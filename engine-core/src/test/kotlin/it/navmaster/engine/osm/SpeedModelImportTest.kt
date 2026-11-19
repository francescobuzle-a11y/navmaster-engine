package it.navmaster.engine.osm

import com.carrotsearch.hppc.LongHashSet
import com.carrotsearch.hppc.LongObjectHashMap
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvLegalSpeeds
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.NodeDelays
import it.navmaster.engine.profile.RoadContext
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.profile.VehicleProfileOptions
import it.navmaster.engine.profile.ZoneInfo
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.weighting.FastestWeighting
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import kotlin.math.min
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** Il modello delle velocità all'import: ORS identico a prima, NAVMASTER senza gli artefatti delle way corte. */
class SpeedModelImportTest {
    private val pbf = File("../data/heidelberg.test.pbf")

    private fun navProfiles(country: String?): List<VehicleProfile> = listOf(
        CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = country)),
        HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = country)),
    )

    private fun <T> imported(profiles: List<VehicleProfile>, source: OsmSource? = null, options: ImportOptions = ImportOptions(), block: (BaseGraph, GraphEncoding, ImportStats) -> T): T {
        val encoding = GraphEncoding(profiles)
        val dir = Files.createTempDirectory("nme-sm").toFile().also { it.deleteOnExit() }
        return BaseGraph(GraphDirectory(dir), encoding.registry).use { graph ->
            val importer = OsmGraphImporter(graph, encoding, null, options)
            val stats = if (source != null) importer.import(source) else importer.import(pbf)
            block(graph, encoding, stats)
        }
    }

    /** Impronta di nodi, distanze, flag grezzi, pesi e tempi (fastest e recommended, due sensi) di tutti gli archi. */
    private fun fingerprint(graph: BaseGraph, encoding: GraphEncoding): Pair<Long, Long> {
        var hFlags = 17L; var hW = 17L
        val ws = encoding.profiles.flatMap { pe -> listOf(WeightingFactory.create(encoding, pe, WeightingMethod.FASTEST), WeightingFactory.create(encoding, pe, WeightingMethod.RECOMMENDED)) }
        val it = graph.createEdgeIterator()
        for (e in 0 until graph.edgeCount) {
            hFlags = hFlags * 31 + graph.nodeA(e); hFlags = hFlags * 31 + graph.nodeB(e)
            hFlags = hFlags * 31 + java.lang.Double.doubleToLongBits(graph.distance(e))
            for (f in graph.getFlags(e)) hFlags = hFlags * 31 + f
            it.setEdge(e, graph.nodeA(e))
            for (w in ws) for (rev in listOf(false, true)) {
                // al millesimo: le ultime cifre dei calcoli in virgola mobile cambiano fra macchine e JVM
                // (Mac e Linux della CI), le velocità e le distanze no (sono nell'impronta dei flag)
                val wt = w.edgeWeight(it, rev)
                hW = hW * 31 + (if (wt.isInfinite()) Long.MAX_VALUE else Math.round(wt * 1000.0))
                hW = hW * 31 + w.edgeMillis(it, rev)
            }
        }
        return hFlags to hW
    }

    @Test
    fun `the ORS model gives the same Heidelberg graph as before`() = imported(listOf(CarProfile(), HgvProfile())) { graph, encoding, stats ->
        // impronta presa col motore di prima del modello NAVMASTER (stesso estratto, stessi profili)
        assertEquals(21146, graph.edgeCount)
        assertEquals(6, graph.intsForFlags)
        val (flags, weights) = fingerprint(graph, encoding)
        assertEquals(-1038835248884379400L, flags, "flag degli archi diversi da prima")
        assertEquals(120975768520584473L, weights, "pesi o tempi degli archi diversi da prima")
        assertEquals(SpeedModel.ORS, stats.speedModel)
        assertTrue(stats.delayEvents.isEmpty())
        for (e in 0 until graph.edgeCount) {
            val f = graph.getFlags(e)
            for (pe in encoding.profiles) { assertEquals(0.0, pe.delay!!.get(f, false)); assertEquals(0.0, pe.delay!!.get(f, true)) }
        }
    }

    @Test
    fun `the NAVMASTER model on Heidelberg - no absurd speeds, delays at junctions only`() {
        // le way primary e secondary col loro cartello, lette a parte dal file
        class W(val highway: String, val signed: Double, val zone: ZoneInfo?, val plain: Boolean)
        val ways = LongObjectHashMap<W>()
        val controlCoords = LongHashSet()
        OsmPbfReader(pbf).read(object : OsmSink {
            override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) {
                if (NodeDelays.classify(tags) != 0) for (dl in doubleArrayOf(-2e-7, 2e-7)) for (dn in doubleArrayOf(-2e-7, 2e-7)) controlCoords.add(key(lat + dl, lon + dn))
            }
            override fun way(way: OsmWay) {
                val h = way.tag("highway") ?: return
                if (h != "primary" && h != "secondary") return
                val signed = ZoneInfo.parseSigned(way.tag("maxspeed"))
                val plain = !way.hasTag("junction") && !way.hasTag("surface", "cobblestone", "sett", "paving_stones", "gravel", "unpaved", "compacted", "cement", "fine_gravel") &&
                    way.tags.keys.none { it.startsWith("maxspeed:") } && !way.hasTag("motorroad", "yes")
                ways.put(way.id, W(h, signed, ZoneInfo.of(way, h), plain))
            }
        }, relations = false)

        // il modello di base, senza le curve del camion (provate a parte, sotto)
        imported(navProfiles("DE"), options = ImportOptions(curves = false)) { graph, encoding, stats ->
            println("NAVMASTER Heidelberg: $stats")
            assertEquals(6, graph.intsForFlags)
            val hgv = encoding.profile("heavyvehicle")
            val it = graph.createEdgeIterator()
            var checked = 0; var delayed = 0; var badDegree2 = 0
            val degree = IntArray(graph.nodeCount)
            for (e in 0 until graph.edgeCount) if (graph.distance(e) > 0) { degree[graph.nodeA(e)]++; degree[graph.nodeB(e)]++ }
            for (e in 0 until graph.edgeCount) {
                val f = graph.getFlags(e)
                val wid = encoding.shared.osmWayId.get(f).toLong()
                val w = ways.get(wid)
                if (w != null && w.plain && !w.signed.isNaN() && w.signed > 50 && hgv.access.get(f, false)) {
                    val ctx = RoadContext.of(w.highway, w.zone?.context, w.signed)
                    val legal = HgvLegalSpeeds.limit(w.zone?.country ?: "DE", ctx) * HgvLegalSpeeds.factor(ctx)
                    // min(0,9 × cartello, legale), meno l'arrotondamento a 5 km/h
                    val minimum = min(0.9 * w.signed, legal) - 5
                    val v = hgv.speed.get(f, false)
                    assertTrue(v >= minimum, "arco $e (way $wid ${w.highway}, cartello ${w.signed}): camion a $v km/h, atteso ≥ $minimum")
                    checked++
                }
                val d = hgv.delay!!
                if (d.get(f, false) > 0 || d.get(f, true) > 0) {
                    delayed++
                    // un arco fra due nodi di grado ≤ 2, senza controlli ai capi né nei punti intermedi, non ha ritardi
                    val a = graph.nodeA(e); val b = graph.nodeB(e)
                    if (degree[a] <= 2 && degree[b] <= 2) {
                        it.setEdge(e, a)
                        val pts = it.fetchWayGeometry(FetchMode.ALL)
                        val anyControl = (0 until pts.size).any { k -> controlCoords.contains(key(pts.lat(k), pts.lon(k))) }
                        if (!anyControl) badDegree2++
                    }
                }
            }
            println("NAVMASTER Heidelberg: $checked archi primary/secondary controllati, $delayed archi con ritardo del camion")
            assertTrue(checked > 300, "pochi archi controllati: $checked")
            assertTrue(delayed > 100, "pochi archi con ritardo: $delayed")
            assertEquals(0, badDegree2, "ritardi su nodi di grado 2 senza controllo")
            assertTrue(stats.delayEvents.getValue("SIGNALS") > 50, "semafori: ${stats.delayEvents}")
            assertTrue(stats.delayEvents.getValue("IMPLIED_GIVE_WAY") > 100, "precedenze implicite: ${stats.delayEvents}")
            assertTrue(stats.delayEvents.getValue("ROUNDABOUT_ENTRY") > 5, "rotatorie: ${stats.delayEvents}")
            assertTrue(stats.hgvSlowSignedShare < 0.01, "camion sotto 30 km/h con cartello > 50: ${stats.hgvSlowSignedShare}")
        }
    }

    @Test
    fun `the NAVMASTER curves on Heidelberg - the truck only slower, never absurd, the car unchanged`() {
        // senza DEM: solo le curve del camion (il DEM ha le sue prove in TerrainImportTest)
        val plain = imported(navProfiles("DE"), options = ImportOptions(curves = false)) { graph, _, _ -> (0 until graph.edgeCount).map { graph.getFlags(it) } }
        imported(navProfiles("DE")) { graph, encoding, stats ->
            val t = stats.terrain!!
            println("NAVMASTER Heidelberg, curve del camion: $t")
            val hgv = encoding.profile("heavyvehicle"); val car = encoding.profile("car-ors")
            val main = setOf(it.navmaster.engine.profile.RoadClass.MOTORWAY, it.navmaster.engine.profile.RoadClass.TRUNK, it.navmaster.engine.profile.RoadClass.PRIMARY,
                it.navmaster.engine.profile.RoadClass.SECONDARY, it.navmaster.engine.profile.RoadClass.TERTIARY)
            var slower = 0
            for (e in 0 until graph.edgeCount) {
                val f = graph.getFlags(e); val p = plain[e]
                for (rev in listOf(false, true)) {
                    assertEquals(car.speed.get(p, rev), car.speed.get(f, rev), "auto, arco $e")
                    val before = hgv.speed.get(p, rev); val now = hgv.speed.get(f, rev)
                    assertTrue(now <= before, "arco $e: il camion più veloce con le curve ($before → $now)")
                    if (now < before) slower++
                    if (encoding.shared.roadClass.get(f) in main && hgv.access.get(f, rev)) assertTrue(now >= 10.0, "arco $e: camion a $now km/h")
                }
            }
            println("sensi d'arco più lenti per le curve: $slower su ${2 * graph.edgeCount}")
            assertEquals(t.changedEdgeDirections.toDouble(), slower.toDouble(), t.changedEdgeDirections * 0.2 + 5, "conteggio dei sensi cambiati")
            assertTrue(t.curves > 100, "curve: ${t.curves}")
            // in media pochi secondi al km: le curve contano nelle anse e negli incroci, non dappertutto
            val secPerKm = t.curveLossMin * 60 / t.km
            assertTrue(secPerKm in 0.5..6.0, "curve: $secPerKm s/km")
            assertTrue(slower < graph.edgeCount / 5, "troppi archi rallentati: $slower")
        }
    }

    @Test
    fun `a flat DSM full of forest blocks and villages - almost no truck km slowed by slope on Heidelberg`() {
        // pianura a 100 m con blocchi di bosco o di case alti 15 m (300 × 300 m, quattro su dieci) e 1 m di rumore
        // ogni 30 m: come un DEM di superficie sulla Val Padana; le strade di Heidelberg ci passano sopra in ogni verso
        fun hash(a: Long, b: Long): Long { var h = a * 0x9E3779B97F4A7C15uL.toLong() + b * 0xC2B2AE3D27D4EB4FuL.toLong(); h = h xor (h ushr 31); h *= 0x94D049BB133111EBuL.toLong(); return h xor (h ushr 29) }
        val dsm = it.navmaster.engine.terrain.ElevationSource { lat, lon ->
            val y = lat * 111_195.0; val x = lon * 111_195.0 * 0.65
            val block = if (Math.floorMod(hash(Math.floorDiv(x.toLong(), 300L), Math.floorDiv(y.toLong(), 300L)), 10L) < 4) 15.0 else 0.0
            val noise = Math.floorMod(hash(Math.floorDiv(x.toLong(), 30L) + 7, Math.floorDiv(y.toLong(), 30L)), 1000L) / 1000.0 * 2 - 1
            100 + block + noise
        }
        // senza i tag incline (veri, sulle salite di Heidelberg: sulle way corte valgono al posto del DEM): conta solo il DEM
        val reader = OsmPbfReader(pbf)
        val noIncline = object : OsmSource {
            override val nodeCount get() = reader.nodeCount
            override val wayCount get() = reader.wayCount
            override fun read(sink: OsmSink, nodes: Boolean, ways: Boolean, relations: Boolean) = reader.read(object : OsmSink {
                override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) = sink.node(id, lat, lon, tags)
                override fun way(way: OsmWay) = sink.way(if (way.hasTag("incline")) OsmWay(way.id, way.tags - "incline", way.nodeIds) else way)
                override fun relation(id: Long, tags: Map<String, String>, members: List<RelationMember>) = sink.relation(id, tags, members)
            }, nodes, ways, relations)
        }
        imported(navProfiles("DE"), noIncline, options = ImportOptions(dem = dsm, curves = false)) { _, _, stats ->
            val t = stats.terrain!!
            println("NAVMASTER Heidelberg su una pianura con boschi: ${"%.1f".format(t.slopeKm)} km frenati su ${"%.0f".format(t.km)}; per classe ${t.slopeShareByClass.mapValues { "%.2f %%".format(100 * it.value) }}")
            assertTrue(stats.demUsed)
            assertTrue(t.slopeKm / t.km < 0.002, "km frenati dalla pendenza in pianura: ${t.slopeKm} su ${t.km}")
            for ((c, share) in t.slopeShareByClass) assertTrue(share < 0.005, "$c: ${100 * share} % dei km frenati in pianura")
        }
    }

    // ---------------------------------------------------------------- dati in memoria

    private class Node(val id: Long, val lat: Double, val lon: Double, val tags: Map<String, String>? = null)

    private class MemoryOsm(val nodes: List<Node>, val ways: List<OsmWay>) : OsmSource {
        override val nodeCount: Long get() = nodes.size.toLong()
        override val wayCount: Long get() = ways.size.toLong()
        override fun read(sink: OsmSink, nodes: Boolean, ways: Boolean, relations: Boolean) {
            if (nodes) for (n in this.nodes) sink.node(n.id, n.lat, n.lon, n.tags)
            if (ways) for (w in this.ways) sink.way(w)
        }
    }

    /** 21 nodi in fila (100 m l'uno dall'altro), un semaforo sul nodo 10, divisi in [parts] way uguali. */
    private fun straightRoad(parts: Int, tags: Map<String, String>): MemoryOsm {
        val nodes = (0..20).map { i -> Node(100L + i, 49.0 + i * 0.000899, 8.0, if (i == 10) mapOf("highway" to "traffic_signals") else null) }
        val per = 20 / parts
        val ways = (0 until parts).map { p -> OsmWay(1000L + p, tags, LongArray(per + 1) { 100L + p * per + it }) }
        return MemoryOsm(nodes, ways)
    }

    /** Il tempo (s) da un capo all'altro della fila: la somma dei pesi in avanti (gli archi seguono le way). */
    private fun chainSeconds(graph: BaseGraph, encoding: GraphEncoding, profile: String): Double {
        val w = FastestWeighting(encoding.profile(profile))
        val it = graph.createEdgeIterator()
        var s = 0.0
        for (e in 0 until graph.edgeCount) { it.setEdge(e, graph.nodeA(e)); s += w.edgeWeight(it, false) }
        return s
    }

    @Test
    fun `a road split in 10 short ways takes as long as one way`() {
        val tags = mapOf("highway" to "primary", "maxspeed" to "90", "ref" to "52")
        val one = imported(navProfiles("CZ"), straightRoad(1, tags)) { g, enc, _ ->
            assertEquals(1, g.edgeCount); chainSeconds(g, enc, "heavyvehicle") to chainSeconds(g, enc, "car-ors") }
        val ten = imported(navProfiles("CZ"), straightRoad(10, tags)) { g, enc, _ ->
            assertEquals(10, g.edgeCount); chainSeconds(g, enc, "heavyvehicle") to chainSeconds(g, enc, "car-ors") }
        println("2 km di primary a 90 in CZ: camion ${one.first} s (1 way) / ${ten.first} s (10 way), auto ${one.second} / ${ten.second}")
        assertEquals(one.first, ten.first, 0.01)
        assertEquals(one.second, ten.second, 0.01)
        // 2 km a 70 km/h (72 arrotondato) più un semaforo a 70
        val signal = Math.round(NodeDelays.seconds(NodeDelays.Control.SIGNALS, 70.0, 0.5, 1.0)).toDouble()
        assertEquals(2000.0 / 70 * 3.6 + signal, one.first, 1.0)

        // il modello ORS: la stessa strada spezzata costa molto di più al camion (l'artefatto della 52 ceca)
        val orsOne = imported(listOf(CarProfile(), HgvProfile()), straightRoad(1, tags)) { g, enc, _ -> chainSeconds(g, enc, "heavyvehicle") }
        val orsTen = imported(listOf(CarProfile(), HgvProfile()), straightRoad(10, tags)) { g, enc, _ -> chainSeconds(g, enc, "heavyvehicle") }
        println("modello ORS, camion: $orsOne s (1 way) / $orsTen s (10 way)")
        assertTrue(orsTen > orsOne * 1.2, "ORS: $orsOne / $orsTen")
    }

    @Test
    fun `the country sets the legal truck speed`() {
        val tags = mapOf("highway" to "primary", "maxspeed" to "90")
        fun truckSpeed(country: String?) = imported(navProfiles(country), straightRoad(1, tags)) { g, enc, _ -> enc.profile("heavyvehicle").speed.get(g.getFlags(0), false) }
        assertEquals(70.0, truckSpeed("CZ")) // 72
        assertEquals(65.0, truckSpeed("IT")) // 63
        assertEquals(55.0, truckSpeed("DE")) // 54
        assertEquals(70.0, truckSpeed(null)) // riserva 80 × 0,9
        assertEquals(80.0, truckSpeed("SK")) // 90 × 0,9 = 81 (zákon 8/2009 §16)
        assertEquals(70.0, truckSpeed("HR")) // 80 × 0,9 = 72 (ZSPC art. 55)
        assertEquals(55.0, truckSpeed("AL")) // 60 × 0,9 = 54
        // le trunk (EXPRESS, fattore pieno) di Spagna e Polonia sono quasi sempre a una carreggiata: limite delle strade normali
        val trunk = mapOf("highway" to "trunk")
        fun trunkSpeed(country: String?) = imported(navProfiles(country), straightRoad(1, trunk)) { g, enc, _ -> enc.profile("heavyvehicle").speed.get(g.getFlags(0), false) }
        assertEquals(80.0, trunkSpeed("ES"))
        assertEquals(70.0, trunkSpeed("PL"))
        assertEquals(80.0, trunkSpeed("TR")) // 85 legale, ma senza cartello la trunk resta alla velocità di base del camion (80)
    }

    @Test
    fun `customs lanes marked hgv=private on a main road - open to the truck only with the NAVMASTER model`() {
        // valico di Mikulov (E461): statale 52 → corsia della vecchia dogana hgv=private → statale 52; e una
        // terziaria privata, che resta chiusa
        val nodes = listOf(Node(1, 48.786, 16.638), Node(2, 48.789, 16.637), Node(3, 48.790, 16.636), Node(4, 48.793, 16.634),
            Node(5, 48.789, 16.640), Node(6, 48.790, 16.641))
        val ways = listOf(
            OsmWay(10, mapOf("highway" to "primary", "ref" to "52"), longArrayOf(1, 2)),
            OsmWay(11, mapOf("highway" to "primary", "hgv" to "private", "oneway" to "no"), longArrayOf(2, 3)),
            OsmWay(12, mapOf("highway" to "primary", "ref" to "52"), longArrayOf(3, 4)),
            OsmWay(13, mapOf("highway" to "tertiary", "hgv" to "private"), longArrayOf(5, 6)),
            OsmWay(14, mapOf("highway" to "primary", "hgv" to "no"), longArrayOf(2, 5)),
        )
        fun open(profiles: List<VehicleProfile>): Map<Long, Boolean> = imported(profiles, MemoryOsm(nodes, ways)) { g, enc, _ ->
            val hgv = enc.profile("heavyvehicle")
            val filter = it.navmaster.engine.weighting.EdgeFilterFactory.create(enc, "heavyvehicle")
            val iter = g.createEdgeIterator()
            (0 until g.edgeCount).associate { e ->
                iter.setEdge(e, g.nodeA(e))
                enc.shared.osmWayId.get(g.getFlags(e)).toLong() to (hgv.access.get(g.getFlags(e), false) && filter.accept(iter))
            }
        }
        val nav = open(navProfiles("CZ"))
        assertEquals(true, nav[11], "corsia di dogana: aperta col modello NAVMASTER")
        assertEquals(false, nav[13] ?: false, "terziaria privata")
        assertEquals(false, nav[14] ?: false, "hgv=no")
        assertEquals(true, nav[10]); assertEquals(true, nav[12])
        val ors = open(listOf(CarProfile(), HgvProfile()))
        assertEquals(false, ors[11] ?: false, "ORS: hgv=private chiude, come prima")
    }

    @Test
    fun `an interior stop counts toward the nearest end, without an implied give way on top`() {
        // tertiary 0→1→2→3 (100 m, 100 m, 10 m) con lo stop sul nodo 2, poi la primary 3 che prosegue (4–3–5)
        val nodes = listOf(
            Node(1, 49.0, 8.0), Node(2, 49.000899, 8.0), Node(3, 49.001798, 8.0, mapOf("highway" to "stop")), Node(4, 49.001888, 8.0),
            Node(5, 49.001888, 7.99), Node(6, 49.001888, 8.01),
        )
        val ways = listOf(
            OsmWay(10, mapOf("highway" to "tertiary", "maxspeed" to "50"), longArrayOf(1, 2, 3, 4)),
            OsmWay(11, mapOf("highway" to "primary", "maxspeed" to "70"), longArrayOf(5, 4, 6)),
        )
        imported(navProfiles("IT"), MemoryOsm(nodes, ways)) { g, enc, stats ->
            val hgv = enc.profile("heavyvehicle")
            val tertiary = (0 until g.edgeCount).first { enc.shared.osmWayId.get(g.getFlags(it)) == 10 }
            val f = g.getFlags(tertiary)
            val stop = Math.round(NodeDelays.seconds(NodeDelays.Control.STOP, 45.0, 0.5, 1.0)).toDouble()
            assertEquals(stop, hgv.delay!!.get(f, false), "verso la primary: solo lo stop")
            assertEquals(0.0, hgv.delay!!.get(f, true), "verso il capo lontano: nulla")
            assertEquals(0, stats.delayEvents.getValue("IMPLIED_GIVE_WAY"))
            assertEquals(1, stats.delayEvents.getValue("STOP"))
        }
    }

    @Test
    fun `an approach signal without direction counts only toward the junction it is next to`() {
        // primary 1→2→3→4 (100 m, 80 m, 20 m) col semaforo sul nodo 3, a 20 m dall'incrocio 4; la primary
        // prosegue (4→5) e una tertiary entra in 4 (6→4); il semaforo sul nodo 8, a metà di 7→8→9, vale nei due sensi
        val nodes = listOf(
            Node(1, 49.0, 8.0), Node(2, 49.000899, 8.0), Node(3, 49.001618, 8.0, mapOf("highway" to "traffic_signals")), Node(4, 49.001798, 8.0),
            Node(5, 49.002697, 8.0), Node(6, 49.001798, 8.01),
            Node(7, 49.1, 8.0), Node(8, 49.100899, 8.0, mapOf("highway" to "traffic_signals")), Node(9, 49.101798, 8.0),
        )
        val ways = listOf(
            OsmWay(10, mapOf("highway" to "primary", "maxspeed" to "50"), longArrayOf(1, 2, 3, 4)),
            OsmWay(11, mapOf("highway" to "primary", "maxspeed" to "50"), longArrayOf(4, 5)),
            OsmWay(12, mapOf("highway" to "tertiary", "maxspeed" to "50"), longArrayOf(6, 4)),
            OsmWay(13, mapOf("highway" to "primary", "maxspeed" to "50"), longArrayOf(7, 8, 9)),
        )
        imported(navProfiles("DE"), MemoryOsm(nodes, ways)) { g, enc, _ ->
            val hgv = enc.profile("heavyvehicle")
            fun edgeOf(way: Long) = (0 until g.edgeCount).first { enc.shared.osmWayId.get(g.getFlags(it)).toLong() == way }
            val signal = Math.round(NodeDelays.seconds(NodeDelays.Control.SIGNALS, 45.0, 0.5, 1.0)).toDouble()
            val approach = g.getFlags(edgeOf(10))
            assertEquals(signal, hgv.delay!!.get(approach, false), "verso l'incrocio: il semaforo")
            assertEquals(0.0, hgv.delay!!.get(approach, true), "in uscita dall'incrocio: nulla")
            val middle = g.getFlags(edgeOf(13))
            assertEquals(signal, hgv.delay!!.get(middle, false)); assertEquals(signal, hgv.delay!!.get(middle, true))
        }
    }

    /** Coordinate arrotondate al decimetro circa (il grafo le tronca a 1e-7 gradi). */
    private fun key(lat: Double, lon: Double): Long = Math.round(lat * 1e6) shl 32 or (Math.round(lon * 1e6) and 0xffffffffL)
}
