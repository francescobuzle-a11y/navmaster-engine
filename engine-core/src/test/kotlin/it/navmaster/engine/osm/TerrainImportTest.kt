package it.navmaster.engine.osm

import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.ClimbModel
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.profile.VehicleProfileOptions
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.terrain.ElevationSource
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.FastestWeighting
import java.nio.file.Files
import kotlin.math.abs
import kotlin.math.cos
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Salite, discese e curve del camion all'import (modello NAVMASTER): pendenze dal DEM per way intera,
 * archi letti a pezzi (anche dopo anelli, barriere e nodi mancanti), ponti, `incline`, tornanti; i
 * grafi ORS e l'auto identici con o senza DEM.
 */
class TerrainImportTest {
    private val lat0 = 46.0
    private val lon0 = 11.0
    private val kx = cos(Math.toRadians(lat0)) * DistanceCalc.METERS_PER_DEGREE

    private class Node(val id: Long, val lat: Double, val lon: Double, val tags: Map<String, String>? = null)

    private class MemoryOsm(val nodes: List<Node>, val ways: List<OsmWay>) : OsmSource {
        override val nodeCount: Long get() = nodes.size.toLong()
        override val wayCount: Long get() = ways.size.toLong()
        override fun read(sink: OsmSink, nodes: Boolean, ways: Boolean, relations: Boolean) {
            if (nodes) for (n in this.nodes) sink.node(n.id, n.lat, n.lon, n.tags)
            if (ways) for (w in this.ways) sink.way(w)
        }
    }

    /** Un nodo a [x] m a est e [y] m a nord di (46, 11). */
    private fun node(id: Long, x: Double, y: Double, tags: Map<String, String>? = null) =
        Node(id, lat0 + y / DistanceCalc.METERS_PER_DEGREE, lon0 + x / kx, tags)

    /** Un piano che sale verso nord del [grade] (frazione). */
    private fun plane(grade: Double) = ElevationSource { lat, _ -> 500 + (lat - lat0) * DistanceCalc.METERS_PER_DEGREE * grade }

    private fun navProfiles(): List<VehicleProfile> = listOf(
        CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "IT")),
        HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "IT")),
    )

    private fun orsProfiles(): List<VehicleProfile> = listOf(CarProfile(), HgvProfile())

    private fun <T> imported(profiles: List<VehicleProfile>, osm: MemoryOsm, options: ImportOptions, block: (BaseGraph, GraphEncoding, ImportStats) -> T): T {
        val encoding = GraphEncoding(profiles)
        val dir = Files.createTempDirectory("nme-terrain").toFile().also { it.deleteOnExit() }
        return BaseGraph(GraphDirectory(dir), encoding.registry).use { graph ->
            val stats = OsmGraphImporter(graph, encoding, null, options).import(osm)
            block(graph, encoding, stats)
        }
    }

    /** Il tempo (s) del camion sugli archi veri, nel senso delle way ([forward]) o contro. */
    private fun seconds(graph: BaseGraph, encoding: GraphEncoding, forward: Boolean, profile: String = "heavyvehicle"): Double {
        val w = FastestWeighting(encoding.profile(profile))
        val it = graph.createEdgeIterator()
        var s = 0.0
        for (e in 0 until graph.edgeCount) { if (graph.distance(e) <= 0) continue; it.setEdge(e, graph.nodeA(e)); s += w.edgeWeight(it, !forward) }
        return s
    }

    /** Le velocità del camion (avanti, indietro) degli archi veri, in ordine. */
    private fun speeds(graph: BaseGraph, encoding: GraphEncoding, profile: String = "heavyvehicle"): List<Pair<Double, Double>> {
        val pe = encoding.profile(profile)
        return (0 until graph.edgeCount).filter { graph.distance(it) > 0 }.map { e -> val f = graph.getFlags(e); pe.speed.get(f, false) to pe.speed.get(f, true) }
    }

    /** Una strada diritta verso nord: [count] tratti di [step] m, spezzata in [parts] way uguali. */
    private fun northRoad(count: Int, step: Double, parts: Int, tags: Map<String, String>): MemoryOsm {
        val nodes = (0..count).map { node(100L + it, 0.0, it * step) }
        val per = count / parts
        val ways = (0 until parts).map { p -> OsmWay(1000L + p, tags, LongArray(per + 1) { 100L + p * per + it }) }
        return MemoryOsm(nodes, ways)
    }

    private val primary = mapOf("highway" to "primary", "maxspeed" to "90")

    @Test
    fun `a 6 percent road - slower uphill, downhill cap downhill, A to B differs from B to A`() {
        val road = northRoad(20, 100.0, 1, primary)
        val flat = imported(navProfiles(), road, ImportOptions(dem = plane(0.0))) { g, enc, _ -> speeds(g, enc).single() }
        assertEquals(65.0 to 65.0, flat, "in piano la velocità di base (63 → 65)")
        imported(navProfiles(), road, ImportOptions(dem = plane(0.06), demSource = "prova")) { g, enc, stats ->
            val (fwd, bwd) = speeds(g, enc).single()
            println("6 %: camion in salita $fwd km/h, in discesa $bwd km/h; ${stats.terrain}")
            assertEquals(40.0, fwd, "in salita al 6 %: 38 km/h, arrotondati a 40")
            assertEquals(50.0, bwd, "in discesa al 6 %: il tetto di 50 km/h")
            val up = seconds(g, enc, true); val down = seconds(g, enc, false)
            assertEquals(2000 / 40.0 * 3.6, up, 0.5)
            assertEquals(2000 / 50.0 * 3.6, down, 0.5)
            assertTrue(up > down + 30, "A→B $up s, B→A $down s")
            // l'auto non cambia
            assertEquals(speeds(g, enc, "car-ors").single().first, speeds(g, enc, "car-ors").single().second)
            val t = assertNotNull(stats.terrain)
            assertTrue(t.dem); assertEquals(1.0, t.demCoverage, 1e-9)
            assertEquals(4.0, t.km, 0.01) // 2 km per due sensi
            assertEquals(4.0, t.slopeKm, 0.05)
            assertTrue(t.slopeLossMin > 1.0, "minuti persi: ${t.slopeLossMin}")
            assertEquals(0, t.curves)
            assertEquals(2, t.changedEdgeDirections)
        }
    }

    @Test
    fun `ten short ways on a slope take as long as one long way`() {
        // 1,5 km al 6 %: una way (pendenza dalla regressione), 10 way da 150 m e 30 da 50 m (dai capi)
        val one = imported(navProfiles(), northRoad(30, 50.0, 1, primary), ImportOptions(dem = plane(0.06))) { g, enc, _ -> seconds(g, enc, true) to seconds(g, enc, false) }
        val ten = imported(navProfiles(), northRoad(30, 50.0, 10, primary), ImportOptions(dem = plane(0.06))) { g, enc, _ -> assertEquals(10, g.edgeCount); seconds(g, enc, true) to seconds(g, enc, false) }
        val thirty = imported(navProfiles(), northRoad(30, 50.0, 30, primary), ImportOptions(dem = plane(0.06))) { g, enc, _ -> seconds(g, enc, true) to seconds(g, enc, false) }
        println("1,5 km al 6 %: 1 way $one s, 10 way $ten s, 30 way $thirty s")
        assertEquals(one.first, ten.first, one.first * 0.02)
        assertEquals(one.second, ten.second, one.second * 0.02)
        assertEquals(one.first, thirty.first, one.first * 0.02)
        assertEquals(one.second, thirty.second, one.second * 0.02)
        assertTrue(one.first > one.second + 20)
    }

    @Test
    fun `a bridge over a valley keeps the grade of its ends`() {
        // una valle profonda 60 m in mezzo a 1 km di strada piana
        val valley = ElevationSource { lat, _ ->
            val d = (lat - lat0) * DistanceCalc.METERS_PER_DEGREE
            500 - 60 * maxOf(0.0, 1 - abs(d - 500) / 300)
        }
        val bridge = northRoad(20, 50.0, 1, primary + ("bridge" to "yes") + ("layer" to "1"))
        val road = northRoad(20, 50.0, 1, primary)
        imported(navProfiles(), bridge, ImportOptions(dem = valley)) { g, enc, _ -> assertEquals(65.0 to 65.0, speeds(g, enc).single(), "ponte: in piano") }
        imported(navProfiles(), road, ImportOptions(dem = valley)) { g, enc, _ ->
            val (fwd, bwd) = speeds(g, enc).single()
            println("la stessa strada senza ponte: $fwd / $bwd km/h")
            assertTrue(fwd < 50 && bwd < 50, "senza ponte scende e risale: $fwd / $bwd")
        }
        // una galleria sotto un monte, coi capi alla stessa quota
        val mountain = ElevationSource { lat, _ -> val d = (lat - lat0) * DistanceCalc.METERS_PER_DEGREE; 500 + 200 * maxOf(0.0, 1 - abs(d - 500) / 400) }
        imported(navProfiles(), northRoad(20, 50.0, 1, primary + ("tunnel" to "yes")), ImportOptions(dem = mountain)) { g, enc, _ -> assertEquals(65.0 to 65.0, speeds(g, enc).single(), "galleria") }
    }

    /** Una strada a tornanti: [legs] tratti diritti di [leg] m alternati nord/sud, uniti da tornanti di raggio [r] m. */
    private fun serpentine(legs: Int, leg: Double, r: Double, tags: Map<String, String>): Pair<MemoryOsm, Double> {
        val pts = ArrayList<Pair<Double, Double>>()
        var x = 0.0
        for (l in 0 until legs) {
            val north = l % 2 == 0
            val y0 = if (north) 0.0 else leg; val y1 = if (north) leg else 0.0
            val n = (leg / 20).toInt()
            for (k in 0..n) pts += x to y0 + (y1 - y0) * k / n
            if (l == legs - 1) break
            // mezzo giro intorno a (x + r, y1), un punto ogni ~3 m
            for (k in 1 until 12) {
                val a = Math.PI - k * Math.PI / 12
                pts += (x + r + r * Math.cos(a)) to (y1 + (if (north) 1 else -1) * r * Math.sin(a))
            }
            x += 2 * r
        }
        val nodes = pts.mapIndexed { i, p -> node(100L + i, p.first, p.second) }
        var len = 0.0
        for (i in 1 until nodes.size) len += DistanceCalc.haversine(nodes[i - 1].lat, nodes[i - 1].lon, nodes[i].lat, nodes[i].lon)
        return MemoryOsm(nodes, listOf(OsmWay(1000, tags, LongArray(nodes.size) { 100L + it }))) to len
    }

    @Test
    fun `five hairpins of 12 m make the road at least 90 s slower than a straight one of the same length`() {
        val tags = mapOf("highway" to "secondary", "maxspeed" to "90")
        val (zigzag, len) = serpentine(6, 400.0, 12.0, tags)
        val straight = northRoad((len / 20).toInt(), len / (len / 20).toInt(), 1, tags)
        val tStraight = imported(navProfiles(), straight, ImportOptions()) { g, enc, _ -> seconds(g, enc, true) }
        imported(navProfiles(), zigzag, ImportOptions()) { g, enc, stats ->
            val t = seconds(g, enc, true); val back = seconds(g, enc, false)
            println("serpentina di ${"%.0f".format(len)} m: $t s (indietro $back s), diritta $tStraight s; ${stats.terrain}")
            assertTrue(t - tStraight >= 90, "5 tornanti: solo ${t - tStraight} s in più")
            assertTrue(t - tStraight <= 200, "5 tornanti: ${t - tStraight} s in più, troppi")
            assertEquals(t, back, 5.0)
            assertEquals(5, stats.terrain!!.curves)
            assertTrue(stats.terrain!!.curveLossMin > 1.5)
            // l'auto non ha curve (per ora)
            val car = seconds(g, enc, true, "car-ors")
            val carStraight = imported(navProfiles(), straight, ImportOptions()) { g2, enc2, _ -> seconds(g2, enc2, true, "car-ors") }
            assertEquals(carStraight, car, carStraight * 0.01)
        }
        // senza curve (solo per le prove): come la diritta
        imported(navProfiles(), zigzag, ImportOptions(curves = false)) { g, enc, stats ->
            assertEquals(tStraight, seconds(g, enc, true), tStraight * 0.01)
            assertNull(stats.terrain)
        }
        // le rotatorie hanno già il loro tetto: nessuna curva
        val (ring, _) = serpentine(6, 400.0, 12.0, tags + ("junction" to "roundabout"))
        imported(navProfiles(), ring, ImportOptions()) { _, _, stats -> assertEquals(0, stats.terrain!!.curves) }
    }

    @Test
    fun `a numeric incline replaces the DEM on short ways and where the DEM is missing`() {
        val flat = ElevationSource { _, _ -> 200.0 }
        val none = ElevationSource { _, _ -> Double.NaN }
        // 150 m con incline=10 % (statale: tetto 10 %), DEM piano
        val short = northRoad(3, 50.0, 1, primary + ("incline" to "10%"))
        imported(navProfiles(), short, ImportOptions(dem = flat)) { g, enc, _ -> assertEquals(25.0 to 35.0, speeds(g, enc).single(), "incline 10 %: 24 in salita, 35 in discesa") }
        imported(navProfiles(), northRoad(3, 50.0, 1, primary + ("incline" to "-8 %")), ImportOptions(dem = flat)) { g, enc, _ -> assertEquals(40.0 to 30.0, speeds(g, enc).single(), "incline -8 %") }
        imported(navProfiles(), northRoad(3, 50.0, 1, primary + ("incline" to "up")), ImportOptions(dem = flat)) { g, enc, _ -> assertEquals(65.0 to 65.0, speeds(g, enc).single(), "incline=up non dice quanto") }
        // 1 km fuori dal DEM con incline=8 %: vale l'incline
        val long = northRoad(20, 50.0, 1, primary + ("incline" to "8%"))
        imported(navProfiles(), long, ImportOptions(dem = none)) { g, enc, stats ->
            assertEquals(30.0 to 40.0, speeds(g, enc).single(), "fuori dal DEM: incline 8 %")
            assertEquals(0.0, stats.terrain!!.demCoverage, 1e-9)
        }
        // 1 km col DEM piano: il DEM vince sull'incline delle way lunghe
        imported(navProfiles(), long, ImportOptions(dem = flat)) { g, enc, _ -> assertEquals(65.0 to 65.0, speeds(g, enc).single()) }
        // senza DEM niente pendenze, nemmeno dall'incline
        imported(navProfiles(), short, ImportOptions()) { g, enc, _ -> assertEquals(65.0 to 65.0, speeds(g, enc).single()) }
    }

    @Test
    fun `edges read their own piece of the way - closed loop, barrier copy, missing node`() {
        val up = 40.0 to 50.0 // al 6 %: in salita 40, in discesa 50 (base 65)
        // una barriera a metà (cancello: il camion non passa, ma i due pezzi di strada restano)
        val nodes = (0..20).map { node(100L + it, 0.0, it * 100.0, if (it == 10) mapOf("barrier" to "gate") else null) }
        imported(navProfiles(), MemoryOsm(nodes, listOf(OsmWay(1000, primary, LongArray(21) { 100L + it }))), ImportOptions(dem = plane(0.06))) { g, enc, _ ->
            assertEquals(3, g.edgeCount, "due pezzi più l'arco della barriera")
            assertEquals(listOf(up, up), speeds(g, enc))
        }
        // un nodo mancante dall'estratto taglia la way: due archi, ciascuno al 6 %
        val holed = (0..20).filter { it != 10 }.map { node(100L + it, 0.0, it * 100.0) }
        imported(navProfiles(), MemoryOsm(holed, listOf(OsmWay(1000, primary, LongArray(21) { 100L + it }))), ImportOptions(dem = plane(0.06))) { g, enc, _ ->
            assertEquals(listOf(up, up), speeds(g, enc))
        }
        // un anello chiuso: 1 km a nord in salita, 700 m a est in piano, 1 km a sud in discesa, 700 m a ovest in piano;
        // l'anello si spezza sul penultimo nodo: l'ultimo arco (100 m verso ovest, in piano, 600 m dopo l'angolo) resta alla base
        val ring = ArrayList<Node>()
        for (k in 0..10) ring += node(100L + k, 0.0, k * 100.0)
        for (k in 1..7) ring += node(110L + k, k * 100.0, 1000.0)
        for (k in 1..10) ring += node(117L + k, 700.0, 1000 - k * 100.0)
        for (k in 1..6) ring += node(127L + k, 700 - k * 100.0, 0.0)
        val ids = ring.map { it.id } + 100L
        imported(navProfiles(), MemoryOsm(ring, listOf(OsmWay(1000, primary, ids.toLongArray()))), ImportOptions(dem = plane(0.06))) { g, enc, _ ->
            val s = speeds(g, enc)
            assertEquals(2, s.size)
            assertEquals(65.0 to 65.0, s[1], "l'ultimo pezzo dell'anello, in piano")
            assertEquals(s[0].first, s[0].second, 5.0, "il primo pezzo sale e scende quanto scende e sale: ${s[0]}")
            assertTrue(s[0].first < 65.0)
        }
    }

    /** I flag di tutti gli archi. */
    private fun allFlags(g: BaseGraph): List<List<Int>> = (0 until g.edgeCount).map { g.getFlags(it).toList() }

    @Test
    fun `ORS graphs and the car are the same with or without a DEM`() {
        val (zigzag, _) = serpentine(4, 300.0, 12.0, mapOf("highway" to "secondary"))
        val osm = MemoryOsm(zigzag.nodes + northRoad(20, 100.0, 1, primary).nodes.map { Node(it.id + 10_000, it.lat, it.lon + 0.01) },
            zigzag.ways + OsmWay(5000, primary, LongArray(21) { 10_100L + it }))
        val dem = plane(0.08)
        // ORS: nessun terreno (neanche le curve), identico con o senza DEM
        val orsPlain = imported(orsProfiles(), osm, ImportOptions()) { g, _, stats -> assertNull(stats.terrain); allFlags(g) }
        val orsDem = imported(orsProfiles(), osm, ImportOptions(dem = dem)) { g, _, stats -> assertNull(stats.terrain); allFlags(g) }
        assertEquals(orsPlain, orsDem)
        assertNull(HgvProfile().climb, "il camion ORS non ha un modello di salita")
        assertNull(CarProfile(VehicleProfileOptions(speedModel = SpeedModel.NAVMASTER)).climb, "l'auto nemmeno")
        assertEquals(ClimbModel.HGV, HgvProfile(VehicleProfileOptions(speedModel = SpeedModel.NAVMASTER)).climb)
        // NAVMASTER: l'auto identica con o senza DEM, il camion no
        fun carAndTruck(o: ImportOptions) = imported(navProfiles(), osm, o) { g, enc, _ ->
            val car = enc.profile("car-ors"); val hgv = enc.profile("heavyvehicle")
            (0 until g.edgeCount).map { e -> val f = g.getFlags(e); listOf(car.speed.get(f, false), car.speed.get(f, true), car.delay!!.get(f, false), car.delay!!.get(f, true)) } to
                (0 until g.edgeCount).map { e -> val f = g.getFlags(e); hgv.speed.get(f, false) to hgv.speed.get(f, true) }
        }
        val plain = carAndTruck(ImportOptions(curves = false))
        val withDem = carAndTruck(ImportOptions(dem = dem))
        assertEquals(plain.first, withDem.first, "auto")
        assertTrue(plain.second != withDem.second, "camion")
    }
}
