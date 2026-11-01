package it.navmaster.engine.osm

import com.carrotsearch.hppc.LongIntHashMap
import com.carrotsearch.hppc.LongArrayList
import com.carrotsearch.hppc.LongObjectHashMap
import it.navmaster.engine.profile.GuidanceParser
import it.navmaster.engine.storage.GuidanceStore
import it.navmaster.engine.storage.PointStore
import it.navmaster.engine.profile.Access
import it.navmaster.engine.profile.WayAttributeParser
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.util.DistanceCalc
import java.io.File

/** Divieto di svolta letto da una relazione `type=restriction` (applicato in una fase successiva). */
class TurnRestrictionRelation(val id: Long, val fromWay: Long, val viaNode: Long, val toWay: Long, val restriction: String, val vehicle: String?, val except: Set<String>)

data class ImportStats(
    val osmNodes: Long, val osmWays: Long, val acceptedWays: Long,
    val towers: Int, val pillars: Int, val edges: Int, val barrierEdges: Int,
    /** Divieti di svolta applicati al grafo (0 se non è stato passato un TurnCostStorage). */
    val turnRestrictions: Int,
    /** Relazioni di divieto lette dal file. */
    val turnRestrictionRelations: Int,
    val millis: Long,
)

/**
 * Costruisce un [BaseGraph] da un file PBF in due passate, come GraphHopper:
 *
 * 1. legge le way accettate da almeno un profilo e conta quante volte ogni nodo è usato;
 *    raccoglie le relazioni di divieto di svolta;
 * 2. legge i nodi (torre = usato da ≥2 way, estremo di way, o barriera; pillar = intermedio),
 *    poi le way: le spezza alle torri e crea un arco per segmento con geometria, distanza e flag.
 *
 * I nodi assenti dall'estratto tagliano il segmento. Un nodo barriera all'interno di una way
 * viene sdoppiato: un arco di lunghezza zero fra le due copie porta il blocco per i profili interessati.
 */
/** Opzioni di costruzione del grafo. */
data class ImportOptions(
    /** Vedi [WayAttributeParser.parse]: con `false` il tag `width` non diventa un limite di larghezza. */
    val widthFromCarriageway: Boolean = true,
    /** Scrive il file `guidance` (limiti di velocità, corsie, destinazioni, svincoli, caselli): solo dati, nessun effetto sul calcolo. */
    val guidance: Boolean = true,
)

class OsmGraphImporter(
    private val graph: BaseGraph, private val encoding: GraphEncoding, private val turnCosts: TurnCostStorage? = null,
    private val options: ImportOptions = ImportOptions(),
    /** Se presente, riceve i dati di guida di ogni arco (file `guidance`). */
    private val guidance: GuidanceStore? = null,
    /** Se presente, riceve autovelox, Tutor e fotocamere ai semafori (file `points`). */
    private val points: PointStore? = null,
) {
    private val profiles = encoding.profiles
    private val junctions = LongObjectHashMap<String>()   // id OSM → svincolo "ref|nome"
    private val specialNodes = LongObjectHashMap<String>() // id OSM → casello / portale / dogana
    private class Enforcement(val type: Int, val maxspeed: Int, val device: Long, val from: Long, val to: Long)
    private val enforcements = ArrayList<Enforcement>()
    private val neededNodes = LongObjectHashMap<DoubleArray>() // nodi delle relazioni enforcement → [lat, lon]
    private val cameraNodes = com.carrotsearch.hppc.LongHashSet()  // nodi highway=speed_camera già scritti
    private val relationSigns = HashMap<Long, MutableList<Pair<Long, String>>>() // way "to" → (nodo intersection, cartello)
    private val nodeUse = LongIntHashMap()       // id OSM → bit: 1 = visto una volta, 2 = più volte, 4 = estremo di way
    private val towers = LongIntHashMap()        // id OSM → id nodo del grafo
    private val pillars = LongIntHashMap()       // id OSM → indice pillar
    private var pillarLat = DoubleArray(1024)
    private var pillarLon = DoubleArray(1024)
    private var pillarCount = 0
    private val barrierTags = LongObjectHashMap<Map<String, String>>()
    val turnRestrictions = ArrayList<TurnRestrictionRelation>()

    private var acceptedWays = 0L
    private var barrierEdges = 0
    private val wayAccess = Array(profiles.size) { Access.CAN_SKIP }

    fun import(pbf: File): ImportStats {
        val start = System.currentTimeMillis()
        val reader = OsmPbfReader(pbf)
        reader.read(Pass1(), nodes = false)
        reader.read(Pass2Nodes(), ways = false, relations = false)
        reader.read(Pass2Ways(), nodes = false, relations = false)
        // autovelox e Tutor delle relazioni enforcement (il nodo device con highway=speed_camera è già scritto)
        if (points != null) for (e in enforcements) {
            fun at(id: Long): DoubleArray? = if (id < 0) null else neededNodes.get(id)?.takeIf { it.size == 2 }
            when (e.type) {
                PointStore.AVERAGE_SPEED_START -> {
                    at(e.from)?.let { points.add(it[0], it[1], PointStore.AVERAGE_SPEED_START, e.maxspeed, -1) }
                    at(e.to)?.let { points.add(it[0], it[1], PointStore.AVERAGE_SPEED_END, e.maxspeed, -1) }
                }
                else -> if (e.device < 0 || !cameraNodes.contains(e.device)) (at(e.device) ?: at(e.from))?.let { points.add(it[0], it[1], e.type, e.maxspeed, -1) }
            }
        }
        var appliedRestrictions = 0
        if (turnCosts != null) {
            val applier = TurnRestrictionApplier(graph, encoding, turnCosts)
            applier.apply(turnRestrictions, towers)
            appliedRestrictions = applier.applied
        }
        graph.freeze()
        return ImportStats(reader.nodeCount, reader.wayCount, acceptedWays, graph.nodeCount, pillarCount, graph.edgeCount, barrierEdges, appliedRestrictions, turnRestrictions.size, System.currentTimeMillis() - start)
    }

    /** Vero se almeno un profilo può percorrere la way; riempie [wayAccess]. */
    private fun acceptWay(way: OsmWay): Boolean {
        if (way.nodeIds.size < 2 || way.tags.isEmpty()) return false
        var any = false
        for (i in profiles.indices) {
            wayAccess[i] = profiles[i].profile.access(way)
            if (!wayAccess[i].canSkip) any = true
        }
        return any
    }

    private inner class Pass1 : OsmSink {
        override fun way(way: OsmWay) {
            if (!acceptWay(way)) return
            acceptedWays++
            val ids = way.nodeIds
            for (i in ids.indices) {
                val id = ids[i]
                var v = nodeUse.getOrDefault(id, 0)
                v = if (v and 3 == 0) v or 1 else v or 2
                if (i == 0 || i == ids.size - 1) v = v or 4
                nodeUse.put(id, v)
            }
        }

        override fun relation(id: Long, tags: Map<String, String>, members: List<RelationMember>) {
            if (guidance != null) when (tags["type"]) {
                "enforcement" -> {
                    val t = when (tags["enforcement"]) {
                        "maxspeed" -> PointStore.SPEED_CAMERA; "average_speed" -> PointStore.AVERAGE_SPEED_START
                        "traffic_signals" -> PointStore.TRAFFIC_SIGNALS_CAMERA; else -> 0
                    }
                    if (t == 0) return
                    fun node(role: String) = members.firstOrNull { it.type == MemberType.NODE && it.role == role }?.ref ?: -1L
                    val e = Enforcement(t, GuidanceParser.speed(tags["maxspeed"]), node("device"), node("from"), node("to"))
                    enforcements += e
                    for (n in longArrayOf(e.device, e.from, e.to)) if (n >= 0) neededNodes.put(n, DoubleArray(0))
                    return
                }
                "destination_sign" -> {
                    val sign = GuidanceParser.relationSign(tags) ?: return
                    val to = members.firstOrNull { it.type == MemberType.WAY && it.role == "to" }?.ref ?: return
                    val inter = members.firstOrNull { it.type == MemberType.NODE && it.role == "intersection" }?.ref ?: -1L
                    relationSigns.getOrPut(to) { ArrayList(2) } += inter to sign
                    return
                }
            }
            if (tags["type"] != "restriction") return
            var restriction: String? = null; var vehicle: String? = null
            for ((k, v) in tags) {
                if (k == "restriction") { restriction = v; vehicle = null; break }
                if (k.startsWith("restriction:")) { restriction = v; vehicle = k.substring(12) }
            }
            val r = restriction ?: return
            val from = members.firstOrNull { it.type == MemberType.WAY && it.role == "from" } ?: return
            val to = members.firstOrNull { it.type == MemberType.WAY && it.role == "to" } ?: return
            val via = members.firstOrNull { it.type == MemberType.NODE && it.role == "via" } ?: return // via-way non supportata
            val except = tags["except"]?.split(';')?.map { it.trim() }?.toSet() ?: emptySet()
            turnRestrictions += TurnRestrictionRelation(id, from.ref, via.ref, to.ref, r, vehicle, except)
        }
    }

    private inner class Pass2Nodes : OsmSink {
        override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) {
            if (points != null) {
                if (tags != null && tags["highway"] == "speed_camera") {
                    points.add(lat, lon, PointStore.SPEED_CAMERA, GuidanceParser.speed(tags["maxspeed"]), bearingOf(tags["direction"]))
                    cameraNodes.add(id)
                }
                if (!neededNodes.isEmpty && neededNodes.containsKey(id)) neededNodes.put(id, doubleArrayOf(lat, lon))
            }
            val use = nodeUse.getOrDefault(id, 0)
            if (use == 0) return
            var isBarrier = false
            if (tags != null && guidance != null) {
                GuidanceParser.junction(tags)?.let { junctions.put(id, it) }
                GuidanceParser.specialNode(tags)?.let { specialNodes.put(id, it) }
            }
            if (tags != null && (tags.containsKey("barrier") || tags["ford"] == "yes" || tags["highway"] == "ford")) {
                isBarrier = profiles.any { it.profile.isBarrier(tags) }
                if (isBarrier) barrierTags.put(id, tags)
            }
            if (use and 2 != 0 || use and 4 != 0 || isBarrier) {
                towers.put(id, graph.addNode(lat, lon))
            } else {
                if (pillarCount == pillarLat.size) {
                    pillarLat = pillarLat.copyOf(pillarCount * 2); pillarLon = pillarLon.copyOf(pillarCount * 2)
                }
                pillarLat[pillarCount] = lat; pillarLon[pillarCount] = lon
                pillars.put(id, pillarCount++)
            }
        }
    }

    private inner class Pass2Ways : OsmSink {
        private val lats = DoubleArrayList(); private val lons = DoubleArrayList()
        private val segIds = LongArrayList() // id OSM dei punti del segmento corrente, in parallelo a lats/lons
        private var segmentStart = -1 // nodo del grafo in cui inizia il segmento corrente

        override fun way(way: OsmWay) {
            if (!acceptWay(way)) return
            val ids = way.nodeIds
            segmentStart = -1
            lats.clear(); lons.clear(); segIds.clear()
            for (i in ids.indices) {
                val id = ids[i]
                var tower = towers.getOrDefault(id, -1)
                if (tower < 0) {
                    val p = pillars.getOrDefault(id, -1)
                    if (p < 0) { // nodo assente dall'estratto: taglia il segmento
                        closeSegment(way, promoteLast = true)
                        continue
                    }
                    if (segmentStart < 0) { // il segmento deve iniziare da una torre: promuovi il pillar
                        tower = promote(id, p)
                    } else {
                        lats.add(pillarLat[p]); lons.add(pillarLon[p]); segIds.add(id); continue
                    }
                }
                if (segmentStart >= 0) {
                    lats.add(graph.lat(tower)); lons.add(graph.lon(tower)); segIds.add(id)
                    addEdge(way, segmentStart, tower)
                }
                // nodo barriera interno: sdoppia e collega con un arco di lunghezza zero
                val bt = if (i in 1 until ids.size - 1) barrierTags.get(id) else null
                val next = if (bt != null) splitBarrier(way, tower, bt) else tower
                segmentStart = next
                lats.clear(); lons.clear(); segIds.clear()
                lats.add(graph.lat(next)); lons.add(graph.lon(next)); segIds.add(id)
            }
        }

        /** Chiude il segmento aperto su un nodo mancante: l'ultimo pillar diventa torre. */
        private fun closeSegment(way: OsmWay, promoteLast: Boolean) {
            if (segmentStart >= 0 && lats.size > 1 && promoteLast) {
                val end = graph.addNode(lats[lats.size - 1], lons[lons.size - 1])
                addEdge(way, segmentStart, end)
            }
            segmentStart = -1
            lats.clear(); lons.clear(); segIds.clear()
        }

        private fun promote(id: Long, pillarIndex: Int): Int {
            val t = graph.addNode(pillarLat[pillarIndex], pillarLon[pillarIndex])
            towers.put(id, t)
            pillars.remove(id)
            return t
        }

        private fun splitBarrier(way: OsmWay, tower: Int, tags: Map<String, String>): Int {
            val copy = graph.addNode(graph.lat(tower), graph.lon(tower))
            val e = graph.addEdge(tower, copy, 0.0)
            val flags = graph.encodedValues.newFlags()
            val attrs = WayAttributeParser.parse(way, isFerry = false, widthFromCarriageway = options.widthFromCarriageway)
            encoding.shared.write(flags, attrs)
            for (i in profiles.indices) {
                val pe = profiles[i]
                val r = pe.profile.analyse(way, wayAccess[i], 0.0)
                val blocked = pe.profile.isBarrier(tags)
                pe.write(flags, if (r == null || blocked) null else r)
            }
            graph.setFlags(e, flags)
            barrierEdges++
            return copy
        }

        private fun addEdge(way: OsmWay, from: Int, to: Int) {
            val n = lats.size
            if (from == to) {
                if (n <= 2) return // anello di due nodi: scartato
                // anello chiuso: l'ultimo pillar diventa torre e si creano due archi
                val mid = graph.addNode(lats[n - 2], lons[n - 2])
                val savedLat = lats[n - 1]; val savedLon = lons[n - 1]
                val midId = segIds.get(n - 2); val savedId = segIds.get(n - 1)
                lats.size = n - 1; lons.size = n - 1; segIds.elementsCount = n - 1
                addEdge(way, from, mid)
                lats.clear(); lons.clear(); segIds.clear()
                lats.add(graph.lat(mid)); lons.add(graph.lon(mid)); lats.add(savedLat); lons.add(savedLon)
                segIds.add(midId); segIds.add(savedId)
                addEdge(way, mid, to)
                return
            }
            var dist = DistanceCalc.length(lats.array, lons.array, n)
            if (dist < 0.001) dist = 0.001
            val edge = graph.addEdge(from, to, dist)
            if (n > 2) {
                val pl = PointList(n - 2)
                for (i in 1 until n - 1) pl.add(lats[i], lons[i])
                graph.setWayGeometry(edge, pl)
            }
            val isFerry = way.tag("highway") == null && way.hasTag("route", "ferry", "shuttle_train")
            val flags = graph.encodedValues.newFlags()
            encoding.shared.write(flags, WayAttributeParser.parse(way, isFerry, widthFromCarriageway = options.widthFromCarriageway))
            val ferryDuration = if (isFerry) OsmValues.parseDurationSeconds(way.tag("duration")) else -1L
            for (i in profiles.indices) {
                profiles[i].write(flags, profiles[i].profile.analyse(way, wayAccess[i], dist, ferryDuration))
            }
            graph.setFlags(edge, flags)
            val name = wayName(way)
            if (name.isNotEmpty()) graph.setName(edge, name)
            guidance?.let { g ->
                var special: StringBuilder? = null
                if (!specialNodes.isEmpty && segIds.size() == n) {
                    for (k in 0 until n) {
                        val t = specialNodes.get(segIds.get(k)) ?: continue
                        if (special == null) special = StringBuilder() else special.append(';')
                        special.append(t).append('@').append(k)
                    }
                }
                val jA = if (segIds.size() == n) junctions.get(segIds.get(0)) else null
                val jB = if (segIds.size() == n) junctions.get(segIds.get(n - 1)) else null
                // cartelli delle relazioni destination_sign verso questa way: all'imbocco dal nodo dell'incrocio
                val rs = relationSigns[way.id]
                val sA = if (rs != null && segIds.size() == n) rs.firstOrNull { it.first == segIds.get(0) }?.second else null
                val sB = if (rs != null && segIds.size() == n) rs.firstOrNull { it.first == segIds.get(n - 1) }?.second else null
                g.set(edge, GuidanceParser.parse(way), jA, jB, special?.toString(), sA, sB)
            }
        }
    }

    /** Il tag `direction` di un autovelox in gradi (numero o punto cardinale), -1 se assente. */
    private fun bearingOf(raw: String?): Int {
        val v = raw?.trim()?.substringBefore(';')?.lowercase() ?: return -1
        v.toDoubleOrNull()?.let { return ((it.toInt() % 360) + 360) % 360 }
        return when (v) {
            "n", "north" -> 0; "ne" -> 45; "e", "east" -> 90; "se" -> 135; "s", "south" -> 180; "sw" -> 225; "w", "west" -> 270; "nw" -> 315
            else -> -1
        }
    }

    /** Nome come GraphHopper: `name` (o `name:<lingua>`), più `, ref`; `;` diventa `, `. */
    private fun wayName(way: OsmWay): String {
        val name = way.tag("name") ?: ""
        val ref = way.tag("ref") ?: ""
        val s = when {
            name.isEmpty() -> ref
            ref.isEmpty() -> name
            else -> "$name, $ref"
        }
        return s.replace(";", ", ")
    }

    /** Lista di double senza boxing, con accesso diretto all'array per i calcoli di distanza. */
    private class DoubleArrayList {
        var array = DoubleArray(64); private set
        var size = 0
        fun add(v: Double) { if (size == array.size) array = array.copyOf(size * 2); array[size++] = v }
        fun clear() { size = 0 }
        operator fun get(i: Int) = array[i]
    }
}
