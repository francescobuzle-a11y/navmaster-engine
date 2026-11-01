package it.navmaster.engine.osm

import com.carrotsearch.hppc.LongIntHashMap
import com.carrotsearch.hppc.LongObjectHashMap
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
)

class OsmGraphImporter(private val graph: BaseGraph, private val encoding: GraphEncoding, private val turnCosts: TurnCostStorage? = null, private val options: ImportOptions = ImportOptions()) {
    private val profiles = encoding.profiles
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
            val use = nodeUse.getOrDefault(id, 0)
            if (use == 0) return
            var isBarrier = false
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
        private var segmentStart = -1 // nodo del grafo in cui inizia il segmento corrente

        override fun way(way: OsmWay) {
            if (!acceptWay(way)) return
            val ids = way.nodeIds
            segmentStart = -1
            lats.clear(); lons.clear()
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
                        lats.add(pillarLat[p]); lons.add(pillarLon[p]); continue
                    }
                }
                if (segmentStart >= 0) {
                    lats.add(graph.lat(tower)); lons.add(graph.lon(tower))
                    addEdge(way, segmentStart, tower)
                }
                // nodo barriera interno: sdoppia e collega con un arco di lunghezza zero
                val bt = if (i in 1 until ids.size - 1) barrierTags.get(id) else null
                val next = if (bt != null) splitBarrier(way, tower, bt) else tower
                segmentStart = next
                lats.clear(); lons.clear()
                lats.add(graph.lat(next)); lons.add(graph.lon(next))
            }
        }

        /** Chiude il segmento aperto su un nodo mancante: l'ultimo pillar diventa torre. */
        private fun closeSegment(way: OsmWay, promoteLast: Boolean) {
            if (segmentStart >= 0 && lats.size > 1 && promoteLast) {
                val end = graph.addNode(lats[lats.size - 1], lons[lons.size - 1])
                addEdge(way, segmentStart, end)
            }
            segmentStart = -1
            lats.clear(); lons.clear()
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
                lats.size = n - 1; lons.size = n - 1
                addEdge(way, from, mid)
                lats.clear(); lons.clear()
                lats.add(graph.lat(mid)); lons.add(graph.lon(mid)); lats.add(savedLat); lons.add(savedLon)
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
