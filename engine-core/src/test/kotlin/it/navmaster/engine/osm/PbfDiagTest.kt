package it.navmaster.engine.osm

import java.io.File
import kotlin.test.Test

/** Diagnostica (`NM_PBF`, `NM_WAY_NAMES=a;b` o `NM_WAY_IDS=1;2`): tag delle way, dei loro nodi con tag e delle relazioni che le toccano. */
class PbfDiagTest {
    @Test
    fun show() {
        val pbf = System.getenv("NM_PBF") ?: return
        // NM_BARRIER_SURVEY: i nodi barrier=gate|lift_gate senza altri tag, con quante way li usano e in che posizione
        if (System.getenv("NM_BARRIER_SURVEY") != null) {
            val gates = HashMap<Long, DoubleArray>()
            OsmPbfReader(File(pbf)).read(object : OsmSink {
                override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) {
                    if (tags != null && tags.size == 1 && (tags["barrier"] == "lift_gate" || tags["barrier"] == "gate")) gates[id] = doubleArrayOf(lat, lon)
                }
            }, ways = false, relations = false)
            val uses = HashMap<Long, ArrayList<String>>(); val neigh = HashMap<Long, ArrayList<Long>>()
            OsmPbfReader(File(pbf)).read(object : OsmSink {
                override fun way(way: OsmWay) {
                    val hw = way.tag("highway") ?: return
                    for ((i, n) in way.nodeIds.withIndex()) if (n in gates) {
                        uses.getOrPut(n) { ArrayList() }.add("${way.id}:$hw:${if (i == 0) "primo" else if (i == way.nodeIds.size - 1) "ultimo" else "interno"}")
                        if (i > 0) neigh.getOrPut(n) { ArrayList() }.add(way.nodeIds[i - 1]); if (i < way.nodeIds.size - 1) neigh.getOrPut(n) { ArrayList() }.add(way.nodeIds[i + 1])
                    }
                }
            }, nodes = false, relations = false)
            val wantedN = neigh.values.flatten().toHashSet(); val coords = HashMap<Long, DoubleArray>()
            OsmPbfReader(File(pbf)).read(object : OsmSink {
                override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) { if (id in wantedN) coords[id] = doubleArrayOf(lat, lon) }
            }, ways = false, relations = false)
            for ((n, u) in uses.entries.sortedBy { it.value.size }) if (u.all { it.contains(":track:") || it.contains(":service:") || it.contains(":unclassified:") || it.contains(":residential:") })
                println("sbarra $n %.6f,%.6f way ${u.size}: ${u.joinToString(" ")} vicini ${neigh[n]!!.mapNotNull { coords[it] }.joinToString(" ") { "%.6f,%.6f".format(it[0], it[1]) }}".format(gates[n]!![0], gates[n]!![1]))
            return
        }
        val names = (System.getenv("NM_WAY_NAMES") ?: "").split(';').filter { it.isNotBlank() }
        val ids = (System.getenv("NM_WAY_IDS") ?: "").split(';').filter { it.isNotBlank() }.map { it.toLong() }.toSet()
        // NM_NODE_IDS: anche le way che contengono quei nodi (con la posizione del nodo)
        val nodeIds = (System.getenv("NM_NODE_IDS") ?: "").split(';').filter { it.isNotBlank() }.map { it.toLong() }.toSet()
        val ways = ArrayList<OsmWay>(); val wanted = HashSet<Long>()
        OsmPbfReader(File(pbf)).read(object : OsmSink {
            override fun way(way: OsmWay) {
                val hit = way.nodeIds.indexOfFirst { it in nodeIds }
                if (way.id in ids || names.any { way.tag("name") == it } || hit >= 0) {
                    if (hit >= 0) println("way ${way.id} contiene il nodo ${way.nodeIds[hit]} in posizione $hit di ${way.nodeIds.size}")
                    ways += way; wanted += way.nodeIds.toList()
                }
            }
        })
        val nodeTags = HashMap<Long, Map<String, String>>()
        OsmPbfReader(File(pbf)).read(object : OsmSink {
            override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) { if (id in wanted && !tags.isNullOrEmpty()) nodeTags[id] = tags }
            override fun relation(id: Long, tags: Map<String, String>, members: List<RelationMember>) {
                if (members.any { (it.type == MemberType.WAY && ways.any { w -> w.id == it.ref }) }) println("relazione $id $tags membri ${members.map { "${it.role}:${it.type}:${it.ref}" }}")
            }
        })
        for (w in ways) {
            println("way ${w.id} ${w.tags} nodi ${w.nodeIds.size}: ${if (w.nodeIds.size <= 30) w.nodeIds.joinToString(",") else "primo ${w.nodeIds.first()} ultimo ${w.nodeIds.last()}"}")
            for ((i, n) in w.nodeIds.withIndex()) nodeTags[n]?.let { println("   nodo[$i] $n $it") }
        }
    }
}
