package it.navmaster.engine.osm

import java.io.File
import kotlin.test.Test

/** Diagnostica (`NM_PBF`, `NM_WAY_NAMES=a;b` o `NM_WAY_IDS=1;2`): tag delle way, dei loro nodi con tag e delle relazioni che le toccano. */
class PbfDiagTest {
    @Test
    fun show() {
        val pbf = System.getenv("NM_PBF") ?: return
        val names = (System.getenv("NM_WAY_NAMES") ?: "").split(';').filter { it.isNotBlank() }
        val ids = (System.getenv("NM_WAY_IDS") ?: "").split(';').filter { it.isNotBlank() }.map { it.toLong() }.toSet()
        val ways = ArrayList<OsmWay>(); val wanted = HashSet<Long>()
        OsmPbfReader(File(pbf)).read(object : OsmSink {
            override fun way(way: OsmWay) {
                if (way.id in ids || names.any { way.tag("name") == it }) { ways += way; wanted += way.nodeIds.toList() }
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
            println("way ${w.id} ${w.tags} nodi ${w.nodeIds.size}: primo ${w.nodeIds.first()} ultimo ${w.nodeIds.last()}")
            for ((i, n) in w.nodeIds.withIndex()) nodeTags[n]?.let { println("   nodo[$i] $n $it") }
        }
    }
}
