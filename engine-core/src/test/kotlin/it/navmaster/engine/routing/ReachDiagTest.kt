package it.navmaster.engine.routing

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import kotlin.test.Test

/** Diagnostica (`NM_GRAPH`, `NM_PTS=lat,lon;lat,lon`): perché B non si raggiunge da A per l'auto: l'arco agganciato e le way che chiudono la zona di B. */
class ReachDiagTest {
    @Test
    fun why() {
        val path = System.getenv("NM_GRAPH") ?: return
        val pts = (System.getenv("NM_PTS") ?: return).split(';').map { it.split(',').let { p -> p[0].toDouble() to p[1].toDouble() } }
        RoutingGraph.load(File(path), mmap = false).use { g ->
            val bg = g.graph; val pe = g.encoding.profile("car-ors"); val sh = g.encoding.shared
            val r = g.router(RouteOptions(profile = "driving-car", turnCosts = false, allowCH = false))
            val ei = bg.createEdgeIterator()
            fun describe(e: Int, base: Int): String { ei.setEdge(e, base); return "arco $e way ${ei.get(sh.osmWayId)} '${ei.name}' classe ${ei.get(sh.roadClass)} accesso ${ei.get(pe.access)} velocità ${ei.get(pe.speed)}/${ei.getReverse(pe.speed)} dist %.0f".format(ei.distance) }
            val snaps = pts.map { (la, lo) -> r.snap(la, lo, 400.0, 0) }
            for ((i, s) in snaps.withIndex()) println("punto $i: ${describe(s.edge, bg.nodeA(s.edge))} posizione ${s.position} a %.0f m".format(s.distance))
            // all'indietro da B: i nodi da cui si arriva a B; le way sulla frontiera (arrivano al nodo ma non si percorrono verso B)
            val w = WeightingFactory.create(g.encoding, pe, WeightingMethod.FASTEST)
            val b = snaps.last()
            val seen = HashSet<Int>(); val queue = ArrayDeque<Int>()
            for (n in listOf(bg.nodeA(b.edge), bg.nodeB(b.edge))) { seen += n; queue += n }
            val frontier = LinkedHashMap<Any, String>()
            while (queue.isNotEmpty() && seen.size < 200_000) {
                val n = queue.removeFirst()
                ei.setBaseNode(n)
                while (ei.next()) {
                    // verso n = contro l'iteratore
                    if (w.edgeWeight(ei, true).isFinite()) { if (seen.add(ei.adjNode)) queue += ei.adjNode }
                    else frontier.putIfAbsent(ei.get(sh.osmWayId), "way ${ei.get(sh.osmWayId)} '${ei.name}' classe ${ei.get(sh.roadClass)} accesso ${ei.get(pe.access)} velocità ${ei.get(pe.speed)}/${ei.getReverse(pe.speed)} a nodo $n (%.5f,%.5f)".format(bg.lat(n), bg.lon(n)))
                }
            }
            println("nodi che raggiungono B: ${seen.size} su ${bg.nodeCount}")
            // in avanti da A: i nodi raggiunti e, se pochi, la frontiera
            val a = snaps.first()
            val fwd = HashSet<Int>(); val q2 = ArrayDeque<Int>()
            for (n in listOf(bg.nodeA(a.edge), bg.nodeB(a.edge))) { fwd += n; q2 += n }
            val fFront = LinkedHashMap<Any, String>()
            while (q2.isNotEmpty() && fwd.size < 200_000) {
                val n = q2.removeFirst()
                ei.setBaseNode(n)
                while (ei.next()) {
                    if (w.edgeWeight(ei, false).isFinite()) { if (fwd.add(ei.adjNode)) q2 += ei.adjNode }
                    else fFront.putIfAbsent(ei.get(sh.osmWayId), "way ${ei.get(sh.osmWayId)} '${ei.name}' classe ${ei.get(sh.roadClass)} accesso ${ei.get(pe.access)} velocità ${ei.get(pe.speed)}/${ei.getReverse(pe.speed)} da nodo $n (%.5f,%.5f)".format(bg.lat(n), bg.lon(n)))
                }
            }
            println("nodi raggiunti da A: ${fwd.size}; capi dell'arco di B raggiunti: A ${bg.nodeA(b.edge) in fwd}, B ${bg.nodeB(b.edge) in fwd}")
            // gli archi vicini a B (entro 40 m dal punto chiesto), col verso percorribile e i capi raggiunti da A
            val (bla, blo) = pts.last()
            val near = ArrayList<String>()
            for (e in 0 until bg.edgeCount) {
                val pl = bg.fetchWayGeometry(e, false, it.navmaster.engine.storage.FetchMode.ALL)
                var dmin = Double.MAX_VALUE
                for (i in 0 until pl.size) dmin = minOf(dmin, it.navmaster.engine.util.DistanceCalc.haversine(bla, blo, pl.lat(i), pl.lon(i)))
                if (dmin < 40) { ei.setEdge(e, bg.nodeA(e)); near += "arco $e way ${ei.get(sh.osmWayId)} '${ei.name}' ${ei.get(sh.roadClass)} a %.0f m, peso A→B %s B→A %s, capo A raggiunto ${bg.nodeA(e) in fwd}, capo B ${bg.nodeB(e) in fwd}".format(dmin, w.edgeWeight(ei, false).let { if (it.isFinite()) "%.0f".format(it) else "∞" }, w.edgeWeight(ei, true).let { if (it.isFinite()) "%.0f".format(it) else "∞" }) }
            }
            near.forEach { println("  vicino: $it") }
            println("frontiera in avanti (${fFront.size} way):"); fFront.values.take(12).forEach { println("  $it") }
            println("frontiera (${frontier.size} way):"); frontier.values.take(25).forEach { println("  $it") }
        }
    }
}
