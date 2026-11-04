package it.navmaster.engine.routing

import it.navmaster.engine.RoutingGraph
import java.io.File
import kotlin.test.Test

/** Diagnostica (`NM_GRAPH`, `NM_NAMES=a;b`): gli attributi per il camion degli archi con quei nomi. */
class WayAttrDiagTest {
    @Test
    fun show() {
        val path = System.getenv("NM_GRAPH") ?: return
        val names = (System.getenv("NM_NAMES") ?: "").split(';').filter { it.isNotBlank() }
        RoutingGraph.load(File(path), mmap = false).use { g ->
            val bg = g.graph; val sh = g.encoding.shared; val hgv = g.encoding.profile("heavyvehicle"); val car = g.encoding.profile("car-ors")
            val ei = bg.createEdgeIterator()
            // NM_ROUTE=lat,lon;lat,lon: il percorso del camion (evita pedaggi) arco per arco
            System.getenv("NM_ROUTE")?.let { spec ->
                val pts = spec.split(';').map { it.split(',').let { p -> p[0].toDouble() to p[1].toDouble() } }
                val prof = System.getenv("NM_PROFILE") ?: "driving-hgv"
                val r = g.router(it.navmaster.engine.RouteOptions(profile = prof, avoidFeatures = if (System.getenv("NM_AVOID_TOLLS") != null) it.navmaster.engine.profile.AvoidFeatureFlags.TOLLWAYS else 0))
                val path = r.route(r.snap(pts[0].first, pts[0].second, 400.0, 0), r.snap(pts[1].first, pts[1].second, 400.0, 1))
                println("percorso ${r.algorithm}: %.0f m, peso %.0f".format(path.distance, path.weight))
                for (e in path.edges) { ei.setEdge(e.edge, e.baseNode); println("  way ${ei.get(sh.osmWayId)} '${ei.name}' ${ei.get(sh.roadClass)} accesso-strada ${ei.get(sh.roadAccess)} v ${if (prof == "driving-car") ei.get(car.speed) else ei.get(hgv.speed)} dist %.0f peso %.1f".format(e.distance, e.weight)) }
            }
            val seen = HashSet<Any>()
            for (e in 0 until bg.edgeCount) {
                ei.setEdge(e, bg.nodeA(e))
                if (names.none { ei.name.startsWith(it) }) continue
                if (!seen.add(ei.get(sh.osmWayId))) continue
                println("way ${ei.get(sh.osmWayId)} '${ei.name}' ${ei.get(sh.roadClass)} auto accesso ${ei.get(car.access)}/${ei.getReverse(car.access)} v ${ei.get(car.speed)}; camion accesso ${ei.get(hgv.access)}/${ei.getReverse(hgv.access)} v ${ei.get(hgv.speed)} hgvBlocked ${ei.get(sh.hgvBlocked)} h ${ei.get(sh.maxHeight)} w ${ei.get(sh.maxWidth)} l ${ei.get(sh.maxLength)} t ${ei.get(sh.maxWeight)} asse ${ei.get(sh.maxAxleLoad)} hazmat ${ei.get(sh.hazmatForbidden)} accesso-strada ${ei.get(sh.roadAccess)}")
            }
        }
    }
}
