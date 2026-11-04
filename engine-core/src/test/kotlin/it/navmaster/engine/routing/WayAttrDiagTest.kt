package it.navmaster.engine.routing

import it.navmaster.engine.RoutingGraph
import java.io.File
import kotlin.test.Test

/** Diagnostica (`NM_GRAPH`, `NM_NAMES=a;b`): gli attributi per il camion degli archi con quei nomi. */
class WayAttrDiagTest {
    @Test
    fun show() {
        val path = System.getenv("NM_GRAPH") ?: return
        val names = (System.getenv("NM_NAMES") ?: return).split(';')
        RoutingGraph.load(File(path), mmap = false).use { g ->
            val bg = g.graph; val sh = g.encoding.shared; val hgv = g.encoding.profile("heavyvehicle"); val car = g.encoding.profile("car-ors")
            val ei = bg.createEdgeIterator()
            val seen = HashSet<Any>()
            for (e in 0 until bg.edgeCount) {
                ei.setEdge(e, bg.nodeA(e))
                if (names.none { ei.name.startsWith(it) }) continue
                if (!seen.add(ei.get(sh.osmWayId))) continue
                println("way ${ei.get(sh.osmWayId)} '${ei.name}' ${ei.get(sh.roadClass)} auto accesso ${ei.get(car.access)}/${ei.getReverse(car.access)} v ${ei.get(car.speed)}; camion accesso ${ei.get(hgv.access)}/${ei.getReverse(hgv.access)} v ${ei.get(hgv.speed)} hgvBlocked ${ei.get(sh.hgvBlocked)} h ${ei.get(sh.maxHeight)} w ${ei.get(sh.maxWidth)} l ${ei.get(sh.maxLength)} t ${ei.get(sh.maxWeight)} asse ${ei.get(sh.maxAxleLoad)} hazmat ${ei.get(sh.hazmatForbidden)}")
            }
        }
    }
}
