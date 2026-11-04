package it.navmaster.engine.ch

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import kotlin.test.Test

/**
 * Diagnostica su un grafo vero (`NM_LM=/…/italy`): la copertura dei landmark del core del camion nel
 * file e quella dei landmark ricalcolati con la scelta attuale; una ricerca lunga con gli uni e
 * con gli altri (nodi visitati, tempo). `NM_LM_TRIP=lat,lon;lat,lon`.
 */
class LandmarkRealDiagTest {
    @Test
    fun compare() {
        val path = System.getenv("NM_LM") ?: return
        val g = RoutingGraph.load(File(path), true)
        val key = "heavyvehicle_recommended_core"
        val storage = g.chStorages.getValue(key)
        val n = g.graph.nodeCount
        val core = (0 until n).filter { storage.level(it) >= n }
        fun coverage(lm: CoreLandmarks): String = (0 until lm.landmarkCount).joinToString { l ->
            val sample = core.indices.step(maxOf(1, core.size / 5000)).map { core[it] }
            "${100 * sample.count { lm.fromLandmark(l, it).isFinite() } / sample.size}/${100 * sample.count { lm.toLandmark(l, it).isFinite() } / sample.size}"
        }
        val old = g.landmarks.getValue(key)
        println("landmark nel file (raggiunge/raggiunto da, %): ${coverage(old)}")
        val w = WeightingFactory.create(g.encoding, g.encoding.profile("heavyvehicle"), WeightingMethod.RECOMMENDED)
        val t0 = System.currentTimeMillis()
        val fresh = CoreLandmarks.prepare(CHGraph(g.graph, storage, w), w, 16, log = { if (it.startsWith("landmark:")) println(it) })
        println("ricalcolati in ${System.currentTimeMillis() - t0} ms: ${coverage(fresh)}")
        for (l in 0 until fresh.landmarkCount) { val v = fresh.landmarkNodes[l]; print(" %.3f,%.3f".format(g.graph.lat(v), g.graph.lon(v))) }
        println()
        val trip = (System.getenv("NM_LM_TRIP") ?: "45.4103,9.9428;46.9971,11.5029").split(';').map { p -> p.split(',').let { it[0].toDouble() to it[1].toDouble() } }
        val params = VehicleParameters(height = 4.0, width = 2.55, length = 16.5, weight = 27.0, axleLoad = 11.5)
        for ((name, lm) in listOf("file" to old, "ricalcolati" to fresh, "file" to old, "ricalcolati" to fresh)) {
            g.landmarks[key] = lm
            val r = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            val a = r.snap(trip[0].first, trip[0].second, 400.0, 0); val b = r.snap(trip[1].first, trip[1].second, 400.0, 1)
            val ts = System.currentTimeMillis()
            val p = r.route(a, b)
            println("landmark $name: peso %.1f, %.1f km, ${System.currentTimeMillis() - ts} ms, ${r.lastStats}".format(p.weight, p.distance / 1000))
        }
        g.landmarks[key] = old
        g.close()
    }
}
