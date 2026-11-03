package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/** maximum_speed on long lorry trips (only locally, NM_IT = the folder of the Italian graph). */
class MaxSpeedRealTest {
    @Test
    fun longLorryTrips() {
        val dir = System.getenv("NM_IT") ?: return
        val g = RoutingGraph.load(File(dir), true)
        val service = DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0))
        fun run(name: String, from: Pair<Double, Double>, to: Pair<Double, Double>, fast: Boolean?): Pair<Double, Double> {
            val body = linkedMapOf<String, Any?>("coordinates" to listOf(listOf(from.second, from.first), listOf(to.second, to.first)), "instructions" to false,
                "maximum_speed" to 80.0, "options" to mapOf("profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 40.0))))
            val req = DirectionsRequest.fromJson("driving-hgv", body)
            val t0 = System.currentTimeMillis()
            val rt = Runtime.getRuntime(); System.gc(); val before = rt.totalMemory() - rt.freeMemory()
            val res = if (fast == false) {
                // the exact search (as ORS) forced, for the comparison
                val o = service.routeOptions(req)
                val router = g.router(o)
                val a = router.snap(from.first, from.second, 400.0, 0); val b = router.snap(to.first, to.second, 400.0, 1)
                val p = router.route(a, b)
                p.distance to p.millis / 1000.0
            } else {
                @Suppress("UNCHECKED_CAST")
                val f = (service.handle(req, ResponseFormat.GEOJSON)["features"] as List<Map<String, Any?>>)[0]
                @Suppress("UNCHECKED_CAST")
                val s = (f["properties"] as Map<String, Any?>)["summary"] as Map<String, Any?>
                (s["distance"] as Double) to (s["duration"] as Double)
            }
            val used = (rt.totalMemory() - rt.freeMemory() - before) / 1048576
            println("%s%s: %.1f km, %.0f min, %d ms, memoria ~%d MB".format(name, if (fast == false) " (esatto)" else "", res.first / 1000, res.second / 60, System.currentTimeMillis() - t0, used))
            return res
        }
        val milano = 45.4642 to 9.1900; val bari = 41.1171 to 16.8719
        val brescia = 45.5416 to 10.2118; val verona = 45.4384 to 10.9916
        run("Milano → Bari camion 80", milano, bari, null)
        val fast = run("Brescia → Verona camion 80", brescia, verona, null)
        val exact = run("Brescia → Verona camion 80", brescia, verona, false)
        println("differenza: %.2f km, %.1f min".format((fast.first - exact.first) / 1000, (fast.second - exact.second) / 60))
        assertTrue(kotlin.math.abs(fast.second - exact.second) / exact.second < 0.03)
    }
}
