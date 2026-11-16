package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * "Evita" a penalità tra Paesi su grafi veri (solo in locale): `NM_SI="si=/…/slovenia,at=/…/austria,sk=/…/slovakia,hu=/…/hungary"`.
 * Un camion senza pedaggi per Bratislava (dall'Austria l'ultimo tratto in città è a pedaggio): la
 * penalità solo nel Paese dove il divieto ostacola un punto contro il divieto ovunque (il giro per l'Ungheria).
 */
class AvoidShutInRealTest {
    @Test
    @Suppress("UNCHECKED_CAST")
    fun lorryWithoutTolls() {
        val spec = System.getenv("NM_SI") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            CountryGraph(id, DirectionsService(RoutingGraph.load(File(path), true), DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0, 60_000.0)))
        }
        val cache = HashMap<String, List<Transfer>>()
        val trips = listOf(
            "Lubiana → Bratislava" to listOf(listOf(14.5058, 46.0569), listOf(17.1077, 48.1486)),
            "Villach → Bratislava" to listOf(listOf(13.8500, 46.6167), listOf(17.1077, 48.1486)),
            "Graz → Bratislava" to listOf(listOf(15.4200, 47.0400), listOf(17.1077, 48.1486)),
            "Vienna → Bratislava" to listOf(listOf(16.3738, 48.2082), listOf(17.1077, 48.1486)),
            "Bratislava → Villach" to listOf(listOf(17.1077, 48.1486), listOf(13.8500, 46.6167)),
        )
        for ((name, pts) in trips) {
            val res = HashMap<Boolean, Pair<Double, Double>>()
            for (check in listOf(true, false)) {
                val cb = CrossBorderDirections(graphs, { a, b -> cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) { Transfers.find(a.graph, b.graph) } },
                    { if (it.trim().startsWith("evita") || it.startsWith("viaggio")) println("    $it") })
                cb.shutInCheck = check
                val body = linkedMapOf<String, Any?>("coordinates" to pts,
                    "options" to mapOf("avoid_features" to listOf("tollways"), "profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 40.0))),
                    "navmaster" to mapOf("avoid_penalty" to true))
                println("== $name, " + if (check) "penalità dove il divieto ostacola" else "divieto ovunque")
                val t0 = System.currentTimeMillis()
                val r = cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf(null, null))
                val ms = System.currentTimeMillis() - t0
                val sum = ((r["features"] as List<Map<String, Any?>>)[0]["properties"] as Map<String, Any?>)["summary"] as Map<String, Any?>
                val d = (sum["distance"] as Number).toDouble(); val t = (sum["duration"] as Number).toDouble()
                println("   %.1f km, %.0f min in %d ms".format(d / 1000, t / 60, ms))
                res[check] = d to t
            }
            // the ban everywhere is a valid trip for the penalty where the ban hampers: never slower
            assertTrue(res[true]!!.second <= res[false]!!.second + 60, "$name: ${res[true]} contro ${res[false]}")
        }
    }
}
