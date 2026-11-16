package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Alternative dei viaggi lunghi su grafi veri (solo in locale):
 * `NM_LA="si=/…/slovenia,hr=/…/croatia,hu=/…/hungary,sk=/…/slovakia"`. Per ogni viaggio stampa i
 * percorsi trovati (tempo, km, parte in comune col primo) e il tempo di calcolo.
 */
class LongAlternativesRealTest {
    @Test
    @Suppress("UNCHECKED_CAST")
    fun realTrips() {
        val spec = System.getenv("NM_LA") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            CountryGraph(id, DirectionsService(RoutingGraph.load(File(path), true), DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0, 60_000.0)).also { s -> s.alternativesLog = { println("  $it") } })
        }
        val cache = HashMap<String, List<Transfer>>()
        val cb = CrossBorderDirections(graphs, { a, b -> cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) { Transfers.find(a.graph, b.graph) } }, { if (it.startsWith("altern")) println("  $it") })
        val trips = if (System.getenv("NM_LA_PENALTY") != null) emptyList() else listOf(
            Triple("Capodistria → Maribor (camion, un Paese)", "driving-hgv", listOf(45.5469 to 13.7294, 46.5547 to 15.6459)),
            Triple("Lubiana → Bratislava (camion, tra Paesi)", "driving-hgv", listOf(46.0569 to 14.5058, 48.1486 to 17.1077)),
            Triple("Capodistria → Košice (camion, tra Paesi)", "driving-hgv", listOf(45.5469 to 13.7294, 48.7164 to 21.2611)),
            Triple("Zagabria → Budapest (auto, senza autostrade e pedaggi)", "driving-car", listOf(45.8150 to 15.9819, 47.4979 to 19.0402)),
            Triple("Zagabria → Budapest (auto, senza autostrade e pedaggi, penalità)", "driving-car", listOf(45.8150 to 15.9819, 47.4979 to 19.0402)),
        )
        // the cost of avoid as a penalty against avoid as a ban, on one route (no alternatives)
        if (System.getenv("NM_LA_PENALTY") != null) for (pen in listOf(false, true, false, true)) {
            val body = linkedMapOf<String, Any?>("coordinates" to listOf(listOf(15.9819, 45.8150), listOf(19.0402, 47.4979)), "options" to mapOf("avoid_features" to listOf("highways", "tollways")))
            if (pen) body["navmaster"] = mapOf("avoid_penalty" to true)
            val t0 = System.currentTimeMillis()
            val res = cb.handle(DirectionsRequest.fromJson("driving-car", body), listOf(null, null))
            val f = (res["features"] as List<Map<String, Any?>>)[0]
            println("   penalità=$pen: ${(LongAlternatives.duration(f) / 60).toInt()} min in ${System.currentTimeMillis() - t0} ms")
            val body2 = linkedMapOf<String, Any?>("coordinates" to listOf(listOf(15.9819, 45.8150), listOf(16.4000, 46.3000)), "options" to mapOf("avoid_features" to listOf("highways", "tollways")))
            if (pen) body2["navmaster"] = mapOf("avoid_penalty" to true)
            val t1 = System.currentTimeMillis()
            cb.handle(DirectionsRequest.fromJson("driving-car", body2), listOf(null, null))
            println("   penalità=$pen, solo Croazia: ${System.currentTimeMillis() - t1} ms")
        }
        var withAlternatives = 0
        for ((name, profile, pts) in trips) {
            val body = linkedMapOf<String, Any?>(
                "coordinates" to pts.map { listOf(it.second, it.first) }, "instructions" to true, "language" to "it", "maneuvers" to true,
                "navmaster" to mapOf("roads" to true),
                "alternative_routes" to mapOf("target_count" to 3, "weight_factor" to 1.4, "share_factor" to 0.6),
            )
            val options = linkedMapOf<String, Any?>()
            if (profile == "driving-hgv") options["profile_params"] = mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 40.0, "hazmat" to true, "adr_tunnel_code" to "D"))
            if (name.contains("senza autostrade")) options["avoid_features"] = listOf("highways", "tollways")
            if (options.isNotEmpty()) body["options"] = options
            if (name.contains("penalità")) body["navmaster"] = mapOf("roads" to true, "avoid_penalty" to true)
            println("== $name")
            val t0 = System.currentTimeMillis()
            val res = try { cb.handle(DirectionsRequest.fromJson(profile, body), List(pts.size) { null }) } catch (e: ApiException) { println("   ERRORE ${e.code} ${e.message}"); continue }
            val features = res["features"] as List<Map<String, Any?>>
            val s0 = LongAlternatives.segments(features[0])
            for ((i, f) in features.withIndex()) {
                val sum = (f["properties"] as Map<String, Any?>)["summary"] as Map<String, Any?>
                println("   %d) %.0f min, %.0f km, in comune col primo %.0f %%".format(i, (sum["duration"] as Number).toDouble() / 60, (sum["distance"] as Number).toDouble() / 1000,
                    LongAlternatives.shared(LongAlternatives.segments(f), s0) * 100))
            }
            println("   ${features.size} percorsi in ${System.currentTimeMillis() - t0} ms")
            if (features.size > 1) withAlternatives++
        }
        if (System.getenv("NM_LA_PENALTY") == null) assertTrue(withAlternatives >= 2, "alternative trovate solo per $withAlternatives viaggi")
    }
}
