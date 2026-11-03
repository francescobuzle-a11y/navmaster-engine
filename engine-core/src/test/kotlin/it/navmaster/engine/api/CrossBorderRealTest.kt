package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Viaggi tra Paesi su grafi veri (solo in locale): `NM_CB="it=/…/italy/nm,si=/…/slovenia,hr=/…/croatia"`.
 * Stampa per ogni viaggio i Paesi, la distanza, il tempo, le ricerche fatte e controlla che la
 * linea sia continua e che le istruzioni non abbiano arrivi intermedi al confine.
 */
class CrossBorderRealTest {
    @Test
    fun realTrips() {
        val spec = System.getenv("NM_CB") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val t0 = System.currentTimeMillis()
            val g = RoutingGraph.load(File(path), true)
            println("$id aperto in ${System.currentTimeMillis() - t0} ms: ${g.graph.nodeCount} nodi, ${g.graph.edgeCount} archi")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val cache = HashMap<String, List<Transfer>>()
        val cb = CrossBorderDirections(graphs, { a, b ->
            val key = listOf(a.id, b.id).sorted().joinToString("-")
            cache.getOrPut(key) {
                val t0 = System.currentTimeMillis()
                Transfers.find(a.graph, b.graph).also { l -> println("passaggi $key: ${l.size} (${l.count { it.rank <= 3 }} su strade principali) in ${System.currentTimeMillis() - t0} ms: ${Transfers.timing}") }
            }
        }, { println("  $it") })
        val trips = listOf(
            Triple("Trieste → Lubiana (auto)", "driving-car", listOf(45.6495 to 13.7768, 46.0569 to 14.5058)),
            Triple("Gorizia → Nova Gorica (auto)", "driving-car", listOf(45.9415 to 13.6219, 45.9558 to 13.6483)),
            Triple("Lubiana → Zagabria (auto)", "driving-car", listOf(46.0569 to 14.5058, 45.8150 to 15.9819)),
            Triple("Zagabria → Fiume (auto, solo Croazia)", "driving-car", listOf(45.8150 to 15.9819, 45.3271 to 14.4422)),
            Triple("Trieste → Fiume (auto, transito Slovenia)", "driving-car", listOf(45.6495 to 13.7768, 45.3271 to 14.4422)),
            Triple("Udine → Zagabria (camion 4 m, 40 t)", "driving-hgv", listOf(46.0711 to 13.2346, 45.8150 to 15.9819)),
            Triple("Venezia → Lubiana → Zagabria (tappa)", "driving-car", listOf(45.4408 to 12.3155, 46.0569 to 14.5058, 45.8150 to 15.9819)),
        )
        val problems = ArrayList<String>()
        for ((name, profile, pts) in trips) {
            val body = linkedMapOf<String, Any?>(
                "coordinates" to pts.map { listOf(it.second, it.first) }, "instructions" to true, "language" to "it", "maneuvers" to true,
                "extra_info" to listOf("tollways", "waycategory", "waytype"), "navmaster" to mapOf("roads" to true),
            )
            if (profile == "driving-hgv") body["options"] = mapOf("profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 40.0)))
            val req = DirectionsRequest.fromJson(profile, body)
            val t0 = System.currentTimeMillis()
            val res = try { cb.handle(req, List(pts.size) { null }) } catch (e: ApiException) { problems += "$name: errore ${e.code} ${e.message}"; println("$name: ERRORE ${e.code} ${e.message}"); continue }
            val ms = System.currentTimeMillis() - t0
            @Suppress("UNCHECKED_CAST") val f = (res["features"] as List<Map<String, Any?>>)[0]
            @Suppress("UNCHECKED_CAST") val props = f["properties"] as Map<String, Any?>
            @Suppress("UNCHECKED_CAST") val coords = (f["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            @Suppress("UNCHECKED_CAST") val summary = props["summary"] as Map<String, Any?>
            @Suppress("UNCHECKED_CAST") val segs = props["segments"] as List<Map<String, Any?>>
            @Suppress("UNCHECKED_CAST") val wps = props["way_points"] as List<Int>
            var maxGap = 0.0
            for (i in 1 until coords.size) maxGap = maxOf(maxGap, DistanceCalc.haversine(coords[i - 1][1], coords[i - 1][0], coords[i][1], coords[i][0]))
            @Suppress("UNCHECKED_CAST") val steps = segs.flatMap { it["steps"] as List<Map<String, Any?>> }
            val finishes = steps.count { it["type"] == 10 }
            @Suppress("UNCHECKED_CAST") val metadata = res["metadata"] as Map<String, Any?>
            println("$name: ${metadata["countries"] ?: "un grafo"}, %.1f km, %.0f min, ${coords.size} punti, salto massimo %.0f m, ${segs.size} tratti, ${steps.size} istruzioni, $ms ms"
                .format((summary["distance"] as Double) / 1000, (summary["duration"] as Double) / 60, maxGap))
            if (segs.size != pts.size - 1) problems += "$name: ${segs.size} tratti invece di ${pts.size - 1}"
            if (wps.size != pts.size || wps.last() != coords.size - 1) problems += "$name: way_points $wps (${coords.size} punti)"
            if (finishes != pts.size - 1) problems += "$name: $finishes arrivi invece di ${pts.size - 1}"
            if (maxGap > 5000) problems += "$name: linea interrotta (salto di %.0f m)".format(maxGap)
            // le istruzioni attorno ai confini
            @Suppress("UNCHECKED_CAST")
            val roads = ((props["navmaster"] as? Map<String, Any?>)?.get("roads") as? List<Map<String, Any?>>).orEmpty()
            if (roads.isNotEmpty() && (roads.last()["to"] as Int) != coords.size - 1) problems += "$name: fatti stradali fino a ${roads.last()["to"]} su ${coords.size - 1}"
            if (name.startsWith("Trieste → Lubiana")) for (s in steps) println("    ${s["type"]} ${s["instruction"]} · ${s["distance"]} m · ${s["way_points"]}")
        }
        // ORS bearings: on the Slovenian motorway of Trieste → Ljubljana, the way the route goes and the opposite one
        graphs.firstOrNull { it.id == "si" }?.let { si ->
            val base = DirectionsRequest.fromJson("driving-car", linkedMapOf("coordinates" to listOf(listOf(13.7768, 45.6495), listOf(14.5058, 46.0569)), "instructions" to false))
            @Suppress("UNCHECKED_CAST")
            val all = ((cb.handle(base, listOf(null, null))["features"] as List<Map<String, Any?>>)[0]["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            val k = all.size * 2 / 3
            val a = all[k]; val b = all[k + 3]
            val dir = DistanceCalc.bearing(a[1], a[0], b[1], b[0])
            fun dist(bearing: Double?): Double {
                val body = linkedMapOf<String, Any?>("coordinates" to listOf(listOf(a[0], a[1]), listOf(14.5058, 46.0569)), "instructions" to false)
                if (bearing != null) body["bearings"] = listOf(listOf(bearing, 45.0), emptyList<Double>())
                @Suppress("UNCHECKED_CAST")
                val f = (si.service.handle(DirectionsRequest.fromJson("driving-car", body), ResponseFormat.GEOJSON)["features"] as List<Map<String, Any?>>)[0]
                @Suppress("UNCHECKED_CAST")
                return ((f["properties"] as Map<String, Any?>)["summary"] as Map<String, Any?>)["distance"] as Double
            }
            val none = dist(null); val with = dist(dir); val against = dist((dir + 180) % 360)
            println("direzioni sull'autostrada (%.0f°): senza %.1f km, nel verso %.1f km, contro %.1f km".format(dir, none / 1000, with / 1000, against / 1000))
            if (with > none + 50) problems += "bearings: nel verso del percorso più lungo (%.0f m invece di %.0f)".format(with, none)
            if (against < with + 500) problems += "bearings: contromano non più lungo (%.0f m contro %.0f)".format(against, with)
        }
        println(if (problems.isEmpty()) "TUTTO OK" else "PROBLEMI:\n" + problems.joinToString("\n"))
        assertTrue(problems.isEmpty(), problems.joinToString("\n"))
    }
}
