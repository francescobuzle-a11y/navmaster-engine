package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import kotlin.test.Test

/**
 * Diagnostica su grafi veri (CI, flusso «Valichi su grafi veri»): `NM_CB="italy=/…,austria=/…"`,
 * `NM_TRIPS="lat,lon;lat,lon|…"` (predefinito: Soncino → Innsbruck). Per ogni viaggio la richiesta
 * dell'app (camion con le misure, evita traghetti, maximum_speed 80), il registro della scelta del
 * valico e, sul primo confine, per ogni passaggio il limite inferiore della ricerca sul core e il
 * peso vero della stessa ricerca, dai due capi; la copertura dei landmark.
 */
class CrossBorderRealDiagTest {
    @Test
    fun diagnose() {
        val spec = System.getenv("NM_CB") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val t0 = System.currentTimeMillis()
            val g = RoutingGraph.load(File(path), true)
            println("$id aperto in ${System.currentTimeMillis() - t0} ms: ${g.graph.nodeCount} nodi, ${g.graph.edgeCount} archi, landmark ${g.landmarks.mapValues { it.value.landmarkCount }}")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val cache = HashMap<String, List<Transfer>>()
        val transfers = { a: CountryGraph, b: CountryGraph ->
            cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) {
                val t0 = System.currentTimeMillis()
                Transfers.find(a.graph, b.graph).also { println("passaggi ${a.id}-${b.id}: ${it.size} in ${System.currentTimeMillis() - t0} ms") }
            }
        }
        val trips = (System.getenv("NM_TRIPS") ?: "45.4103,9.9428;47.2692,11.4041").split('|').map { t ->
            t.split(';').map { p -> p.split(',').let { it[0].trim().toDouble() to it[1].trim().toDouble() } }
        }
        val cb = CrossBorderDirections(graphs, transfers, { println("  $it") })
        for (pts in trips) {
            val body = linkedMapOf<String, Any?>(
                "coordinates" to pts.map { listOf(it.second, it.first) }, "preference" to "recommended", "instructions" to true, "language" to "it",
                "options" to linkedMapOf<String, Any?>("avoid_features" to listOf("ferries"), "vehicle_type" to "hgv",
                    "profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "width" to 2.55, "length" to 16.5, "weight" to 27.0, "axleload" to 11.5))),
                "maximum_speed" to 80.0,
            )
            val req = DirectionsRequest.fromJson("driving-hgv", body)
            println("== viaggio ${pts.joinToString(" → ") { "%.4f,%.4f".format(it.first, it.second) }}")
            val t0 = System.currentTimeMillis()
            val res = runCatching { cb.handle(req, List(pts.size) { null }) }
            println("risultato in ${System.currentTimeMillis() - t0} ms: ${res.exceptionOrNull()?.message ?: (res.getOrThrow()["metadata"] as Map<*, *>)["countries"]}")
            if (cb.lastPieces.size < 2) continue
            // il primo confine: limiti e pesi veri dai due capi, per ogni passaggio
            val (ga, _) = cb.lastPieces[0]; val (gb, _) = cb.lastPieces[1]
            val all = transfers(ga, gb)
            for ((g, root, rootIsTarget) in listOf(Triple(ga, pts.first(), false), Triple(gb, pts.last(), true))) {
                val o = g.service.routeOptions(req).copy(hasBearings = false, maximumSpeedTimeOnly = true)
                val router = g.graph.router(o)
                val sr = router.snap(root.first, root.second, 400.0, 0)
                val usable = all.filter { t -> runCatching { router.snap(t.lat, t.lon, 3.0, 0, checkNetwork = false) }.isSuccess }
                val snaps = usable.map { t -> router.snap(t.lat, t.lon, 3.0, 0, checkNetwork = false) }
                val tm = System.currentTimeMillis()
                val m = router.manyToOne(sr, snaps, rootIsTarget)!!
                println("${g.id}: molti-a-uno preparato in ${System.currentTimeMillis() - tm} ms (${m.stats}); ${router.algorithm}")
                val rows = usable.indices.map { i ->
                    val t = usable[i]
                    val ts = System.currentTimeMillis()
                    val w = try { (if (rootIsTarget) router.route(snaps[i], sr, exact = false) else router.route(sr, snaps[i], exact = false)).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
                    Triple(t, m.lowerBound(i) to w, System.currentTimeMillis() - ts)
                }.sortedBy { it.second.second }
                for ((t, lw, ms) in rows.take(25)) {
                    val (lb, w) = lw
                    println("   %.5f,%.5f r${t.rank} a %.0f km: limite %s, peso %s (%s), ricerca $ms ms".format(t.lat, t.lon,
                        DistanceCalc.haversine(root.first, root.second, t.lat, t.lon) / 1000,
                        if (lb.isFinite()) "%.0f".format(lb) else "∞", if (w.isFinite()) "%.0f".format(w) else "∞",
                        if (lb.isFinite() && w.isFinite() && w > 0) "%.0f %%".format(100 * lb / w) else "-"))
                }
                // i landmark: dove sono e quanti nodi del core li raggiungono
                val lm = g.graph.landmarks.values.firstOrNull { it.landmarkCount > 0 }
                if (lm != null) {
                    val bg = g.graph.graph
                    val sample = (0 until bg.nodeCount step maxOf(1, bg.nodeCount / 20000)).filter { lm.isCoreNode(it) }
                    for (l in 0 until lm.landmarkCount) {
                        val n = lm.landmarkNodes[l]
                        val fin = sample.count { lm.fromLandmark(l, it).isFinite() }
                        println("   landmark $l: %.4f,%.4f, raggiunge ${100 * fin / maxOf(1, sample.size)} %% del core".format(bg.lat(n), bg.lon(n)))
                    }
                }
            }
        }
    }
}
