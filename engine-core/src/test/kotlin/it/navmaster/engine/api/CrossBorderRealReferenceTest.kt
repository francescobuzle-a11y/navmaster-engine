package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Le prove per Paese sui grafi veri (CI, flusso «Valichi per Paese»): `NM_CB="italy=/…,austria=/…"`.
 * Per ogni coppia di grafi con passaggi: viaggi a cavallo del confine (partenza e arrivo entro
 * qualche decina di km da un passaggio), la scelta del valico contro il riferimento esaustivo
 * ([CrossBorderReference], tutti i passaggi), con le richieste dell'app. Per ogni grafo: viaggi
 * interni, la ricerca sul core contro la ricerca piena edge-based e la CH contro Dijkstra.
 * `NM_CB_TRIPS` viaggi per confine (6), `NM_INTERNAL` viaggi interni per Paese (12), `NM_SEED`.
 */
class CrossBorderRealReferenceTest {
    private fun same(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || abs(x - y) <= 0.05 + abs(y) * 1e-5

    @Test
    fun countries() {
        val spec = System.getenv("NM_CB") ?: return
        val tripsPerBorder = System.getenv("NM_CB_TRIPS")?.toIntOrNull() ?: 6
        val internal = System.getenv("NM_INTERNAL")?.toIntOrNull() ?: 12
        val rnd = Random(System.getenv("NM_SEED")?.toLongOrNull() ?: 1L)
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val g = RoutingGraph.load(File(path), true)
            println("$id: ${g.graph.nodeCount} nodi, ${g.graph.edgeCount} archi, preparazioni ${g.chStorages.keys}, landmark ${g.landmarks.mapValues { it.value.landmarkCount }}")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val cache = HashMap<String, List<Transfer>>()
        val transfers = { a: CountryGraph, b: CountryGraph ->
            cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) { Transfers.find(a.graph, b.graph) }
        }
        val problems = ArrayList<String>()
        val params = VehicleParameters(height = 4.0, width = 2.55, length = 16.5, weight = 27.0, axleLoad = 11.5)

        /** Un punto stradale a caso nel grafo, a una distanza fra [min] e [max] m da (lat, lon). */
        fun pointNear(g: CountryGraph, r: Router, lat: Double, lon: Double, min: Double, max: Double): Snap? {
            val bg = g.graph.graph
            repeat(200_000) {
                val n = rnd.nextInt(bg.nodeCount)
                val d = DistanceCalc.haversine(lat, lon, bg.lat(n), bg.lon(n))
                if (d in min..max) try { return r.snap(bg.lat(n), bg.lon(n), 400.0, 0) } catch (e: PointNotFoundException) { }
            }
            return null
        }

        // ---- viaggi a cavallo di ogni confine
        val variants = listOf<Pair<String, (List<Snap>) -> Pair<String, Map<String, Any?>>>>(
            "camion dell'app" to { pts -> "driving-hgv" to appBody(pts, avoid = listOf("ferries"), speed = 80.0) },
            "camion, evita autostrade" to { pts -> "driving-hgv" to appBody(pts, avoid = listOf("ferries", "highways"), speed = 80.0) },
            "camion, evita pedaggi" to { pts -> "driving-hgv" to appBody(pts, avoid = listOf("ferries", "tollways"), speed = null) },
            "auto" to { pts -> "driving-car" to linkedMapOf("coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false) },
        )
        for (i in graphs.indices) for (j in graphs.indices) {
            if (i == j) continue
            val a = graphs[i]; val b = graphs[j]
            val all = transfers(a, b)
            if (all.isEmpty()) continue
            println("== confine ${a.id} → ${b.id}: ${all.size} passaggi (${all.count { it.rank <= 3 }} su strade principali)")
            val ra = a.graph.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            val rb = b.graph.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            val cb = CrossBorderDirections(listOf(a, b), transfers, null)
            val ref = CrossBorderReference(listOf(a, b), transfers)
            repeat(tripsPerBorder) { k ->
                val t = all[rnd.nextInt(all.size)]
                val p = pointNear(a, ra, t.lat, t.lon, 3_000.0, 60_000.0) ?: return@repeat
                val q = pointNear(b, rb, t.lat, t.lon, 3_000.0, 60_000.0) ?: return@repeat
                val (name, make) = variants[(k + i) % variants.size]
                val (profile, body) = make(listOf(p, q))
                val req = DirectionsRequest.fromJson(profile, body)
                val t0 = System.currentTimeMillis()
                val got = runCatching { cb.handle(req, listOf(a.id, b.id)) }
                val cbMs = System.currentTimeMillis() - t0
                val w = if (got.isSuccess) ref.weightOf(cb.lastPieces) else Double.POSITIVE_INFINITY
                val t1 = System.currentTimeMillis()
                val before = ref.pieceEvaluations
                val r = ref.best(req, listOf(a, b))
                val refMs = System.currentTimeMillis() - t1
                val ok = same(w, r.weight)
                val line = "  ${a.id}→${b.id} $name %.4f,%.4f → %.4f,%.4f: peso %s, riferimento %s (%d pezzi in %d ms), scelta in %d ms%s".format(
                    p.queryLat, p.queryLon, q.queryLat, q.queryLon, fmt(w), fmt(r.weight), ref.pieceEvaluations - before, refMs, cbMs,
                    if (ok) "" else " — DIVERSO: valico ${cb.lastPieces.drop(1).map { it.second.coordinates[0] }} invece di ${r.crossings.map { it.lat to it.lon }}" +
                        (got.exceptionOrNull()?.let { " (${it.message?.take(80)})" } ?: ""))
                println(line)
                if (!ok) problems += line.trim()
            }
        }

        // ---- i landmark di ogni core: ognuno deve raggiungere ed essere raggiunto da quasi tutto il core
        for (g in graphs) for ((key, lm) in g.graph.landmarks) {
            val st = g.graph.chStorages[key] ?: continue
            val n = g.graph.graph.nodeCount
            val sample = (0 until n step maxOf(1, n / 200_000)).filter { st.level(it) >= n }
            val cov = (0 until lm.landmarkCount).map { l ->
                100 * sample.count { lm.fromLandmark(l, it).isFinite() } / maxOf(1, sample.size) to 100 * sample.count { lm.toLandmark(l, it).isFinite() } / maxOf(1, sample.size)
            }
            val worst = cov.minOfOrNull { minOf(it.first, it.second) } ?: 100
            val line = "  ${g.id} landmark $key: ${cov.joinToString(" ") { "${it.first}/${it.second}" }} (raggiunge/raggiunto da, % del core)"
            println(line)
            if (worst < 50) problems += "${g.id} landmark $key: un landmark copre solo il $worst % del core (limiti nulli, ricerche lente)"
        }

        // ---- viaggi interni di ogni Paese: core contro ricerca piena, CH contro Dijkstra
        for (g in graphs) {
            val core = g.graph.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            val full = g.graph.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false))
            val ch = g.graph.router(RouteOptions(profile = "driving-car", turnCosts = false))
            val plain = g.graph.router(RouteOptions(profile = "driving-car", turnCosts = false, allowCH = false))
            println("== interni ${g.id}: ${core.algorithm} contro ${full.algorithm}, ${ch.algorithm} contro ${plain.algorithm}")
            var equal = 0; var loops = 0
            val bg = g.graph.graph
            repeat(internal) { k ->
                val n = rnd.nextInt(bg.nodeCount)
                val (r1, r2) = if (k % 2 == 0) core to full else ch to plain
                val p = pointNear(g, r1, bg.lat(n), bg.lon(n), 0.0, 1_000.0) ?: return@repeat
                val q = pointNear(g, r1, p.queryLat, p.queryLon, 2_000.0, 30_000.0) ?: return@repeat
                var unsnapped = false
                fun path(r: Router) = try { r.route(r.snap(p.queryLat, p.queryLon, 400.0, 0), r.snap(q.queryLat, q.queryLon, 400.0, 1)) }
                    catch (e: RouteNotFoundException) { null } catch (e: PointNotFoundException) { unsnapped = true; null }
                val p1 = path(r1); val p2 = path(r2)
                if (unsnapped) return@repeat
                val w1 = p1?.weight ?: Double.POSITIVE_INFINITY; val w2 = p2?.weight ?: Double.POSITIVE_INFINITY
                if (same(w1, w2)) equal++ else {
                    // la ricerca piena edge-based può girare attorno a un isolato e ripassare da un nodo (inversioni
                    // vietate); fuori dal core la CH è node-based e non lo fa, come in ORS: limite noto, non un errore
                    val loop = p2 != null && p1 != null && w2 < w1 && p2.edges.map { it.baseNode }.let { it.size != it.toSet().size } && full.isLegal(p2)
                    val line = "  ${g.id} interno ${r1.algorithm}/${r2.algorithm} %.4f,%.4f → %.4f,%.4f: %s contro %s%s".format(p.queryLat, p.queryLon, q.queryLat, q.queryLon, fmt(w1), fmt(w2),
                        if (loop) " (la ricerca piena ripassa da un nodo: limite noto del core)" else "")
                    println(line)
                    if (loop) loops++ else problems += line.trim()
                }
            }
            println("  ${g.id}: $equal interni uguali, $loops col giro della ricerca piena")
        }
        println(if (problems.isEmpty()) "TUTTO OK" else "DIFFERENZE:\n" + problems.joinToString("\n"))
        assertTrue(problems.isEmpty(), problems.joinToString("\n"))
    }

    private fun fmt(x: Double) = if (x.isFinite()) "%.1f".format(x) else "∞"

    /** La richiesta dell'app per un camion (vedi NmRouting.request nell'app). */
    private fun appBody(pts: List<Snap>, avoid: List<String>, speed: Double?) = linkedMapOf<String, Any?>(
        "coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "preference" to "recommended", "instructions" to false,
        "options" to linkedMapOf<String, Any?>("avoid_features" to avoid, "vehicle_type" to "hgv",
            "profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "width" to 2.55, "length" to 16.5, "weight" to 27.0, "axleload" to 11.5))),
    ).apply { if (speed != null) put("maximum_speed", speed) }
}
