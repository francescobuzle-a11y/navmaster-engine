package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.PriorityModel
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import java.util.concurrent.Executors
import kotlin.test.Test

/**
 * Sonda su grafi veri (CI, flusso «Valichi su grafi veri», `NM_CB="italy=/…,austria=/…"`, prova
 * `it.navmaster.engine.api.AlpineRouteProbeTest`): da Canneto sull'Oglio il camion va a Wien per la Val
 * Pusteria (SS49 → Lienz → A10 → Murtal B96/B317 → S36 → A2) invece che per Tarvisio (A4 → A23 → A2).
 *
 * Per ogni valico italia/austria vicino a Tarvisio, Prato alla Drava/Sillian (Pusteria) e al Brennero:
 * i due pezzi obbligati come quelli del motore (Canneto → valico nel grafo italiano, valico → Wien in quello
 * austriaco), coi km, i minuti e il peso; il tempo e i km per codice di priorità e il peso che il percorso
 * avrebbe con ciascun modello (ORS e NAVMASTER) e col solo tempo; i km e la velocità media per classe di
 * strada; le strade principali. Poi il migliore del motore Canneto → Wien. Tutte le righe cominciano con
 * «sonda». Senza `NM_CB` non fa niente. Nessuna asserzione: è una misura.
 */
class AlpineRouteProbeTest {
    private val restrictions = mapOf("height" to 4.0, "width" to 2.55, "length" to 16.5, "weight" to 27.0, "axleload" to 11.5)

    private fun request(pts: List<Pair<Double, Double>>, via: List<Boolean>? = null): DirectionsRequest {
        val nm = linkedMapOf<String, Any?>("roads" to true)
        if (via != null && via.any { it }) nm["pass_through"] = via
        return DirectionsRequest.fromJson("driving-hgv", linkedMapOf<String, Any?>(
            "coordinates" to pts.map { listOf(it.second, it.first) }, "preference" to "recommended", "instructions" to true, "language" to "it",
            "options" to linkedMapOf<String, Any?>("avoid_features" to listOf("ferries"), "vehicle_type" to "hgv", "profile_params" to mapOf("restrictions" to restrictions)),
            "navmaster" to nm, "maximum_speed" to 80.0,
        ))
    }

    private val canneto = 45.1502 to 10.3792
    private val wien = 48.2082 to 16.3738
    private val places = listOf(
        "Tarvisio" to (46.5130 to 13.6500),
        "Pusteria (Prato alla Drava/Sillian)" to (46.7400 to 12.3900),
        "Brennero" to (47.0030 to 11.5060),
    )

    private class Leg(val rg: RoutingGraph, val path: RoutePath)

    /** Tempo e km per codice di priorità, il peso con ogni modello e col solo tempo; km e velocità media per classe di strada. */
    private fun breakdown(tag: String, legs: List<Leg>) {
        val s = DoubleArray(8); val m = DoubleArray(8)
        val clsS = LinkedHashMap<String, Double>(); val clsM = LinkedHashMap<String, Double>()
        var w = 0.0
        for (l in legs) {
            val rg = l.rg; val hv = rg.encoding.profile("heavyvehicle"); val sh = rg.encoding.shared
            val it = rg.graph.createEdgeIterator()
            for (pe in l.path.edges) {
                it.setEdge(pe.edge, rg.graph.nodeA(pe.edge))
                val c = Math.round(it.get(hv.priority) * 7).toInt().coerceIn(0, 7)
                s[c] += pe.millis / 1000.0; m[c] += pe.distance
                val cls = it.get(sh.roadClass).osm + if (it.get(sh.roadClassLink)) "_link" else ""
                clsS[cls] = (clsS[cls] ?: 0.0) + pe.millis / 1000.0; clsM[cls] = (clsM[cls] ?: 0.0) + pe.distance
            }
            w += l.path.weight
        }
        println("$tag priorità: " + (7 downTo 0).filter { s[it] > 0 }.joinToString(", ") { c -> "codice $c %.1f min %.1f km".format(s[c] / 60, m[c] / 1000) })
        println("$tag pesi: tempo %.1f min, peso del grafo %.0f; ".format(s.sum() / 60, w) +
            PriorityModel.entries.joinToString(", ") { md -> "peso ${md.id} %.0f".format((0..7).sumOf { c -> s[c] * md.factor(c / 7.0) }) })
        println("$tag classi: " + clsM.entries.sortedByDescending { it.value }.joinToString(", ") { (k, v) ->
            "$k %.1f km %.1f min (%.0f km/h)".format(v / 1000, clsS[k]!! / 60, v / clsS[k]!! * 3.6) })
    }

    /** Le strade di seguito con lo stesso ref (o nome) lunghe almeno 3 km. */
    private fun roads(tag: String, legs: List<Leg>) {
        val runs = ArrayList<Triple<String, Double, Double>>()
        for (l in legs) for (pe in l.path.edges) {
            val key = l.rg.guidance?.ref(pe.edge).orEmpty().ifEmpty { l.rg.graph.name(pe.edge).ifEmpty { "-" } }
            if (runs.isNotEmpty() && runs.last().first == key) runs[runs.size - 1] = Triple(key, runs.last().second + pe.distance, runs.last().third + pe.millis / 1000.0)
            else runs += Triple(key, pe.distance, pe.millis / 1000.0)
        }
        println("$tag strade: " + runs.filter { it.second >= 3_000 }.joinToString(" → ") { "${it.first} %.0f km %.0f km/h".format(it.second / 1000, it.second / it.third * 3.6) })
    }

    @Test
    fun probe() {
        val spec = System.getenv("NM_CB") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val g = RoutingGraph.load(File(path), true)
            println("sonda: $id aperto: ${g.graph.nodeCount} nodi, modello di priorità ${g.encoding.priorityModel.id}")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val ita = graphs.firstOrNull { g -> g.id == "italy" }; val at = graphs.firstOrNull { g -> g.id == "austria" }
        if (ita == null || at == null) { println("sonda: servono italy e austria"); return }
        val all = Transfers.find(ita.graph, at.graph)
        println("sonda: ${all.size} passaggi italy/austria")
        val truckReq = request(listOf(canneto, wien))
        fun piece(g: CountryGraph, pts: List<Pair<Double, Double>>, transferFirst: Boolean): RoutePath {
            val r = g.service.maximumSnappingRadius; val tr = CrossBorderDirections.TRANSFER_RADIUS
            val req = truckReq.withPoints(pts, if (transferFirst) doubleArrayOf(tr, r) else doubleArrayOf(r, tr), if (transferFirst) booleanArrayOf(true, false) else booleanArrayOf(false, true))
            return g.service.solveAsHandled(req, RequestContext()).legs.single()
        }
        class Forced(val place: String, val t: Transfer, val itLeg: RoutePath, val atLeg: RoutePath) {
            val weight get() = itLeg.weight + atLeg.weight
            val millis get() = itLeg.millis + atLeg.millis
            val dist get() = itLeg.distance + atLeg.distance
        }
        val best = ArrayList<Forced>()
        for ((name, c) in places) {
            val near = all.filter { t -> DistanceCalc.haversine(t.lat, t.lon, c.first, c.second) <= 15_000 }
            val found = ArrayList<Forced>()
            for (t in near) {
                val a = runCatching { piece(ita, listOf(canneto, t.lat to t.lon), transferFirst = false) }.getOrNull() ?: continue
                val b = runCatching { piece(at, listOf(t.lat to t.lon, wien), transferFirst = true) }.getOrNull() ?: continue
                found += Forced(name, t, a, b)
            }
            println("sonda $name: ${near.size} passaggi entro 15 km, ${found.size} con percorso")
            for (f in found.sortedBy { f -> f.weight }) println("sonda $name   %.5f,%.5f r${f.t.rank}: %.1f km, %.1f min, peso %.0f".format(f.t.lat, f.t.lon, f.dist / 1000, f.millis / 60000.0, f.weight))
            found.minByOrNull { f -> f.weight }?.let { best += it }
        }
        val wMin = best.minOfOrNull { it.weight } ?: return
        val tMin = best.minOf { it.millis }
        for (f in best.sortedBy { it.weight }) {
            val tag = "sonda == ${f.place} (%.5f,%.5f)".format(f.t.lat, f.t.lon)
            println("$tag: %.1f km, %.1f min (%.2f h), peso %.0f; peso ×%.3f, tempo ×%.3f sul migliore (Italia %.1f km %.1f min, Austria %.1f km %.1f min)".format(
                f.dist / 1000, f.millis / 60000.0, f.millis / 3_600_000.0, f.weight, f.weight / wMin, f.millis.toDouble() / tMin,
                f.itLeg.distance / 1000, f.itLeg.millis / 60000.0, f.atLeg.distance / 1000, f.atLeg.millis / 60000.0))
            val legs = listOf(Leg(ita.graph, f.itLeg), Leg(at.graph, f.atLeg))
            breakdown(tag, legs)
            roads(tag, legs)
        }

        // il migliore del motore tra Paesi, e lo stesso obbligato per ogni valico (punto di passaggio 3 km dopo il valico)
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { t -> t.isDaemon = true } }
        try {
            val tcache = HashMap<String, List<Transfer>>()
            val transfers = { a: CountryGraph, b: CountryGraph -> synchronized(tcache) { tcache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) { Transfers.find(a.graph, b.graph) } } }
            var verbose = false
            val vlog: (String) -> Unit = { line -> if (verbose) println("sonda registro: $line") }
            val cb = CrossBorderDirections(graphs, transfers, vlog)
            fun trip(tag: String, pts: List<Pair<Double, Double>>) {
                val via = pts.indices.map { i -> i in 1 until pts.size - 1 }
                val t0 = System.currentTimeMillis()
                val doc = try { cb.handle(request(pts, via), List(pts.size) { null }, RequestContext(CancelToken(), RequestContext.MAIN, pool, SearchSlots(2), vlog)) }
                    catch (e: Exception) { println("$tag: errore ${e.message}"); return }
                @Suppress("UNCHECKED_CAST") val f = (doc["features"] as List<Map<String, Any?>>).first()
                val summary = (f["properties"] as Map<*, *>)["summary"] as Map<*, *>
                println("$tag: %.1f min, %.1f km, peso %.0f, valichi %s, in %d ms".format((summary["duration"] as Number).toDouble() / 60,
                    (summary["distance"] as Number).toDouble() / 1000, cb.lastWeight, cb.lastCrossings.joinToString { c -> "%.5f,%.5f".format(c.lat, c.lon) }, System.currentTimeMillis() - t0))
            }
            // con Cechia e Polonia: Canneto → Bielsko-Biała, il migliore (col registro del motore) e obbligato per ogni valico alpino
            if (graphs.any { g -> g.id == "czech-republic" } && graphs.any { g -> g.id == "poland" }) {
                val bielsko = 49.8224 to 19.0444
                verbose = true
                trip("sonda motore Canneto → Bielsko-Biała migliore", listOf(canneto, bielsko))
                verbose = false
                for (f in best) {
                    val (lat, lon) = f.atLeg.points().let { p -> val i = minOf(p.size - 1, p.size / 40); p.lat(i) to p.lon(i) }
                    trip("sonda motore Canneto → Bielsko-Biała per ${f.place}", listOf(canneto, lat to lon, bielsko))
                }
                // e obbligato per Tarvisio e poi Mikulov (A5 prima del confine)
                best.firstOrNull { f -> f.place == "Tarvisio" }?.let { f ->
                    val (lat, lon) = f.atLeg.points().let { p -> val i = minOf(p.size - 1, p.size / 40); p.lat(i) to p.lon(i) }
                    trip("sonda motore Canneto → Bielsko-Biała per Tarvisio e Drasenhofen", listOf(canneto, lat to lon, 48.7505 to 16.6380, bielsko))
                }
            }
            trip("sonda motore Canneto → Wien migliore", listOf(canneto, wien))
            for (f in best) {
                val (lat, lon) = f.atLeg.points().let { p -> val i = minOf(p.size - 1, p.size / 40); p.lat(i) to p.lon(i) }
                trip("sonda motore Canneto → Wien per ${f.place}", listOf(canneto, lat to lon, wien))
            }
        } finally { pool.shutdownNow() }
        println("sonda: fine")
    }
}
