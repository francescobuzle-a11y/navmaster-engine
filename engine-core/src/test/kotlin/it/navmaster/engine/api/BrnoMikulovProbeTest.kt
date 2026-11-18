package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.profile.PriorityModel
import it.navmaster.engine.routing.Router
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import java.util.concurrent.Executors
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.test.Test

/**
 * Sonda su grafi veri (CI, flusso «Valichi su grafi veri», `NM_CB="italy=/…,austria=/…,czech-republic=/…,…"`)
 * per tre segnalazioni dal tablet sul viaggio del camion Canneto sull'Oglio → Bielsko-Biała:
 *
 * A) lo svincolo D1/D2 di Brno (Brno-jih): arrivando da sud sulla D52/D2 e dovendo andare sulla D1
 *    verso est, il percorso va sulla D1 a ovest fino a Brno-západ/Bohunice, torna indietro e riprende
 *    la D1 verso est. Stampa gli archi intorno ai due svincoli (accesso, velocità, pesi, limiti, divieti
 *    di svolta), il percorso del camion e dell'auto, il confronto core/ricerca piena e i percorsi obbligati
 *    sulle rampe dello svincolo.
 * B) il valico di Mikulov/Drasenhofen (A5 → strada 52 → D52 → Brno) contro quello di Hatě/Kleinhaugsdorf
 *    (S3, Znojmo): i passaggi di [Transfers] vicini, gli archi delle due strade, Wien → Brno per ciascun
 *    passaggio (due pezzi come quelli del motore: Wien → passaggio nel grafo austriaco, passaggio → Brno in quello
 *    ceco), il confronto Mikulov/Hatě, le velocità del camion e dell'auto lungo la 52 e la A5/B7, e il viaggio
 *    intero obbligato per Mikulov e per Hatě contro il migliore.
 * C) la tappa in Austria (Drasenhofen, Wien) come «passa di qui»: tempi e ricerche per sequenza, con le
 *    righe del registro del motore (sul tablet 202 s).
 *
 * Tutte le righe cominciano con «sonda» (per trovarle nel registro). Senza `NM_CB` la prova non fa
 * nulla (e passa). Nessuna asserzione: è una diagnosi.
 */
class BrnoMikulovProbeTest {
    /** Le misure del camion dell'app (NmRouting): 4 m, 2,55 m, 16,5 m, 27 t, 11,5 t per asse. */
    private val restrictions = mapOf("height" to 4.0, "width" to 2.55, "length" to 16.5, "weight" to 27.0, "axleload" to 11.5)

    /** Il corpo della richiesta come lo fa l'app: camion (maximum_speed 80) o auto, evita traghetti, roads. */
    private fun body(pts: List<Pair<Double, Double>>, car: Boolean, via: List<Boolean>? = null, bearings: List<Double?>? = null): Map<String, Any?> {
        val nm = linkedMapOf<String, Any?>("roads" to true)
        if (via != null && via.any { it }) nm["pass_through"] = via
        return linkedMapOf<String, Any?>(
            "coordinates" to pts.map { listOf(it.second, it.first) }, "preference" to "recommended", "instructions" to true, "language" to "it",
            "options" to (if (car) linkedMapOf<String, Any?>("avoid_features" to listOf("ferries"))
                else linkedMapOf<String, Any?>("avoid_features" to listOf("ferries"), "vehicle_type" to "hgv", "profile_params" to mapOf("restrictions" to restrictions))),
            "navmaster" to nm,
        ).apply {
            if (!car) put("maximum_speed", 80.0)
            if (bearings != null && bearings.any { it != null }) put("bearings", bearings.map { b -> if (b == null) emptyList<Double>() else listOf(b, 45.0) })
        }
    }

    private fun request(pts: List<Pair<Double, Double>>, car: Boolean, via: List<Boolean>? = null, bearings: List<Double?>? = null): DirectionsRequest =
        DirectionsRequest.fromJson(if (car) "driving-car" else "driving-hgv", body(pts, car, via, bearings))

    // ------------------------------------------------------------------ geometria

    /** Il punto di [pl] più vicino a (lat, lon): distanza in metri, segmento, punto proiettato. */
    private class Near(val dist: Double, val seg: Int, val lat: Double, val lon: Double)

    private fun nearest(pl: PointList, lat: Double, lon: Double): Near {
        if (pl.size == 0) return Near(Double.POSITIVE_INFINITY, 0, lat, lon)
        if (pl.size == 1) return Near(DistanceCalc.haversine(lat, lon, pl.lat(0), pl.lon(0)), 0, pl.lat(0), pl.lon(0))
        val kx = 111_320.0 * cos(Math.toRadians(lat)); val ky = 110_540.0
        var best = Near(Double.POSITIVE_INFINITY, 0, pl.lat(0), pl.lon(0))
        for (i in 0 until pl.size - 1) {
            val ax = (pl.lon(i) - lon) * kx; val ay = (pl.lat(i) - lat) * ky
            val bx = (pl.lon(i + 1) - lon) * kx; val by = (pl.lat(i + 1) - lat) * ky
            val dx = bx - ax; val dy = by - ay
            val l2 = dx * dx + dy * dy
            val t = if (l2 > 0) ((-ax * dx - ay * dy) / l2).coerceIn(0.0, 1.0) else 0.0
            val px = ax + t * dx; val py = ay + t * dy
            val d = sqrt(px * px + py * py)
            if (d < best.dist) best = Near(d, i, lat + py / ky, lon + px / kx)
        }
        return best
    }

    private fun bearingOf(pl: PointList, seg: Int): Double =
        if (pl.size < 2) 0.0 else { val i = seg.coerceIn(0, pl.size - 2); (DistanceCalc.bearing(pl.lat(i), pl.lon(i), pl.lat(i + 1), pl.lon(i + 1)) + 360) % 360 }

    private fun angle(a: Double, b: Double): Double = Math.abs(((a - b + 540.0) % 360.0) - 180.0)

    /** Il punto a [at] metri lungo [pl] (negativo: dalla fine) e la direzione lì. */
    private fun along(pl: PointList, at: Double): Triple<Double, Double, Double> {
        var total = 0.0
        for (i in 1 until pl.size) total += DistanceCalc.haversine(pl.lat(i - 1), pl.lon(i - 1), pl.lat(i), pl.lon(i))
        val target = (if (at < 0) total + at else at).coerceIn(0.0, total)
        var acc = 0.0
        for (i in 1 until pl.size) {
            val d = DistanceCalc.haversine(pl.lat(i - 1), pl.lon(i - 1), pl.lat(i), pl.lon(i))
            if (acc + d >= target && d > 0) {
                val t = (target - acc) / d
                return Triple(pl.lat(i - 1) + (pl.lat(i) - pl.lat(i - 1)) * t, pl.lon(i - 1) + (pl.lon(i) - pl.lon(i - 1)) * t, bearingOf(pl, i - 1))
            }
            acc += d
        }
        return Triple(pl.lat(pl.size - 1), pl.lon(pl.size - 1), bearingOf(pl, pl.size - 2))
    }

    /** Gli archi di [rg] entro [radius] metri da (lat, lon), coi metri, dal più vicino. */
    private fun edgesNear(rg: RoutingGraph, lat: Double, lon: Double, radius: Double): List<Pair<Int, Double>> {
        val dLat = radius / 111_000.0; val dLon = radius / (111_000.0 * cos(Math.toRadians(lat)))
        return rg.index.query(lat - dLat, lon - dLon, lat + dLat, lon + dLon)
            .map { e -> e to nearest(rg.graph.fetchWayGeometry(e, false, FetchMode.ALL), lat, lon).dist }
            .filter { it.second <= radius }.sortedBy { it.second }
    }

    // ------------------------------------------------------------------ accesso per veicolo

    /** Le regole di un router (filtro del veicolo, peso, tempi) lette su un arco nei due versi. */
    private class Access(val rg: RoutingGraph, router: Router) {
        val acc = router.access(rg.encoding.shared)
        private val it = rg.graph.createEdgeIterator()
        @Synchronized fun filterOk(e: Int): Boolean { it.setEdge(e, rg.graph.nodeA(e)); return acc.filter.accept(it) }
        @Synchronized fun weight(e: Int, reverse: Boolean): Double { it.setEdge(e, rg.graph.nodeA(e)); return acc.weighting.edgeWeight(it, reverse) }
        @Synchronized fun millis(e: Int, reverse: Boolean): Long { it.setEdge(e, rg.graph.nodeA(e)); return acc.timeWeighting.edgeMillis(it, reverse) }
        fun open(e: Int, reverse: Boolean): Boolean = filterOk(e) && weight(e, reverse).isFinite()
        fun desc(e: Int, reverse: Boolean): String = when {
            !filterOk(e) -> "vietato dal filtro del veicolo"
            weight(e, reverse).isInfinite() -> "chiuso (accesso o senso unico)"
            else -> "%.1f s, peso %.1f".format(millis(e, reverse) / 1000.0, weight(e, reverse))
        }
    }

    private fun refOf(rg: RoutingGraph, e: Int): String = rg.guidance?.ref(e).orEmpty()

    /** Una riga per l'arco [e]: nodi, capi, ref e nome, classe, accesso e pesi nei due versi, velocità, pedaggio, limiti, guida. */
    private fun describe(rg: RoutingGraph, e: Int, truck: Access, car: Access?): String {
        val g = rg.graph; val sh = rg.encoding.shared; val gd = rg.guidance
        val a = g.nodeA(e); val b = g.nodeB(e)
        val it = g.createEdgeIterator().setEdge(e, a)
        val sb = StringBuilder()
        sb.append("arco $e nodi $a→$b (%.5f,%.5f → %.5f,%.5f) %.0f m".format(g.lat(a), g.lon(a), g.lat(b), g.lon(b), g.distance(e)))
        sb.append(" | ref «${refOf(rg, e)}» nome «${g.name(e)}» | ${it.get(sh.roadClass).osm}${if (it.get(sh.roadClassLink)) "_link" else ""}")
        sb.append(" | camion A→B ${truck.desc(e, false)}; B→A ${truck.desc(e, true)}")
        if (car != null) sb.append(" | auto A→B ${car.desc(e, false)}; B→A ${car.desc(e, true)}")
        runCatching { rg.encoding.profile("heavyvehicle") }.getOrNull()?.let { hv -> sb.append(" | vel. camion %.0f/%.0f km/h".format(it.get(hv.speed), it.getReverse(hv.speed))) }
        runCatching { rg.encoding.profile("car-ors") }.getOrNull()?.let { cp -> sb.append(", auto %.0f/%.0f".format(it.get(cp.speed), it.getReverse(cp.speed))) }
        sb.append(" | pedaggio ${it.get(sh.toll)} accesso ${it.get(sh.roadAccess).osm}")
        val lim = ArrayList<String>()
        it.get(sh.maxWeight).takeIf { v -> v.isFinite() && v > 0 }?.let { v -> lim += "peso %.1f t".format(v) }
        it.get(sh.maxAxleLoad).takeIf { v -> v.isFinite() && v > 0 }?.let { v -> lim += "asse %.1f t".format(v) }
        it.get(sh.maxHeight).takeIf { v -> v.isFinite() && v > 0 }?.let { v -> lim += "altezza %.2f m".format(v) }
        it.get(sh.maxWidth).takeIf { v -> v.isFinite() && v > 0 }?.let { v -> lim += "larghezza %.2f m".format(v) }
        it.get(sh.maxLength).takeIf { v -> v.isFinite() && v > 0 }?.let { v -> lim += "lunghezza %.2f m".format(v) }
        it.get(sh.hgvBlocked).takeIf { v -> v != 0 }?.let { v -> lim += "mezzi pesanti bloccati (bit $v)" }
        if (it.get(sh.hazmatForbidden)) lim += "merci pericolose vietate"
        sb.append(" | limiti ${lim.joinToString(", ").ifEmpty { "nessuno" }}")
        if (gd != null) {
            val gl = ArrayList<String>()
            gl += "limite %d/%d, camion %d/%d, senso unico %d".format(gd.maxSpeed(e, false), gd.maxSpeed(e, true), gd.maxSpeedHgv(e, false), gd.maxSpeedHgv(e, true), gd.oneway(e))
            gd.destination(e, false).takeIf { it.isNotEmpty() }?.let { gl += "direzione A→B «$it»" }
            gd.destination(e, true).takeIf { it.isNotEmpty() }?.let { gl += "direzione B→A «$it»" }
            gd.destinationRef(e, false).takeIf { it.isNotEmpty() }?.let { gl += "ref direzione A→B «$it»" }
            gd.destinationRef(e, true).takeIf { it.isNotEmpty() }?.let { gl += "ref direzione B→A «$it»" }
            gd.junctionAtStart(e, false).takeIf { it.isNotEmpty() }?.let { gl += "svincolo in A «$it»" }
            gd.junctionAtStart(e, true).takeIf { it.isNotEmpty() }?.let { gl += "svincolo in B «$it»" }
            gd.conditional(e).takeIf { it.isNotEmpty() }?.let { gl += "a orario «$it»" }
            sb.append(" | ${gl.joinToString(", ")}")
        }
        return sb.toString()
    }

    /** I divieti di svolta nei [nodes] (coppie entrata → uscita vietate al camion o all'auto). */
    private fun printTurnRestrictions(tag: String, rg: RoutingGraph, nodes: Collection<Int>) {
        val tc = rg.turnCosts; val enc = tc.encoding; val g = rg.graph
        val it = g.createEdgeIterator()
        var n = 0
        for (node in nodes.toSortedSet()) {
            if (!tc.hasEntries(node)) continue
            val edges = ArrayList<Int>()
            it.setBaseNode(node)
            while (it.next()) edges += it.edge
            for (inE in edges) for (outE in edges) {
                if (inE == outE) continue
                val f = tc.flags(inE, node, outE)
                val hv = enc.isForbidden(f, "heavyvehicle"); val car = enc.isForbidden(f, "car-ors")
                if (!hv && !car) continue
                n++
                println("$tag divieto nel nodo $node (%.5f,%.5f): arco $inE «${refOf(rg, inE)}» «${g.name(inE)}» → arco $outE «${refOf(rg, outE)}» «${g.name(outE)}»: %s".format(
                    g.lat(node), g.lon(node), listOfNotNull(if (hv) "vietata al camion" else null, if (car) "vietata all'auto" else null).joinToString(", ")))
            }
        }
        println("$tag: $n svolte vietate in ${nodes.size} nodi")
    }

    /** Il percorso [legs] arco per arco ([full]) e per tratti dello stesso ref/nome, coi totali. */
    private fun printRoute(tag: String, rg: RoutingGraph, legs: List<RoutePath>, full: Boolean) {
        val edges = legs.flatMap { it.edges }
        println("$tag: %.2f km, %.1f min, peso %.1f, ${edges.size} archi".format(legs.sumOf { it.distance } / 1000, legs.sumOf { it.millis } / 60000.0, legs.sumOf { it.weight }))
        val sh = rg.encoding.shared
        val it = rg.graph.createEdgeIterator()
        if (full) for (pe in edges) {
            it.setEdge(pe.edge, rg.graph.nodeA(pe.edge))
            println("$tag   arco ${pe.edge} ${if (pe.reverse) "B→A" else "A→B"} ${pe.baseNode}→${pe.adjNode} (%.5f,%.5f) ref «${refOf(rg, pe.edge)}» nome «${rg.graph.name(pe.edge)}» ${it.get(sh.roadClass).osm}${if (it.get(sh.roadClassLink)) "_link" else ""} %.3f km %.1f s peso %.1f".format(
                rg.graph.lat(pe.adjNode), rg.graph.lon(pe.adjNode), pe.distance / 1000, pe.millis / 1000.0, pe.weight))
        }
        // per tratti: lo stesso ref (o nome) di seguito
        val runs = ArrayList<Pair<String, Double>>()
        for (pe in edges) {
            val key = refOf(rg, pe.edge).ifEmpty { rg.graph.name(pe.edge).ifEmpty { "-" } }
            if (runs.isNotEmpty() && runs.last().first == key) runs[runs.size - 1] = key to runs.last().second + pe.distance else runs += key to pe.distance
        }
        println("$tag tratti: " + runs.joinToString(" → ") { "${it.first} %.2f km".format(it.second / 1000) })
        val pl = PointList(); legs.forEach { l -> val p = l.points(); for (i in 0 until p.size) pl.add(p.lat(i), p.lon(i)) }
        if (pl.size > 0) println("$tag: punto più a ovest %.5f; a %.0f m da Brno-západ/Bohunice (49.1800,16.5800), a %.0f m da Brno-jih (49.1600,16.6450)".format(
            (0 until pl.size).minOf { pl.lon(it) }, nearest(pl, 49.18, 16.58).dist, nearest(pl, 49.16, 16.645).dist))
    }

    /**
     * Un punto di una strada di ref [refs] (grezzi, senza spazi, maiuscoli) entro [radius] da (lat, lon), percorribile
     * dal camion in un verso entro 80° da [wantBearing]: (lat, lon, direzione, arco).
     */
    private fun roadPoint(rg: RoutingGraph, truck: Access, refs: Set<String>, lat: Double, lon: Double, wantBearing: Double, radius: Double = 6_000.0): DoubleArray? {
        var best: DoubleArray? = null; var bestD = Double.POSITIVE_INFINITY
        for ((e, _) in edgesNear(rg, lat, lon, radius)) {
            val toks = refOf(rg, e).split(';', ',').map { it.replace(" ", "").uppercase() }
            if (toks.none { it in refs }) continue
            val pl = rg.graph.fetchWayGeometry(e, false, FetchMode.ALL)
            val nr = nearest(pl, lat, lon)
            for (rev in listOf(false, true)) {
                if (!truck.open(e, rev)) continue
                val b = (bearingOf(pl, nr.seg) + if (rev) 180.0 else 0.0) % 360
                if (angle(b, wantBearing) > 80) continue
                if (nr.dist < bestD) { bestD = nr.dist; best = doubleArrayOf(nr.lat, nr.lon, b, e.toDouble()) }
            }
        }
        return best
    }

    private fun usedMb(): Long { val r = Runtime.getRuntime(); return (r.totalMemory() - r.freeMemory()) / (1024 * 1024) }

    @Suppress("UNCHECKED_CAST")
    @Test
    fun probe() {
        val spec = System.getenv("NM_CB") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val t0 = System.currentTimeMillis()
            val g = RoutingGraph.load(File(path), true)
            println("sonda: $id aperto in ${System.currentTimeMillis() - t0} ms: ${g.graph.nodeCount} nodi, ${g.graph.edgeCount} archi, profili ${g.profileNames}")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val byId = graphs.associateBy { it.id }
        val cache = HashMap<String, List<Transfer>>()
        val transfers = { a: CountryGraph, b: CountryGraph ->
            synchronized(cache) {
                cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) {
                    val t0 = System.currentTimeMillis()
                    Transfers.find(a.graph, b.graph).also { println("sonda: passaggi ${a.id}-${b.id}: ${it.size} in ${System.currentTimeMillis() - t0} ms") }
                }
            }
        }
        // come sul tablet: 2 posti di ricerca, un thread aiutante
        val slots = SearchSlots(2)
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try {
            byId["czech-republic"]?.let { runCatching { brno(it) }.onFailure { e -> println("sonda A: errore ${e.javaClass.simpleName} ${e.message}"); e.printStackTrace(System.out) } }
            // C prima di B: la tappa in Austria con le memorie del motore fredde (come sul tablet dopo il migliore)
            if (graphs.size >= 2) runCatching { austrianStop(graphs, transfers, pool, slots) }.onFailure { e -> println("sonda C: errore ${e.javaClass.simpleName} ${e.message}"); e.printStackTrace(System.out) }
            val at = byId["austria"]; val cz = byId["czech-republic"]
            if (at != null && cz != null) runCatching { mikulov(graphs, at, cz, transfers, pool, slots) }.onFailure { e -> println("sonda B: errore ${e.javaClass.simpleName} ${e.message}"); e.printStackTrace(System.out) }
            else println("sonda B: servono i grafi austria e czech-republic")
        } finally { pool.shutdownNow() }
        println("sonda: fine, memoria ${usedMb()} MB")
    }

    // ------------------------------------------------------------------ A) svincolo di Brno

    private fun brno(cz: CountryGraph) {
        val tag = "sonda A"
        val svc = cz.service; val rg = svc.routingGraph
        val truckOpts = svc.routeOptions(request(listOf(49.13 to 16.65, 49.15 to 16.73), car = false))
        val truck = Access(rg, rg.router(truckOpts))
        val hasCar = "car-ors" in rg.profileNames
        val car = if (hasCar) Access(rg, rg.router(svc.routeOptions(request(listOf(49.13 to 16.65, 49.15 to 16.73), car = true)))) else null
        println("$tag: grafo czech-republic, profili ${rg.profileNames}, guida ${rg.guidance != null}")

        // 1) gli archi intorno ai due svincoli e i divieti di svolta nei loro nodi
        for ((name, c) in listOf("Brno-jih (D1/D2)" to (49.1600 to 16.6450), "Brno-západ/Bohunice" to (49.1800 to 16.5800))) {
            val near = edgesNear(rg, c.first, c.second, 1_500.0)
            println("$tag archi entro 1,5 km da $name ${c.first},${c.second}: ${near.size}")
            // prima le strade principali (autostrade, superstrade, primarie e rampe), poi le altre, ognuna per distanza
            val sh = rg.encoding.shared
            val it = rg.graph.createEdgeIterator()
            val main = near.sortedBy { (e, d) ->
                it.setEdge(e, rg.graph.nodeA(e))
                val cls = it.get(sh.roadClass).ordinal
                (if (cls in 1..4) 0.0 else 1e7) + d
            }
            for ((e, d) in main) println("$tag [$name] a %.0f m: ${describe(rg, e, truck, car)}".format(d))
            val nodes = near.flatMap { (e, _) -> listOf(rg.graph.nodeA(e), rg.graph.nodeB(e)) }.toSet()
            printTurnRestrictions("$tag [$name]", rg, nodes)
        }

        // 2) i punti: sulla D2/D52 a sud dello svincolo, verso nord; sulla D1 a est, verso est (nord-est)
        val south = roadPoint(rg, truck, setOf("D2", "D52", "E65", "E461"), 49.1300, 16.6500, 0.0)
        val east = roadPoint(rg, truck, setOf("D1", "E50", "E462"), 49.1500, 16.7300, 70.0)
        println("$tag punto a sud: ${south?.let { "%.5f,%.5f direzione %.0f° arco %d «%s»".format(it[0], it[1], it[2], it[3].toInt(), refOf(rg, it[3].toInt())) } ?: "nessuna D2/D52 percorribile verso nord entro 6 km, uso 49.1300,16.6500"}")
        println("$tag punto a est: ${east?.let { "%.5f,%.5f direzione %.0f° arco %d «%s»".format(it[0], it[1], it[2], it[3].toInt(), refOf(rg, it[3].toInt())) } ?: "nessuna D1 percorribile verso est entro 6 km, uso 49.1500,16.7300"}")
        val from = south?.let { it[0] to it[1] } ?: (49.1300 to 16.6500)
        val to = east?.let { it[0] to it[1] } ?: (49.1500 to 16.7300)
        val fromB = south?.get(2); val toB = east?.get(2)

        fun solve(car: Boolean, pts: List<Pair<Double, Double>>, via: List<Boolean>?, bearings: List<Double?>?): DirectionsService.Solved? =
            try { svc.solveAsHandled(request(pts, car, via, bearings), RequestContext()) } catch (e: Exception) { println("$tag: nessun percorso (${e.javaClass.simpleName}: ${e.message})"); null }

        // 3) il migliore del camion e dell'auto, arco per arco
        val bestTruck = solve(false, listOf(from, to), null, listOf(fromB, toB))
        bestTruck?.let { printRoute("$tag camion migliore", rg, it.legs, full = true); println("$tag camion migliore: opzioni ${it.options}") }
        val bestCar = if (hasCar) solve(true, listOf(from, to), null, listOf(fromB, toB)) else null
        bestCar?.let { printRoute("$tag auto migliore", rg, it.legs, full = true) }
        if (!hasCar) println("$tag: il grafo non ha il profilo dell'auto")

        // 4) core contro ricerca piena sugli stessi agganci (senza direzioni): un percorso del core più caro è un errore del core
        for ((label, solved) in listOf("camion" to bestTruck, "auto" to bestCar)) {
            val s = solved ?: continue
            val o = s.options.copy(hasBearings = false)
            val r0 = rg.router(o); val rf = rg.router(o.copy(allowCH = false, maxVisitedNodes = 50_000_000))
            val p0 = runCatching { r0.route(s.snaps[0], s.snaps[1]) }.getOrNull()
            val pf = runCatching { rf.route(s.snaps[0], s.snaps[1]) }.getOrNull()
            println("$tag $label: ${r0.algorithm} peso %s, %s peso %s; stessi archi: %s".format(p0?.weight?.let { "%.1f".format(it) } ?: "nessuno", rf.algorithm,
                pf?.weight?.let { "%.1f".format(it) } ?: "nessuno", if (p0 != null && pf != null) (p0.edges.map { it.edge } == pf.edges.map { it.edge }).toString() else "-"))
            if (p0 != null && pf != null && p0.edges.map { it.edge } != pf.edges.map { it.edge }) printRoute("$tag $label ricerca piena", rg, listOf(pf), full = false)
        }

        // 5) il camion obbligato su ogni rampa dello svincolo D1/D2 (punto di passaggio a metà rampa, nel suo verso)
        val sh = rg.encoding.shared
        val it = rg.graph.createEdgeIterator()
        val ramps = edgesNear(rg, 49.1600, 16.6450, 1_500.0).filter { (e, _) -> it.setEdge(e, rg.graph.nodeA(e)); it.get(sh.roadClassLink) }
        println("$tag rampe entro 1,5 km da Brno-jih: ${ramps.size}")
        val bestW = bestTruck?.legs?.sumOf { it.weight }
        val rows = ArrayList<Pair<Double, String>>()
        for ((e, _) in ramps) {
            val pl = rg.graph.fetchWayGeometry(e, false, FetchMode.ALL)
            for (rev in listOf(false, true)) {
                val truckOpen = truck.open(e, rev); val carOpen = car?.open(e, rev) ?: false
                if (!truckOpen && !carOpen) continue
                val (mLat, mLon, mB0) = along(pl, rg.graph.distance(e) / 2)
                val mB = (mB0 + if (rev) 180.0 else 0.0) % 360
                val dest = rg.guidance?.destination(e, rev).orEmpty()
                val head = "rampa $e ${if (rev) "B→A" else "A→B"} «${refOf(rg, e)}» «${rg.graph.name(e)}» direzione «$dest» %.0f m verso %.0f° (camion: ${truck.desc(e, rev)}; auto: ${car?.desc(e, rev) ?: "-"})".format(rg.graph.distance(e), mB)
                val pts = listOf(from, mLat to mLon, to); val via = listOf(false, true, false); val brg = listOf(fromB, mB, toB)
                val tr = if (truckOpen) runCatching { svc.solveAsHandled(request(pts, false, via, brg), RequestContext()) } else null
                val ca = if (carOpen && hasCar) runCatching { svc.solveAsHandled(request(pts, true, via, brg), RequestContext()) } else null
                val tw = tr?.getOrNull()?.legs?.sumOf { l -> l.weight }
                val sb = StringBuilder(head)
                tr?.getOrNull()?.let { s ->
                    val w = s.legs.sumOf { l -> l.weight }
                    sb.append(" | camion obbligato: %.2f km, %.1f min, peso %.1f%s, aggancio sull'arco ${s.snaps[1].edge}%s".format(s.legs.sumOf { l -> l.distance } / 1000, s.legs.sumOf { l -> l.millis } / 60000.0, w,
                        bestW?.let { b -> " (%+.1f sul migliore%s)".format(w - b, if (w < b - 1e-6) ", PIÙ LEGGERO DEL MIGLIORE" else "") } ?: "", if (s.snaps[1].edge != e) " (NON la rampa)" else ""))
                } ?: tr?.exceptionOrNull()?.let { x -> sb.append(" | camion obbligato: nessun percorso (${x.message})") }
                ca?.getOrNull()?.let { s -> sb.append(" | auto obbligata: %.2f km, %.1f min, peso %.1f".format(s.legs.sumOf { l -> l.distance } / 1000, s.legs.sumOf { l -> l.millis } / 60000.0, s.legs.sumOf { l -> l.weight })) }
                    ?: ca?.exceptionOrNull()?.let { x -> sb.append(" | auto obbligata: nessun percorso (${x.message})") }
                rows += (tw ?: Double.POSITIVE_INFINITY) to sb.toString()
                // il percorso intero dei candidati più promettenti: quelli con «Olomouc/Ostrava/Vyškov/Brno-východ» nei cartelli
                if (tr?.getOrNull() != null && Regex("Olomouc|Ostrava|Vyškov|Vyskov|Brno-v|Hrušovany|Hrusovany", RegexOption.IGNORE_CASE).containsMatchIn(dest))
                    printRoute("$tag camion obbligato su rampa $e", rg, tr.getOrThrow().legs, full = true)
            }
        }
        for ((_, line) in rows.sortedBy { it.first }) println("$tag $line")
    }

    // ------------------------------------------------------------------ B) valico di Mikulov

    private class TripResult(val duration: Double, val distance: Double, val weight: Double, val countries: List<String>)

    /** Un viaggio tra Paesi con [cb]: tempo, km, peso, Paesi, valichi, vicinanza ai punti chiave, strade principali dei passi. */
    private fun crossTrip(tag: String, cb: CrossBorderDirections, req: DirectionsRequest, rc: RequestContext): TripResult? {
        val t0 = System.currentTimeMillis()
        val doc = try { cb.handle(req, List(req.coordinates.size) { null }, rc) } catch (e: Exception) {
            println("$tag: errore ${e.javaClass.simpleName} ${e.message} in ${System.currentTimeMillis() - t0} ms"); return null
        }
        val ms = System.currentTimeMillis() - t0
        val f = (doc["features"] as List<Map<String, Any?>>).first()
        val props = f["properties"] as Map<String, Any?>
        val summary = props["summary"] as Map<*, *>
        val d = (summary["duration"] as Number).toDouble(); val l = (summary["distance"] as Number).toDouble()
        val countries = ((doc["metadata"] as? Map<*, *>)?.get("countries") as? List<*>)?.map { it.toString() }.orEmpty()
        println("$tag: %.1f min (%.2f h), %.1f km, peso %.0f, Paesi $countries, valichi %s, in $ms ms".format(d / 60, d / 3600, l / 1000, cb.lastWeight,
            cb.lastCrossings.joinToString { "%.5f,%.5f r%d".format(it.lat, it.lon, it.rank) }))
        println("$tag: ${rc.stats.line()}")
        if (rc.stats.crossings.isNotEmpty()) println("$tag valichi: ${rc.stats.crossings}")
        synchronized(rc.stats.children) { rc.stats.children.toList() }.forEach { (n, s) -> println("$tag parte $n: ${s.line()}${if (s.crossings.isNotEmpty()) " | valichi ${s.crossings}" else ""}") }
        val coords = (f["geometry"] as Map<*, *>)["coordinates"] as List<*>
        val pl = PointList()
        for (c in coords) when (c) {
            is DoubleArray -> pl.add(c[1], c[0])
            is List<*> -> pl.add((c[1] as Number).toDouble(), (c[0] as Number).toDouble())
        }
        if (pl.size > 0) println("$tag: a %.1f km da Mikulov (48.7550,16.6400), a %.1f km da Hatě (48.74228,16.06031), a %.1f km da Brno-jih, a %.1f km da Brno-západ, a %.1f km da Bratislava (48.1486,17.1077)".format(
            nearest(pl, 48.7550, 16.6400).dist / 1000, nearest(pl, 48.74228, 16.06031).dist / 1000, nearest(pl, 49.16, 16.645).dist / 1000, nearest(pl, 49.18, 16.58).dist / 1000,
            nearest(pl, 48.1486, 17.1077).dist / 1000))
        // le strade dei passi lunghi (≥ 5 km di seguito con lo stesso nome)
        val steps = (props["segments"] as? List<*>).orEmpty().flatMap { s -> ((s as Map<*, *>)["steps"] as? List<*>).orEmpty() }.map { it as Map<*, *> }
        val runs = ArrayList<Pair<String, Double>>()
        for (s in steps) {
            val n = s["name"]?.toString() ?: "-"; val dd = (s["distance"] as? Number)?.toDouble() ?: 0.0
            if (runs.isNotEmpty() && runs.last().first == n) runs[runs.size - 1] = n to runs.last().second + dd else runs += n to dd
        }
        println("$tag strade: " + runs.filter { it.second >= 5_000 }.joinToString(" → ") { "${it.first} %.0f km".format(it.second / 1000) })
        return TripResult(d, l, cb.lastWeight, countries)
    }

    /**
     * Le velocità del camion e dell'auto arco per arco lungo [legs] (solo gli archi coi ref [refs]; vuoto: tutti):
     * velocità del grafo nel verso percorso, limite OSM, tempo del percorso; poi i totali (tempo alle velocità del
     * camion e a quelle dell'auto) e gli archi dove il camion va sotto 40 km/h con un limite di almeno 70.
     */
    private fun speedsAlong(tag: String, rg: RoutingGraph, legs: List<RoutePath>, refs: Set<String>, perEdge: Boolean = true) {
        val hv = runCatching { rg.encoding.profile("heavyvehicle") }.getOrNull()
        val cp = runCatching { rg.encoding.profile("car-ors") }.getOrNull()
        if (hv == null) { println("$tag: il grafo non ha il profilo del camion"); return }
        val gd = rg.guidance
        val it = rg.graph.createEdgeIterator()
        var m = 0.0; var sTruck = 0.0; var sCar = 0.0; var sRoute = 0.0; var n = 0
        var slow = 0; var slowM = 0.0; var slowS = 0.0; var slowCarS = 0.0
        for (pe in legs.flatMap { l -> l.edges }) {
            val ref = refOf(rg, pe.edge)
            if (refs.isNotEmpty() && ref.split(';', ',').map { x -> x.replace(" ", "").uppercase() }.none { x -> x in refs }) continue
            it.setEdge(pe.edge, rg.graph.nodeA(pe.edge))
            val vT = if (pe.reverse) it.getReverse(hv.speed) else it.get(hv.speed)
            val vC = cp?.let { c -> if (pe.reverse) it.getReverse(c.speed) else it.get(c.speed) } ?: Double.NaN
            val lim = gd?.maxSpeed(pe.edge, pe.reverse) ?: 0
            val d = pe.distance
            val tT = if (vT > 0) d / (vT / 3.6) else Double.POSITIVE_INFINITY
            val tC = if (vC > 0) d / (vC / 3.6) else Double.NaN
            n++; m += d; sTruck += tT; if (!tC.isNaN()) sCar += tC; sRoute += pe.millis / 1000.0
            if (vT < 40 && lim >= 70) { slow++; slowM += d; slowS += tT; if (!tC.isNaN()) slowCarS += tC }
            if (perEdge) println("$tag   arco ${pe.edge} ${if (pe.reverse) "B→A" else "A→B"} «$ref» ${it.get(rg.encoding.shared.roadClass).osm} %.0f m (arco %.0f m): camion %.0f km/h, auto %.0f km/h, limite %d; nel percorso %.1f s%s".format(
                d, rg.graph.distance(pe.edge), vT, vC, lim, pe.millis / 1000.0, if (vT < 40 && lim >= 70) " ← camion lento" else ""))
        }
        if (n == 0) { println("$tag: nessun arco${if (refs.isEmpty()) "" else " con ref ${refs.joinToString("/")}"}"); return }
        println("$tag: $n archi, %.2f km; alle velocità del camion %.1f min (media %.0f km/h), dell'auto %.1f min (media %.0f km/h), nel percorso del camion %.1f min".format(
            m / 1000, sTruck / 60, m / sTruck * 3.6, sCar / 60, if (sCar > 0) m / sCar * 3.6 else Double.NaN, sRoute / 60))
        println("$tag: $slow archi col camion sotto 40 km/h e limite ≥ 70: %.2f km, camion %.1f min, auto %.1f min".format(slowM / 1000, slowS / 60, slowCarS / 60))
    }

    /**
     * Secondi e km del percorso per codice di priorità (0 pessimo … 4 neutro … 7 ottimo) e il peso che
     * avrebbe con ogni [PriorityModel] (tempo con i ritardi × fattore della priorità, senza le penalità
     * d'accesso): dice quale modello sceglie quale valico.
     */
    private fun byPriority(tag: String, pieces: List<Pair<RoutingGraph, RoutePath>>) {
        val s = DoubleArray(8); val m = DoubleArray(8); var weight = 0.0
        for ((rg, leg) in pieces) {
            val hv = rg.encoding.profile("heavyvehicle")
            val it = rg.graph.createEdgeIterator()
            for (pe in leg.edges) {
                it.setEdge(pe.edge, rg.graph.nodeA(pe.edge))
                val c = Math.round(it.get(hv.priority) * 7).toInt().coerceIn(0, 7)
                s[c] += pe.millis / 1000.0; m[c] += pe.distance
            }
            weight += leg.weight
        }
        val models = PriorityModel.entries
        println("$tag: " + (7 downTo 0).filter { s[it] > 0 }.joinToString(", ") { c -> "codice $c %.1f min %.1f km".format(s[c] / 60, m[c] / 1000) } +
            "; tempo %.1f min, peso del grafo %.0f; ".format(s.sum() / 60, weight) +
            models.joinToString(", ") { md -> "peso ${md.id} %.0f".format((0..7).sumOf { c -> s[c] * md.factor(c / 7.0) }) })
    }

    private fun mikulov(graphs: List<CountryGraph>, at: CountryGraph, cz: CountryGraph, transfers: (CountryGraph, CountryGraph) -> List<Transfer>,
                        pool: java.util.concurrent.ExecutorService, slots: SearchSlots) {
        val tag = "sonda B"
        val mik = 48.7550 to 16.6400; val hate = 48.74228 to 16.06031
        val all = transfers(at, cz)
        val near = all.filter { t -> DistanceCalc.haversine(t.lat, t.lon, mik.first, mik.second) <= 15_000 || DistanceCalc.haversine(t.lat, t.lon, hate.first, hate.second) <= 15_000 }
        println("$tag: ${all.size} passaggi austria/czech-republic, ${near.size} entro 15 km da Mikulov o da Hatě")
        val truckReq = request(listOf(48.2082 to 16.3738, 49.1951 to 16.6068), car = false)
        // le opzioni dei pezzi lunghi del motore: maximum_speed sui soli tempi (fastMaximumSpeed), niente direzioni
        fun opts(g: CountryGraph) = g.service.routeOptions(truckReq).copy(hasBearings = false, maximumSpeedTimeOnly = true)
        val atRouter = at.graph.router(opts(at)); val czRouter = cz.graph.router(opts(cz))
        val atAcc = Access(at.graph, atRouter); val czAcc = Access(cz.graph, czRouter)
        for (t in near) {
            val sb = StringBuilder("passaggio %.5f,%.5f r${t.rank}: a %.1f km da Mikulov, %.1f km da Hatě".format(t.lat, t.lon,
                DistanceCalc.haversine(t.lat, t.lon, mik.first, mik.second) / 1000, DistanceCalc.haversine(t.lat, t.lon, hate.first, hate.second) / 1000))
            for ((g, r, a) in listOf(Triple(at, atRouter, atAcc), Triple(cz, czRouter, czAcc))) {
                val s = runCatching { r.snap(t.lat, t.lon, CrossBorderDirections.TRANSFER_RADIUS, 0, checkNetwork = false) }
                sb.append(" | ${g.id}: ").append(s.getOrNull()?.let { x -> "arco ${x.edge} «${refOf(g.graph, x.edge)}» «${g.graph.graph.name(x.edge)}» a %.1f m, camion A→B ${a.desc(x.edge, false)}; B→A ${a.desc(x.edge, true)}".format(x.distance) }
                    ?: "non agganciato per il camion (${s.exceptionOrNull()?.message})")
            }
            println("$tag $sb")
        }

        // gli archi della strada 52/D52 (Cechia) e della A5/B7 (Austria) vicino al confine di Mikulov
        for ((g, refs, acc) in listOf(Triple(cz, setOf("52", "D52", "I/52", "E461"), czAcc), Triple(at, setOf("A5", "B7", "E461"), atAcc))) {
            val es = edgesNear(g.graph, mik.first, mik.second, 10_000.0).filter { (e, _) -> refOf(g.graph, e).split(';', ',').map { it.replace(" ", "").uppercase() }.any { it in refs } }
            println("$tag ${g.id}: ${es.size} archi con ref ${refs.joinToString("/")} entro 10 km da Mikulov")
            for ((e, d) in es) println("$tag [${g.id} vicino al confine] a %.0f m: ${describe(g.graph, e, acc, null)}".format(d))
        }

        // Wien → Brno: il migliore del motore e i due pezzi per ogni passaggio vicino (obbligati)
        val log: (String) -> Unit = { println("$tag motore: $it") }
        val cb = CrossBorderDirections(graphs, transfers, log)
        fun ctx() = RequestContext(CancelToken(), RequestContext.MAIN, pool, slots, log)
        val wien = 48.2082 to 16.3738; val brno = 49.1951 to 16.6068
        crossTrip("$tag Wien → Brno migliore", cb, truckReq, ctx())
        class Forced(val t: Transfer, val atLeg: RoutePath, val czLeg: RoutePath) { val weight get() = atLeg.weight + czLeg.weight }
        val forced = ArrayList<Forced>()
        // due pezzi come quelli del motore ([CrossBorderDirections]: la richiesta dell'app coi punti del pezzo, il passaggio
        // agganciato esatto con raggio TRANSFER_RADIUS): Wien → passaggio nel grafo austriaco, passaggio → Brno in quello ceco.
        // Prima un percorso del router fra due agganci a mano dava sempre «nessun percorso»
        fun piece(g: CountryGraph, pts: List<Pair<Double, Double>>, transferFirst: Boolean): RoutePath {
            val r = g.service.maximumSnappingRadius; val tr = CrossBorderDirections.TRANSFER_RADIUS
            val req = truckReq.withPoints(pts, if (transferFirst) doubleArrayOf(tr, r) else doubleArrayOf(r, tr), if (transferFirst) booleanArrayOf(true, false) else booleanArrayOf(false, true))
            return g.service.solveAsHandled(req, RequestContext()).legs.single()
        }
        var noRoute = 0
        for (t in near) {
            val a = runCatching { piece(at, listOf(wien, t.lat to t.lon), transferFirst = false) }
            val c = if (a.isFailure) null else runCatching { piece(cz, listOf(t.lat to t.lon, brno), transferFirst = true) }
            if (a.isFailure || c == null || c.isFailure) {
                noRoute++
                println("$tag Wien → %.5f,%.5f → Brno: nessun percorso (${if (a.isFailure) "austria" else "czech-republic"}: ${(a.exceptionOrNull() ?: c?.exceptionOrNull())?.message})".format(t.lat, t.lon))
                continue
            }
            forced += Forced(t, a.getOrThrow(), c.getOrThrow())
        }
        println("$tag Wien → passaggio → Brno: ${forced.size} passaggi con percorso, $noRoute senza")
        forced.sortBy { it.weight }
        val fBest = forced.firstOrNull()?.weight
        for (f in forced) println("$tag Wien → %.5f,%.5f r${f.t.rank} → Brno: %.1f km, %.1f min, peso %.0f (austria %.0f + cechia %.0f)%s".format(f.t.lat, f.t.lon,
            (f.atLeg.distance + f.czLeg.distance) / 1000, (f.atLeg.millis + f.czLeg.millis) / 60000.0, f.weight, f.atLeg.weight, f.czLeg.weight,
            fBest?.let { b -> ", fattore %.3f sul più leggero".format(f.weight / b) } ?: ""))
        val fMik = forced.minByOrNull { DistanceCalc.haversine(it.t.lat, it.t.lon, mik.first, mik.second) }
        val fHate = forced.minByOrNull { DistanceCalc.haversine(it.t.lat, it.t.lon, hate.first, hate.second) }
        fMik?.let { printRoute("$tag Wien → Mikulov (pezzo austriaco)", at.graph, listOf(it.atLeg), full = false); printRoute("$tag Mikulov → Brno (pezzo ceco)", cz.graph, listOf(it.czLeg), full = false) }
        fHate?.let { printRoute("$tag Wien → Hatě (pezzo austriaco)", at.graph, listOf(it.atLeg), full = false); printRoute("$tag Hatě → Brno (pezzo ceco)", cz.graph, listOf(it.czLeg), full = false) }
        // Mikulov contro Hatě: i due pezzi obbligati e il migliore
        if (fMik != null && fHate != null) println("$tag confronto Mikulov/Hatě: Mikulov %.1f km, %.1f min, peso %.0f; Hatě %.1f km, %.1f min, peso %.0f; Mikulov/Hatě tempo %.3f, peso %.3f".format(
            (fMik.atLeg.distance + fMik.czLeg.distance) / 1000, (fMik.atLeg.millis + fMik.czLeg.millis) / 60000.0, fMik.weight,
            (fHate.atLeg.distance + fHate.czLeg.distance) / 1000, (fHate.atLeg.millis + fHate.czLeg.millis) / 60000.0, fHate.weight,
            (fMik.atLeg.millis + fMik.czLeg.millis).toDouble() / (fHate.atLeg.millis + fHate.czLeg.millis), fMik.weight / fHate.weight))
        // il tempo per codice di priorità nei due percorsi obbligati e il peso che avrebbero con ciascun modello di priorità
        for ((name, f) in listOf("Mikulov" to fMik, "Hatě" to fHate)) if (f != null) byPriority("$tag priorità per $name", listOf(at.graph to f.atLeg, cz.graph to f.czLeg))
        // le velocità del camion e dell'auto lungo la 52 (Cechia) e la A5/B7 (Austria) nei pezzi per Mikulov: il modello
        // d'accelerazione di ORS (velocità sulla linea d'aria della way OSM intera) sulle way corte della 52
        fMik?.let {
            speedsAlong("$tag velocità lungo la 52 (Mikulov → Brno)", cz.graph, listOf(it.czLeg), setOf("52", "I/52", "D52", "E461"))
            speedsAlong("$tag velocità lungo A5/B7 (Wien → Mikulov)", at.graph, listOf(it.atLeg), setOf("A5", "B7", "E461"))
            speedsAlong("$tag velocità nel pezzo ceco per Mikulov (tutto)", cz.graph, listOf(it.czLeg), emptySet(), perEdge = false)
        }
        fHate?.let { speedsAlong("$tag velocità nel pezzo ceco per Hatě (tutto)", cz.graph, listOf(it.czLeg), emptySet(), perEdge = false) }

        // il viaggio intero Canneto → Bielsko-Biała: migliore, obbligato per Mikulov e per Hatě (punti di passaggio 2 km prima e
        // dopo il valico sulle strade dei pezzi qui sopra, e sulla D1 a est di Brno)
        val canneto = 45.1502 to 10.3792; val bielsko = 49.8224 to 19.0444
        val best = crossTrip("$tag Canneto → Bielsko-Biała migliore", cb, request(listOf(canneto, bielsko), car = false), ctx())
        val d1 = roadPoint(cz.graph, czAcc, setOf("D1", "E50", "E462"), 49.1500, 16.7300, 70.0)
        println("$tag punto sulla D1 a est di Brno: ${d1?.let { "%.5f,%.5f direzione %.0f°".format(it[0], it[1], it[2]) } ?: "nessuno"}")
        for ((name, f) in listOf("Mikulov" to fMik, "Hatě" to fHate)) {
            if (f == null) { println("$tag $name: nessun passaggio con percorso"); continue }
            val (aLat, aLon, aB) = along(f.atLeg.points(), -2_000.0)
            val (cLat, cLon, cB) = along(f.czLeg.points(), 2_000.0)
            for (withD1 in listOf(false, true)) {
                if (withD1 && d1 == null) continue
                val pts = mutableListOf(canneto, aLat to aLon, cLat to cLon); val brg = mutableListOf<Double?>(null, aB, cB)
                if (withD1) { pts += d1!![0] to d1[1]; brg += d1[2] }
                pts += bielsko; brg += null
                val via = pts.indices.map { it in 1 until pts.size - 1 }
                val label = "$tag Canneto → Bielsko-Biała per $name${if (withD1) " e la D1 a est di Brno" else ""} (passaggi %.5f,%.5f e %.5f,%.5f)".format(aLat, aLon, cLat, cLon)
                val r = crossTrip(label, cb, request(pts, false, via, brg), ctx()) ?: continue
                if (best != null) println("$label: fattore tempo %.3f, km %.3f, peso %.3f sul migliore".format(r.duration / best.duration, r.distance / best.distance, r.weight / best.weight))
            }
        }
    }

    // ------------------------------------------------------------------ C) tappa in Austria

    private fun austrianStop(graphs: List<CountryGraph>, transfers: (CountryGraph, CountryGraph) -> List<Transfer>, pool: java.util.concurrent.ExecutorService, slots: SearchSlots) {
        val tag = "sonda C"
        val log: (String) -> Unit = { println("$tag motore: $it") }
        val cb = CrossBorderDirections(graphs, transfers, log)
        fun ctx() = RequestContext(CancelToken(), RequestContext.MAIN, pool, slots, log)
        // i passaggi di tutti i confini prima dei viaggi (l'app li ha già scritti): il tempo non comprende la loro ricerca
        for (i in graphs.indices) for (j in i + 1 until graphs.size) if (graphs[i].graph.graph.bounds.intersects(graphs[j].graph.graph.bounds)) transfers(graphs[i], graphs[j])
        val canneto = 45.1502 to 10.3792; val bielsko = 49.8224 to 19.0444
        val trips = listOf(
            "Canneto → Bielsko-Biała (migliore, come prima della tappa)" to listOf(canneto, bielsko),
            "Canneto → Drasenhofen (passa di qui) → Bielsko-Biała" to listOf(canneto, 48.7505 to 16.6380, bielsko),
            "Canneto → Wien (passa di qui) → Bielsko-Biała" to listOf(canneto, 48.2082 to 16.3738, bielsko),
        )
        for ((name, pts) in trips) {
            println("$tag == $name")
            val via = pts.indices.map { it in 1 until pts.size - 1 }
            val m0 = usedMb()
            val t0 = System.currentTimeMillis()
            val r = crossTrip("$tag $name", cb, request(pts, false, via, null), ctx())
            println("$tag $name: tempo totale ${System.currentTimeMillis() - t0} ms, memoria $m0 → ${usedMb()} MB${if (r == null) ", nessun percorso" else ""}")
        }
        // la stessa tappa di nuovo: quanto resta dalle memorie del motore
        val t1 = System.currentTimeMillis()
        crossTrip("$tag di nuovo Canneto → Drasenhofen → Bielsko-Biała", cb, request(trips[1].second, false, listOf(false, true, false), null), ctx())
        println("$tag di nuovo: ${System.currentTimeMillis() - t1} ms")
    }
}
