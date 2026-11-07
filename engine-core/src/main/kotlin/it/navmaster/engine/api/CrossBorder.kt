package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.ch.CoreManyToOne
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.instructions.InstructionType
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import java.io.File

/** Il grafo di un Paese, per i viaggi tra Paesi. */
class CountryGraph(val id: String, val service: DirectionsService) {
    val graph: RoutingGraph get() = service.routingGraph
}

/**
 * Un punto in cui si passa da un grafo all'altro: lo stesso punto della stessa strada nei due grafi
 * (le strade che attraversano il confine sono in entrambi gli estratti OSM dei due Paesi). [rank]:
 * l'importanza della strada (1 autostrada … 7 strada di servizio).
 */
class Transfer(val lat: Double, val lon: Double, val rank: Int)

/** I punti di passaggio fra due grafi, trovati una volta e salvati. */
object Transfers {
    /** Quanto è durata l'ultima ricerca dei passaggi, per il registro. */
    @Volatile var timing: String = ""

    /**
     * I punti di passaggio fra [a] e [b]: i nodi senza uscita di un grafo (dove il suo estratto
     * finisce, oltre il confine) che nell'altro sono un punto di una strada che prosegue.
     */
    fun find(a: RoutingGraph, b: RoutingGraph): List<Transfer> {
        val ba = a.graph.bounds; val bb = b.graph.bounds
        if (!ba.isValid || !bb.isValid || !ba.intersects(bb)) return emptyList()
        val overlap = BBox.empty().apply {
            include(maxOf(ba.minLat, bb.minLat), maxOf(ba.minLon, bb.minLon))
            include(minOf(ba.maxLat, bb.maxLat), minOf(ba.maxLon, bb.maxLon))
        }
        val t0 = System.currentTimeMillis()
        val degA = degrees(a); val degB = degrees(b)
        val t1 = System.currentTimeMillis()
        val out = ArrayList<Transfer>()
        val qa = scan(a, degA, b, degB, overlap, out)
        val t2 = System.currentTimeMillis()
        val qb = scan(b, degB, a, degA, overlap, out)
        timing = "gradi ${t1 - t0} ms, A→B ${t2 - t1} ms ($qa ricerche), B→A ${System.currentTimeMillis() - t2} ms ($qb ricerche)"
        // un punto per luogo: i due capi di una stessa strada di confine restano (possono distare km)
        val kept = ArrayList<Transfer>()
        for (t in out.sortedBy { it.rank }) if (kept.none { DistanceCalc.haversine(it.lat, it.lon, t.lat, t.lon) < 25.0 }) kept += t
        return kept
    }

    /** Il numero di archi di ogni nodo (fino a 3). */
    private fun degrees(g: RoutingGraph): ByteArray {
        val bg = g.graph
        val deg = ByteArray(bg.nodeCount)
        for (e in 0 until bg.edgeCount) {
            val x = bg.nodeA(e); val y = bg.nodeB(e)
            if (x == y) continue
            if (deg[x] < 3) deg[x]++
            if (deg[y] < 3) deg[y]++
        }
        return deg
    }

    private fun scan(src: RoutingGraph, degSrc: ByteArray, dst: RoutingGraph, degDst: ByteArray, overlap: BBox, out: MutableList<Transfer>): Int {
        var queries = 0
        val g = src.graph
        val it = g.createEdgeIterator()
        val rc = src.encoding.shared.roadClass
        for (n in 0 until g.nodeCount) {
            if (degSrc[n].toInt() != 1) continue
            val lat = g.lat(n); val lon = g.lon(n)
            if (!overlap.contains(lat, lon)) continue
            // solo dove l'altro grafo ha strade (la sua griglia è vuota fuori dal suo estratto)
            if (!dst.index.hasEdgesNear(lat, lon)) continue
            queries++
            val s = dst.index.findClosest(lat, lon, maxRings = 1)
            if (!s.isValid || s.distance > 1.0) continue
            // senza uscita anche nell'altro: una strada chiusa nella zona coperta da entrambi, non un passaggio
            if (s.position == SnapPosition.TOWER && degDst[s.closestNode] <= 1) continue
            it.setBaseNode(n)
            if (!it.next()) continue
            out += Transfer(lat, lon, rank(it.get(rc)))
        }
        return queries
    }

    /**
     * I valichi raggruppati per luogo (entro [PLACE] m dal punto della strada più importante), i
     * luoghi in ordine di [detour], al più [places]; in ogni luogo i punti in ordine d'importanza
     * della strada. Un luogo è un valico solo: al Brennero i capi delle due carreggiate
     * dell'autostrada e la statale, a Resia i due capi della stessa strada. Quale punto del luogo
     * serve lo decide il viaggio: un senso unico o un'autostrada vietata al mezzo da un lato non si
     * raggiungono, allora vale il punto successivo.
     */
    fun places(list: List<Transfer>, detour: (Transfer) -> Double, places: Int): List<List<Transfer>> {
        val out = ArrayList<MutableList<Transfer>>()
        for (t in list.sortedWith(compareBy<Transfer>({ it.rank }, { detour(it) }))) {
            val place = out.firstOrNull { DistanceCalc.haversine(it[0].lat, it[0].lon, t.lat, t.lon) < PLACE }
            if (place == null) out += mutableListOf(t) else place += t
        }
        return out.sortedBy { detour(it[0]) }.take(places)
    }

    private const val PLACE = 5_000.0

    fun rank(c: RoadClass): Int = when (c) {
        RoadClass.MOTORWAY -> 1
        RoadClass.TRUNK -> 2
        RoadClass.PRIMARY -> 3
        RoadClass.SECONDARY -> 4
        RoadClass.TERTIARY -> 5
        RoadClass.UNCLASSIFIED, RoadClass.RESIDENTIAL, RoadClass.LIVING_STREET -> 6
        else -> 7
    }

    fun write(f: File, list: List<Transfer>) {
        f.parentFile?.mkdirs()
        val tmp = File(f.path + ".tmp")
        tmp.writeText(buildString { append("NMTRANSFERS 1\n"); for (t in list) append(t.lat).append(',').append(t.lon).append(',').append(t.rank).append('\n') })
        tmp.renameTo(f)
    }

    fun read(f: File): List<Transfer>? {
        if (!f.exists()) return null
        val lines = f.readLines()
        if (lines.firstOrNull() != "NMTRANSFERS 1") return null
        return lines.drop(1).filter { it.isNotBlank() }.mapNotNull { l ->
            val p = l.split(',')
            if (p.size != 3) null else Transfer(p[0].toDouble(), p[1].toDouble(), p[2].toInt())
        }
    }
}

/**
 * Viaggi tra Paesi con un grafo per Paese: ogni tratto che passa da un grafo all'altro è calcolato
 * nei grafi dei Paesi attraversati e cucito ai punti di passaggio del confine ([Transfers]). Fra
 * tutti i passaggi candidati di ciascun confine si sceglie la combinazione col peso totale minimo
 * (lo stesso peso di openrouteservice: tempo, distanza o "recommended" con le restrizioni del mezzo),
 * passando anche per Paesi intermedi (fino a due). La risposta è quella di openrouteservice
 * (GeoJSON) come per un percorso in un grafo solo: le istruzioni proseguono al confine senza un
 * arrivo intermedio, gli extra e i fatti sulle strade sono uniti.
 */
class CrossBorderDirections(
    private val graphs: List<CountryGraph>,
    /** I passaggi fra due grafi (calcolati una volta e conservati da chi chiama). */
    private val transfers: (CountryGraph, CountryGraph) -> List<Transfer>,
    private val log: ((String) -> Unit)? = null,
    /** Fra due Paesi il valico coi limiti dei landmark e una ricerca sola dal lato vicino ([chooseTogether]); false: uno alla volta (prove). */
    private val oneSearch: Boolean = true,
) {
    private class LL(val lat: Double, val lon: Double)

    /** Il peso del viaggio scelto all'ultimo confine e i suoi valichi (diagnostica, prove). */
    internal var lastWeight = Double.NaN; private set
    internal var lastCrossings: List<Transfer> = emptyList(); private set
    /** I pezzi dell'ultimo viaggio tra Paesi: il grafo e la richiesta di ciascuno (prove). */
    internal var lastPieces: List<Pair<CountryGraph, DirectionsRequest>> = emptyList(); private set

    /** Un pezzo del viaggio dentro un grafo: i suoi punti (le tappe e i passaggi di confine). */
    private class Piece(val graph: CountryGraph, val points: MutableList<Pair<Double, Double>>, val radiuses: MutableList<Double>,
                        val exact: MutableList<Boolean> = MutableList(points.size) { false },
                        val bearings: MutableList<Double?> = MutableList(points.size) { null })

    /** Lo stato di una richiesta: router e agganci per grafo, pesi già calcolati. */
    private inner class Ctx(val request: DirectionsRequest) {
        private val routers = HashMap<String, Router>()
        private val snaps = HashMap<String, Snap?>()
        /** The weights found, with the limit they were looked for under (an infinite one below a limit may exist above it). */
        private val weights = HashMap<String, Pair<Double, Double>>()
        var queries = 0

        // the choice of the crossings without the bearings (they only touch the ends): the quick search
        fun router(g: CountryGraph): Router = routers.getOrPut(g.id) {
            val o = g.service.routeOptions(request).copy(hasBearings = false)
            // a trip across a border is a long one: a limited speed on the times only (the quick search)
            g.graph.router(if (!g.graph.coreWide && o.maximumSpeed.isFinite() && o.maximumSpeed >= DirectionsRequest.MAXIMUM_SPEED_LOWER_BOUND) o.copy(maximumSpeedTimeOnly = true) else o)
        }

        /**
         * L'aggancio di un punto come lo fa il pezzo finale ([exact]: un passaggio di confine, senza il
         * controllo delle reti isolate; [heading]: ORS `bearings`).
         */
        fun snap(g: CountryGraph, p: LL, radius: Double, exact: Boolean = radius == TRANSFER_RADIUS, heading: Double = Double.NaN): Snap? =
            snaps.getOrPut("${g.id}:${p.lat}:${p.lon}:$radius:$exact:$heading") {
                try { router(g).snap(p.lat, p.lon, radius, 0, checkNetwork = !exact, heading = heading) } catch (e: PointNotFoundException) { null }
            }

        /**
         * Il peso di un pezzo come lo calcola il percorso finale ([DirectionsService.routes] con tutte le
         * impostazioni della richiesta); +∞ quando non c'è percorso.
         */
        fun pieceWeight(g: CountryGraph, piece: DirectionsRequest): Double {
            queries++
            val t0 = System.currentTimeMillis()
            val w = try { g.service.routes(piece).single().sumOf { it.weight } } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    pezzo ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} in $ms ms")
            return w
        }

        /** The weight from [a] to [b] in [g], or +∞ when there is no path lighter than [limit]. */
        fun weight(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, limit: Double = Double.POSITIVE_INFINITY): Double {
            val key = "${g.id}:${a.lat}:${a.lon}>${b.lat}:${b.lon}"
            weights[key]?.let { (w, l) -> if (w.isFinite() || l >= limit) return if (w < limit) w else Double.POSITIVE_INFINITY }
            val sa = snap(g, a, ra) ?: return Double.POSITIVE_INFINITY
            val sb = snap(g, b, rb) ?: return Double.POSITIVE_INFINITY
            queries++
            val t0 = System.currentTimeMillis()
            // for choosing: the quick search's path also when it uses a road to avoid (the pieces are then exact)
            val w = try { router(g).route(sa, sb, maxWeight = limit, exact = false).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    ricerca ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} (limite ${if (limit.isFinite()) "%.0f".format(limit) else "∞"}) in $ms ms: ${router(g).algorithm} ${router(g).quickStats()}")
            weights[key] = w to limit
            return w
        }
    }

    /**
     * La risposta GeoJSON di openrouteservice per [request]; [pointGraph]: il Paese (id del grafo) di
     * ogni punto quando è noto, altrimenti il grafo che lo aggancia più vicino.
     */
    fun handle(request: DirectionsRequest, pointGraph: List<String?>): Map<String, Any?> {
        val started = System.currentTimeMillis()
        val ctx = Ctx(request)
        val pts = request.coordinates.map { LL(it.first, it.second) }
        val radius = { i: Int -> request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius }
        val of = pts.indices.map { i ->
            val wanted = pointGraph.getOrNull(i)?.let { id -> graphs.firstOrNull { it.id == id } }
            val g = wanted?.takeIf { ctx.snap(it, pts[i], radius(i)) != null } ?: graphs
                .filter { it.graph.graph.bounds.contains(pts[i].lat, pts[i].lon) }
                .mapNotNull { g -> ctx.snap(g, pts[i], radius(i))?.let { s -> g to s.distance } }
                .minByOrNull { it.second }?.first
            g ?: throw ApiException(404, RoutingErrorCodes.POINT_NOT_FOUND,
                "Could not find routable point within a radius of ${radius(i).toLong()} meters of specified coordinate $i: %.7f %.7f.".format(pts[i].lon, pts[i].lat))
        }
        log?.invoke("punti nei grafi: ${of.joinToString { it.id }}")
        if (of.all { it === of[0] }) return of[0].service.handle(request, ResponseFormat.GEOJSON)

        // i pezzi del viaggio: le tappe nello stesso grafo restano insieme, ai confini il passaggio migliore
        val pieces = ArrayList<Piece>()
        var cur = Piece(of[0], mutableListOf(request.coordinates[0]), mutableListOf(radius(0)), bearings = mutableListOf(request.bearings?.getOrNull(0)))
        // i blocchi di punti consecutivi nello stesso grafo; con due confini i due valichi si scelgono insieme
        val blocks = ArrayList<Pair<CountryGraph, IntRange>>()
        run { var s = 0; for (i in 1..pts.size) if (i == pts.size || of[i] !== of[s]) { blocks += of[s] to (s until i); s = i } }
        var twoDone = false
        if (blocks.size == 3 && oneSearch) {
            val (g0, r0) = blocks[0]; val (g1, r1) = blocks[1]; val (g2, r2) = blocks[2]
            // solo coi due confini diretti e senza sequenze alternative per un terzo Paese (quelle le pesa chain)
            fun direct(x: CountryGraph, y: CountryGraph, i: Int, j: Int) = countryPaths(x, y, pts[i], pts[j]).let { it.isNotEmpty() && it.all { pa -> pa.size == 2 } }
            if (direct(g0, g1, r0.last, r1.first) && direct(g1, g2, r1.last, r2.first)) {
                for (i in r0.drop(1)) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i) }
                val two = chooseTwo(ctx, cur, r1, g1, r2, g2, radius, Double.POSITIVE_INFINITY)
                if (two != null) {
                    if (two.first == null) throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${r0.last} (%.7f %.7f) and ${r1.first} (%.7f %.7f).".format(
                        pts[r0.last].lon, pts[r0.last].lat, pts[r1.first].lon, pts[r1.first].lat))
                    val (x, y, w) = two
                    lastWeight = w; lastCrossings = listOf(x!!, y!!)
                    cur.points += x.lat to x.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null
                    pieces += cur
                    cur = Piece(g1, mutableListOf(x.lat to x.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null))
                    for (i in r1) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i) }
                    cur.points += y.lat to y.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null
                    pieces += cur
                    cur = Piece(g2, mutableListOf(y.lat to y.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null))
                    for (i in r2) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i) }
                    twoDone = true
                }
            }
        }
        if (twoDone) { /* pezzi già fatti */ } else for (i in 1 until pts.size) {
            if (of[i] !== cur.graph) {
                // i punti dopo il confine nello stesso grafo (il pezzo che comincia al valico)
                var end = i
                while (end + 1 < pts.size && of[end + 1] === of[i]) end++
                val chain = chain(ctx, cur, i..end, radius, of[i])
                    ?: throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${i - 1} (%.7f %.7f) and $i (%.7f %.7f).".format(
                        pts[i - 1].lon, pts[i - 1].lat, pts[i].lon, pts[i].lat))
                for ((t, next) in chain) {
                    cur.points += t.lat to t.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null
                    pieces += cur
                    cur = Piece(next, mutableListOf(t.lat to t.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null))
                }
            }
            cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i)
        }
        pieces += cur
        val pieceRequests = pieces.map { p -> p.graph to request.withPoints(p.points, p.radiuses.toDoubleArray(), p.exact.toBooleanArray(), p.bearings) }
        lastPieces = pieceRequests
        val docs = pieceRequests.map { (g, r0) ->
            val r = g.service.handle(r0, ResponseFormat.GEOJSON)
            @Suppress("UNCHECKED_CAST")
            (r["features"] as List<Map<String, Any?>>).first()
        }
        log?.invoke("viaggio tra Paesi: ${pieces.joinToString(" → ") { it.graph.id }}, ${ctx.queries} ricerche, ${System.currentTimeMillis() - started} ms")
        val feature = merge(docs, request)
        @Suppress("UNCHECKED_CAST")
        val bbox = feature["bbox"] as DoubleArray
        val metadata = LinkedHashMap<String, Any?>()
        metadata["attribution"] = pieces[0].graph.service.attribution
        metadata["service"] = "routing"
        metadata["timestamp"] = System.currentTimeMillis()
        metadata["query"] = LinkedHashMap<String, Any?>(request.echo).apply { put("profile", request.profile); put("format", "geojson") }
        metadata["engine"] = pieces[0].graph.service.engineInfo()
        metadata["countries"] = pieces.map { it.graph.id }
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bbox, "features" to listOf(feature), "metadata" to metadata)
    }

    // ------------------------------------------------------------------------------- the borders

    /** Le sequenze di grafi da [a] a [b] (al più due intermedi) che stanno vicino alla linea p → q. */
    private fun countryPaths(a: CountryGraph, b: CountryGraph, p: LL, q: LL): List<List<CountryGraph>> {
        val crow = DistanceCalc.haversine(p.lat, p.lon, q.lat, q.lon)
        val reach = crow * 0.35 + 60_000.0
        fun near(g: CountryGraph) = distanceToBox(p, q, g.graph.graph.bounds) <= reach
        fun linked(x: CountryGraph, y: CountryGraph) = x.graph.graph.bounds.intersects(y.graph.graph.bounds) && transfers(x, y).isNotEmpty()
        val out = ArrayList<List<CountryGraph>>()
        fun walk(path: List<CountryGraph>) {
            val last = path.last()
            if (last === b) { out += path; return }
            if (path.size >= 4) return
            for (g in graphs) {
                if (g in path) continue
                if (g !== b && !near(g)) continue
                if (!last.graph.graph.bounds.intersects(g.graph.graph.bounds)) continue
                if (!linked(last, g)) continue
                walk(path + g)
            }
        }
        walk(listOf(a))
        // prima le sequenze più corte
        return out.sortedBy { it.size }.take(6)
    }

    /** Distanza (m, approssimata) fra il segmento p → q e un rettangolo. */
    private fun distanceToBox(p: LL, q: LL, b: BBox): Double {
        var best = Double.MAX_VALUE
        for (i in 0..40) {
            val t = i / 40.0
            val la = p.lat + (q.lat - p.lat) * t; val lo = p.lon + (q.lon - p.lon) * t
            val cla = la.coerceIn(b.minLat, b.maxLat); val clo = lo.coerceIn(b.minLon, b.maxLon)
            best = minOf(best, DistanceCalc.haversine(la, lo, cla, clo))
        }
        return best
    }

    /** I passaggi da provare su un confine: i migliori per deviazione dalla linea p → q, prima le strade principali. */
    private fun candidates(list: List<Transfer>, p: LL, q: LL, transit: Boolean): List<List<Transfer>> {
        fun detour(t: Transfer) = DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon) + DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)
        val major = Transfers.places(list.filter { it.rank <= 3 }, ::detour, if (transit) 5 else 6)
        // le strade minori solo vicino alla partenza o all'arrivo (un viaggio corto oltre il confine)
        val minor = list.filter { it.rank > 3 }.filter { t ->
            minOf(DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon), DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)) < 40_000.0
        }.let { Transfers.places(it, ::detour, if (transit) 2 else 4) }
        return major + minor
    }

    /**
     * I passaggi di confine del percorso migliore da p (grafo [ga]) a q (grafo [gb]), con il grafo
     * in cui si entra a ciascuno; null quando nessuna sequenza di Paesi ha un percorso.
     */
    private fun chain(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, together: Boolean = oneSearch): List<Pair<Transfer, CountryGraph>>? {
        val ga = prefix.graph
        val p = prefix.points.last().let { LL(it.first, it.second) }; val rp = prefix.radiuses.last()
        val q = ctx.request.coordinates[suffix.first].let { LL(it.first, it.second) }; val rq = radius(suffix.first)
        if (together && compareOneByOne && log != null) {
            // diagnostica: la scelta di un valico alla volta, con ricerche sue, per confronto
            val t0 = System.currentTimeMillis()
            val c2 = Ctx(ctx.request)
            val other = chain(c2, prefix, suffix, radius, gb, together = false)
            log.invoke("  confronto, un valico alla volta: ${other?.joinToString { "%.5f,%.5f".format(it.first.lat, it.first.lon) } ?: "nessuno"} peso ${"%.0f".format(lastWeight)}, ${c2.queries} ricerche, ${System.currentTimeMillis() - t0} ms")
        }
        var best = Double.POSITIVE_INFINITY
        var bestChain: List<Pair<Transfer, CountryGraph>>? = null
        val paths = countryPaths(ga, gb, p, q)
        log?.invoke("sequenze ${ga.id} → ${gb.id}: ${paths.joinToString(" | ") { pa -> pa.joinToString("-") { it.id } }}")
        for (path in paths) {
            // programmazione dinamica sui confini: il peso migliore fino a ogni passaggio
            var layer: List<Triple<Transfer, Double, List<Transfer>>> = emptyList()
            var ok = true
            for (k in 1 until path.size) {
                val from = path[k - 1]; val to = path[k]
                val places = candidates(transfers(from, to), p, q, path.size > 2)
                    // il passaggio deve essere percorribile dal mezzo in entrambi i grafi
                    .map { pl -> pl.filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } } }
                    .filter { it.isNotEmpty() }
                // due Paesi: tutti i passaggi (anche uno alla volta: senza core la ricerca è la CH, rapida); in transito i candidati
                val cands = if (path.size == 2) transfers(from, to).filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } }
                    else places.flatten()
                log?.invoke("  ${path.joinToString("-") { it.id }} confine ${from.id}/${to.id}: ${cands.size} candidati${if (path.size > 2) " in ${places.size} luoghi" else ""}")
                if (cands.isEmpty()) { ok = false; break }
                // transito per un Paese: i due valichi insieme, fra tutti i passaggi dei due confini
                if (path.size == 3 && together && k == 1) {
                    val two = chooseTwo(ctx, prefix, null, path[1], suffix, path[2], radius, best)
                    if (two != null) {
                        if (two.first != null && two.third < best) { best = two.third; bestChain = listOf(two.first!! to path[1], two.second!! to path[2]) }
                        ok = false; break
                    }
                }
                // due Paesi: fra tutti i passaggi del confine, pesati come i pezzi finali (sotto, uno alla volta quando non si può)
                if (path.size == 2 && together) {
                    val c = chooseExact(ctx, prefix, suffix, radius, to, cands, best)
                    if (c != null) {
                        if (c.weight < best && c.transfer != null) { best = c.weight; bestChain = listOf(c.transfer to to) }
                        ok = false; break
                    }
                }
                val next = ArrayList<Triple<Transfer, Double, List<Transfer>>>()
                for (t in cands) {
                    val tl = LL(t.lat, t.lon)
                    var w = Double.POSITIVE_INFINITY; var via: List<Transfer> = emptyList()
                    if (k == 1) {
                        // each search only as far as the best whole trip found so far: it cannot win beyond
                        w = ctx.weight(from, p, rp, tl, TRANSFER_RADIUS, best); via = listOf(t)
                        // two countries: the whole trip through this crossing at once, so that the
                        // next crossings are looked for under its weight
                        if (path.size == 2 && w.isFinite()) {
                            // the weights of the search are a lower bound: the whole trip weighed as the final pieces
                            val total = w + ctx.weight(path[1], tl, TRANSFER_RADIUS, q, rq, best - w)
                            if (total < best) {
                                val exact = exactChain(ctx, prefix, suffix, radius, path, listOf(t))
                                if (exact < best) { best = exact; bestChain = listOf(t to path[1]) }
                            }
                            continue
                        }
                    } else for ((pt, pw, pc) in layer) {
                        if (pw >= best || pw >= w) continue
                        val x = pw + ctx.weight(from, LL(pt.lat, pt.lon), TRANSFER_RADIUS, tl, TRANSFER_RADIUS, minOf(best, w) - pw)
                        if (x < w) { w = x; via = pc + t }
                    }
                    if (w.isFinite() && w < best) next += Triple(t, w, via)
                }
                if (path.size == 2) { ok = false; break }
                layer = next.sortedBy { it.second }.take(if (k == path.size - 1) next.size else 8)
                log?.invoke("    pesi ${from.id}/${to.id}: ${layer.joinToString { "%.0f".format(it.second) }}")
                if (layer.isEmpty()) { ok = false; break }
            }
            if (!ok) continue
            val last = path.last()
            for ((t, w, via) in layer) {
                if (w >= best) continue
                val total = w + ctx.weight(last, LL(t.lat, t.lon), TRANSFER_RADIUS, q, rq, best - w)
                if (total >= best) continue
                // come sopra: il viaggio pesato come i pezzi finali
                val exact = exactChain(ctx, prefix, suffix, radius, path, via)
                if (exact < best) { best = exact; bestChain = via.mapIndexed { i, tr -> tr to path[i + 1] } }
            }
        }
        lastWeight = best; lastCrossings = bestChain?.map { it.first }.orEmpty()
        return bestChain
    }

    /**
     * Il peso del viaggio attraverso i passaggi [via] lungo la sequenza di Paesi [path], come i pezzi
     * finali: il pezzo prima del primo confine, quelli nei Paesi di transito, quello dopo l'ultimo.
     */
    private fun exactChain(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, path: List<CountryGraph>, via: List<Transfer>): Double {
        val request = ctx.request
        var w = ctx.pieceWeight(path[0], request.withPoints(prefix.points + (via[0].lat to via[0].lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(),
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null))
        for (k in 1 until via.size) {
            if (w.isInfinite()) return w
            w += ctx.pieceWeight(path[k], request.withPoints(listOf(via[k - 1].lat to via[k - 1].lon, via[k].lat to via[k].lon),
                doubleArrayOf(TRANSFER_RADIUS, TRANSFER_RADIUS), booleanArrayOf(true, true), null))
        }
        if (w.isInfinite()) return w
        val t = via.last()
        return w + ctx.pieceWeight(path.last(), request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) }))
    }

    /** Quante volte due valichi sono stati scelti insieme (diagnostica dei test). */
    internal var twoChoices = 0

    /**
     * La ricerca portata avanti fin oltre una volta e mezza il peso del passaggio più vicino (più
     * mezz'ora): i passaggi più lontani, o irraggiungibili, restano con un limite inferiore (la
     * frontiera), che basta a scartarli; portarla a termine costerebbe tutto il core (Italia: 3 s).
     */
    /**
     * Porta avanti una ricerca di limite fin oltre il passaggio più vicino di [WINDOW_FACTOR] più
     * [WINDOW_SLACK] secondi: per i passaggi più lontani restano i limiti dei landmark e della
     * frontiera. La finestra non tocca l'esattezza (sono limiti inferiori comunque), solo quanti
     * pezzi si pesano poi; con 1,5× + 30 min la ricerca dall'Italia verso l'Austria esplorava 461 000
     * nodi del core (quasi tutta l'Italia: 10 s sul tablet).
     */
    private fun CoreManyToOne.advanceAround() {
        advance {
            var near = Double.POSITIVE_INFINITY
            for (i in 0 until otherCount) if (isFinal(i) && weight(i) < near) near = weight(i)
            near.isFinite() && bound > near * WINDOW_FACTOR + WINDOW_SLACK
        }
    }

    /** Un passo di ricerca che valga la pena: oltre [key] di un poco (il ciclo sopra ripesa le coppie a ogni ritorno). */
    private fun slack(key: Double) = maxOf(60.0, key * 0.02)

    /**
     * I due valichi di un transito (nessuna tappa nel Paese di mezzo), scelti insieme fra tutte le
     * coppie: con molti passaggi sul secondo confine (Austria/Germania: centinaia) una ricerca per
     * ogni X costerebbe troppo. Qui: i pesi di ricerca dal capo a ogni X ([m0], portata a termine);
     * nel Paese di mezzo una ricerca sola da tutte le X insieme (ognuna col suo peso) verso ogni Y,
     * che dà per ogni Y il limite inferiore del viaggio fin lì; dall'altro capo i limiti di [m2].
     * Gli Y si esaminano in ordine di limite: il pezzo finale, poi i pesi di ricerca da ogni X a
     * quell'Y (una ricerca) e, in ordine di limite della coppia, i pezzi esatti finché il limite
     * della coppia successiva non supera il viaggio migliore. I pesi di ricerca sono limiti
     * inferiori dei pezzi (senza avoid_features, velocità massima né direzioni).
     */
    private fun chooseTransit(ctx: Ctx, ga: CountryGraph, gMid: CountryGraph, gEnd: CountryGraph, c01: List<Transfer>, c12: List<Transfer>,
                              inM1: List<Snap>, inM2: List<Snap>, m0: CoreManyToOne, m2: CoreManyToOne,
                              pieceA: (Transfer) -> DirectionsRequest, pieceMid: (Transfer, Transfer) -> DirectionsRequest, pieceEnd: (Transfer) -> DirectionsRequest,
                              ends: Ends, limit: Double, t0: Long): Triple<Transfer?, Transfer?, Double>? {
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        m0.advanceAround()
        // le radici coi limiti inferiori (i pesi, dove definitivi): i limiti restano tali
        val roots = c01.indices.map { inM1[it] to m0.lowerBound(it) }
        if (roots.none { it.second.isFinite() }) return none
        val mm = ctx.router(gMid).manyToOne(roots, inM2, rootIsTarget = false) ?: return null
        mm.advanceAround()
        ctx.queries += 3
        val b1 = DoubleArray(c12.size) { mm.lowerBound(it) }
        val w0 = DoubleArray(c01.size) { Double.NaN }; val w2 = DoubleArray(c12.size) { Double.NaN }
        val done = BooleanArray(c12.size)
        var best = limit; var bx = -1; var by = -1; var evaluated = 0; var searches = 0
        while (true) {
            var ky = -1; var key = Double.POSITIVE_INFINITY
            for (y in c12.indices) if (!done[y]) {
                val k = b1[y] + (if (w2[y].isNaN()) m2.lowerBound(y) else w2[y])
                if (k < key) { key = k; ky = y }
            }
            if (ky < 0 || key >= best) break
            if (w2[ky].isNaN()) {
                if (!m2.isFinal(ky)) { m2.advance { m2.isFinal(ky) || b1[ky] + m2.lowerBound(ky) > key }; continue }
                // il peso di ricerca è quello del pezzo quando niente lo distingue (vedi Ends); se no il pezzo
                w2[ky] = if (ends.endExact) ends.endInner + m2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                continue
            }
            // i pesi di ricerca da ogni X a questo Y, poi le coppie in ordine di limite
            val my = ctx.router(gMid).manyToOne(inM2[ky], inM1, rootIsTarget = true) ?: return null
            my.advanceAround(); searches++
            val order = c01.indices.filter { m0.lowerBound(it).isFinite() && my.lowerBound(it).isFinite() }.sortedBy { m0.lowerBound(it) + my.lowerBound(it) }
            for (x in order) {
                if (m0.lowerBound(x) + my.lowerBound(x) + w2[ky] >= best) continue
                if (w0[x].isNaN()) {
                    // prima il peso vero del capo dalla sua ricerca (va avanti solo fin lì), poi, se serve, il pezzo
                    if (!m0.isFinal(x)) m0.advance { m0.isFinal(x) || m0.lowerBound(x) + my.lowerBound(x) + w2[ky] >= best }
                    if (!m0.isFinal(x)) continue
                    w0[x] = if (ends.startExact) ends.startInner + m0.weight(x) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[x])) }
                }
                if (w0[x] + my.lowerBound(x) + w2[ky] >= best) continue
                // in transito il pezzo di mezzo va da valico a valico senza tappe né direzioni: il peso di ricerca è esatto
                if (!my.isFinal(x)) my.advance { my.isFinal(x) || w0[x] + my.lowerBound(x) + w2[ky] >= best }
                if (!my.isFinal(x)) continue
                val w1 = my.weight(x)
                val total = w0[x] + w1 + w2[ky]
                if (total < best) { best = total; bx = x; by = ky }
            }
            done[ky] = true
        }
        log?.invoke("    due valichi insieme, transito (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)} fra ${c01.size}×${c12.size} coppie, ${done.count { it }} uscite esaminate, $searches ricerche, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; ${ga.id} ${m0.stats}; ${gMid.id} ${mm.stats}; ${gEnd.id} ${m2.stats}")
        return if (bx < 0) none else Triple(c01[bx], c12[by], best)
    }

    /** Il valico scelto e il peso del viaggio intero che ci passa (+∞ senza). */
    private class Choice(val weight: Double, val transfer: Transfer?)

    /**
     * Quando il peso di ricerca ([Router.manyToOne], stessi pesi e filtri del percorso finale) di un
     * capo è esattamente quello del pezzo: senza direzione (`bearings`) al punto del capo e, con
     * più punti nel capo, senza `continue_straight` (che lega le tappe fra loro); allora il pezzo
     * pesa la ricerca più le tappe interne del capo ([startInner]/[endInner], pesate una volta).
     * Se no la ricerca resta un limite inferiore e il pezzo si calcola.
     */
    private class Ends(val startExact: Boolean, val startInner: Double, val endExact: Boolean, val endInner: Double)

    /**
     * I due valichi di un viaggio che attraversa due confini, scelti insieme: da [prefix] (grafo A)
     * al grafo [gMid] (coi punti [mid], o nessuno in transito) al grafo [gEnd] (punti [suffix]).
     * Per ogni coppia (X, Y) il peso è quello dei tre pezzi finali; i limiti inferiori vengono dalla
     * ricerca sul core ([Router.manyToOne]): dai capi del viaggio ai valichi, e nel grafo di mezzo dai
     * punti del pezzo (o, in transito, da ogni X ai Y). Le coppie si valutano in ordine di limite, un
     * pezzo alla volta, finché il limite della successiva non supera il viaggio migliore.
     * Null quando i router non cercano sul core (allora un confine alla volta); (null, null, +∞) senza percorso.
     */
    private fun chooseTwo(ctx: Ctx, prefix: Piece, mid: IntRange?, gMid: CountryGraph, suffix: IntRange, gEnd: CountryGraph, radius: (Int) -> Double, limit: Double): Triple<Transfer?, Transfer?, Double>? {
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        val ga = prefix.graph; val request = ctx.request
        fun usable(from: CountryGraph, to: CountryGraph) = transfers(from, to).filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } }
        val c01 = usable(ga, gMid); val c12 = usable(gMid, gEnd)
        if (c01.isEmpty() || c12.isEmpty()) return none
        val pl = prefix.points.last()
        val sp = ctx.snap(ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN) ?: return none
        val qi = suffix.first
        val sq = ctx.snap(gEnd, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi) ?: Double.NaN) ?: return none
        val inA = c01.map { ctx.snap(ga, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inM1 = c01.map { ctx.snap(gMid, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inM2 = c12.map { ctx.snap(gMid, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inE = c12.map { ctx.snap(gEnd, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val t0 = System.currentTimeMillis()
        twoChoices++
        fun pieceA(x: Transfer) = request.withPoints(prefix.points + (x.lat to x.lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(), (prefix.exact + true).toBooleanArray(), prefix.bearings + null)
        fun pieceMid(x: Transfer, y: Transfer): DirectionsRequest {
            val ps = ArrayList<Pair<Double, Double>>(); val rs = ArrayList<Double>(); val ex = ArrayList<Boolean>(); val bs = ArrayList<Double?>()
            ps += x.lat to x.lon; rs += TRANSFER_RADIUS; ex += true; bs += null
            mid?.forEach { i -> ps += request.coordinates[i]; rs += radius(i); ex += false; bs += request.bearings?.getOrNull(i) }
            ps += y.lat to y.lon; rs += TRANSFER_RADIUS; ex += true; bs += null
            return request.withPoints(ps, rs.toDoubleArray(), ex.toBooleanArray(), bs)
        }
        fun pieceEnd(y: Transfer) = request.withPoints(listOf(y.lat to y.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(), listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) })
        // i limiti inferiori
        val m0 = ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false) ?: return null
        val m2 = ctx.router(gEnd).manyToOne(sq, inE, rootIsTarget = true) ?: return null
        // dove i pesi di ricerca ai capi sono quelli dei pezzi (vedi Ends)
        val startExact = prefix.bearings.last() == null && (prefix.points.size == 1 || !request.continueStraight)
        val startInner = if (!startExact || prefix.points.size == 1) 0.0 else ctx.pieceWeight(ga, request.withPoints(prefix.points, prefix.radiuses.toDoubleArray(), prefix.exact.toBooleanArray(), prefix.bearings))
        val suffixPts = suffix.map { request.coordinates[it] }
        val endExact = request.bearings?.getOrNull(qi) == null && (suffixPts.size == 1 || !request.continueStraight)
        val endInner = if (!endExact || suffixPts.size == 1) 0.0 else ctx.pieceWeight(gEnd, request.withPoints(suffixPts, suffix.map { radius(it) }.toDoubleArray(), suffix.map { false }.toBooleanArray(), suffix.map { request.bearings?.getOrNull(it) }))
        if (startInner.isInfinite() || endInner.isInfinite()) return none
        val ends = Ends(startExact, startInner, endExact, endInner)
        if (mid == null) return chooseTransit(ctx, ga, gMid, gEnd, c01, c12, inM1, inM2, m0, m2, ::pieceA, ::pieceMid, ::pieceEnd, ends, limit, t0)
        val lb1: (Int, Int) -> Double
        /** Il peso vero del pezzo di mezzo dalle ricerche, quando è esatto (null se no): vedi Ends. */
        var midWeight: ((Int, Int) -> Double?)? = null
        var midStats = ""
        run {
            val f = request.coordinates[mid.first]; val l = request.coordinates[mid.last]
            val sm1 = ctx.snap(gMid, LL(f.first, f.second), radius(mid.first), false, request.bearings?.getOrNull(mid.first) ?: Double.NaN) ?: return none
            val sm2 = ctx.snap(gMid, LL(l.first, l.second), radius(mid.last), false, request.bearings?.getOrNull(mid.last) ?: Double.NaN) ?: return none
            val mA = ctx.router(gMid).manyToOne(sm1, inM1, rootIsTarget = true) ?: return null
            val mB = ctx.router(gMid).manyToOne(sm2, inM2, rootIsTarget = false) ?: return null
            // le ricerche nel Paese di mezzo e dal primo capo portate a termine: i limiti dei soli landmark
            // lasciavano pesare decine di pezzi (Soncino → Innsbruck → Monaco: 741 passaggi, 42 s)
            mA.advanceAround(); mB.advanceAround()
            midStats = "${mA.stats} / ${mB.stats}"
            // più le tappe interne del blocco di mezzo (un pezzo da sole pesa al più quanto dentro il pezzo intero)
            val inner = if (mid.first == mid.last) 0.0 else ctx.pieceWeight(gMid, request.withPoints(mid.map { request.coordinates[it] },
                mid.map { radius(it) }.toDoubleArray(), mid.map { false }.toBooleanArray(), mid.map { request.bearings?.getOrNull(it) }))
            if (inner.isInfinite()) return none
            lb1 = { x, y -> mA.lowerBound(x) + mB.lowerBound(y) + inner }
            // esatto senza continue_straight (lega le tappe) e senza direzione all'ultima tappa (vale per la partenza da lì)
            if (!request.continueStraight && request.bearings?.getOrNull(mid.last) == null) midWeight = { x, y ->
                if (!mA.isFinal(x)) mA.advance { mA.isFinal(x) }
                if (!mB.isFinal(y)) mB.advance { mB.isFinal(y) }
                mA.weight(x) + inner + mB.weight(y)
            }
        }
        ctx.queries += 4
        val w0 = DoubleArray(c01.size) { Double.NaN }; val w2 = DoubleArray(c12.size) { Double.NaN }
        val w1 = Array(c01.size) { DoubleArray(c12.size) { Double.NaN } }
        var best = limit; var bx = -1; var by = -1; var evaluated = 0
        while (true) {
            // la coppia col limite più basso del viaggio intero fra quelle non ancora pesate del tutto
            var kx = -1; var ky = -1; var key = Double.POSITIVE_INFINITY
            for (x in c01.indices) {
                val a = if (w0[x].isNaN()) m0.lowerBound(x) else w0[x]
                if (a >= best) continue
                for (y in c12.indices) {
                    if (!w0[x].isNaN() && !w2[y].isNaN() && !w1[x][y].isNaN()) continue
                    val k = a + (if (w1[x][y].isNaN()) lb1(x, y) else w1[x][y]) + (if (w2[y].isNaN()) m2.lowerBound(y) else w2[y])
                    if (k < key) { key = k; kx = x; ky = y }
                }
            }
            if (kx < 0 || key >= best) break
            // prima i pezzi ai capi (servono a molte coppie), poi quello di mezzo
            val rest1 = if (w1[kx][ky].isNaN()) lb1(kx, ky) else w1[kx][ky]
            when {
                // prima il limite vero del capo (la sua ricerca va avanti solo fin lì), poi il pezzo
                !m0.isFinal(kx) && w0[kx].isNaN() -> m0.advance { m0.isFinal(kx) || m0.lowerBound(kx) + rest1 + (if (w2[ky].isNaN()) m2.lowerBound(ky) else w2[ky]) > key + slack(key) }
                w0[kx].isNaN() -> w0[kx] = if (ends.startExact) ends.startInner + m0.weight(kx) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[kx])) }
                !m2.isFinal(ky) && w2[ky].isNaN() -> m2.advance { m2.isFinal(ky) || w0[kx] + rest1 + m2.lowerBound(ky) > key + slack(key) }
                w2[ky].isNaN() -> w2[ky] = if (ends.endExact) ends.endInner + m2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                else -> {
                    w1[kx][ky] = midWeight?.invoke(kx, ky) ?: run { evaluated++; ctx.pieceWeight(gMid, pieceMid(c01[kx], c12[ky])) }
                    val total = w0[kx] + w1[kx][ky] + w2[ky]
                    if (total < best) { best = total; bx = kx; by = ky }
                }
            }
        }
        log?.invoke("    due valichi insieme (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)} fra ${c01.size}×${c12.size} coppie, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; ${ga.id} ${m0.stats}; ${gMid.id} $midStats; ${gEnd.id} ${m2.stats}")
        return if (bx < 0) none else Triple(c01[bx], c12[by], best)
    }

    /**
     * Il valico migliore fra due Paesi, esatto: fra tutti i passaggi del confine quello col peso
     * minimo del viaggio, ogni pezzo pesato come il percorso finale (avoid_features, maximum_speed,
     * bearings, tappe, continue_straight: [Ctx.pieceWeight]). Per non pesarli tutti, un limite
     * inferiore per ognuno su entrambi i lati: la ricerca sul core senza avoid_features né limite di
     * velocità e senza direzioni ([Router.manyToOne]) — togliere strade, rallentarle o vincolare una
     * direzione può solo aumentare un peso. Dal lato vicino al confine una ricerca sola dà i limiti
     * dal suo capo a tutti i passaggi, in ordine e solo fin dove serve; dal lato lontano i landmark.
     * I passaggi si pesano in ordine di limite del viaggio intero, prima il lato vicino (costa poco),
     * finché il limite del successivo non supera il viaggio migliore: nessun altro può batterlo.
     * Null quando i router non cercano sul core: allora un valico alla volta.
     */
    private fun chooseExact(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, cands: List<Transfer>, limit: Double): Choice? {
        val none = Choice(Double.POSITIVE_INFINITY, null)
        if (cands.isEmpty()) return none
        val ga = prefix.graph
        val request = ctx.request
        // i capi del viaggio vicino al confine, agganciati come nei pezzi finali
        val pl = prefix.points.last()
        val sp = ctx.snap(ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN) ?: return none
        val qi = suffix.first
        val sq = ctx.snap(gb, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi) ?: Double.NaN) ?: return none
        val inA = cands.map { ctx.snap(ga, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inB = cands.map { ctx.snap(gb, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        // i pezzi finali con il passaggio t: prima del confine i punti del pezzo più t, dopo t più i punti nel grafo seguente
        fun pieceA(t: Transfer) = request.withPoints(prefix.points + (t.lat to t.lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(),
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null)
        fun pieceB(t: Transfer) = request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) })
        // il lato vicino: il capo più vicino ai suoi passaggi
        val nearA = cands.minOf { DistanceCalc.haversine(sp.queryLat, sp.queryLon, it.lat, it.lon) }
        val nearB = cands.minOf { DistanceCalc.haversine(it.lat, it.lon, sq.queryLat, sq.queryLon) }
        val shortIsB = nearB <= nearA
        val near = if (shortIsB) gb else ga; val far = if (shortIsB) ga else gb
        fun nearPiece(t: Transfer) = if (shortIsB) pieceB(t) else pieceA(t)
        fun farPiece(t: Transfer) = if (shortIsB) pieceA(t) else pieceB(t)
        val t0 = System.currentTimeMillis()
        val mNear = (if (shortIsB) ctx.router(gb).manyToOne(sq, inB, rootIsTarget = true) else ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false)) ?: return null
        val mFar = (if (shortIsB) ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false) else ctx.router(gb).manyToOne(sq, inB, rootIsTarget = true)) ?: return null
        val nearWeight = DoubleArray(cands.size) { Double.NaN }
        val farWeight = DoubleArray(cands.size) { Double.NaN }
        var best = limit; var bestI = -1; var nearCount = 0; var farCount = 0; var farMs = 0L
        while (true) {
            // il passaggio col limite più basso del viaggio intero, non ancora pesato dal lato lontano
            var i = -1; var key = Double.POSITIVE_INFINITY
            for (j in cands.indices) if (farWeight[j].isNaN()) {
                val k = mFar.lowerBound(j) + if (nearWeight[j].isNaN()) mNear.lowerBound(j) else nearWeight[j]
                if (k < key) { key = k; i = j }
            }
            if (i < 0 || key >= best) break
            if (nearWeight[i].isNaN()) {
                // prima il limite vero del lato vicino (la sua ricerca va avanti solo fin lì), poi il pezzo
                if (!mNear.isFinal(i)) { mNear.advance { mNear.isFinal(i) || mFar.lowerBound(i) + mNear.lowerBound(i) > key }; continue }
                nearWeight[i] = ctx.pieceWeight(near, nearPiece(cands[i])); nearCount++
                continue
            }
            // poi il limite vero del lato lontano (una ricerca sola per tutti i passaggi, fin dove serve):
            // i pezzi lontani costano (evita pedaggi: secondi) e così ne serve di solito uno
            if (!mFar.isFinal(i)) { mFar.advance { mFar.isFinal(i) || nearWeight[i] + mFar.lowerBound(i) > key }; continue }
            val t1 = System.currentTimeMillis()
            farWeight[i] = ctx.pieceWeight(far, farPiece(cands[i])); farCount++
            farMs += System.currentTimeMillis() - t1
            val total = nearWeight[i] + farWeight[i]
            if (total < best) { best = total; bestI = i }
        }
        log?.invoke("    valico scelto: ${if (bestI < 0) "nessuno" else "%.5f,%.5f peso %.0f".format(cands[bestI].lat, cands[bestI].lon, best)} fra ${cands.size} passaggi; " +
            "lato vicino ${near.id} ${mNear.stats}, $nearCount pezzi; lato lontano ${far.id} ${mFar.stats}, $farCount pezzi in $farMs ms; in tutto ${System.currentTimeMillis() - t0} ms")
        fun w(x: Double) = if (x.isFinite()) "%.0f".format(x) else "∞"
        for ((j, t) in cands.withIndex()) if (!nearWeight[j].isNaN() || !farWeight[j].isNaN() || mNear.lowerBound(j) + mFar.lowerBound(j) < best + 600)
            log?.invoke("      %.5f,%.5f r${t.rank}: ${near.id} ${if (nearWeight[j].isNaN()) "≥" + w(mNear.lowerBound(j)) else w(nearWeight[j])}, ${far.id} ${if (farWeight[j].isNaN()) "≥" + w(mFar.lowerBound(j)) else w(farWeight[j])}".format(t.lat, t.lon))
        return if (bestI < 0) none else Choice(best, cands[bestI])
    }

    // ------------------------------------------------------------------------------- the answer

    /** I pezzi uniti in un percorso solo, come se fosse stato calcolato in un grafo. */
    @Suppress("UNCHECKED_CAST")
    private fun merge(docs: List<Map<String, Any?>>, request: DirectionsRequest): Map<String, Any?> {
        val coords = ArrayList<DoubleArray>()
        val segments = ArrayList<MutableMap<String, Any?>>()
        val wayPoints = ArrayList<Int>().also { it += 0 }
        val extraValues = LinkedHashMap<String, MutableList<MutableList<Number>>>()
        val extraDist = LinkedHashMap<String, LinkedHashMap<Number, Double>>()
        val roads = ArrayList<Map<String, Any?>>()
        val points = ArrayList<Map<String, Any?>>()
        val bbox = BBox.empty()
        var hasRoads = false
        var cur: MutableMap<String, Any?>? = null
        val translator = InstructionTranslator.forLanguage(request.language)
        val pieceStarts = ArrayList<Int>()
        for ((pi, f) in docs.withIndex()) {
            val props = f["properties"] as Map<String, Any?>
            val geo = (f["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            val offset = if (pi == 0) 0 else coords.size - 1
            pieceStarts += offset
            for (i in (if (pi == 0) 0 else 1) until geo.size) { coords += geo[i]; bbox.include(geo[i][1], geo[i][0]) }
            val segs = props["segments"] as List<Map<String, Any?>>
            val wps = props["way_points"] as List<Int>
            for ((si, seg0) in segs.withIndex()) {
                val seg = LinkedHashMap(seg0)
                val steps: MutableList<Map<String, Any?>>? = (seg["steps"] as? List<Map<String, Any?>>)?.map<Map<String, Any?>, Map<String, Any?>> { shiftStep(it, offset) }?.toMutableList()
                if (steps != null) seg["steps"] = steps
                val c = cur
                if (si == 0 && pi > 0 && c != null) {
                    // dopo un passaggio di confine: lo stesso tratto prosegue
                    c["distance"] = (c["distance"] as Double) + (seg["distance"] as Double)
                    c["duration"] = round1((c["duration"] as Double) + (seg["duration"] as Double))
                    val cs = c["steps"] as MutableList<Map<String, Any?>>?
                    if (cs != null && steps != null) joinSteps(cs, steps, coords, offset, translator)
                } else {
                    cur?.let { segments += it }
                    cur = seg
                }
                val endsAtBorder = si == segs.size - 1 && pi < docs.size - 1
                if (!endsAtBorder) { segments += cur!!; cur = null; wayPoints += wps[si + 1] + offset }
            }
            // gli extra: gli intervalli spostati, uniti al confine quando il valore non cambia
            (props["extras"] as? Map<String, Any?>)?.forEach { (kind, v) ->
                val m = v as Map<String, Any?>
                val vals = extraValues.getOrPut(kind) { ArrayList() }
                for (r in m["values"] as List<List<Number>>) {
                    val from = r[0].toInt() + offset; val to = r[1].toInt() + offset; val value = r[2]
                    val last = vals.lastOrNull()
                    if (last != null && last[2] == value && last[1].toInt() == from) last[1] = to else vals += mutableListOf(from, to, value)
                }
                val dist = extraDist.getOrPut(kind) { LinkedHashMap() }
                for (sm in m["summary"] as List<Map<String, Any?>>) {
                    val value = sm["value"] as Number
                    dist[value] = (dist[value] ?: 0.0) + (sm["distance"] as Number).toDouble()
                }
            }
            (props["navmaster"] as? Map<String, Any?>)?.let { nm ->
                hasRoads = true
                (nm["roads"] as? List<Map<String, Any?>>)?.forEach { r ->
                    val m = LinkedHashMap(r)
                    m["from"] = (r["from"] as Int) + offset; m["to"] = (r["to"] as Int) + offset
                    (r["nodes"] as? List<Map<String, Any?>>)?.let { ns -> m["nodes"] = ns.map { n -> LinkedHashMap(n).apply { put("index", (n["index"] as Int) + offset) } } }
                    roads += m
                }
                (nm["points"] as? List<Map<String, Any?>>)?.forEach { p ->
                    if (points.none { it["lat"] == p["lat"] && it["lon"] == p["lon"] && it["type"] == p["type"] }) {
                        points += LinkedHashMap(p).apply { put("index", (p["index"] as Int) + offset); put("piece", pi) }
                    }
                }
            }
        }
        cur?.let { segments += it }
        // le distanze lungo il percorso dei punti (autovelox…): dall'inizio del percorso unito
        if (points.isNotEmpty()) {
            val cum = DoubleArray(coords.size)
            for (i in 1 until coords.size) cum[i] = cum[i - 1] + DistanceCalc.haversine(coords[i - 1][1], coords[i - 1][0], coords[i][1], coords[i][0])
            for (p in points) {
                val m = p as MutableMap<String, Any?>
                val start = pieceStarts[m.remove("piece") as Int]
                m["offset"] = Math.round(((m["offset"] as Number).toDouble() + cum[start]) * 10) / 10.0
            }
            points.sortBy { (it["offset"] as Number).toDouble() }
        }
        val totalDistance = segments.sumOf { it["distance"] as Double }
        val totalDuration = round1(segments.sumOf { it["duration"] as Double })
        if ("percentage" in request.attributes) for (s in segments) s["percentage"] = if (totalDistance > 0) Math.round((s["distance"] as Double) * 100 / totalDistance * 100) / 100.0 else 0.0
        val properties = LinkedHashMap<String, Any?>()
        properties["segments"] = segments
        properties["way_points"] = wayPoints
        properties["summary"] = linkedMapOf("distance" to DirectionsService.roundForUnits(totalDistance, request.units), "duration" to totalDuration)
        if (extraValues.isNotEmpty()) {
            properties["extras"] = extraValues.mapValuesTo(LinkedHashMap()) { (kind, vals) ->
                val dist = extraDist[kind] ?: LinkedHashMap()
                val total = dist.values.sum()
                linkedMapOf("values" to vals, "summary" to dist.entries.sortedByDescending { it.value }.map { (v, d) ->
                    linkedMapOf("value" to v, "distance" to DirectionsService.roundForUnits(d, request.units), "amount" to (if (total > 0) Math.round(d * 100 / total * 100) / 100.0 else 0.0))
                })
            }
        }
        if (hasRoads) properties["navmaster"] = linkedMapOf("roads" to roads, "points" to points)
        val bboxArr = doubleArrayOf(DirectionsService.r6(bbox.minLon), DirectionsService.r6(bbox.minLat), DirectionsService.r6(bbox.maxLon), DirectionsService.r6(bbox.maxLat))
        return linkedMapOf("bbox" to bboxArr, "type" to "Feature", "properties" to properties,
            "geometry" to linkedMapOf("coordinates" to coords, "type" to "LineString"))
    }

    @Suppress("UNCHECKED_CAST")
    private fun shiftStep(s: Map<String, Any?>, offset: Int): MutableMap<String, Any?> {
        val m = LinkedHashMap(s)
        val wp = s["way_points"] as List<Int>
        m["way_points"] = listOf(wp[0] + offset, wp[1] + offset)
        return m
    }

    /**
     * Le istruzioni al passaggio di confine: via l'arrivo del pezzo prima e la partenza di quello
     * dopo; la strada prosegue (stesso nome: un'istruzione sola) o "Continua dritto su …".
     */
    private fun joinSteps(before: MutableList<Map<String, Any?>>, after: MutableList<Map<String, Any?>>, coords: List<DoubleArray>, offset: Int,
                          t: InstructionTranslator) {
        if (before.lastOrNull()?.get("type") == InstructionType.FINISH.code) before.removeAt(before.size - 1)
        val depart = after.firstOrNull()
        if (depart == null) return
        val last = before.lastOrNull()
        val name = depart["name"] as? String ?: "-"
        val rest = after.drop(1)
        if (last != null && depart["type"] == InstructionType.DEPART.code && name != "-" && last["name"] == name) {
            // la stessa strada oltre il confine: l'istruzione di prima arriva fin dove arrivava la partenza
            val m = LinkedHashMap(last)
            m["distance"] = round1((last["distance"] as Number).toDouble() + (depart["distance"] as Number).toDouble())
            m["duration"] = round1((last["duration"] as Number).toDouble() + (depart["duration"] as Number).toDouble())
            m["way_points"] = listOf((last["way_points"] as List<*>)[0], (depart["way_points"] as List<*>)[1])
            before[before.size - 1] = m
        } else if (depart["type"] == InstructionType.DEPART.code) {
            val m = LinkedHashMap(depart)
            m["type"] = InstructionType.CONTINUE.code
            m["instruction"] = t.continueOn(InstructionType.CONTINUE, if (name == "-") "" else name)
            // la direzione d'arrivo al confine
            (depart["maneuver"] as? Map<*, *>)?.let { mv ->
                val k = offset
                val before0 = if (k >= 1) DistanceCalc.bearing(coords[k - 1][1], coords[k - 1][0], coords[k][1], coords[k][0]) else null
                m["maneuver"] = LinkedHashMap<String, Any?>().apply {
                    putAll(mv.entries.associate { it.key.toString() to it.value })
                    if (before0 != null) put("bearing_before", Math.round((before0 + 360) % 360).toInt())
                }
            }
            before += m
        } else before += depart
        before += rest
    }

    private fun round1(v: Double) = Math.round(v * 10) / 10.0

    companion object {
        /** La finestra delle ricerche di limite oltre il passaggio più vicino (vedi [advanceAround]): un decimo più dieci minuti. */
        const val WINDOW_FACTOR = 1.1
        const val WINDOW_SLACK = 600.0
        /** Il raggio di aggancio di un passaggio di confine: il punto è un nodo della strada. */
        const val TRANSFER_RADIUS = 3.0

        /** Diagnostica: registra anche la scelta di un valico alla volta (con le sue ricerche) per confronto. */
        @Volatile @JvmStatic var compareOneByOne = false
    }
}
