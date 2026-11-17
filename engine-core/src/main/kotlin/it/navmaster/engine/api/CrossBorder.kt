package it.navmaster.engine.api

import com.carrotsearch.hppc.LongHashSet
import com.carrotsearch.hppc.LongIntHashMap
import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.ch.CoreManyToOne
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.instructions.InstructionType
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
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

    /** Un luogo di valico: i passaggi entro questi metri (dal più importante) sono lo stesso valico. */
    internal const val PLACE = 5_000.0

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

    /**
     * La finestra delle ricerche di limite oltre il passaggio più vicino ([advanceAround]): un fattore
     * e dei secondi; [startWindowFactor]/[startWindowSlack] per la ricerca dal primo capo in transito
     * (in Italia la più grande). Non toccano l'esattezza, solo il lavoro: modificabili per le prove.
     */
    internal var windowFactor = WINDOW_FACTOR
    internal var windowSlack = WINDOW_SLACK
    // dal primo capo in transito basta arrivare al passaggio più vicino: le radici del Paese di mezzo
    // prendono i limiti degli altri dalla frontiera (Soncino → Monaco di Baviera: in Italia 92 000 nodi
    // invece di 176 000, stessi valichi; per Komjatice uguale, la ricerca deve arrivare a Tarvisio)
    internal var startWindowFactor = 1.0
    internal var startWindowSlack = 0.0
    /**
     * Il percorso mirato ([Reach]): per un passaggio lontano dal capo più di [focusRatio] volte (in linea
     * d'aria) il passaggio più vicino già fissato dalla ricerca verso tutti, il peso esatto viene da un
     * percorso da punto a punto col tetto. Il limite dei landmark non serve a dirlo: per Tarvisio da
     * Canneto (312 km) è più basso della frontiera fissata al Brennero (214 km).
     * [focusBeyond] negativo: sempre mirato (prove). Non tocca l'esattezza, solo il lavoro.
     */
    internal var focusBeyond = 0.0
    /** false: "avoid" as a penalty in every country as soon as the ban finds nothing (prove, confronto). */
    internal var shutInCheck = true
    internal var focusRatio = 1.25

    /** Il peso del viaggio scelto all'ultimo confine e i suoi valichi (diagnostica, prove). */
    internal var lastWeight = Double.NaN; private set
    internal var lastCrossings: List<Transfer> = emptyList(); private set
    /** I pezzi dell'ultimo viaggio tra Paesi: il grafo e la richiesta di ciascuno (prove). */
    internal var lastPieces: List<Pair<CountryGraph, DirectionsRequest>> = emptyList(); private set

    /** Un pezzo del viaggio dentro un grafo: i suoi punti (le tappe e i passaggi di confine). */
    private class Piece(val graph: CountryGraph, val points: MutableList<Pair<Double, Double>>, val radiuses: MutableList<Double>,
                        val exact: MutableList<Boolean> = MutableList(points.size) { false },
                        val bearings: MutableList<Double?> = MutableList(points.size) { null },
                        /** Per punto: si passa senza fermarsi ([DirectionsRequest.passThroughAt]); mai ai passaggi di confine. */
                        val passThrough: MutableList<Boolean> = MutableList(points.size) { false })

    /** Lo stato di una richiesta: router e agganci per grafo, pesi già calcolati. */
    private inner class Ctx(val request: DirectionsRequest, val penalized: Set<String>? = null,
                            /** Searches without the roads to avoid at all (the yardstick of [shutIn]). */
                            val plain: Boolean = false,
                            /** L'annullamento e i posti di ricerca della richiesta. */
                            val rc: RequestContext = RequestContext.NONE) {
        /** Per i pesi dei pezzi ([pieceWeight]): lo stesso annullamento, misure a parte (non sono tratti della risposta). */
        private val quiet = RequestContext(rc.cancel, rc.priority, rc.executor, rc.slots, rc.log, useCache = rc.useCache)
        private val routers = HashMap<String, Router>()
        private val options = HashMap<String, RouteOptions>()
        private val snaps = HashMap<String, Snap?>()
        /** The weights found, with the limit they were looked for under (an infinite one below a limit may exist above it). */
        private val weights = HashMap<String, Pair<Double, Double>>()
        var queries = 0
        /**
         * Le varianti a valichi diversi (fonte V, `navmaster.alternatives`): fra due Paesi [chooseExactTop]
         * tiene fino a [topK] luoghi entro [topFactor] volte il peso del migliore; in [top] l'ultima scelta (il migliore primo).
         */
        var topK = 1
        var topFactor = 1.0
        var top: List<Choice> = emptyList()
        /**
         * Il tempo delle varianti (`budget_ms`, dall'inizio della richiesta): la parte della scelta
         * che serve solo alla fonte V (oltre il migliore) si ferma alla sua scadenza, tenendo il
         * migliore; [topStopped] se è successo.
         */
        var topBudget: CancelToken? = null
        var topStopped = false

        /**
         * [r] as asked in [g]: "avoid" as a penalty only in the countries of [penalized] (null: all),
         * a ban elsewhere. The weights add up across countries as they are: off the roads to avoid
         * the penalty weighs as the ban.
         */
        fun inGraph(g: CountryGraph, r: DirectionsRequest): DirectionsRequest =
            if (r.avoidPenalty && penalized != null && g.id !in penalized) r.withAvoidPenalty(false) else r

        // the choice of the crossings without the bearings (they only touch the ends): the quick search
        fun router(g: CountryGraph): Router = routers.getOrPut(g.id) { g.graph.router(options(g)).also { it.cancel = rc.cancel } }

        /** Le opzioni di [router] in [g]. */
        fun options(g: CountryGraph): RouteOptions = options.getOrPut(g.id) {
            val o = g.service.routeOptions(inGraph(g, request)).copy(hasBearings = false).let { if (plain) it.copy(avoidFeatures = 0, avoidPenalty = 0.0) else it }
            // a trip across a border is a long one: a limited speed on the times only (the quick search)
            if (!g.graph.coreWide && o.maximumSpeed.isFinite() && o.maximumSpeed >= DirectionsRequest.MAXIMUM_SPEED_LOWER_BOUND) o.copy(maximumSpeedTimeOnly = true) else o
        }

        /**
         * Vero se al punto agganciato [snap] in [g] il tratto seguente dipende da quello che ci arriva:
         * vi si passa senza fermarsi ([through]: `continue_straight` o `pass_through`) a metà di un
         * arco percorribile nei due versi ([Router.directionFree]).
         */
        fun pt(g: CountryGraph, through: Boolean, snap: Snap): Boolean = through && !router(g).directionFree(snap)

        /**
         * L'aggancio di un punto come lo fa il pezzo finale ([exact]: un passaggio di confine, senza il
         * controllo delle reti isolate; [heading]: ORS `bearings`).
         */
        fun snap(g: CountryGraph, p: LL, radius: Double, exact: Boolean = radius == TRANSFER_RADIUS, heading: Double = Double.NaN): Snap? =
            snaps.getOrPut("${g.id}:${p.lat}:${p.lon}:$radius:$exact:$heading") {
                try { g.service.snap(router(g), options(g), 0, p.lat, p.lon, radius, !exact, heading, rc.useCache) } catch (e: PointNotFoundException) { null }
            }

        /**
         * Il peso di un pezzo come lo calcola il percorso finale ([DirectionsService.routes] con tutte le
         * impostazioni della richiesta); +∞ quando non c'è percorso. [token]: l'annullamento della
         * ricerca (un figlio di quello della richiesta, con la scadenza delle varianti).
         */
        fun pieceWeight(g: CountryGraph, piece: DirectionsRequest, token: CancelToken = rc.cancel): Double {
            token.check()
            queries++
            val t0 = System.currentTimeMillis()
            val qrc = if (token === rc.cancel) quiet else RequestContext(token, rc.priority, rc.executor, rc.slots, rc.log, quiet.stats, rc.useCache)
            val w = try { g.service.routes(inGraph(g, piece), qrc).single().sumOf { it.weight } } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    pezzo ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} in $ms ms")
            return w
        }

        /** I pesi dei pezzi ([pieceWeight]) nella riga `stats`: quanti tratti e i loro ms. */
        fun pieceStats(): String {
            val ls = synchronized(quiet.stats.legs) { quiet.stats.legs.toList() }
            return if (ls.isEmpty()) "" else " (pesi dei pezzi: ${ls.size} tratti, ${ls.sumOf { it.ms }} ms)"
        }

        /** The weight from [a] to [b] in [g], or +∞ when there is no path lighter than [limit]. */
        fun weight(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, limit: Double = Double.POSITIVE_INFINITY): Double {
            rc.cancel.check()
            val key = "${g.id}:${a.lat}:${a.lon}>${b.lat}:${b.lon}"
            weights[key]?.let { (w, l) -> if (w.isFinite() || l >= limit) return if (w < limit) w else Double.POSITIVE_INFINITY }
            val sa = snap(g, a, ra) ?: return Double.POSITIVE_INFINITY
            val sb = snap(g, b, rb) ?: return Double.POSITIVE_INFINITY
            queries++
            val t0 = System.currentTimeMillis()
            // for choosing: the quick search's path also when it uses a road to avoid (the pieces are then exact)
            val w = try { router(g).route(sa, sb, maxWeight = limit, exact = false).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    ricerca ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} (limite ${if (limit.isFinite()) "%.0f".format(limit) else "∞"}) in $ms ms: ${router(g).lastAlgorithm} ${router(g).lastSearch}")
            weights[key] = w to limit
            return w
        }
    }

    /**
     * La risposta GeoJSON di openrouteservice per [request]; [pointGraph]: il Paese (id del grafo) di
     * ogni punto quando è noto, altrimenti il grafo che lo aggancia più vicino. [rc]: l'annullamento
     * (le ricerche lanciano [SearchCancelledException][it.navmaster.engine.routing.SearchCancelledException]),
     * un posto di ricerca per tutto il viaggio, le misure dei pezzi.
     */
    fun handle(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext = RequestContext.NONE): Map<String, Any?> {
        val started = System.nanoTime()
        try { return rc.searching { handleIn(request, pointGraph, rc) } } finally {
            rc.stats.profile = request.profile; rc.stats.points = request.coordinates.size
            rc.stats.totalMs = (System.nanoTime() - started) / 1_000_000
        }
    }

    private fun handleIn(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext): Map<String, Any?> {
        if (!request.avoidPenalty || request.avoidFeatures == 0) return route(request, pointGraph, null, rc)
        // "avoid" as a penalty. The countries where the ban hampers a point of the trip (a lorry for
        // Bratislava: from Austria the last stretch in the city is a toll road; with the ban the trip
        // came round through Hungary, Vienna → Bratislava 705 km) and those where a point is on a
        // road to avoid: there the penalty, elsewhere the ban with its landmarks (the penalty in every
        // country took 128 s on the tablet from Lombardy to Bratislava); with none, the ban everywhere
        val started = System.currentTimeMillis()
        val onRoad = request.coordinates.indices.flatMap { i -> graphs.filter { it.service.onAvoidedRoad(request, i) }.map { it.id } }.toSet()
        val shut = onRoad + if (shutInCheck) shutIn(request, pointGraph, rc) else emptySet()
        log?.invoke("evita come penalità in: ${shut.ifEmpty { setOf("nessun Paese") }.joinToString()} (${System.currentTimeMillis() - started} ms)")
        rc.stats.addCrossings("dove la penalità ${System.currentTimeMillis() - started} ms")
        try {
            return if (shut.isEmpty()) route(request.withAvoidPenalty(false), pointGraph, null, rc) else route(request, pointGraph, shut, rc)
        } catch (e: ApiException) {
            if (!notFound(e)) throw e
            log?.invoke("evita: nessun percorso (${e.code}), come penalità in tutti i Paesi")
            rc.stats.profile = request.profile; rc.stats.points = request.coordinates.size
            rc.stats.totalMs = System.currentTimeMillis() - started
            rc.stats.setAside("divieto scartato")
        }
        return route(request, pointGraph, null, rc)
    }

    private fun notFound(e: ApiException) = e.code == RoutingErrorCodes.ROUTE_NOT_FOUND || e.code == RoutingErrorCodes.POINT_NOT_FOUND

    /**
     * The countries where the ban hampers a point of the trip: of the crossings within [LOCAL_REACH]
     * of the point without the roads to avoid (from it; to it for the arrival; both for a stop), one
     * that the ban makes unreachable or much farther ([LOCAL_DETOUR] times plus [LOCAL_SLACK]). A
     * crossing on a road to avoid does not count (another one takes its place). Only near the points:
     * a country blocked in transit is not found here (then the penalty everywhere, see [handle]).
     */
    private fun shutIn(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext): Set<String> {
        val ban = Ctx(request.withAvoidPenalty(false), rc = rc)
        val free = Ctx(request.withAvoidPenalty(false), plain = true, rc = rc)
        val out = LinkedHashSet<String>()
        val last = request.coordinates.lastIndex
        for ((i, c) in request.coordinates.withIndex()) {
            rc.cancel.check()
            val t0 = System.currentTimeMillis()
            val p = LL(c.first, c.second)
            val radius = request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius
            // the point's country as the trip takes it (a point on a road to avoid: already penalised)
            val hint = pointGraph.getOrNull(i)?.let { id -> graphs.firstOrNull { it.id == id } }
            val (g, sb) = (if (hint != null) listOf(hint) else graphs.filter { it.graph.graph.bounds.contains(p.lat, p.lon) })
                .mapNotNull { g -> ban.snap(g, p, radius)?.let { g to it } }.minByOrNull { it.second.distance } ?: continue
            if (g.id in out) continue
            val rb = ban.router(g); val r0 = free.router(g)
            // only where the searches keep to the ban (the wide core; the reduced one searches without it)
            if (!rb.searchesExact) continue
            val s0 = free.snap(g, p, radius) ?: continue
            // the crossings usable with the ban, in both countries
            val exits = graphs.filter { it !== g && it.graph.graph.bounds.intersects(g.graph.graph.bounds) }
                .flatMap { n -> transfers(g, n).map { n to LL(it.lat, it.lon) } }
                .filter { (n, l) -> ban.snap(g, l, TRANSFER_RADIUS) != null && ban.snap(n, l, TRANSFER_RADIUS) != null }
                .map { it.second }
            if (exits.isEmpty()) continue
            val in0 = exits.map { free.snap(g, it, TRANSFER_RADIUS)!! }
            val inB = exits.map { ban.snap(g, it, TRANSFER_RADIUS)!! }
            for (toPoint in when (i) { 0 -> listOf(false); last -> listOf(true); else -> listOf(false, true) }) {
                val m0 = r0.manyToOne(s0, in0, rootIsTarget = toPoint) ?: break
                m0.advance { m0.bound > LOCAL_REACH }
                val near = exits.indices.filter { m0.isFinal(it) && m0.weight(it) <= LOCAL_REACH }
                if (near.isEmpty()) continue
                val mb = rb.manyToOne(sb, inB, rootIsTarget = toPoint) ?: break
                fun limit(k: Int) = m0.weight(k) * LOCAL_DETOUR + LOCAL_SLACK
                // only until every near crossing has its weight or one is proven beyond its limit
                mb.advance { near.all { mb.isFinal(it) } || near.any { !mb.isFinal(it) && mb.bound > limit(it) } }
                val hampered = near.firstOrNull { k -> if (mb.isFinal(k)) mb.weight(k) > limit(k) else mb.bound > limit(k) }
                if (hampered != null) {
                    log?.invoke("  evita: il punto $i in ${g.id} col divieto arriva male al passaggio %.5f,%.5f (%.0f invece di %.0f)".format(
                        exits[hampered].lat, exits[hampered].lon, if (mb.isFinal(hampered)) mb.weight(hampered) else Double.POSITIVE_INFINITY, m0.weight(hampered)))
                    out += g.id; break
                }
            }
            log?.invoke("  evita: punto $i in ${g.id}, ${System.currentTimeMillis() - t0} ms")
        }
        return out
    }

    /** [handle] with "avoid" as a penalty only in the countries of [penalized] (null: all). */
    private fun route(request: DirectionsRequest, pointGraph: List<String?>, penalized: Set<String>?, rc: RequestContext): Map<String, Any?> {
        val started = System.currentTimeMillis()
        val ctx = Ctx(request, penalized, rc = rc)
        // varianti NavMaster: fra due Paesi anche gli altri valichi (fonte V); il loro tempo (budget_ms)
        // conta dall'inizio della richiesta, così comprende la parte della scelta dei valichi che serve solo a loro
        val nav = request.navAlternatives
        val navStart = System.nanoTime()
        val navBudget = nav?.let { rc.cancel.child(navStart + it.budgetMs * 1_000_000) }
        if (nav != null && VariantSource.CROSSING in nav.sources && nav.count > 1) { ctx.topK = nav.count; ctx.topFactor = nav.maxFactor; ctx.topBudget = navBudget }
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
        if (of.all { it === of[0] }) return of[0].service.handle(ctx.inGraph(of[0], request), ResponseFormat.GEOJSON, rc)
        // alternatives across borders: the via points method (each candidate is a trip like this one)
        if (request.alternatives != null && pts.size == 2) {
            return LongAlternatives(graphs.map { it.graph }, log, cancel = rc.cancel) { r, pg -> route(r, pg, penalized, rc) }.handle(request, of.map { it.id })
        }

        // i pezzi del viaggio: le tappe nello stesso grafo restano insieme, ai confini il passaggio migliore
        val pieces = ArrayList<Piece>()
        var cur = Piece(of[0], mutableListOf(request.coordinates[0]), mutableListOf(radius(0)), bearings = mutableListOf(request.bearings?.getOrNull(0)),
            passThrough = mutableListOf(request.passThroughAt(0)))
        // i blocchi di punti consecutivi nello stesso grafo; con due confini i due valichi si scelgono insieme
        val blocks = ArrayList<Pair<CountryGraph, IntRange>>()
        run { var s = 0; for (i in 1..pts.size) if (i == pts.size || of[i] !== of[s]) { blocks += of[s] to (s until i); s = i } }
        var twoDone = false
        if (blocks.size == 3 && oneSearch) {
            val (g0, r0) = blocks[0]; val (g1, r1) = blocks[1]; val (g2, r2) = blocks[2]
            // solo coi due confini diretti e senza sequenze alternative per un terzo Paese (quelle le pesa chain)
            fun direct(x: CountryGraph, y: CountryGraph, i: Int, j: Int) = countryPaths(x, y, pts[i], pts[j]).let { it.isNotEmpty() && it.all { pa -> pa.size == 2 } }
            if (direct(g0, g1, r0.last, r1.first) && direct(g1, g2, r1.last, r2.first)) {
                for (i in r0.drop(1)) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
                val two = chooseTwo(ctx, cur, r1, g1, r2, g2, radius, Double.POSITIVE_INFINITY)
                if (two != null) {
                    if (two.first == null) throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${r0.last} (%.7f %.7f) and ${r1.first} (%.7f %.7f).".format(
                        pts[r0.last].lon, pts[r0.last].lat, pts[r1.first].lon, pts[r1.first].lat))
                    val (x, y, w) = two
                    lastWeight = w; lastCrossings = listOf(x!!, y!!)
                    cur.points += x.lat to x.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
                    pieces += cur
                    cur = Piece(g1, mutableListOf(x.lat to x.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
                    for (i in r1) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
                    cur.points += y.lat to y.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
                    pieces += cur
                    cur = Piece(g2, mutableListOf(y.lat to y.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
                    for (i in r2) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
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
                    cur.points += t.lat to t.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
                    pieces += cur
                    cur = Piece(next, mutableListOf(t.lat to t.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
                }
            }
            cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i)
        }
        pieces += cur
        val pieceRequests = pieces.map { p -> p.graph to request.withPoints(p.points, p.radiuses.toDoubleArray(), p.exact.toBooleanArray(), p.bearings, p.passThrough.toBooleanArray()) }
        lastPieces = pieceRequests
        rc.stats.addCrossings("${pieces.joinToString(" → ") { it.graph.id }}: ${ctx.queries} ricerche in ${System.currentTimeMillis() - started} ms${ctx.pieceStats()}")
        rc.cancel.check()
        // i pezzi finali: i tratti dalla cache (i pesi dei pezzi li hanno già cercati), su due thread con l'aiutante
        val finals = finalPieces(ctx, pieceRequests, rc)
        log?.invoke("viaggio tra Paesi: ${pieces.joinToString(" → ") { it.graph.id }}, ${ctx.queries} ricerche, ${System.currentTimeMillis() - started} ms")
        val feature = merge(finals.map { featureOf(it) }, request)
        @Suppress("UNCHECKED_CAST")
        val bbox = feature["bbox"] as DoubleArray
        val metadata = LinkedHashMap<String, Any?>()
        metadata["attribution"] = pieces[0].graph.service.attribution
        metadata["service"] = "routing"
        metadata["timestamp"] = System.currentTimeMillis()
        metadata["query"] = LinkedHashMap<String, Any?>(request.echo).apply { put("profile", request.profile); put("format", "geojson") }
        metadata["engine"] = pieces[0].graph.service.engineInfo()
        metadata["countries"] = pieces.map { it.graph.id }
        if (nav != null) {
            val (features, alternatives) = variantsAcross(ctx, request, pieces, finals, feature, nav, rc, navStart, navBudget!!)
            metadata["navmaster"] = linkedMapOf("alternatives" to alternatives)
            val all = BBox.empty()
            for (f in features) { val b = f["bbox"] as DoubleArray; all.include(b[1], b[0]); all.include(b[3], b[2]) }
            val bboxAll = if (features.isEmpty()) doubleArrayOf() else
                doubleArrayOf(DirectionsService.r6(all.minLon), DirectionsService.r6(all.minLat), DirectionsService.r6(all.maxLon), DirectionsService.r6(all.maxLat))
            return linkedMapOf("type" to "FeatureCollection", "bbox" to bboxAll, "features" to features, "metadata" to metadata)
        }
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bbox, "features" to listOf(feature), "metadata" to metadata)
    }

    /** Un pezzo finale: il grafo, i tratti ([DirectionsService.solveAsHandled]) e la sua feature (null: non ancora costruita, vedi [featureOf]). */
    private class FinalPiece(val graph: CountryGraph, val solved: DirectionsService.Solved, val feature: Map<String, Any?>?) {
        val millis: Long = solved.legs.sumOf { it.millis }
        val meters: Double = solved.legs.sumOf { it.distance }
        val weight: Double = solved.legs.sumOf { it.weight }
    }

    /** La feature del pezzo [p]: quella di [finalPieces] o, se non l'ha costruita, costruita ora. */
    private fun featureOf(p: FinalPiece): Map<String, Any?> =
        p.feature ?: p.graph.service.feature(p.solved.request, p.solved.legs, p.graph.service.encoder(p.solved.request), null)

    /**
     * I pezzi finali di [pieceRequests]: i tratti come li dà la risposta di un grafo solo
     * ([DirectionsService.solveAsHandled]: dalla cache dei tratti, che i pesi dei pezzi per la scelta
     * dei valichi hanno riempito) e, con [features], la feature di ciascuno; su due thread se c'è un
     * posto da aiutante. Con un errore, quello del pezzo d'indice minimo. Le misure di ogni pezzo in
     * una parte di [RequestContext.stats] ([names], o l'id del grafo).
     */
    private fun finalPieces(ctx: Ctx, pieceRequests: List<Pair<CountryGraph, DirectionsRequest>>, rc: RequestContext, names: List<String>? = null,
                            features: Boolean = true): List<FinalPiece> {
        val n = pieceRequests.size
        val stats = pieceRequests.mapIndexed { i, (g, _) -> rc.stats.child(names?.getOrNull(i) ?: g.id) }
        val out = arrayOfNulls<Any>(n)
        fun one(i: Int, token: CancelToken) {
            val (g, r0) = pieceRequests[i]
            val st = stats[i]
            val prc = RequestContext(token, rc.priority, rc.executor, rc.slots, rc.log, st, rc.useCache)
            val t0 = System.nanoTime()
            out[i] = try {
                val s = g.service.solveAsHandled(ctx.inGraph(g, r0), prc)
                FinalPiece(g, s, if (features) g.service.feature(s.request, s.legs, g.service.encoder(s.request), null, st) else null)
            } catch (e: SearchCancelledException) { throw e } catch (e: RuntimeException) { e } finally {
                st.profile = r0.profile; st.points = r0.coordinates.size; st.totalMs = (System.nanoTime() - t0) / 1_000_000
            }
        }
        val slot = if (n >= 2) helperSlot(rc, rc.cancel) else null
        onTwoThreads(rc, rc.cancel, slot, { t -> for (i in 0 until n step 2) one(i, t) }, if (n >= 2) { t -> for (i in 1 until n step 2) one(i, t) } else null)
        return out.map { v -> if (v is RuntimeException) throw v else v as FinalPiece }
    }

    /**
     * Gli archi di un viaggio tra Paesi in fila, per la regola delle varianti sul viaggio intero
     * ([TripJudge]): id = (indice del grafo in [graphs]) << 32 | arco, con tempi e metri. L'insieme
     * degli id si costruisce alla prima domanda (serve solo per il migliore e per le tenute).
     */
    private class TripEdges(parts: List<Pair<Int, List<RoutePath>>>) {
        val ids: LongArray
        val ms: LongArray
        val m: DoubleArray

        init {
            val n = parts.sumOf { (_, legs) -> legs.sumOf { it.edges.size } }
            ids = LongArray(n); ms = LongArray(n); m = DoubleArray(n)
            var i = 0
            for ((gi, legs) in parts) for (l in legs) for (e in l.edges) {
                ids[i] = (gi.toLong() shl 32) or (e.edge.toLong() and 0xffffffffL); ms[i] = e.millis; m[i] = e.distance; i++
            }
        }

        val set: LongHashSet by lazy { LongHashSet(ids.size * 2).also { s -> for (id in ids) s.add(id) } }

        /** I tratti massimali di archi fuori da [k] (posizioni in [ids], metri, ms). */
        fun runsOff(k: TripEdges): List<Run> {
            val ks = k.set
            val out = ArrayList<Run>()
            var i = 0
            while (i < ids.size) {
                if (ks.contains(ids[i])) { i++; continue }
                var j = i; var mm = 0.0; var t = 0L
                while (j < ids.size && !ks.contains(ids[j])) { mm += m[j]; t += ms[j]; j++ }
                out += Run(i, j - 1, mm, t, 0.0)
                i = j
            }
            return out
        }
    }

    /**
     * Ref e classi degli archi per grafo ([EdgeInfo], con la sua cache dei nomi) e firma e metri per
     * ref dei pezzi già letti (per identità della lista dei tratti: i pezzi del migliore si leggono una
     * volta sola): una per richiesta, sul thread della richiesta.
     */
    private class TripLabels {
        private val infos = HashMap<CountryGraph, Pair<EdgeInfo, CorridorAnalyzer>>()
        private val memo = java.util.IdentityHashMap<List<RoutePath>, Pair<List<Pair<String, Double>>, Map<String, Double>>>()

        private fun of(g: CountryGraph) = infos.getOrPut(g) {
            val rg = g.graph
            val info = EdgeInfo(rg.graph, rg.encoding.shared, rg.guidance)
            info to CorridorAnalyzer(rg.graph, info, rg.guidance)
        }

        fun info(g: CountryGraph): EdgeInfo = of(g).first

        /** Le strade (ref) dei pezzi [parts], per la firma e le strade nuove: la firma unita (km) e i metri per ref. */
        fun refs(parts: List<Pair<CountryGraph, List<RoutePath>>>): Pair<List<Pair<String, Double>>, Map<String, Double>> {
            val sig = ArrayList<Pair<String, Double>>()
            val meters = HashMap<String, Double>()
            for ((g, legs) in parts) {
                val (ps, pm) = memo.getOrPut(legs) {
                    val (info, corridors) = of(g)
                    val mm = HashMap<String, Double>()
                    for (l in legs) for (e in l.edges) info.ref(e.edge)?.let { mm.merge(it, e.distance, Double::plus) }
                    corridors.signature(legs) to mm
                }
                for ((ref, km) in ps) {
                    val last = sig.lastOrNull()
                    if (last != null && last.first == ref) sig[sig.size - 1] = ref to Math.round((last.second + km) * 10) / 10.0 else sig += ref to km
                }
                for ((ref, mm) in pm) meters.merge(ref, mm, Double::plus)
            }
            return sig to meters
        }
    }

    /**
     * La regola di accettazione delle varianti ([VariantRules], come [VariantJudge.evaluate]) sul
     * viaggio tra Paesi intero: per le varianti che cambiano più pezzi (fonte V) e per la distanza fra
     * varianti di origine diversa. Tempi e km sui totali del migliore [best]; la deviazione più lunga
     * dal tenuto più vicino ≥ dMin1 (in scala, [NavAlternativesOptions.minDetourScale]), il tratto
     * deviato, la sanità locale verso il migliore, le autostrade (stessa firma di un tenuto).
     */
    private inner class TripJudge(val best: TripEdges, private val labels: TripLabels, private val nav: NavAlternativesOptions, tripMeters: Double) {
        private val scale = nav.minDetourScale
        val d1 = VariantRules.dMin1(tripMeters) * scale
        /** id → posizione (la prima) nel migliore; i tempi cumulati del migliore (prefisso: prima dell'arco i). */
        private val pos = LongIntHashMap(best.ids.size * 2)
        private val preMs = LongArray(best.ids.size + 1)

        init {
            for (i in best.ids.indices) { if (!pos.containsKey(best.ids[i])) pos.put(best.ids[i], i); preMs[i + 1] = preMs[i] + best.ms[i] }
        }

        private fun mw(id: Long): Boolean = labels.info(graphs[(id ushr 32).toInt()]).isMotorwayOrTrunk(id.toInt())

        /** La quota di autostrada e superstrada del migliore, letta solo se serve. */
        private val bestMwShare: Double by lazy {
            var mwm = 0.0; var all = 0.0
            for (i in best.ids.indices) { all += best.m[i]; if (mw(best.ids[i])) mwm += best.m[i] }
            if (all > 0) mwm / all else 0.0
        }

        /** Il tempo del migliore fra le ancore del tratto [r] di [v] (gli archi strettamente fra le due). */
        private fun bestMsAround(v: TripEdges, r: Run): Long {
            val a = (if (r.from > 0) pos.getOrDefault(v.ids[r.from - 1], -1) else -1) + 1
            val b = if (r.to + 1 < v.ids.size) pos.getOrDefault(v.ids[r.to + 1], best.ids.size) else best.ids.size
            return if (b > a) preMs[b] - preMs[a] else 0L
        }

        private fun sane(v: TripEdges, runs: List<Run>, a: Double, bMs: Long): Boolean = runs.all { r -> r.ms <= a * bestMsAround(v, r) + bMs * scale }

        /** La deviazione minima da ogni tenuto per il livello [tier]. */
        fun minDev(tier: Int): Double = when (tier) {
            1 -> d1
            2 -> VariantRules.T2_D * d1
            else -> maxOf(VariantRules.T3_D_MIN * scale, VariantRules.T3_D * d1)
        }

        /** Vero se [v], già giudicata dal suo pezzo al livello [tier], devia da ognuno di [others] quanto chiede il livello. */
        fun diverse(v: TripEdges, tier: Int, others: List<TripEdges>): Boolean =
            others.all { o -> (v.runsOff(o).maxOfOrNull { it.meters } ?: 0.0) >= minDev(tier) }

        /**
         * Il livello di [v] (firma [sig], tempo [r] e km [rk] sul migliore) rispetto ai tenuti [kept]
         * (il migliore primo, con le firme): 1, 2, 3 (solo con `fill`), 0 con il motivo dello scarto.
         */
        fun tier(v: TripEdges, sig: List<String>, r: Double, rk: Double, kept: List<Pair<TripEdges, List<String>>>): Pair<Int, String> {
            val runsB = v.runsOff(best)
            if (runsB.isEmpty()) return 0 to "doppione"
            // la deviazione più lunga e il suo tempo, dal tenuto più vicino
            var dMax = Double.POSITIVE_INFINITY; var majorMs = Long.MAX_VALUE; var same = false
            for ((k, ks) in kept) {
                val major = v.runsOff(k).maxByOrNull { it.meters }
                dMax = minOf(dMax, major?.meters ?: 0.0); majorMs = minOf(majorMs, major?.ms ?: 0L)
                if (ks == sig) same = true
            }
            if (dMax <= 0.0) return 0 to "doppione"
            val majorB = runsB.maxByOrNull { it.meters }!!
            fun mwOk(): Boolean {
                if (!same || bestMwShare < VariantRules.MW_BEST_SHARE) return true
                var mwm = 0.0
                for (i in majorB.from..majorB.to) if (mw(v.ids[i])) mwm += v.m[i]
                return majorB.meters > 0 && mwm / majorB.meters >= VariantRules.MW_RUN_SHARE
            }
            val tier1Time = (r <= nav.tier1Factor && rk <= nav.tier1KmFactor) || (nav.allowFewerKm && rk < 1.0 && r <= nav.maxFactor)
            val withinMax = r <= nav.maxFactor && (rk <= nav.maxKmFactor || (nav.allowFewerKm && rk < 1.0))
            if (tier1Time && dMax >= d1 && majorMs >= VariantRules.T1_RUN_MS * scale && sane(v, runsB, VariantRules.T1_A, VariantRules.T1_B_MS))
                return if (mwOk()) 1 to "" else 2 to "autostrade"
            if (withinMax && dMax >= minDev(2) && majorMs >= VariantRules.T2_RUN_MS * scale && sane(v, runsB, VariantRules.T2_A, VariantRules.T2_B_MS)) return 2 to ""
            if (nav.fill && withinMax && dMax >= minDev(3) && sane(v, runsB, VariantRules.T3_A, VariantRules.T3_B_MS)) return 3 to ""
            return 0 to when {
                r > nav.maxFactor -> "tempo"
                !withinMax -> "km"
                dMax < minDev(3) || majorMs < VariantRules.T2_RUN_MS * scale -> "diversità"
                !sane(v, runsB, VariantRules.T2_A, VariantRules.T2_B_MS) -> "locale"
                else -> "diversità"
            }
        }
    }

    /**
     * Una variante di un viaggio tra Paesi: livello (quello del suo pezzo; per la fonte V si giudica a
     * ogni scelta, [TripJudge.tier]), fonte, punteggio, tempo e km sul viaggio del migliore, archi e
     * firma (la regola sul viaggio intero), ordine d'arrivo, `variant` della risposta e come si
     * costruisce la feature (solo per le tenute).
     */
    private class CrossVariant(val tier: Int, val source: VariantSource, val score: Double, val r: Double, val rk: Double, val edges: TripEdges,
                               val sig: List<String>, val order: Int, val json: MutableMap<String, Any?>, val build: () -> Map<String, Any?>)

    /** "+4 min, −6 km". */
    private fun diff(extraS: Double, extraM: Double): String {
        val min = Math.round(extraS / 60).toInt(); val km = Math.round(extraM / 1000).toInt()
        fun signed(v: Int) = if (v < 0) "−${-v}" else "+$v"
        return "${signed(min)} min, ${signed(km)} km"
    }

    /** [feature] con `properties.navmaster.variant` = [variant]. */
    @Suppress("UNCHECKED_CAST")
    private fun withVariant(feature: Map<String, Any?>, variant: Map<String, Any?>): Map<String, Any?> {
        val props = feature["properties"] as MutableMap<String, Any?>
        val nm = props["navmaster"] as? MutableMap<String, Any?> ?: linkedMapOf<String, Any?>().also { props["navmaster"] = it }
        nm["variant"] = variant
        return feature
    }

    /**
     * Le varianti NavMaster di un viaggio tra Paesi (`navmaster.alternatives`, passo 7) e
     * `metadata.navmaster.alternatives`. (b) A valichi fissi: le varianti del pezzo più lungo (poi
     * degli altri pezzi ≥ [PIECE_SHARE] del viaggio, finché mancano varianti di livello 1) dal suo
     * servizio ([NavAlternativesEngine] sulla richiesta del pezzo: i valichi sono punti esatti),
     * giudicate sui totali del viaggio ([TripContext]). (c) Fonte V, se (b) non ha già tutte le
     * varianti di livello 1: gli altri luoghi di valico fra due Paesi ([chooseExactTop]) coi loro
     * tratti, dalla cache. La scelta è golosa (per livello, poi fonte M, P, V, poi punteggio): la fonte
     * V si giudica sul viaggio intero rispetto ai tenuti ([TripJudge]), le varianti dei pezzi devono
     * stare abbastanza lontane dalle tenute V. Ogni variante sono i pezzi del migliore con quelli
     * cambiati, uniti ([merge]); le feature solo per le tenute. Il tempo [budget] e i ms contano da
     * [t0] (`System.nanoTime`, l'inizio della richiesta).
     */
    private fun variantsAcross(ctx: Ctx, request: DirectionsRequest, pieces: List<Piece>, finals: List<FinalPiece>, bestFeature: Map<String, Any?>,
                               nav: NavAlternativesOptions, rc: RequestContext, t0: Long, budget: CancelToken): Pair<List<Map<String, Any?>>, Map<String, Any?>> {
        val want = nav.count - 1
        val totalMs = finals.sumOf { it.millis }; val totalM = finals.sumOf { it.meters }; val totalW = finals.sumOf { it.weight }
        val labels = TripLabels()
        val (bestSig, bestRefM) = labels.refs(finals.map { it.graph to it.solved.legs })
        fun edgesOf(parts: List<Pair<CountryGraph, List<RoutePath>>>) = TripEdges(parts.map { (g, legs) -> graphs.indexOf(g) to legs })
        val bestEdges = edgesOf(finals.map { it.graph to it.solved.legs })
        val judge = TripJudge(bestEdges, labels, nav, totalM)
        val cands = ArrayList<CrossVariant>()
        val notes = ArrayList<String>(); val summaries = ArrayList<String>()
        if (ctx.topStopped) notes += "budget"
        fun leftMs() = nav.budgetMs - (System.nanoTime() - t0) / 1_000_000
        fun tier1() = cands.count { it.tier == 1 }
        fun sigJson(sig: List<Pair<String, Double>>) = sig.map { linkedMapOf("ref" to it.first, "km" to it.second) }
        fun factor(x: Double) = Math.round(x * 1000) / 1000.0

        // (b) le varianti dei pezzi lunghi, coi valichi fissi
        val pieceSources = nav.sources - VariantSource.CROSSING
        if (pieceSources.isNotEmpty()) for ((n, j) in finals.indices.sortedByDescending { finals[it].meters }.withIndex()) {
            if (tier1() >= want || (n > 0 && finals[j].meters < PIECE_SHARE * totalM)) break
            val left = leftMs()
            if (left < MIN_PIECE_BUDGET_MS) { notes += "budget"; break }
            val f = finals[j]
            val trip = TripContext(totalMs - f.millis, totalM - f.meters, totalW - f.weight, finals.take(j).sumOf { it.meters })
            val opt = NavAlternativesOptions(want - tier1() + 1, nav.tier1Factor, nav.maxFactor, left, true, nav.tier1KmFactor, nav.maxKmFactor,
                nav.allowFewerKm, nav.fill, pieceSources, nav.minDetourScale).withTrace(nav.trace)
            val prc = RequestContext(rc.cancel, rc.priority, rc.executor, rc.slots, rc.log ?: log, rc.stats.child("varianti ${f.graph.id}"), rc.useCache)
            val res = NavAlternativesEngine(f.graph.service, prc, trip).run(f.solved.request, opt)
            summaries += "${f.graph.id}: ${res.summary}"
            res.note?.let { notes += "${f.graph.id}: $it" }
            val encoder = f.graph.service.encoder(f.solved.request)
            for (a in res.variants) {
                val parts = finals.mapIndexed { i, p -> p.graph to (if (i == j) a.legs else p.solved.legs) }
                val (sig, _) = labels.refs(parts)
                val json = LinkedHashMap(res.json(a))
                json["signature"] = sig.joinToString(" · ") { it.first }; json["refs"] = sigJson(sig)
                val r = (json["factor"] as Number).toDouble(); val rk = (json["km_factor"] as Number).toDouble()
                cands += CrossVariant(a.verdict.tier, a.draft.source, a.verdict.score, r, rk, edgesOf(parts), sig.map { it.first }, cands.size, json) {
                    merge(finals.mapIndexed { i, p -> if (i == j) f.graph.service.feature(f.solved.request, a.legs, encoder, null) else featureOf(p) }, request)
                }
            }
        }

        // (c) fonte V: gli altri valichi fra due Paesi (dalla scelta del valico), coi loro tratti; il livello alla scelta
        var crossings = 0
        val top = ctx.top
        if (VariantSource.CROSSING in nav.sources && pieces.size == 2 && top.size > 1 && top[0].transfer === lastCrossings.singleOrNull()) {
            if (tier1() >= want) summaries += "V no (${tier1()} di livello 1)" else {
                val a = pieces[0]; val b = pieces[1]
                val vrc = RequestContext(budget, rc.priority, rc.executor, rc.slots, rc.log, rc.stats.child("varianti valichi"), rc.useCache)
                try {
                    for (c in top.drop(1)) {
                        val t = c.transfer ?: continue
                        budget.check()
                        val ra = request.withPoints(a.points.dropLast(1) + (t.lat to t.lon), a.radiuses.toDoubleArray(), a.exact.toBooleanArray(), a.bearings, a.passThrough.toBooleanArray())
                        val rb = request.withPoints(listOf(t.lat to t.lon) + b.points.drop(1), b.radiuses.toDoubleArray(), b.exact.toBooleanArray(), b.bearings, b.passThrough.toBooleanArray())
                        // solo i tratti (dalla cache): le feature solo per le tenute
                        val alt = try { finalPieces(ctx, listOf(a.graph to ra, b.graph to rb), vrc, listOf("valico ${a.graph.id}", "valico ${b.graph.id}"), features = false) } catch (e: ApiException) { continue }
                        crossings++
                        val ms = alt.sumOf { it.millis }; val m = alt.sumOf { it.meters }
                        val r = ms.toDouble() / maxOf(1L, totalMs); val rk = m / maxOf(1.0, totalM)
                        val within = r <= nav.maxFactor && (rk <= nav.maxKmFactor || (nav.allowFewerKm && rk < 1.0))
                        log?.invoke("varianti: V valico %.5f,%.5f: ${"%+d".format((ms - totalMs) / 1000)} s, ${"%+.1f".format((m - totalM) / 1000)} km → ".format(t.lat, t.lon) +
                            if (within) "candidata" else "scartata (${if (r > nav.maxFactor) "tempo" else "km"})")
                        if (!within) continue
                        val (sig, refM) = labels.refs(alt.map { it.graph to it.solved.legs })
                        val newRefs = sig.map { it.first }.distinct().filter { ref -> (refM[ref] ?: 0.0) >= VariantRules.NEW_REF_M && (bestRefM[ref] ?: 0.0) < VariantRules.ABSENT_REF_M }
                        val label = when {
                            newRefs.size >= 2 -> "via ${newRefs[0]} + ${newRefs[1]}"
                            newRefs.size == 1 -> "via ${newRefs[0]}"
                            else -> "altro valico"
                        }
                        val extraS = (ms - totalMs) / 1000.0; val extraM = m - totalM
                        // livello e «più lunga» alla scelta
                        val json = linkedMapOf<String, Any?>("rank" to 0, "tier" to 0, "source" to VariantSource.CROSSING.json, "label" to label, "diff" to diff(extraS, extraM),
                            "signature" to sig.joinToString(" · ") { it.first }, "refs" to sigJson(sig), "new_refs" to newRefs,
                            "extra_s" to Math.round(extraS * 10) / 10.0, "extra_m" to Math.round(extraM * 10) / 10.0, "factor" to factor(r), "km_factor" to factor(rk),
                            "longer" to false, "detours" to emptyList<Any>(), "crossing" to linkedMapOf("lat" to t.lat, "lon" to t.lon))
                        // il bonus delle strade nuove come il giudice: solo le autostrade (una superstrada statale, SS o B, no)
                        val score = 0.6 * r + 0.4 * rk - VariantRules.NEW_REF_BONUS * minOf(VariantRules.NEW_REF_MAX, newRefs.count { CorridorAnalyzer.isMotorwayRef(it) })
                        cands += CrossVariant(0, VariantSource.CROSSING, score, r, rk, edgesOf(alt.map { it.graph to it.solved.legs }), sig.map { it.first }, cands.size, json) {
                            merge(alt.map { featureOf(it) }, request)
                        }
                    }
                } catch (e: SearchCancelledException) {
                    if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
                    notes += "budget"
                }
                summaries += "V $crossings"
            }
        }

        // le tenute, golose: per livello, poi fonte (M, P, V), poi punteggio. La fonte V si giudica sul viaggio
        // intero rispetto ai tenuti; le varianti dei pezzi (giudicate dal loro pezzo) devono stare lontane dalle tenute V
        val chosen = ArrayList<CrossVariant>()
        val kept = arrayListOf(bestEdges to bestSig.map { it.first })
        val left = ArrayList(cands)
        val why = HashMap<CrossVariant, String>()
        while (chosen.size < want && left.isNotEmpty()) {
            var pick: CrossVariant? = null; var pickTier = 0
            val out = ArrayList<CrossVariant>()
            for (c in left) {
                val t = if (c.source == VariantSource.CROSSING) judge.tier(c.edges, c.sig, c.r, c.rk, kept).let { (t, w) -> why[c] = w; t }
                    else if (judge.diverse(c.edges, c.tier, chosen.filter { it.source == VariantSource.CROSSING }.map { it.edges })) c.tier else 0.also { why[c] = "diversità" }
                // con più tenuti un livello non migliora: lo scarto è definitivo
                if (t == 0) { out += c; continue }
                val p = pick
                if (p == null || t < pickTier || (t == pickTier && (c.source.ordinal < p.source.ordinal ||
                        (c.source == p.source && (c.score < p.score || (c.score == p.score && c.order < p.order)))))) { pick = c; pickTier = t }
            }
            left.removeAll(out.toSet())
            val p = pick ?: break
            chosen += p; left.remove(p); kept += p.edges to p.sig
            if (p.source == VariantSource.CROSSING) { p.json["tier"] = pickTier; p.json["longer"] = pickTier >= 2 && p.r > nav.tier1Factor }
        }
        for (c in cands) if (c.source == VariantSource.CROSSING) {
            @Suppress("UNCHECKED_CAST") val at = c.json["crossing"] as Map<String, Any?>
            log?.invoke("varianti: V valico %.5f,%.5f: ".format(at["lat"], at["lon"]) +
                if (c in chosen) "tenuta, livello ${c.json["tier"]}" else "scartata (${why[c]?.ifEmpty { null } ?: "posti finiti"})")
        }
        val features = ArrayList<Map<String, Any?>>()
        if (!nav.excludeBest) {
            val top2 = bestSig.sortedByDescending { it.second }.take(2).map { it.first }.toSet()
            val refs = bestSig.map { it.first }.filter { it in top2 }.distinct()
            features += withVariant(bestFeature, linkedMapOf("rank" to 0, "tier" to 0, "source" to VariantSource.BEST.json,
                "label" to (if (refs.isEmpty()) "percorso migliore" else "via " + refs.joinToString(" + ")), "diff" to "",
                "signature" to bestSig.joinToString(" · ") { it.first }, "refs" to sigJson(bestSig), "new_refs" to emptyList<String>(), "extra_s" to 0.0, "extra_m" to 0.0,
                "factor" to 1.0, "km_factor" to 1.0, "longer" to false, "detours" to emptyList<Any>()))
        }
        // le feature delle tenute, su due thread se c'è l'aiutante
        val built = arrayOfNulls<Map<String, Any?>>(chosen.size)
        fun build(parity: Int) { for (i in chosen.indices) if (i % 2 == parity) built[i] = chosen[i].build() }
        onTwoThreads(rc, rc.cancel, if (chosen.size >= 2) helperSlot(rc, rc.cancel) else null, { build(0) }, if (chosen.size >= 2) { _ -> build(1) } else null)
        for ((i, v) in chosen.withIndex()) { v.json["rank"] = i + 1; features += withVariant(built[i]!!, v.json) }
        val ms = (System.nanoTime() - t0) / 1_000_000
        val note = if (chosen.size >= want) null else notes.firstOrNull() ?: "nessun'altra strada entro +${Math.round((nav.maxFactor - 1) * 100)} %"
        val summary = summaries.joinToString(" | ").ifEmpty { "nessuna fonte" }
        log?.invoke("varianti: tra Paesi ${chosen.size}/$want in $ms ms ($summary)" + (note?.let { "; $it" } ?: ""))
        rc.stats.variants = "${chosen.size}/$want in $ms ms ($summary)"
        return features to linkedMapOf("asked" to nav.count, "found" to 1 + chosen.size, "variants" to chosen.size, "ms" to ms, "note" to note, "candidates" to summary)
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
            val c2 = Ctx(ctx.request, ctx.penalized, rc = ctx.rc)
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
                    val cs = chooseExactTop(ctx, prefix, suffix, radius, to, cands, best, ctx.topK, ctx.topFactor)
                    if (cs != null && ctx.topK > 1) ctx.top = cs
                    val c = cs?.first()
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
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null, (prefix.passThrough + false).toBooleanArray()))
        for (k in 1 until via.size) {
            if (w.isInfinite()) return w
            w += ctx.pieceWeight(path[k], request.withPoints(listOf(via[k - 1].lat to via[k - 1].lon, via[k].lat to via[k].lon),
                doubleArrayOf(TRANSFER_RADIUS, TRANSFER_RADIUS), booleanArrayOf(true, true), null))
        }
        if (w.isInfinite()) return w
        val t = via.last()
        return w + ctx.pieceWeight(path.last(), request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) }, (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray()))
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
    private fun CoreManyToOne.advanceAround(factor: Double = windowFactor, slack: Double = windowSlack) {
        advance {
            var near = Double.POSITIVE_INFINITY
            for (i in 0 until otherCount) if (isFinal(i) && weight(i) < near) near = weight(i)
            near.isFinite() && bound > near * factor + slack
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
                              inM1: List<Snap>, inM2: List<Snap>, r0: Reach, r2: Reach,
                              pieceA: (Transfer) -> DirectionsRequest, pieceMid: (Transfer, Transfer) -> DirectionsRequest, pieceEnd: (Transfer) -> DirectionsRequest,
                              ends: Ends, limit: Double, t0: Long): Triple<Transfer?, Transfer?, Double>? {
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        val m0 = r0.base; val m2 = r2.base
        m0.advanceAround(startWindowFactor, startWindowSlack)
        val m0Window = m0.stats
        // le radici coi limiti inferiori (i pesi, dove definitivi): i limiti restano tali
        val roots = c01.indices.map { inM1[it] to r0.lowerBound(it) }
        if (roots.none { it.second.isFinite() }) return none
        val mm = ctx.router(gMid).manyToOne(roots, inM2, rootIsTarget = false) ?: return null
        mm.advanceAround()
        ctx.queries += 3
        val b1 = DoubleArray(c12.size) { mm.lowerBound(it) }
        val midExact = ctx.router(gMid).searchesExact
        val w0 = DoubleArray(c01.size) { Double.NaN }; val w2 = DoubleArray(c12.size) { Double.NaN }
        val done = BooleanArray(c12.size)
        var best = limit; var bx = -1; var by = -1; var evaluated = 0; var searches = 0
        while (true) {
            var ky = -1; var key = Double.POSITIVE_INFINITY
            for (y in c12.indices) if (!done[y]) {
                val k = b1[y] + (if (w2[y].isNaN()) r2.lowerBound(y) else w2[y])
                if (k < key) { key = k; ky = y }
            }
            if (ky < 0 || key >= best) break
            if (w2[ky].isNaN()) {
                if (!r2.isFinal(ky)) { r2.advanceFor(ky, best - b1[ky]) { lb -> b1[ky] + lb > key }; continue }
                // il peso di ricerca è quello del pezzo quando niente lo distingue (vedi Ends); se no il pezzo
                w2[ky] = if (ends.endExact) ends.endInner + r2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                continue
            }
            // i pesi di ricerca da ogni X a questo Y, poi le coppie in ordine di limite
            val my = ctx.router(gMid).manyToOne(inM2[ky], inM1, rootIsTarget = true) ?: return null
            my.advanceAround(); searches++
            val order = c01.indices.filter { r0.lowerBound(it).isFinite() && my.lowerBound(it).isFinite() }.sortedBy { r0.lowerBound(it) + my.lowerBound(it) }
            for (x in order) {
                if (r0.lowerBound(x) + my.lowerBound(x) + w2[ky] >= best) continue
                if (w0[x].isNaN()) {
                    // prima il peso vero del capo dalla sua ricerca (o da una mirata, se lontano), poi, se serve, il pezzo
                    if (!r0.isFinal(x)) r0.advanceFor(x, best - my.lowerBound(x) - w2[ky]) { lb -> lb + my.lowerBound(x) + w2[ky] >= best }
                    if (!r0.isFinal(x)) continue
                    w0[x] = if (ends.startExact) ends.startInner + r0.weight(x) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[x])) }
                }
                if (w0[x] + my.lowerBound(x) + w2[ky] >= best) continue
                // in transito il pezzo di mezzo va da valico a valico senza tappe né direzioni: il peso di ricerca è esatto
                if (!my.isFinal(x)) my.advance { my.isFinal(x) || w0[x] + my.lowerBound(x) + w2[ky] >= best }
                if (!my.isFinal(x)) continue
                // (con la ricerca rapida del core ridotto il peso di ricerca è solo un limite: allora il pezzo)
                val w1 = if (midExact) my.weight(x) else { evaluated++; ctx.pieceWeight(gMid, pieceMid(c01[x], c12[ky])) }
                val total = w0[x] + w1 + w2[ky]
                if (total < best) { best = total; bx = x; by = ky }
            }
            done[ky] = true
        }
        log?.invoke("    due valichi insieme, transito (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)} fra ${c01.size}×${c12.size} coppie, ${done.count { it }} uscite esaminate, $searches ricerche, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; ${ga.id} ${r0.stats} (finestra: $m0Window); ${gMid.id} ${mm.stats}; ${gEnd.id} ${r2.stats}")
        return if (bx < 0) none else Triple(c01[bx], c12[by], best)
    }

    /**
     * Una ricerca di limite da un capo verso i passaggi di un confine ([base]), più percorsi mirati per
     * i passaggi lontani. La ricerca verso tutti i passaggi ha un potenziale ALT debole (il passaggio più
     * vicino): per fissare il peso di uno lontano esplora tutto ciò che sta più vicino di lui (Canneto →
     * Tarvisio: 254 000 nodi del core, mezza Italia); anche una sola direzione verso il suo gruppo ne
     * esplora 190 000. Un percorso da punto a punto (core-ALT bidirezionale, [Router.route]) con un tetto
     * ([advanceFor]: si ferma appena è dimostrato che il passaggio non può vincere) segue un corridoio.
     * Stessi pesi, filtri e svolte: i pesi definitivi sono identici; i limiti sono il massimo dei due.
     */
    private inner class Reach(private val ctx: Ctx, private val g: CountryGraph, private val root: Snap, private val snaps: List<Snap>,
                              private val where: List<Transfer>, private val rootIsTarget: Boolean, val base: CoreManyToOne) {
        /** Il peso esatto da un percorso mirato (NaN: non calcolato) e il limite dimostrato da uno col tetto. */
        private val exact = DoubleArray(snaps.size) { Double.NaN }
        private val proven = DoubleArray(snaps.size)
        private var routes = 0
        private val routeStats = ArrayList<String>()

        fun isFinal(i: Int): Boolean = base.isFinal(i) || !exact[i].isNaN()
        fun weight(i: Int): Double = if (base.isFinal(i)) base.weight(i) else exact[i]
        fun lowerBound(i: Int): Double = if (!exact[i].isNaN()) exact[i] else maxOf(base.lowerBound(i), proven[i])

        /** In linea d'aria dal capo al passaggio [k], in metri. */
        private fun beeline(k: Int) = DistanceCalc.haversine(root.queryLat, root.queryLon, where[k].lat, where[k].lon)

        /** Vero se [i] è lontano: oltre [focusRatio] volte il passaggio più vicino (in linea d'aria) già fissato dalla ricerca verso tutti. */
        private fun far(i: Int): Boolean {
            if (focusBeyond < 0) return true
            var near = Double.POSITIVE_INFINITY
            for (k in where.indices) if (base.isFinal(k) && base.weight(k).isFinite()) near = minOf(near, beeline(k))
            return near.isFinite() && beeline(i) > near * focusRatio
        }

        /**
         * Avanti finché [i] ha il peso definitivo o [enough] (sul suo limite inferiore) è vero. [cap]: il
         * peso oltre il quale [i] non può più vincere (dal viaggio migliore già trovato; +∞ se non ce n'è
         * uno), il tetto del percorso mirato. Non il confine di [enough]: quello è il limite del passaggio
         * che viene dopo, appena sopra il suo, e un percorso con quel tetto non trova niente (giri a vuoto).
         * Senza percorso trovato il peso è almeno il tetto: allora [i] non vince e si torna al chiamante,
         * che non lo sceglie più ([CAP_MARGIN] sopra il tetto, così gli arrotondamenti non lo riportano sotto).
         */
        fun advanceFor(i: Int, cap: Double, enough: (Double) -> Boolean) {
            if (isFinal(i) || enough(lowerBound(i))) return
            if (far(i) && cap > lowerBound(i)) {
                // un percorso trova solo pesi sotto il tetto (anche uguali, col nextUp)
                val top = if (cap.isFinite()) Math.nextUp(cap) else cap
                val r = ctx.router(g)
                try {
                    val p = if (rootIsTarget) r.route(snaps[i], root, maxWeight = top) else r.route(root, snaps[i], maxWeight = top)
                    exact[i] = p.weight
                } catch (e: RouteNotFoundException) {
                    proven[i] = maxOf(proven[i], top + CAP_MARGIN)
                }
                routes++; ctx.queries++
                if (routeStats.size < 4) routeStats += r.lastStats
                return
            }
            base.advance { base.isFinal(i) || enough(lowerBound(i)) }
        }

        val stats: String get() = base.stats + if (routes == 0) "" else " + $routes mirati (${routeStats.joinToString(" / ")})"
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

    /** Gli [Ends] di un viaggio da [prefix] (grafo di partenza) ai punti [suffix] in [gEnd]; null quando un capo con più tappe non ha percorso. */
    private fun ends(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gEnd: CountryGraph): Ends? {
        val request = ctx.request
        // i pesi di ricerca sono quelli dei pezzi solo se la ricerca non passa dalla rapida senza avoid_features (core ridotto)
        // con più punti nel capo, il tratto dal (o al) punto vicino al confine dipende dal resto del capo solo se lì si passa senza fermarsi (Ctx.pt)
        val startPt = prefix.points.size > 1 && prefix.passThrough.last() && prefix.points.last().let { pl ->
            ctx.snap(prefix.graph, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN)?.let { ctx.pt(prefix.graph, true, it) } ?: true }
        val startExact = ctx.router(prefix.graph).searchesExact && prefix.bearings.last() == null && !startPt
        val startInner = if (!startExact || prefix.points.size == 1) 0.0 else ctx.pieceWeight(prefix.graph, request.withPoints(prefix.points, prefix.radiuses.toDoubleArray(), prefix.exact.toBooleanArray(), prefix.bearings, prefix.passThrough.toBooleanArray()))
        val suffixPts = suffix.map { request.coordinates[it] }
        val endPt = suffixPts.size > 1 && request.passThroughAt(suffix.first) && suffixPts[0].let { q ->
            ctx.snap(gEnd, LL(q.first, q.second), radius(suffix.first), false, request.bearings?.getOrNull(suffix.first) ?: Double.NaN)?.let { ctx.pt(gEnd, true, it) } ?: true }
        val endExact = ctx.router(gEnd).searchesExact && request.bearings?.getOrNull(suffix.first) == null && !endPt
        val endInner = if (!endExact || suffixPts.size == 1) 0.0 else ctx.pieceWeight(gEnd, request.withPoints(suffixPts, suffix.map { radius(it) }.toDoubleArray(), suffix.map { false }.toBooleanArray(), suffix.map { request.bearings?.getOrNull(it) },
            suffix.map { request.passThroughAt(it) }.toBooleanArray()))
        if (startInner.isInfinite() || endInner.isInfinite()) return null
        return Ends(startExact, startInner, endExact, endInner)
    }

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
        fun pieceA(x: Transfer) = request.withPoints(prefix.points + (x.lat to x.lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(), (prefix.exact + true).toBooleanArray(), prefix.bearings + null,
            (prefix.passThrough + false).toBooleanArray())
        fun pieceMid(x: Transfer, y: Transfer): DirectionsRequest {
            val ps = ArrayList<Pair<Double, Double>>(); val rs = ArrayList<Double>(); val ex = ArrayList<Boolean>(); val bs = ArrayList<Double?>(); val pt = ArrayList<Boolean>()
            ps += x.lat to x.lon; rs += TRANSFER_RADIUS; ex += true; bs += null; pt += false
            mid?.forEach { i -> ps += request.coordinates[i]; rs += radius(i); ex += false; bs += request.bearings?.getOrNull(i); pt += request.passThroughAt(i) }
            ps += y.lat to y.lon; rs += TRANSFER_RADIUS; ex += true; bs += null; pt += false
            return request.withPoints(ps, rs.toDoubleArray(), ex.toBooleanArray(), bs, pt.toBooleanArray())
        }
        fun pieceEnd(y: Transfer) = request.withPoints(listOf(y.lat to y.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(), listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) },
            (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray())
        // i limiti inferiori
        val m0 = ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false) ?: return null
        val m2 = ctx.router(gEnd).manyToOne(sq, inE, rootIsTarget = true) ?: return null
        val r0 = Reach(ctx, ga, sp, inA, c01, false, m0)
        val r2 = Reach(ctx, gEnd, sq, inE, c12, true, m2)
        // dove i pesi di ricerca ai capi sono quelli dei pezzi (vedi Ends)
        val ends = ends(ctx, prefix, suffix, radius, gEnd) ?: return none
        if (mid == null) return chooseTransit(ctx, ga, gMid, gEnd, c01, c12, inM1, inM2, r0, r2, ::pieceA, ::pieceMid, ::pieceEnd, ends, limit, t0)
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
                mid.map { radius(it) }.toDoubleArray(), mid.map { false }.toBooleanArray(), mid.map { request.bearings?.getOrNull(it) }, mid.map { request.passThroughAt(it) }.toBooleanArray()))
            if (inner.isInfinite()) return none
            lb1 = { x, y -> mA.lowerBound(x) + mB.lowerBound(y) + inner }
            // esatto dove non si passa senza fermarsi (lega i tratti: Ctx.pt) alla prima e all'ultima tappa e senza direzione all'ultima (vale per la partenza da lì)
            if (ctx.router(gMid).searchesExact && !ctx.pt(gMid, request.passThroughAt(mid.first), sm1) && !ctx.pt(gMid, request.passThroughAt(mid.last), sm2) &&
                request.bearings?.getOrNull(mid.last) == null) midWeight = { x, y ->
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
                val a = if (w0[x].isNaN()) r0.lowerBound(x) else w0[x]
                if (a >= best) continue
                for (y in c12.indices) {
                    if (!w0[x].isNaN() && !w2[y].isNaN() && !w1[x][y].isNaN()) continue
                    val k = a + (if (w1[x][y].isNaN()) lb1(x, y) else w1[x][y]) + (if (w2[y].isNaN()) r2.lowerBound(y) else w2[y])
                    if (k < key) { key = k; kx = x; ky = y }
                }
            }
            if (kx < 0 || key >= best) break
            // prima i pezzi ai capi (servono a molte coppie), poi quello di mezzo
            val rest1 = if (w1[kx][ky].isNaN()) lb1(kx, ky) else w1[kx][ky]
            when {
                // prima il limite vero del capo (la sua ricerca va avanti solo fin lì), poi il pezzo
                !r0.isFinal(kx) && w0[kx].isNaN() -> { val rest2 = if (w2[ky].isNaN()) r2.lowerBound(ky) else w2[ky]; r0.advanceFor(kx, best - rest1 - rest2) { lb -> lb + rest1 + rest2 > key + slack(key) } }
                w0[kx].isNaN() -> w0[kx] = if (ends.startExact) ends.startInner + r0.weight(kx) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[kx])) }
                !r2.isFinal(ky) && w2[ky].isNaN() -> r2.advanceFor(ky, best - w0[kx] - rest1) { lb -> w0[kx] + rest1 + lb > key + slack(key) }
                w2[ky].isNaN() -> w2[ky] = if (ends.endExact) ends.endInner + r2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                else -> {
                    w1[kx][ky] = midWeight?.invoke(kx, ky) ?: run { evaluated++; ctx.pieceWeight(gMid, pieceMid(c01[kx], c12[ky])) }
                    val total = w0[kx] + w1[kx][ky] + w2[ky]
                    if (total < best) { best = total; bx = kx; by = ky }
                }
            }
        }
        log?.invoke("    due valichi insieme (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)} fra ${c01.size}×${c12.size} coppie, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; ${ga.id} ${r0.stats}; ${gMid.id} $midStats; ${gEnd.id} ${r2.stats}")
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
     *
     * Con [k] > 1 (fonte V delle varianti) la scelta va avanti oltre il migliore: anche i passaggi
     * col limite sotto [factor] volte il suo peso e sotto il peso del k-esimo luogo già trovato; un
     * passaggio che non può più essere il migliore né migliorare il suo luogo ([Transfers.PLACE]) si
     * salta. Ritorna il migliore e poi un passaggio per luogo, in ordine di peso, al più [k] in tutto.
     * Con [k] = 1 è la scelta di sempre (stessi passi, stesso valico).
     */
    private fun chooseExactTop(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, cands: List<Transfer>, limit: Double,
                               k: Int, factor: Double): List<Choice>? {
        val none = listOf(Choice(Double.POSITIVE_INFINITY, null))
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
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null, (prefix.passThrough + false).toBooleanArray())
        fun pieceB(t: Transfer) = request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) }, (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray())
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
        // il lato lontano con le ricerche mirate per i passaggi lontani dalla sua frontiera (vedi Reach)
        val rFar = if (shortIsB) Reach(ctx, ga, sp, inA, cands, false, mFar) else Reach(ctx, gb, sq, inB, cands, true, mFar)
        // dove i pesi di ricerca sono quelli dei pezzi (vedi Ends): allora niente pezzo, la ricerca va avanti fino al passaggio
        val ends = ends(ctx, prefix, suffix, radius, gb) ?: return none
        val nearExact = if (shortIsB) ends.endExact else ends.startExact; val nearInner = if (shortIsB) ends.endInner else ends.startInner
        val farExact = if (shortIsB) ends.startExact else ends.endExact; val farInner = if (shortIsB) ends.startInner else ends.endInner
        val nearWeight = DoubleArray(cands.size) { Double.NaN }
        val farWeight = DoubleArray(cands.size) { Double.NaN }
        var best = limit; var bestI = -1; var nearCount = 0; var farCount = 0; var farMs = 0L
        // k > 1: i passaggi saltati (il loro luogo ha già un viaggio più leggero del loro limite)
        val skip = BooleanArray(cands.size)
        fun total(j: Int) = nearWeight[j] + farWeight[j]
        fun samePlace(a: Int, b: Int) = DistanceCalc.haversine(cands[a].lat, cands[a].lon, cands[b].lat, cands[b].lon) < Transfers.PLACE
        // un passaggio per luogo fra quelli pesati del tutto, in ordine di peso
        fun places(): List<Int> {
            val out = ArrayList<Int>()
            for (j in cands.indices.filter { !farWeight[it].isNaN() && total(it).isFinite() }.sortedBy { total(it) }) if (out.none { samePlace(it, j) }) out += j
            return out
        }
        // fin dove pesare: il migliore; con k > 1 fino a factor volte e al k-esimo luogo
        fun cut(): Double = if (k == 1) best else minOf(best * factor, places().let { ps -> if (ps.size >= k) total(ps[k - 1]) else Double.POSITIVE_INFINITY })
        // k > 1, oltre il migliore (key ≥ best: con k = 1 la scelta sarebbe finita, il migliore non cambia
        // più): solo per la fonte V, entro il tempo delle varianti; alla scadenza si tiene quel che c'è
        val tb = ctx.topBudget
        var stopped = false
        fun piece(extra: Boolean, g: CountryGraph, r: DirectionsRequest): Double? {
            if (!extra || tb == null) return ctx.pieceWeight(g, r)
            return try { ctx.pieceWeight(g, r, tb) } catch (e: SearchCancelledException) {
                if (e.reason != SearchCancelledException.Reason.DEADLINE || ctx.rc.cancel.isCancelled) throw e
                null
            }
        }
        while (true) {
            // il passaggio col limite più basso del viaggio intero, non ancora pesato dal lato lontano
            var i = -1; var key = Double.POSITIVE_INFINITY
            for (j in cands.indices) if (farWeight[j].isNaN() && !skip[j]) {
                val lb = rFar.lowerBound(j) + if (nearWeight[j].isNaN()) mNear.lowerBound(j) else nearWeight[j]
                if (lb < key) { key = lb; i = j }
            }
            if (i < 0 || key >= cut()) break
            val extra = k > 1 && key >= best
            if (extra && tb != null && tb.isCancelled) { ctx.rc.cancel.check(); stopped = true; break }
            if (k > 1 && key >= best && cands.indices.any { j -> !farWeight[j].isNaN() && total(j) <= key && samePlace(i, j) }) { skip[i] = true; continue }
            if (nearWeight[i].isNaN()) {
                // prima il limite vero del lato vicino (la sua ricerca va avanti solo fin lì), poi il pezzo
                if (!mNear.isFinal(i)) { mNear.advance { mNear.isFinal(i) || rFar.lowerBound(i) + mNear.lowerBound(i) > key }; continue }
                nearWeight[i] = if (nearExact) nearInner + mNear.weight(i) else { nearCount++; piece(extra, near, nearPiece(cands[i])) ?: run { stopped = true; Double.NaN } }
                if (stopped) break
                continue
            }
            // poi il limite vero del lato lontano (una ricerca sola per tutti i passaggi, fin dove serve):
            // i pezzi lontani costano (evita pedaggi: secondi) e così ne serve di solito uno
            if (!rFar.isFinal(i)) { rFar.advanceFor(i, cut() - nearWeight[i]) { lb -> nearWeight[i] + lb > key }; continue }
            val t1 = System.currentTimeMillis()
            farWeight[i] = if (farExact) farInner + rFar.weight(i) else { farCount++; piece(extra, far, farPiece(cands[i])) ?: run { stopped = true; Double.NaN } }
            farMs += System.currentTimeMillis() - t1
            if (stopped) break
            val total = nearWeight[i] + farWeight[i]
            if (total < best) { best = total; bestI = i }
        }
        log?.invoke("    valico scelto: ${if (bestI < 0) "nessuno" else "%.5f,%.5f peso %.0f".format(cands[bestI].lat, cands[bestI].lon, best)} fra ${cands.size} passaggi; " +
            "lato vicino ${near.id} ${mNear.stats}, $nearCount pezzi; lato lontano ${far.id} ${rFar.stats}, $farCount pezzi in $farMs ms; in tutto ${System.currentTimeMillis() - t0} ms")
        fun w(x: Double) = if (x.isFinite()) "%.0f".format(x) else "∞"
        for ((j, t) in cands.withIndex()) if (!nearWeight[j].isNaN() || !farWeight[j].isNaN() || mNear.lowerBound(j) + rFar.lowerBound(j) < best + 600)
            log?.invoke("      %.5f,%.5f r${t.rank}: ${near.id} ${if (nearWeight[j].isNaN()) "≥" + w(mNear.lowerBound(j)) else w(nearWeight[j])}, ${far.id} ${if (farWeight[j].isNaN()) "≥" + w(rFar.lowerBound(j)) else w(farWeight[j])}".format(t.lat, t.lon))
        if (bestI < 0) return none
        val out = arrayListOf(Choice(best, cands[bestI]))
        if (k > 1) {
            // gli altri luoghi entro factor volte il migliore, in ordine di peso
            val kept = arrayListOf(bestI)
            for (j in places()) if (out.size < k && total(j) <= best * factor && kept.none { samePlace(it, j) }) { kept += j; out += Choice(total(j), cands[j]) }
            if (stopped) ctx.topStopped = true
            log?.invoke("varianti: valichi ${out.drop(1).joinToString { "%.5f,%.5f peso %.0f".format(it.transfer!!.lat, it.transfer.lon, it.weight) }.ifEmpty { "nessun altro" }} " +
                "(${skip.count { it }} saltati${if (stopped) "; tempo finito" else ""})")
        }
        return out
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


    companion object {
        /**
         * Le istruzioni al passaggio di confine: via l'arrivo del pezzo prima e la partenza di quello
         * dopo; la strada prosegue (stesso nome: un'istruzione sola) o "Continua dritto su …".
         */
        internal fun joinSteps(before: MutableList<Map<String, Any?>>, after: MutableList<Map<String, Any?>>, coords: List<DoubleArray>, offset: Int,
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

        internal fun round1(v: Double) = Math.round(v * 10) / 10.0

        /** La finestra delle ricerche di limite oltre il passaggio più vicino (vedi [advanceAround]): un decimo più dieci minuti. */
        const val WINDOW_FACTOR = 1.1
        const val WINDOW_SLACK = 600.0
        /** Il raggio di aggancio di un passaggio di confine: il punto è un nodo della strada. */
        const val TRANSFER_RADIUS = 3.0
        /**
         * "Evita" ([shutIn]): i passaggi entro mezz'ora dal punto (senza le strade da evitare); col divieto
         * il punto è ostacolato se uno di questi diventa irraggiungibile o costa più del doppio e un quarto d'ora.
         */
        const val LOCAL_REACH = 1800.0
        const val LOCAL_DETOUR = 2.0
        const val LOCAL_SLACK = 900.0
        /**
         * Quanto sopra il tetto va il limite dimostrato da un percorso mirato senza esito (secondi): molto
         * più degli arrotondamenti delle somme, molto meno di qualsiasi differenza fra due viaggi.
         */
        const val CAP_MARGIN = 1e-6

        /** Varianti tra Paesi: oltre al pezzo più lungo, anche i pezzi di almeno questa quota del viaggio. */
        const val PIECE_SHARE = 0.25
        /** Sotto questo tempo rimasto non si cercano le varianti di un altro pezzo (ms). */
        const val MIN_PIECE_BUDGET_MS = 1_000L

        /** Diagnostica: registra anche la scelta di un valico alla volta (con le sue ricerche) per confronto. */
        @Volatile @JvmStatic var compareOneByOne = false
    }
}
