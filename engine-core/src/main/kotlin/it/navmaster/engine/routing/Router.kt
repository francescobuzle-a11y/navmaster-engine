package it.navmaster.engine.routing

import it.navmaster.engine.ch.CHGraph
import it.navmaster.engine.ch.CoreDijkstra
import it.navmaster.engine.ch.CoreManyToOne
import it.navmaster.engine.ch.DijkstraBidirectionCH
import it.navmaster.engine.index.LocationIndex
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting

/** Un arco percorso in una certa direzione, con la geometria effettivamente usata (anche parziale). */
class PathEdge(val edge: Int, val reverse: Boolean, val baseNode: Int, val adjNode: Int, val geometry: PointList, val distance: Double, val millis: Long, val weight: Double, val name: String)

/** Percorso completo fra due coordinate agganciate alla rete. */
class RoutePath(val edges: List<PathEdge>, val distance: Double, val millis: Long, val weight: Double) {
    val found get() = edges.isNotEmpty() || distance == 0.0

    /** Geometria concatenata senza punti duplicati ai raccordi. */
    fun points(): PointList {
        val pl = PointList()
        for (e in edges) {
            val start = if (pl.size > 0) 1 else 0
            for (i in start until e.geometry.size) pl.add(e.geometry.lat(i), e.geometry.lon(i), e.geometry.ele(i))
        }
        return pl
    }
}

class RouteNotFoundException(message: String) : RuntimeException(message)
class PointNotFoundException(message: String, val pointIndex: Int) : RuntimeException(message)

/**
 * Calcola un percorso fra coordinate: aggancio, ricerca bidirezionale fra le torri degli archi
 * agganciati con i costi dei tratti parziali, ricostruzione della geometria.
 *
 * Un punto a metà arco viene gestito senza nodi virtuali: la ricerca parte (o arriva) da entrambe
 * le torri dell'arco con il costo del tratto parziale, e il caso "stesso arco" è valutato a parte.
 */
class Router(
    private val graph: BaseGraph,
    private val index: LocationIndex,
    private val weighting: Weighting,
    private val filter: EdgeFilter = EdgeFilter.ALL,
    /** Se presente, la ricerca è edge-based e rispetta divieti di svolta e inversioni a U. */
    private val turnCosts: TurnCostProvider? = null,
    /** Se presente, la ricerca usa la Contraction Hierarchy (ignora filtri e costi di svolta, come la modalità "static" di ORS). */
    private val ch: CHGraph? = null,
    /** Se presente (e [ch] è assente), la ricerca usa il Core: CH fuori dal core, filtri e costi di svolta dentro. */
    private val core: CHGraph? = null,
    /** Landmark del core: la fase 2 diventa un A* bidirezionale (core-ALT). */
    private val landmarks: it.navmaster.engine.ch.CoreLandmarks? = null,
    private val maxVisitedNodes: Int = Int.MAX_VALUE,
    /** The times of the path (normally the weighting's; with maximumSpeedTimeOnly the limited speed). */
    private val timeWeighting: Weighting = weighting,
    /** The quick search to try first (see RoutingGraph.router: avoid_features), and what its path must pass. */
    private val quick: Router? = null,
    private val quickAccepts: EdgeFilter? = null,
) {
    /** Quale algoritmo verrà usato: utile per diagnostica e test. */
    /** The phases of the last core search (diagnostics). */
    var lastStats = ""; private set

    /** The same, of the quick search when there is one. */
    fun quickStats(): String = quick?.let { "rapida ${it.algorithm}: ${it.lastStats}" } ?: lastStats

    val algorithm: String get() = when {
        ch != null -> "ch"
        core != null -> (if (landmarks != null) "core_alt" else "core") + (if (turnCosts != null) "_edge_based" else "")
        turnCosts != null -> "dijkstra_edge_based"
        else -> "dijkstra"
    }
    private val it = graph.createEdgeIterator()

    /**
     * L'aggancio di una coordinata, come il `DefaultSnapFilter` di ORS: solo archi che il profilo può
     * percorrere in almeno un verso e che non stanno in una piccola rete isolata (ORS
     * `min_network_size`: 200), cioè da cui si raggiungono e da cui sono raggiungibili almeno
     * [MIN_NETWORK_SIZE] nodi. Altrimenti un punto in una zona pedonale o su un tratto staccato
     * dal resto delle strade non avrebbe nessun percorso.
     */
    fun snap(lat: Double, lon: Double, maxDistance: Double, pointIndex: Int, checkNetwork: Boolean = true, heading: Double = Double.NaN): Snap {
        val excluded = HashSet<Int>()
        val snapFilter = EdgeFilter { e ->
            e.edge !in excluded && filter.accept(e) && (weighting.edgeWeight(e, false).isFinite() || weighting.edgeWeight(e, true).isFinite())
        }
        // GraphHopper `HeadingEdgeFilter` (ORS bearings): first a road within 20 m that goes the
        // given way (30°) in a direction the vehicle may drive; else the nearest road, as without
        // (a lorry on the motorway: its own carriageway, not the opposite one a few metres nearer)
        if (!heading.isNaN()) {
            val withHeading = EdgeFilter { e -> snapFilter.accept(e) && headingMatches(e, lat, lon, heading) }
            for (attempt in 0 until 4) {
                val s = index.findClosest(lat, lon, withHeading)
                if (!s.isValid || s.distance > minOf(maxDistance, HEADING_SNAP_DISTANCE)) break
                if (!checkNetwork) return s
                val small = smallNetwork(s.edge) ?: return s
                excluded.addAll(small)
            }
        }
        for (attempt in 0 until 8) {
            val s = index.findClosest(lat, lon, snapFilter)
            if (!s.isValid || s.distance > maxDistance) break
            if (!checkNetwork) return s
            val small = smallNetwork(s.edge) ?: return s
            excluded.addAll(small)
        }
        throw PointNotFoundException("Nessuna strada percorribile entro %.0f m dal punto %d (%.6f, %.6f)".format(maxDistance, pointIndex, lon, lat), pointIndex)
    }

    /** L'arco, vicino al punto (20 m), va nella direzione [heading] (30°) in un verso percorribile. */
    private fun headingMatches(e: it.navmaster.engine.storage.BaseGraph.EdgeIterator, lat: Double, lon: Double, heading: Double): Boolean {
        val g = graph.fetchWayGeometry(e.edge, false, FetchMode.ALL)
        val k = Math.cos(Math.toRadians(lat))
        var best = HEADING_SNAP_DISTANCE
        var bearing = Double.NaN
        for (i in 0 until g.size - 1) {
            val ax = (g.lon(i) - lon) * k * DistanceCalc.METERS_PER_DEGREE; val ay = (g.lat(i) - lat) * DistanceCalc.METERS_PER_DEGREE
            val bx = (g.lon(i + 1) - lon) * k * DistanceCalc.METERS_PER_DEGREE; val by = (g.lat(i + 1) - lat) * DistanceCalc.METERS_PER_DEGREE
            val dx = bx - ax; val dy = by - ay
            val l2 = dx * dx + dy * dy
            val t = if (l2 > 0) (-(ax * dx + ay * dy) / l2).coerceIn(0.0, 1.0) else 0.0
            val d = Math.hypot(ax + dx * t, ay + dy * t)
            if (d < best) { best = d; bearing = DistanceCalc.bearing(g.lat(i), g.lon(i), g.lat(i + 1), g.lon(i + 1)) }
        }
        if (bearing.isNaN()) return false
        // il verso A→B dell'arco rispetto all'iteratore
        val aToB = e.baseNode == graph.nodeA(e.edge)
        val forwardOk = weighting.edgeWeight(e, !aToB).isFinite()
        val backwardOk = weighting.edgeWeight(e, aToB).isFinite()
        return (angle(bearing, heading) < HEADING_SNAP_TOLERANCE && forwardOk) || (angle((bearing + 180.0) % 360.0, heading) < HEADING_SNAP_TOLERANCE && backwardOk)
    }

    /** Gli archi della piccola rete isolata di [edge], o null quando l'arco è nella rete principale. */
    private fun smallNetwork(edge: Int): Set<Int>? {
        val fwd = explore(edge, forward = true)
        if (fwd != null) return fwd
        return explore(edge, forward = false)
    }

    /** Gli archi raggiunti da [edge] (o che lo raggiungono), o null appena si superano [MIN_NETWORK_SIZE] nodi. */
    private fun explore(edge: Int, forward: Boolean): Set<Int>? {
        val e = graph.createEdgeIterator()
        val visited = HashSet<Int>()
        val edges = HashSet<Int>().also { it += edge }
        val queue = ArrayDeque<Int>()
        for (n in intArrayOf(graph.nodeA(edge), graph.nodeB(edge))) if (visited.add(n)) queue.addLast(n)
        while (queue.isNotEmpty()) {
            val n = queue.removeFirst()
            e.setBaseNode(n)
            while (e.next()) {
                if (!filter.accept(e)) continue
                // avanti: da n verso il nodo adiacente; indietro: dal nodo adiacente verso n
                if (weighting.edgeWeight(e, !forward).isInfinite()) continue
                edges += e.edge
                if (visited.add(e.adjNode)) {
                    // in un grafo piccolo (prove) la rete principale è quella con almeno metà dei nodi
                    if (visited.size >= minOf(MIN_NETWORK_SIZE, maxOf(2, graph.nodeCount / 2))) return null
                    queue.addLast(e.adjNode)
                }
            }
        }
        return edges
    }

    companion object {
        /** ORS `min_network_size`. */
        const val MIN_NETWORK_SIZE = 200
        /** GraphHopper `heading_penalty` (ORS `DEFAULT_HEADING_PENALTY`): 300 s. */
        const val HEADING_PENALTY = 300.0
        /** Oltre questa differenza (gradi) un tratto non va nella direzione chiesta (GraphHopper: 100°). */
        const val HEADING_TOLERANCE = 100.0
        /** GraphHopper `HeadingEdgeFilter`: entro 30° e 20 m. */
        const val HEADING_SNAP_TOLERANCE = 30.0
        const val HEADING_SNAP_DISTANCE = 20.0
    }

    /**
     * Percorso fra due punti agganciati. [arrival]: con `continue_straight`, l'arco (e verso) con cui
     * il tratto precedente è arrivato in [from]: il nuovo tratto riparte nello stesso verso, senza
     * invertire sull'arco. Per un aggancio esatto su una torre l'arco d'arrivo è già alle spalle e
     * nessuna parte viene esclusa.
     */
    /**
     * [maxWeight]: only a path lighter than this (none found otherwise): a search that cannot beat
     * a known path stops there instead of exploring all it can reach. [exact] false: the quick
     * search's path is taken even when it uses a road to avoid (an estimate, for choosing).
     */
    fun route(from: Snap, to: Snap, arrival: PathEdge? = null, fromHeading: Double = Double.NaN, toHeading: Double = Double.NaN,
              maxWeight: Double = Double.POSITIVE_INFINITY, exact: Boolean = true): RoutePath {
        if (quick != null && quickAccepts != null) {
            val p = try { quick.route(from, to, arrival, fromHeading, toHeading, maxWeight) } catch (e: RouteNotFoundException) { null }
            if (p != null && (!exact || p.edges.all { e -> it.setEdge(e.edge, e.baseNode); quickAccepts.accept(it) })) return p
            // no path even without the filter (or none under the limit): none with it either
            if (p == null) throw RouteNotFoundException("Nessun percorso trovato fra i punti richiesti")
        }
        var fromParts = splitEdge(from, asTarget = false).map { penalised(it, from, fromHeading, asTarget = false) }
        val toParts = splitEdge(to, asTarget = true).map { penalised(it, to, toHeading, asTarget = true) }
        var best: RoutePath? = null
        val keepDirection = arrival != null && from.position != SnapPosition.TOWER && arrival.edge == from.edge
        if (keepDirection) {
            val same = fromParts.filter { part -> part.reverse == arrival!!.reverse }
            if (same.isNotEmpty()) fromParts = same
        }

        // caso speciale: stesso arco, tratto diretto fra i due punti
        if (from.edge == to.edge && from.position != SnapPosition.TOWER || (from.edge == to.edge && to.position != SnapPosition.TOWER)) {
            best = sameEdgeRoute(from, to)
            // la direzione chiesta (ORS bearings) vale anche per il tratto diretto sullo stesso arco
            if (best != null && best.edges.isNotEmpty() && (!fromHeading.isNaN() || !toHeading.isNaN())) {
                val dir = travelBearing(from, !best.edges[0].reverse)
                val bad = (!fromHeading.isNaN() && angle(dir, fromHeading) > HEADING_TOLERANCE) || (!toHeading.isNaN() && angle(dir, toHeading) > HEADING_TOLERANCE)
                if (bad) best = RoutePath(best.edges, best.distance, best.millis, best.weight + HEADING_PENALTY)
            }
            // continue_straight: il tratto diretto vale solo se prosegue nel verso d'arrivo
            if (keepDirection && best != null && best.edges.isNotEmpty() && best.edges[0].reverse != arrival!!.reverse) best = null
        }

        val sp = search(fromParts.map { it.node to it.weight }, toParts.map { it.node to it.weight }, maxWeight)
        if (sp != null) {
            val startPart = fromParts.first { it.node == sp.startNode }
            val endPart = toParts.first { it.node == sp.endNode }
            val candidate = assemble(sp, startPart, endPart)
            if (best == null || candidate.weight < best.weight) best = candidate
        }
        return best ?: throw RouteNotFoundException("Nessun percorso trovato fra i punti richiesti")
    }

    /**
     * I pesi fra [root] e ciascuno di [others] trovati insieme (vedi [CoreManyToOne]):
     * dagli altri alla radice quando [rootIsTarget], altrimenti dalla radice agli altri. Gli stessi
     * pesi di [route] (con la ricerca rapida quando c'è, come `route(exact = false)`); null quando
     * questo router non cerca sul core.
     */
    fun manyToOne(root: Snap, others: List<Snap>, rootIsTarget: Boolean): CoreManyToOne? {
        if (quick != null && quickAccepts != null) return quick.manyToOne(root, others, rootIsTarget)
        val c = core ?: return null
        val m = CoreManyToOne(c, weighting, filter, turnCosts, landmarks, rootIsTarget, others.size)
        for (p in splitEdge(root, asTarget = rootIsTarget)) m.addRoot(p.node, p.weight)
        for ((i, s) in others.withIndex()) {
            // sullo stesso arco della radice vale anche il tratto diretto (come in route)
            if (s.edge == root.edge && (s.position != SnapPosition.TOWER || root.position != SnapPosition.TOWER)) {
                (if (rootIsTarget) sameEdgeRoute(s, root) else sameEdgeRoute(root, s))?.let { m.addDirect(i, it.weight) }
            }
            for (p in splitEdge(s, asTarget = !rootIsTarget)) m.addOther(i, p.node, p.weight)
        }
        m.prepare()
        return m
    }

    /** La ricerca fra i nodi dati (coi pesi iniziali) con l'algoritmo del router. */
    private fun search(sources: List<Pair<Int, Double>>, targets: List<Pair<Int, Double>>, maxWeight: Double): SearchPath? = if (ch != null) {
        val algo = DijkstraBidirectionCH(ch, weighting)
        if (maxWeight.isFinite()) algo.bestWeight = maxWeight
        for ((n, w) in sources) algo.addSource(n, w)
        for ((n, w) in targets) algo.addTarget(n, w)
        if (algo.run()) algo.extractPath() else null
    } else if (core != null) {
        val algo = CoreDijkstra(core, weighting, filter, turnCosts, landmarks)
        if (maxWeight.isFinite()) algo.bestWeight = maxWeight
        for ((n, w) in sources) algo.addSource(n, w)
        for ((n, w) in targets) algo.addTarget(n, w)
        val found = algo.run()
        lastStats = algo.stats
        if (found) algo.extractPath() else null
    } else if (turnCosts != null) {
        val algo = EdgeBasedBidirectionalDijkstra(graph, weighting, turnCosts, filter, maxVisitedNodes)
        if (maxWeight.isFinite()) algo.bestWeight = maxWeight
        for ((n, w) in sources) algo.addSource(n, w)
        for ((n, w) in targets) algo.addTarget(n, w)
        if (algo.run()) algo.extractPath() else null
    } else {
        val algo = BidirectionalDijkstra(graph, weighting, filter, maxVisitedNodes)
        if (maxWeight.isFinite()) algo.bestWeight = maxWeight
        for ((n, w) in sources) algo.addSource(n, w)
        for ((n, w) in targets) algo.addTarget(n, w)
        if (algo.run()) algo.extractPath() else null
    }

    /**
     * Il percorso migliore e fino a [targetCount] − 1 alternative (metodo dei plateau, vedi
     * [AlternativeRoutes]) con la ricerca bidirezionale node-based piena, come la modalità
     * flessibile di ORS: niente CH, costi di svolta non considerati.
     */
    fun alternatives(from: Snap, to: Snap, targetCount: Int, weightFactor: Double, shareFactor: Double): List<RoutePath> {
        // il percorso migliore con l'algoritmo normale (divieti di svolta compresi)
        val best = route(from, to)
        val out = ArrayList<RoutePath>().also { it += best }
        if (targetCount <= 1) return out
        val fromParts = splitEdge(from, asTarget = false)
        val toParts = splitEdge(to, asTarget = true)
        // le alternative dalla ricerca piena (edge-based con i divieti di svolta, altrimenti node-based):
        // valgono solo se non ricalcano il percorso migliore
        val candidates = if (turnCosts != null) {
            val algo = EdgeBasedBidirectionalDijkstra(graph, weighting, turnCosts, filter, maxVisitedNodes, explorationFactor = weightFactor)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            EdgeBasedAlternatives(algo, targetCount + 2, weightFactor, shareFactor).run()
        } else {
            val algo = BidirectionalDijkstra(graph, weighting, filter, maxVisitedNodes, explorationFactor = weightFactor)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            AlternativeRoutes(algo, targetCount + 2, weightFactor, shareFactor).run()
        }
        val bestEdges = best.edges.map { it.edge }.toHashSet()
        for (sp in candidates) {
            if (out.size >= targetCount) break
            val startPart = fromParts.firstOrNull { it.node == sp.startNode } ?: continue
            val endPart = toParts.firstOrNull { it.node == sp.endNode } ?: continue
            val path = assemble(sp, startPart, endPart)
            if (!isLegal(path)) continue
            if (path.weight > best.weight * weightFactor) continue
            var shared = 0.0
            for (e in path.edges) if (e.edge in bestEdges) shared += e.distance
            if (shared > shareFactor * path.distance) continue
            if (out.drop(1).any { o -> val oe = o.edges.map { it.edge }.toHashSet(); path.edges.sumOf { e -> if (e.edge in oe) e.distance else 0.0 } > shareFactor * path.distance }) continue
            out += path
        }
        return out
    }

    /** Vero se ogni svolta del percorso è consentita (con i costi di svolta del profilo; sempre vero senza). */
    fun isLegal(path: RoutePath): Boolean {
        val tc = turnCosts ?: return true
        for (i in 1 until path.edges.size) {
            val prev = path.edges[i - 1]; val cur = path.edges[i]
            if (prev.edge == cur.edge) continue
            if (tc.turnWeight(prev.edge, prev.adjNode, cur.edge).isInfinite()) return false
        }
        return true
    }

    /**
     * ORS `bearings` (GraphHopper `heading`): un tratto iniziale che parte (o finale che arriva) in
     * una direzione diversa di oltre [HEADING_TOLERANCE] gradi da quella chiesta costa
     * [HEADING_PENALTY] in più (`heading_penalty`, 300 s): si parte nel verso del mezzo, salvo non
     * ci sia altra via.
     */
    private fun penalised(p: EdgePart, s: Snap, heading: Double, asTarget: Boolean): EdgePart {
        if (heading.isNaN() || s.position == SnapPosition.TOWER) return p
        // la direzione di marcia sul tratto: verso B (A→B) se la torre è B, verso A altrimenti; un
        // arrivo percorre il tratto dalla torre verso il punto (il verso opposto)
        val towardB = (p.node == graph.nodeB(s.edge)) != asTarget
        val dir = travelBearing(s, towardB)
        return if (angle(dir, heading) > HEADING_TOLERANCE) EdgePart(p.node, p.weight + HEADING_PENALTY, p.distance, p.millis, p.geometry, p.edge, p.reverse) else p
    }

    /** La direzione (gradi) del segmento dell'arco su cui sta [s], nel verso A→B ([towardB]) o B→A. */
    private fun travelBearing(s: Snap, towardB: Boolean): Double {
        val all = graph.fetchWayGeometry(s.edge, reverse = false, FetchMode.ALL)
        val i = s.wayIndex.coerceIn(0, maxOf(0, all.size - 2))
        if (all.size < 2) return 0.0
        val b = DistanceCalc.bearing(all.lat(i), all.lon(i), all.lat(i + 1), all.lon(i + 1))
        return if (towardB) b else (b + 180.0) % 360.0
    }

    private fun angle(a: Double, b: Double): Double = Math.abs(((a - b + 540.0) % 360.0) - 180.0)

    /** Tratto parziale di un arco agganciato: dal punto agganciato alla torre [node] (o viceversa). */
    private class EdgePart(val node: Int, val weight: Double, val distance: Double, val millis: Long, val geometry: PointList, val edge: Int, val reverse: Boolean)

    /**
     * Le due estremità raggiungibili da un punto agganciato. Per uno snap su torre c'è un solo
     * elemento a costo zero. Per uno snap a metà arco: verso nodeB nella direzione A→B e verso
     * nodeA nella direzione inversa, ciascuna con peso proporzionale alla frazione di arco.
     * [asTarget]: il punto è un arrivo, quindi i tratti parziali vengono percorsi verso lo snap
     * (dalla torre B in direzione B→A con il peso inverso, dalla torre A in direzione A→B): conta per
     * i sensi unici. `reverse` resta "la torre della parte è A".
     */
    private fun splitEdge(s: Snap, asTarget: Boolean = false): List<EdgePart> {
        if (s.position == SnapPosition.TOWER) return listOf(EdgePart(s.closestNode, 0.0, 0.0, 0L, PointList().also { it.add(graph.lat(s.closestNode), graph.lon(s.closestNode)) }, s.edge, false))
        val a = graph.nodeA(s.edge); val b = graph.nodeB(s.edge)
        val all = graph.fetchWayGeometry(s.edge, reverse = false, FetchMode.ALL)
        val total = graph.distance(s.edge)
        // geometria A→snap e snap→B
        val toSnap = PointList(); val fromSnap = PointList()
        for (i in 0..s.wayIndex) toSnap.add(all.lat(i), all.lon(i))
        toSnap.add(s.snappedLat, s.snappedLon)
        fromSnap.add(s.snappedLat, s.snappedLon)
        for (i in s.wayIndex + 1 until all.size) fromSnap.add(all.lat(i), all.lon(i))
        val dA = length(toSnap); val dB = length(fromSnap)
        val fracA = if (total > 0) dA / total else 0.5; val fracB = if (total > 0) dB / total else 0.5
        it.setEdge(s.edge, a)
        val wFwd = weighting.edgeWeight(it, false); val wBwd = weighting.edgeWeight(it, true)
        val msFwd = timeWeighting.edgeMillis(it, false); val msBwd = timeWeighting.edgeMillis(it, true)
        val parts = ArrayList<EdgePart>(2)
        // partenza: verso B percorrendo A→B (peso diretto); arrivo: da B verso lo snap percorrendo B→A (peso inverso)
        val wToB = if (asTarget) wBwd else wFwd; val msToB = if (asTarget) msBwd else msFwd
        if (!wToB.isInfinite()) parts += EdgePart(b, wToB * fracB, dB, (msToB * fracB).toLong(), fromSnap, s.edge, false)
        // partenza: verso A percorrendo B→A (peso inverso); arrivo: da A verso lo snap percorrendo A→B (peso diretto)
        val wToA = if (asTarget) wFwd else wBwd; val msToA = if (asTarget) msFwd else msBwd
        if (!wToA.isInfinite()) parts += EdgePart(a, wToA * fracA, dA, (msToA * fracA).toLong(), toSnap, s.edge, true)
        if (parts.isEmpty()) {
            // arco non percorribile in nessuna direzione per questo peso: la ricerca parte comunque dalle torri a costo zero
            parts += EdgePart(a, 0.0, dA, 0L, toSnap, s.edge, true)
            parts += EdgePart(b, 0.0, dB, 0L, fromSnap, s.edge, false)
        }
        return parts
    }

    private fun length(pl: PointList): Double {
        var d = 0.0
        for (i in 1 until pl.size) d += DistanceCalc.haversine(pl.lat(i - 1), pl.lon(i - 1), pl.lat(i), pl.lon(i))
        return d
    }

    private fun sameEdgeRoute(from: Snap, to: Snap): RoutePath? {
        val all = graph.fetchWayGeometry(from.edge, reverse = false, FetchMode.ALL)
        val total = graph.distance(from.edge)
        it.setEdge(from.edge, graph.nodeA(from.edge))
        // posizione lungo l'arco (indice del segmento + frazione) per decidere il verso
        val posFrom = alongPosition(all, from); val posTo = alongPosition(all, to)
        if (posFrom == posTo) return RoutePath(emptyList(), 0.0, 0L, 0.0)
        val forward = posTo > posFrom
        val w = weighting.edgeWeight(it, !forward)
        if (w.isInfinite()) return null
        val geom = PointList()
        geom.add(from.snappedLat, from.snappedLon)
        if (forward) for (i in from.wayIndex + 1..to.wayIndex) geom.add(all.lat(i), all.lon(i))
        else for (i in from.wayIndex downTo to.wayIndex + 1) geom.add(all.lat(i), all.lon(i))
        geom.add(to.snappedLat, to.snappedLon)
        val d = length(geom)
        val frac = if (total > 0) d / total else 1.0
        val ms = (timeWeighting.edgeMillis(it, !forward) * frac).toLong()
        val base = if (forward) graph.nodeA(from.edge) else graph.nodeB(from.edge)
        val adj = if (forward) graph.nodeB(from.edge) else graph.nodeA(from.edge)
        return RoutePath(listOf(PathEdge(from.edge, !forward, base, adj, geom, d, ms, w * frac, graph.name(from.edge))), d, ms, w * frac)
    }

    private fun alongPosition(all: PointList, s: Snap): Double {
        if (s.position == SnapPosition.TOWER) return if (s.closestNode == graph.nodeA(s.edge)) 0.0 else (all.size - 1).toDouble()
        val i = s.wayIndex
        if (i >= all.size - 1) return i.toDouble()
        val segLen = DistanceCalc.planeDistance(all.lat(i), all.lon(i), all.lat(i + 1), all.lon(i + 1))
        val part = DistanceCalc.planeDistance(all.lat(i), all.lon(i), s.snappedLat, s.snappedLon)
        return i + if (segLen > 0) (part / segLen).coerceIn(0.0, 1.0) else 0.0
    }

    private fun assemble(sp: SearchPath, start: EdgePart, end: EdgePart): RoutePath {
        val edges = ArrayList<PathEdge>(sp.edges.size + 2)
        var distance = 0.0; var millis = 0L; var weight = 0.0
        if (start.geometry.size > 1) {
            val base = if (start.reverse) graph.nodeB(start.edge) else graph.nodeA(start.edge)
            edges += PathEdge(start.edge, start.reverse, base, start.node, start.geometry.also { if (start.reverse) it.reverse() }, start.distance, start.millis, start.weight, graph.name(start.edge))
            distance += start.distance; millis += start.millis; weight += start.weight
        }
        for (i in sp.edges.indices) {
            val e = sp.edges[i]; val base = sp.nodes[i]
            it.setEdge(e, base)
            val w = weighting.edgeWeight(it, false); val ms = timeWeighting.edgeMillis(it, false)
            edges += PathEdge(e, it.reverse, base, it.adjNode, it.fetchWayGeometry(FetchMode.ALL), it.distance, ms, w, it.name)
            distance += it.distance; millis += ms; weight += w
        }
        if (end.geometry.size > 1) {
            // il tratto finale va dalla torre al punto agganciato
            val geom = end.geometry
            if (!end.reverse) geom.reverse() // fromSnap era snap→B: lo percorriamo B→snap
            val adj = if (end.reverse) graph.nodeB(end.edge) else graph.nodeA(end.edge)
            edges += PathEdge(end.edge, !end.reverse, end.node, adj, geom, end.distance, end.millis, end.weight, graph.name(end.edge))
            distance += end.distance; millis += end.millis; weight += end.weight
        }
        return RoutePath(edges, distance, millis, weight)
    }
}
