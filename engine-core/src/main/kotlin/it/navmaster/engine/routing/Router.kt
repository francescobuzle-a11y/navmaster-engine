package it.navmaster.engine.routing

import it.navmaster.engine.ch.CHGraph
import it.navmaster.engine.ch.CoreDijkstra
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
) {
    /** Quale algoritmo verrà usato: utile per diagnostica e test. */
    val algorithm: String get() = when {
        ch != null -> "ch"
        core != null -> (if (landmarks != null) "core_alt" else "core") + (if (turnCosts != null) "_edge_based" else "")
        turnCosts != null -> "dijkstra_edge_based"
        else -> "dijkstra"
    }
    private val it = graph.createEdgeIterator()

    fun snap(lat: Double, lon: Double, maxDistance: Double, pointIndex: Int): Snap {
        val s = index.findClosest(lat, lon, filter)
        if (!s.isValid || s.distance > maxDistance) {
            throw PointNotFoundException("Nessuna strada percorribile entro %.0f m dal punto %d (%.6f, %.6f)".format(maxDistance, pointIndex, lon, lat), pointIndex)
        }
        return s
    }

    /**
     * Percorso fra due punti agganciati. [arrival]: con `continue_straight`, l'arco (e verso) con cui
     * il tratto precedente è arrivato in [from]: il nuovo tratto riparte nello stesso verso, senza
     * invertire sull'arco. Per un aggancio esatto su una torre l'arco d'arrivo è già alle spalle e
     * nessuna parte viene esclusa.
     */
    fun route(from: Snap, to: Snap, arrival: PathEdge? = null): RoutePath {
        var fromParts = splitEdge(from, asTarget = false)
        val toParts = splitEdge(to, asTarget = true)
        var best: RoutePath? = null
        val keepDirection = arrival != null && from.position != SnapPosition.TOWER && arrival.edge == from.edge
        if (keepDirection) {
            val same = fromParts.filter { part -> part.reverse == arrival!!.reverse }
            if (same.isNotEmpty()) fromParts = same
        }

        // caso speciale: stesso arco, tratto diretto fra i due punti
        if (from.edge == to.edge && from.position != SnapPosition.TOWER || (from.edge == to.edge && to.position != SnapPosition.TOWER)) {
            best = sameEdgeRoute(from, to)
            // continue_straight: il tratto diretto vale solo se prosegue nel verso d'arrivo
            if (keepDirection && best != null && best.edges.isNotEmpty() && best.edges[0].reverse != arrival!!.reverse) best = null
        }

        val sp: SearchPath? = if (ch != null) {
            val algo = DijkstraBidirectionCH(ch, weighting)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            if (algo.run()) algo.extractPath() else null
        } else if (core != null) {
            val algo = CoreDijkstra(core, weighting, filter, turnCosts, landmarks)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            if (algo.run()) algo.extractPath() else null
        } else if (turnCosts != null) {
            val algo = EdgeBasedBidirectionalDijkstra(graph, weighting, turnCosts, filter, maxVisitedNodes)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            if (algo.run()) algo.extractPath() else null
        } else {
            val algo = BidirectionalDijkstra(graph, weighting, filter, maxVisitedNodes)
            for (p in fromParts) algo.addSource(p.node, p.weight)
            for (p in toParts) algo.addTarget(p.node, p.weight)
            if (algo.run()) algo.extractPath() else null
        }
        if (sp != null) {
            val startPart = fromParts.first { it.node == sp.startNode }
            val endPart = toParts.first { it.node == sp.endNode }
            val candidate = assemble(sp, startPart, endPart)
            if (best == null || candidate.weight < best.weight) best = candidate
        }
        return best ?: throw RouteNotFoundException("Nessun percorso trovato fra i punti richiesti")
    }

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
        val msFwd = weighting.edgeMillis(it, false); val msBwd = weighting.edgeMillis(it, true)
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
        val ms = (weighting.edgeMillis(it, !forward) * frac).toLong()
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
            val w = weighting.edgeWeight(it, false); val ms = weighting.edgeMillis(it, false)
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
