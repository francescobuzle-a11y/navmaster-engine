package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntHashSet

/**
 * Percorsi alternativi con il metodo dei "plateau" (GraphHopper `AlternativeRoute`, usato da ORS in
 * modalità flessibile): la ricerca bidirezionale esplora oltre il percorso migliore (fino a
 * `weightFactor` volte il suo peso); ogni nodo raggiunto da entrambe le direzioni dà un candidato
 * (percorso in avanti fino al nodo + percorso all'indietro dal nodo). Un candidato è buono se il
 * tratto in cui i due alberi coincidono (il plateau) è lungo, il peso non supera `weightFactor`
 * volte il migliore e condivide con i percorsi già scelti meno di `shareFactor` del suo peso.
 *
 * @param targetCount percorsi totali voluti (migliore compreso), al massimo 3 come in ORS.
 */
class AlternativeRoutes(
    private val algo: BidirectionalDijkstra,
    private val targetCount: Int = 2,
    private val weightFactor: Double = 1.4,
    private val shareFactor: Double = 0.6,
    private val minPlateauFactor: Double = 0.2,
) {
    private class Candidate(val node: Int, val weight: Double, val plateau: Double, var path: SearchPath? = null, var edges: IntHashSet? = null)

    /** Il percorso migliore seguito dalle alternative trovate (al massimo [targetCount] − 1). */
    fun run(): List<SearchPath> {
        if (!algo.run()) return emptyList()
        val best = algo.extractPath() ?: return emptyList()
        val out = ArrayList<SearchPath>()
        out += best
        if (targetCount <= 1) return out
        val bestEdges = IntHashSet(best.edges.size).also { s -> best.edges.forEach { s.add(it) } }
        val chosen = ArrayList<IntHashSet>().also { it += bestEdges }
        val maxWeight = best.weight * weightFactor
        val candidates = ArrayList<Candidate>()
        val fwd = algo.fwd; val bwd = algo.bwd
        for (c in fwd.weight) {
            val node = c.key
            val b = bwd.weightOf(node)
            if (b.isInfinite()) continue
            val total = c.value + b
            if (total > maxWeight || total <= best.weight + 1e-9) continue
            // il plateau: da questo nodo, in avanti lungo l'albero all'indietro finché l'albero in
            // avanti lo segue, e all'indietro lungo l'albero in avanti finché l'albero all'indietro lo segue
            var plateau = 0.0
            var v = node
            while (true) {
                val next = bwd.parentNode.getOrDefault(v, -1)
                if (next < 0 || fwd.parentNode.getOrDefault(next, -2) != v) break
                plateau += bwd.weightOf(v) - bwd.weightOf(next)
                v = next
            }
            v = node
            while (true) {
                val prev = fwd.parentNode.getOrDefault(v, -1)
                if (prev < 0 || bwd.parentNode.getOrDefault(prev, -2) != v) break
                plateau += fwd.weightOf(v) - fwd.weightOf(prev)
                v = prev
            }
            if (plateau < minPlateauFactor * total) continue
            candidates += Candidate(node, total, plateau)
        }
        // i candidati con il plateau più lungo rispetto al peso prima (come GraphHopper: plateau e peso insieme)
        candidates.sortBy { it.weight - it.plateau * 0.5 }
        for (cand in candidates) {
            if (out.size >= targetCount) break
            val path = pathVia(cand.node)
            if (path.isEmpty) continue
            val edges = IntHashSet(path.edges.size).also { s -> path.edges.forEach { s.add(it) } }
            // condivisione con i percorsi già scelti, in peso (gli archi in comune sul peso del candidato)
            var tooSimilar = false
            for (other in chosen) {
                var shared = 0.0
                for (i in path.edges.indices) if (other.contains(path.edges[i])) shared += edgeWeight(path, i)
                if (shared > shareFactor * path.weight) { tooSimilar = true; break }
            }
            if (tooSimilar) continue
            out += path
            chosen += edges
        }
        return out
    }

    private val weightsCache = HashMap<Int, Double>()

    /** Il peso dell'arco i del percorso: differenza dei pesi dell'albero che lo contiene. */
    private fun edgeWeight(path: SearchPath, i: Int): Double {
        val a = path.nodes[i]; val b = path.nodes[i + 1]
        val fa = algo.fwd.weightOf(a); val fb = algo.fwd.weightOf(b)
        if (fa.isFinite() && fb.isFinite() && fb >= fa) return fb - fa
        val ba = algo.bwd.weightOf(a); val bb = algo.bwd.weightOf(b)
        if (ba.isFinite() && bb.isFinite() && ba >= bb) return ba - bb
        return 0.0
    }

    /** Il percorso che passa per [node]: catena in avanti fino al nodo, poi catena all'indietro. */
    private fun pathVia(node: Int): SearchPath {
        val fwd = algo.fwd; val bwd = algo.bwd
        val edges = ArrayList<Int>(); val nodes = ArrayList<Int>()
        var n = node
        while (true) {
            nodes.add(n)
            val e = fwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            edges.add(e); n = fwd.parentNode.get(n)
        }
        edges.reverse(); nodes.reverse()
        n = node
        val seen = IntHashSet(nodes.size).also { s -> nodes.forEach { s.add(it) } }
        while (true) {
            val e = bwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            edges.add(e); n = bwd.parentNode.get(n)
            if (!seen.add(n)) return SearchPath(IntArray(0), IntArray(0), Double.POSITIVE_INFINITY) // un anello: scartato
            nodes.add(n)
        }
        return SearchPath(nodes.toIntArray(), edges.toIntArray(), fwd.weightOf(node) + bwd.weightOf(node))
    }
}
