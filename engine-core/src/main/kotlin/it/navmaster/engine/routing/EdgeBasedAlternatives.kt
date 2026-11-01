package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntHashSet

/**
 * Percorsi alternativi con il metodo dei plateau sulla ricerca **edge-based** (vedi
 * [AlternativeRoutes] per l'idea): lo stato è la chiave d'arco, quindi i candidati rispettano
 * divieti di svolta e inversioni. Un candidato nasce da una chiave raggiunta da entrambe le
 * direzioni: percorso in avanti fino all'arco + percorso all'indietro da esso.
 */
class EdgeBasedAlternatives(
    private val algo: EdgeBasedBidirectionalDijkstra,
    private val targetCount: Int = 2,
    private val weightFactor: Double = 1.4,
    private val shareFactor: Double = 0.6,
    private val minPlateauFactor: Double = 0.2,
) {
    private class Candidate(val key: Int, val weight: Double, val plateau: Double)

    /** Il percorso migliore seguito dalle alternative trovate (al massimo [targetCount] − 1). */
    fun run(): List<SearchPath> {
        if (!algo.run()) return emptyList()
        val best = algo.extractPath() ?: return emptyList()
        val out = ArrayList<SearchPath>()
        out += best
        if (targetCount <= 1) return out
        val chosen = ArrayList<IntHashSet>().also { it += IntHashSet(best.edges.size).also { s -> best.edges.forEach { e -> s.add(e) } } }
        val maxWeight = best.weight * weightFactor
        val fwd = algo.fwd; val bwd = algo.bwd
        val candidates = ArrayList<Candidate>()
        for (c in fwd.weight) {
            val key = c.key
            val b = bwd.weightOf(key)
            if (b.isInfinite()) continue
            val total = c.value + b - algo.edgeWeightOf(key)
            if (total > maxWeight || total <= best.weight + 1e-9) continue
            var plateau = 0.0
            var k = key
            while (true) {
                if (bwd.parentEdge.getOrDefault(k, -1) < 0) break
                val next = bwd.parentNode.get(k)
                if (fwd.parentEdge.getOrDefault(next, -1) < 0 || fwd.parentNode.get(next) != k) break
                plateau += bwd.weightOf(k) - bwd.weightOf(next)
                k = next
            }
            k = key
            while (true) {
                if (fwd.parentEdge.getOrDefault(k, -1) < 0) break
                val prev = fwd.parentNode.get(k)
                if (bwd.parentEdge.getOrDefault(prev, -1) < 0 || bwd.parentNode.get(prev) != k) break
                plateau += fwd.weightOf(k) - fwd.weightOf(prev)
                k = prev
            }
            if (plateau < minPlateauFactor * total) continue
            candidates += Candidate(key, total, plateau)
        }
        candidates.sortBy { it.weight - it.plateau * 0.5 }
        val seenPaths = HashSet<Int>()
        for (cand in candidates) {
            if (out.size >= targetCount) break
            val path = algo.pathVia(cand.key) ?: continue
            if (path.isEmpty || !seenPaths.add(path.edges.contentHashCode())) continue
            val edges = IntHashSet(path.edges.size).also { s -> path.edges.forEach { e -> s.add(e) } }
            var tooSimilar = false
            for (other in chosen) {
                var shared = 0.0
                for (i in path.edges.indices) if (other.contains(path.edges[i])) shared += algo.edgeWeightOfEdge(path.edges[i])
                if (shared > shareFactor * path.weight) { tooSimilar = true; break }
            }
            if (tooSimilar) continue
            out += path
            chosen += edges
        }
        return out
    }
}
