package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntDoubleHashMap
import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.Weighting

/** Heap binario di (nodo, chiave) senza boxing, con cancellazione pigra. */
class NodeHeap(initialCapacity: Int = 1024) {
    private var nodes = IntArray(initialCapacity)
    private var keys = DoubleArray(initialCapacity)
    var size = 0; private set

    val isEmpty get() = size == 0
    fun peekKey() = keys[0]
    fun peekNode() = nodes[0]

    fun push(node: Int, key: Double) {
        if (size == nodes.size) { nodes = nodes.copyOf(size * 2); keys = keys.copyOf(size * 2) }
        var i = size++
        while (i > 0) {
            val parent = (i - 1) ushr 1
            if (keys[parent] <= key) break
            nodes[i] = nodes[parent]; keys[i] = keys[parent]; i = parent
        }
        nodes[i] = node; keys[i] = key
    }

    fun pop(): Int {
        val top = nodes[0]
        size--
        if (size > 0) {
            val n = nodes[size]; val k = keys[size]
            var i = 0
            while (true) {
                var child = 2 * i + 1
                if (child >= size) break
                if (child + 1 < size && keys[child + 1] < keys[child]) child++
                if (keys[child] >= k) break
                nodes[i] = nodes[child]; keys[i] = keys[child]; i = child
            }
            nodes[i] = n; keys[i] = k
        }
        return top
    }

    fun clear() { size = 0 }
}

/** Stato di una direzione di ricerca: pesi, genitori e archi usati, indicizzati per nodo. */
class SearchState(expected: Int) {
    val weight = IntDoubleHashMap(expected)
    val parentNode = IntIntHashMap(expected)
    val parentEdge = IntIntHashMap(expected)
    val settled = com.carrotsearch.hppc.IntHashSet(expected)
    val heap = NodeHeap(maxOf(16, expected / 4))

    fun weightOf(node: Int) = weight.getOrDefault(node, Double.POSITIVE_INFINITY)

    fun relax(node: Int, w: Double, from: Int, edge: Int): Boolean {
        if (w >= weightOf(node)) return false
        weight.put(node, w); parentNode.put(node, from); parentEdge.put(node, edge)
        heap.push(node, w)
        return true
    }
}

/**
 * Dijkstra bidirezionale node-based su un [BaseGraph]: la ricerca in avanti usa il peso diretto,
 * quella all'indietro il peso inverso. Sorgenti e destinazioni possono essere più nodi con un
 * costo iniziale (punti di partenza agganciati a metà arco).
 */
class BidirectionalDijkstra(
    private val graph: BaseGraph,
    private val weighting: Weighting,
    private val filter: EdgeFilter = EdgeFilter.ALL,
    private val maxVisitedNodes: Int = Int.MAX_VALUE,
    /** > 1: la ricerca continua oltre il percorso migliore fino a questo multiplo del suo peso (percorsi alternativi). */
    private val explorationFactor: Double = 1.0,
) {
    val fwd = SearchState(1024)
    val bwd = SearchState(1024)
    private val fwdIt = graph.createEdgeIterator()
    private val bwdIt = graph.createEdgeIterator()

    var bestWeight = Double.POSITIVE_INFINITY; private set
    var meetingNode = -1; private set
    var visitedNodes = 0; private set

    fun addSource(node: Int, initialWeight: Double) { fwd.relax(node, initialWeight, -1, -1) }
    fun addTarget(node: Int, initialWeight: Double) { bwd.relax(node, initialWeight, -1, -1) }

    /** Esegue la ricerca; true se è stato trovato un percorso. */
    fun run(): Boolean {
        // un nodo che è insieme sorgente e destinazione (o già raggiunto da entrambi) conta subito
        for (c in fwd.weight) checkMeeting(c.key)
        while (!fwd.heap.isEmpty || !bwd.heap.isEmpty) {
            if (visitedNodes >= maxVisitedNodes) return false
            if (finished()) break
            // espandi la direzione con il minimo più piccolo
            val fwdKey = if (fwd.heap.isEmpty) Double.POSITIVE_INFINITY else fwd.heap.peekKey()
            val bwdKey = if (bwd.heap.isEmpty) Double.POSITIVE_INFINITY else bwd.heap.peekKey()
            if (fwdKey <= bwdKey) settleNext(fwd, fwdIt, reverse = false) else settleNext(bwd, bwdIt, reverse = true)
        }
        return meetingNode >= 0
    }

    private fun finished(): Boolean {
        if (meetingNode < 0) return false
        val fwdKey = if (fwd.heap.isEmpty) Double.POSITIVE_INFINITY else fwd.heap.peekKey()
        val bwdKey = if (bwd.heap.isEmpty) Double.POSITIVE_INFINITY else bwd.heap.peekKey()
        return fwdKey + bwdKey >= bestWeight * explorationFactor
    }

    private fun settleNext(state: SearchState, it: BaseGraph.EdgeIterator, reverse: Boolean) {
        val node = state.heap.pop()
        if (!state.settled.add(node)) return // voce pigra già superata
        visitedNodes++
        val base = state.weightOf(node)
        it.setBaseNode(node)
        while (it.next()) {
            if (!filter.accept(it)) continue
            val w = weighting.edgeWeight(it, reverse)
            if (w.isInfinite()) continue
            val adj = it.adjNode
            if (state.settled.contains(adj)) continue
            if (state.relax(adj, base + w, node, it.edge)) checkMeeting(adj)
        }
    }

    private fun checkMeeting(node: Int) {
        val f = fwd.weightOf(node); val b = bwd.weightOf(node)
        if (f.isInfinite() || b.isInfinite()) return
        val total = f + b
        if (total < bestWeight) { bestWeight = total; meetingNode = node }
    }

    /** Archi del percorso dal nodo sorgente al nodo destinazione, con i rispettivi nodi. */
    fun extractPath(): SearchPath? {
        if (meetingNode < 0) return null
        val edges = ArrayList<Int>(); val nodes = ArrayList<Int>()
        var n = meetingNode
        while (true) {
            nodes.add(n)
            val e = fwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            edges.add(e); n = fwd.parentNode.get(n)
        }
        edges.reverse(); nodes.reverse()
        n = meetingNode
        while (true) {
            val e = bwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            edges.add(e); n = bwd.parentNode.get(n); nodes.add(n)
        }
        return SearchPath(nodes.toIntArray(), edges.toIntArray(), bestWeight)
    }
}

/** Percorso come sequenza di nodi (n+1) e archi (n) nel grafo base. */
class SearchPath(val nodes: IntArray, val edges: IntArray, val weight: Double) {
    val isEmpty get() = edges.isEmpty()
    val startNode get() = nodes.first()
    val endNode get() = nodes.last()
}
