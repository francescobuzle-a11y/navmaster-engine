package it.navmaster.engine.ch

import com.carrotsearch.hppc.IntDoubleHashMap
import com.carrotsearch.hppc.IntHashSet
import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.routing.NodeHeap
import it.navmaster.engine.routing.SearchPath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.weighting.Weighting

/**
 * Vista del grafo per le interrogazioni CH: archi base verso nodi di livello superiore
 * più gli shortcut registrati sul nodo (che vanno sempre "verso l'alto").
 */
class CHGraph(val base: BaseGraph, val storage: CHStorage, private val weighting: Weighting) {
    private val baseEdges = base.edgeCount

    /** Iteratore riutilizzabile sugli archi CH uscenti (o entranti, per la ricerca indietro) di un nodo. */
    inner class Iterator {
        private val baseIt = base.createEdgeIterator()
        private var node = -1
        private var sc = -1          // indice dello shortcut corrente, -1 quando si passa agli archi base
        private var inShortcuts = true
        private var reverseSearch = false
        var chEdge = -1; private set
        var adjNode = -1; private set
        var weight = Double.POSITIVE_INFINITY; private set
        val isShortcut get() = chEdge >= baseEdges

        /**
         * Whether [filter] lets the current edge through. A shortcut always does (as ORS's
         * `AvoidFeaturesCoreEdgeFilter`): the roads a request can exclude are core edges, and the
         * core's shortcuts are made of non-core edges only.
         */
        fun accepts(filter: it.navmaster.engine.weighting.EdgeFilter): Boolean = isShortcut || filter.accept(baseIt)

        fun setBaseNode(n: Int, reverse: Boolean): Iterator {
            node = n; reverseSearch = reverse
            sc = storage.lastShortcut(n); inShortcuts = true
            if (sc < 0) { inShortcuts = false; baseIt.setBaseNode(n) }
            return this
        }

        fun next(): Boolean {
            while (inShortcuts) {
                if (sc < 0 || storage.nodeA(sc) != node) { inShortcuts = false; baseIt.setBaseNode(node); break }
                val cur = sc--
                val ok = if (!reverseSearch) storage.fwd(cur) else storage.bwd(cur)
                if (!ok) continue
                chEdge = baseEdges + cur; adjNode = storage.nodeB(cur); weight = storage.weight(cur)
                return true
            }
            val myLevel = storage.level(node)
            while (baseIt.next()) {
                if (storage.level(baseIt.adjNode) < myLevel) continue // solo verso l'alto (i pari livello sono il core)
                val w = weighting.edgeWeight(baseIt, reverseSearch)
                if (w.isInfinite()) continue
                chEdge = baseIt.edge; adjNode = baseIt.adjNode; weight = w
                return true
            }
            return false
        }
    }

    fun iterator() = Iterator()

    /** Scompatta ricorsivamente un arco CH nei suoi archi base, nell'ordine di percorrenza da [fromNode]. */
    fun unpack(chEdge: Int, fromNode: Int, out: MutableList<Int>) {
        if (chEdge < baseEdges) { out.add(chEdge); return }
        val sc = chEdge - baseEdges
        val a = storage.nodeA(sc); val b = storage.nodeB(sc)
        val s1 = storage.skip1(sc); val s2 = storage.skip2(sc) // orientati a→b: s1 da a al nodo via, s2 dal via a b
        if (fromNode == a) {
            val via = otherEnd(s1, a)
            unpack(s1, a, out); unpack(s2, via, out)
        } else {
            check(fromNode == b) { "nodo $fromNode non è estremo dello shortcut $sc" }
            val via = otherEnd(s2, b)
            unpack(s2, b, out); unpack(s1, via, out)
        }
    }

    /** L'altro estremo di un arco CH rispetto a [node]. */
    fun otherEnd(chEdge: Int, node: Int): Int {
        if (chEdge < baseEdges) return if (base.nodeA(chEdge) == node) base.nodeB(chEdge) else base.nodeA(chEdge)
        val sc = chEdge - baseEdges
        return if (storage.nodeA(sc) == node) storage.nodeB(sc) else storage.nodeA(sc)
    }
}

/**
 * Dijkstra bidirezionale su CH con stall-on-demand (GraphHopper `DijkstraBidirectionCH`).
 * Entrambe le ricerche salgono di livello; la condizione di arresto è `minF + minB ≥ best`.
 */
class DijkstraBidirectionCH(private val ch: CHGraph, private val weighting: Weighting, private val stallOnDemand: Boolean = true) {
    private class State {
        val weight = IntDoubleHashMap(1024)
        val parentNode = IntIntHashMap(1024)
        val parentEdge = IntIntHashMap(1024)
        val settled = IntHashSet(1024)
        val heap = NodeHeap(256)
        fun weightOf(n: Int) = weight.getOrDefault(n, Double.POSITIVE_INFINITY)
        fun relax(n: Int, w: Double, from: Int, edge: Int): Boolean {
            if (w >= weightOf(n)) return false
            weight.put(n, w); parentNode.put(n, from); parentEdge.put(n, edge); heap.push(n, w); return true
        }
    }

    private val fwd = State(); private val bwd = State()
    private val fwdIt = ch.iterator(); private val bwdIt = ch.iterator()
    private val stallIt = ch.iterator()
    var bestWeight = Double.POSITIVE_INFINITY; internal set
    private var meetingNode = -1
    var visitedNodes = 0; private set

    fun addSource(node: Int, w: Double) { if (fwd.relax(node, w, -1, -1)) checkMeeting(node) }
    fun addTarget(node: Int, w: Double) { if (bwd.relax(node, w, -1, -1)) checkMeeting(node) }

    fun run(): Boolean {
        while (!fwd.heap.isEmpty || !bwd.heap.isEmpty) {
            val fk = if (fwd.heap.isEmpty) Double.POSITIVE_INFINITY else fwd.heap.peekKey()
            val bk = if (bwd.heap.isEmpty) Double.POSITIVE_INFINITY else bwd.heap.peekKey()
            if (fk >= bestWeight && bk >= bestWeight) break
            if (fk <= bk) settle(fwd, fwdIt, false) else settle(bwd, bwdIt, true)
        }
        return meetingNode >= 0
    }

    private fun settle(s: State, it: CHGraph.Iterator, reverse: Boolean) {
        val node = s.heap.pop()
        if (!s.settled.add(node)) return
        visitedNodes++
        val w = s.weightOf(node)
        if (stallOnDemand && isStalled(node, w, reverse)) return
        it.setBaseNode(node, reverse)
        while (it.next()) {
            val adj = it.adjNode
            if (s.settled.contains(adj)) continue
            if (s.relax(adj, w + it.weight, node, it.chEdge)) checkMeeting(adj)
        }
    }

    /** Il nodo è "stallato" se un vicino già raggiunto lo raggiunge con peso minore passando per un arco verso di lui. */
    private fun isStalled(node: Int, w: Double, reverse: Boolean): Boolean {
        val s = if (reverse) bwd else fwd
        // archi nella direzione opposta alla ricerca: (vicino → nodo)
        stallIt.setBaseNode(node, !reverse)
        while (stallIt.next()) {
            val nb = stallIt.adjNode
            val nbW = s.weightOf(nb)
            if (nbW.isInfinite()) continue
            if (nbW + stallIt.weight - w < -0.001) return true
        }
        return false
    }

    private fun checkMeeting(node: Int) {
        val f = fwd.weightOf(node); val b = bwd.weightOf(node)
        if (f.isInfinite() || b.isInfinite()) return
        if (f + b < bestWeight) { bestWeight = f + b; meetingNode = node }
    }

    /** Percorso scompattato in archi base. */
    fun extractPath(): SearchPath? {
        if (meetingNode < 0) return null
        val chEdgesFwd = ArrayList<Int>(); val nodesFwd = ArrayList<Int>()
        var n = meetingNode
        while (true) {
            nodesFwd.add(n)
            val e = fwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            chEdgesFwd.add(e); n = fwd.parentNode.get(n)
        }
        chEdgesFwd.reverse(); nodesFwd.reverse()
        val baseEdgesOut = ArrayList<Int>()
        for (i in chEdgesFwd.indices) ch.unpack(chEdgesFwd[i], nodesFwd[i], baseEdgesOut)
        n = meetingNode
        while (true) {
            val e = bwd.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            ch.unpack(e, n, baseEdgesOut)
            n = bwd.parentNode.get(n)
        }
        // ricostruisci la sequenza di nodi dagli archi base
        val startNode = nodesFwd.first()
        val nodes = IntArray(baseEdgesOut.size + 1)
        nodes[0] = startNode
        for (i in baseEdgesOut.indices) {
            val e = baseEdgesOut[i]
            nodes[i + 1] = if (ch.base.nodeA(e) == nodes[i]) ch.base.nodeB(e) else ch.base.nodeA(e)
        }
        return SearchPath(nodes, baseEdgesOut.toIntArray(), bestWeight)
    }
}
