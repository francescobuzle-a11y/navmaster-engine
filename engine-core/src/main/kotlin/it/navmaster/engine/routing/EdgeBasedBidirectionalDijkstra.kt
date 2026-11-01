package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntHashSet
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting

/**
 * Dijkstra bidirezionale **edge-based**: lo stato è la chiave d'arco `edge<<1 | reverse`
 * (arco e verso di percorrenza), così i costi di svolta e i divieti di inversione si applicano
 * tra arco entrante e arco uscente. Le due ricerche si incontrano sulla stessa chiave d'arco:
 * peso totale = avanti(k) + indietro(k) − peso(k), perché entrambe includono l'arco k.
 */
class EdgeBasedBidirectionalDijkstra(
    private val graph: BaseGraph,
    private val weighting: Weighting,
    private val turnCosts: TurnCostProvider,
    private val filter: EdgeFilter = EdgeFilter.ALL,
    private val maxVisitedNodes: Int = Int.MAX_VALUE,
) {
    private val fwd = SearchState(1024)   // chiave: edgeKey percorso in avanti
    private val bwd = SearchState(1024)   // chiave: edgeKey nel verso in cui lo percorrerebbe il viaggio (stessa chiave di fwd)
    private val fwdIt = graph.createEdgeIterator()
    private val bwdIt = graph.createEdgeIterator()
    private val fwdSources = IntHashSet(); private val bwdTargets = IntHashSet()
    private val sourceWeights = com.carrotsearch.hppc.IntDoubleHashMap()
    private val targetWeights = com.carrotsearch.hppc.IntDoubleHashMap()

    var bestWeight = Double.POSITIVE_INFINITY; private set
    private var meetingKey = -1
    /** Incontro diretto su un nodo comune a sorgenti e destinazioni (percorso senza archi). */
    private var meetingNode = -1
    var visitedNodes = 0; private set

    fun addSource(node: Int, initialWeight: Double) { fwdSources.add(node); sourceWeights.put(node, initialWeight) }
    fun addTarget(node: Int, initialWeight: Double) { bwdTargets.add(node); targetWeights.put(node, initialWeight) }

    fun run(): Boolean {
        for (c in sourceWeights) if (targetWeights.containsKey(c.key)) {
            val w = c.value + targetWeights.get(c.key)
            if (w < bestWeight) { bestWeight = w; meetingNode = c.key }
        }
        // prime mosse: dagli archi uscenti delle sorgenti e dagli archi entranti delle destinazioni, senza costo di svolta
        for (c in sourceWeights) {
            fwdIt.setBaseNode(c.key)
            while (fwdIt.next()) {
                if (!filter.accept(fwdIt)) continue
                val w = weighting.edgeWeight(fwdIt, false)
                if (w.isInfinite()) continue
                if (fwd.relax(fwdIt.edgeKey, c.value + w, c.key, -1)) checkMeeting(fwdIt.edgeKey, fwdIt)
            }
        }
        for (c in targetWeights) {
            bwdIt.setBaseNode(c.key)
            while (bwdIt.next()) {
                if (!filter.accept(bwdIt)) continue
                // arrivare al nodo destinazione percorrendo l'arco verso di esso = verso opposto all'iteratore
                val w = weighting.edgeWeight(bwdIt, true)
                if (w.isInfinite()) continue
                val key = travelKey(bwdIt)
                if (bwd.relax(key, c.value + w, c.key, -1)) checkMeeting(key, bwdIt)
            }
        }
        while (!fwd.heap.isEmpty || !bwd.heap.isEmpty) {
            if (visitedNodes >= maxVisitedNodes) return false
            if (finished()) break
            val fk = if (fwd.heap.isEmpty) Double.POSITIVE_INFINITY else fwd.heap.peekKey()
            val bk = if (bwd.heap.isEmpty) Double.POSITIVE_INFINITY else bwd.heap.peekKey()
            if (fk <= bk) settleForward() else settleBackward()
        }
        return meetingKey >= 0 || meetingNode >= 0
    }

    /** Chiave dell'arco nel verso in cui verrebbe percorso nel viaggio: per la ricerca indietro è l'opposto dell'iteratore. */
    private fun travelKey(it: BaseGraph.EdgeIterator): Int = (it.edge shl 1) or (if (it.reverse) 0 else 1)

    private fun finished(): Boolean {
        if (bestWeight.isInfinite()) return false
        val fk = if (fwd.heap.isEmpty) Double.POSITIVE_INFINITY else fwd.heap.peekKey()
        val bk = if (bwd.heap.isEmpty) Double.POSITIVE_INFINITY else bwd.heap.peekKey()
        return fk + bk >= bestWeight
    }

    private fun settleForward() {
        val key = fwd.heap.pop()
        if (!fwd.settled.add(key)) return
        visitedNodes++
        val edge = key ushr 1
        // nodo raggiunto: adj dell'arco percorso nel verso della chiave
        val arrivedAt = if (key and 1 == 0) graph.nodeB(edge) else graph.nodeA(edge)
        val base = fwd.weightOf(key)
        fwdIt.setBaseNode(arrivedAt)
        while (fwdIt.next()) {
            if (!filter.accept(fwdIt)) continue
            val w = weighting.edgeWeight(fwdIt, false)
            if (w.isInfinite()) continue
            val turn = turnCosts.turnWeight(edge, arrivedAt, fwdIt.edge)
            if (turn.isInfinite()) continue
            val nk = fwdIt.edgeKey
            if (fwd.settled.contains(nk)) continue
            if (fwd.relax(nk, base + w + turn, key, edge)) checkMeeting(nk, fwdIt)
        }
    }

    private fun settleBackward() {
        val key = bwd.heap.pop()
        if (!bwd.settled.add(key)) return
        visitedNodes++
        val edge = key ushr 1
        // l'arco con chiave `key` viene percorso base→adj nel viaggio; la ricerca indietro è arrivata alla sua base
        val startsAt = if (key and 1 == 0) graph.nodeA(edge) else graph.nodeB(edge)
        val base = bwd.weightOf(key)
        bwdIt.setBaseNode(startsAt)
        while (bwdIt.next()) {
            if (!filter.accept(bwdIt)) continue
            val w = weighting.edgeWeight(bwdIt, true) // percorso verso startsAt
            if (w.isInfinite()) continue
            // nel viaggio: si arriva a startsAt con bwdIt.edge e si svolta su `edge`
            val turn = turnCosts.turnWeight(bwdIt.edge, startsAt, edge)
            if (turn.isInfinite()) continue
            val nk = travelKey(bwdIt)
            if (bwd.settled.contains(nk)) continue
            if (bwd.relax(nk, base + w + turn, key, edge)) checkMeeting(nk, bwdIt)
        }
    }

    private fun checkMeeting(key: Int, it: BaseGraph.EdgeIterator) {
        val f = fwd.weightOf(key); val b = bwd.weightOf(key)
        if (f.isInfinite() || b.isInfinite()) return
        val edgeW = weighting.edgeWeight(it, (key and 1 == 1) != it.reverse)
        val total = f + b - edgeW
        if (total < bestWeight) { bestWeight = total; meetingKey = key; meetingNode = -1 }
    }

    fun extractPath(): SearchPath? {
        if (meetingNode >= 0 && meetingKey < 0) return SearchPath(intArrayOf(meetingNode), IntArray(0), bestWeight)
        if (meetingKey < 0) return null
        val edges = ArrayList<Int>(); val nodes = ArrayList<Int>()
        // all'indietro lungo la catena avanti: ogni chiave → arco; parentNode della prima voce è il nodo sorgente
        var k = meetingKey
        while (true) {
            edges.add(k ushr 1)
            val pe = fwd.parentEdge.getOrDefault(k, -1)
            val pn = fwd.parentNode.get(k)
            if (pe < 0) { nodes.add(pn); break } // pn è il nodo sorgente
            k = pn // per la ricerca edge-based parentNode contiene la chiave precedente
        }
        edges.reverse()
        // ricostruisci i nodi: dal nodo sorgente seguendo gli archi
        val nodeList = ArrayList<Int>(edges.size + 1)
        var n = nodes[0]
        nodeList.add(n)
        for (e in edges) { n = if (graph.nodeA(e) == n) graph.nodeB(e) else graph.nodeA(e); nodeList.add(n) }
        // poi in avanti lungo la catena indietro, saltando l'arco d'incontro già incluso
        k = bwd.parentNode.getOrDefault(meetingKey, Int.MIN_VALUE)
        var pe = bwd.parentEdge.getOrDefault(meetingKey, -1)
        while (pe >= 0 && k != Int.MIN_VALUE) {
            val e = k ushr 1
            edges.add(e)
            n = if (graph.nodeA(e) == n) graph.nodeB(e) else graph.nodeA(e)
            nodeList.add(n)
            pe = bwd.parentEdge.getOrDefault(k, -1)
            k = bwd.parentNode.getOrDefault(k, Int.MIN_VALUE)
        }
        return SearchPath(nodeList.toIntArray(), edges.toIntArray(), bestWeight)
    }
}
