package it.navmaster.engine.ch

import com.carrotsearch.hppc.IntDoubleHashMap
import com.carrotsearch.hppc.IntHashSet
import com.carrotsearch.hppc.IntIntHashMap
import com.carrotsearch.hppc.IntObjectHashMap
import it.navmaster.engine.routing.NodeHeap
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting

/**
 * I pesi fra un punto (la radice) e molti altri sul core, trovati insieme: le ricerche CH verso
 * l'alto di ogni punto (fase 1, come in [CoreDijkstra]), poi una ricerca sola dentro il core dagli
 * ingressi della radice, A* verso gli ingressi degli altri (core-ALT, coi landmark), che fissa gli
 * altri punti in ordine di peso. In ogni momento dà per ogni altro punto il suo peso quando è già
 * definitivo, altrimenti un limite inferiore; la ricerca va avanti solo quanto chiesto ([advance]).
 *
 * [rootIsTarget]: i pesi dagli altri punti alla radice (le ricerche della radice vanno all'indietro),
 * altrimenti dalla radice agli altri. Archi, filtri e svolte come in [CoreDijkstra].
 */
class CoreManyToOne(
    private val ch: CHGraph,
    private val weighting: Weighting,
    private val filter: EdgeFilter,
    private val turnCosts: TurnCostProvider?,
    landmarks: CoreLandmarks?,
    private val rootIsTarget: Boolean,
    val otherCount: Int,
) {
    private val base = ch.base
    private val storage = ch.storage
    private val coreLevel = base.nodeCount
    private fun inCore(node: Int) = storage.level(node) >= coreLevel
    private val edgeBased get() = turnCosts != null
    private val potentials: CorePotentials? = landmarks?.takeIf { it.landmarkCount > 0 }?.let { CorePotentials(it) }
    /** Le ricerche della radice vanno all'indietro quando è l'arrivo; quelle degli altri nel verso opposto. */
    private val rootReverse = rootIsTarget
    private val otherReverse = !rootIsTarget

    private class State(capacity: Int) {
        val weight = IntDoubleHashMap(capacity)
        val parentNode = IntIntHashMap(capacity)
        val parentEdge = IntIntHashMap(capacity)
        val settled = IntHashSet(capacity)
        val heap = NodeHeap(capacity)
        fun weightOf(n: Int) = weight.getOrDefault(n, Double.POSITIVE_INFINITY)
        fun relax(n: Int, w: Double, from: Int, edge: Int, key: Double = w): Boolean {
            if (w >= weightOf(n)) return false
            weight.put(n, w); parentNode.put(n, from); parentEdge.put(n, edge); heap.push(n, key); return true
        }
    }

    /** Un ingresso al core di un altro punto: la chiave (nodo o arco d'arrivo/partenza) e il peso fin lì. */
    private class Seed(val other: Int, val key: Int, val weight: Double)

    private val root1 = State(1024)
    private val others1 = Array(otherCount) { State(64) }
    private val core = State(4096)
    private val seedsAt = IntObjectHashMap<ArrayList<Seed>>()
    private val tentative = DoubleArray(otherCount) { Double.POSITIVE_INFINITY }
    private val final = BooleanArray(otherCount)
    private val entersCore = BooleanArray(otherCount)
    private val it1 = ch.iterator()
    private val coreIt = ch.iterator()
    private val baseIt = base.createEdgeIterator()
    private var prepared = false
    private var exhausted = false

    var visitedNodes = 0; private set
    private var phase1Nodes = 0
    private var phase1Ms = 0L
    private var coreNanos = 0L

    /** Un capo dell'arco della radice, col peso del tratto fino al punto. */
    fun addRoot(node: Int, w: Double) { root1.relax(node, w, -1, -1) }

    /** Un capo dell'arco dell'altro punto [other]. */
    fun addOther(other: Int, node: Int, w: Double) { others1[other].relax(node, w, -1, -1) }

    /** Un percorso diretto già noto fra la radice e [other] (lo stesso arco). */
    fun addDirect(other: Int, w: Double) { if (w < tentative[other]) tentative[other] = w }

    /** Fase 1 di tutti i punti e semina della ricerca nel core. */
    fun prepare() {
        check(!prepared) { "già preparata" }
        prepared = true
        val t0 = System.nanoTime()
        phase1(root1, rootReverse)
        for (s in others1) phase1(s, otherReverse)
        phase1Nodes = visitedNodes
        // incontri in fase 1 (come CoreDijkstra: un nodo raggiunto da entrambi, anche del core)
        for ((i, s) in others1.withIndex()) for (c in s.weight) {
            val r = root1.weightOf(c.key)
            if (r.isFinite() && c.value + r < tentative[i]) tentative[i] = c.value + r
        }
        // gli ingressi al core degli altri: dove la ricerca della radice li incontra; e i potenziali verso di loro
        for ((i, s) in others1.withIndex()) for (c in s.weight) {
            val node = c.key
            if (!inCore(node) || !s.settled.contains(node)) continue
            val key = if (!edgeBased) node else lastBaseEdge(s, node).let { e -> if (e < 0) virtualKey(node) else edgeKeyArriving(e, node, otherReverse) }
            (seedsAt.get(node) ?: ArrayList<Seed>(2).also { seedsAt.put(node, it) }).add(Seed(i, key, c.value))
            potentials?.let { p -> if (rootIsTarget) p.addSource(node, c.value) else p.addTarget(node, c.value) }
            entersCore[i] = true
        }
        // senza ingressi al core un punto ha solo gli incontri di fase 1: già definitivo
        for (i in 0 until otherCount) if (!entersCore[i]) final[i] = true
        // gli ingressi al core della radice: i semi della ricerca
        for (c in root1.weight) {
            val node = c.key
            if (!inCore(node) || !root1.settled.contains(node)) continue
            val key = if (!edgeBased) node else lastBaseEdge(root1, node).let { e -> if (e < 0) virtualKey(node) else edgeKeyArriving(e, node, rootReverse) }
            relaxCore(key, c.value, -1, -1)
        }
        phase1Ms = (System.nanoTime() - t0) / 1_000_000
    }

    /** Vero quando il peso di [other] è definitivo ([weight]). */
    fun isFinal(other: Int): Boolean = final[other]

    /** Vero quando almeno un punto ha il suo peso definitivo ed è raggiungibile. */
    val anyReached: Boolean get() = (0 until otherCount).any { final[it] && tentative[it].isFinite() }

    /** Il peso di [other] (+∞ se irraggiungibile) quando è definitivo, altrimenti un limite inferiore. */
    fun lowerBound(other: Int): Double = if (final[other]) tentative[other] else minOf(tentative[other], frontier())

    /** Un limite inferiore del peso di ogni punto non ancora definitivo (+∞ a ricerca finita). */
    val bound: Double get() = frontier()

    /** Il peso definitivo di [other]. */
    fun weight(other: Int): Double { check(final[other]); return tentative[other] }

    /**
     * Prosegue la ricerca nel core finché [stop] è vero o non c'è più niente da esplorare (allora
     * tutti i pesi sono definitivi). I pesi diventano definitivi in ordine crescente.
     */
    fun advance(stop: () -> Boolean) {
        check(prepared) { "manca prepare()" }
        val t0 = System.nanoTime()
        try {
            while (true) {
                val k = frontier()
                for (i in 0 until otherCount) if (!final[i] && tentative[i] <= k) final[i] = true
                if (k.isInfinite()) { exhausted = true; return }
                if (stop()) return
                settleCore()
            }
        } finally { coreNanos += System.nanoTime() - t0 }
    }

    /** Tempi e nodi (diagnostica). */
    val stats: String get() = "fase1 $phase1Nodes nodi $phase1Ms ms, core ${visitedNodes - phase1Nodes} nodi ${coreNanos / 1_000_000} ms" +
        (if (potentials != null) " ALT" else "") + (if (exhausted) ", esaurita" else "") + ", definitivi ${final.count { it }}/$otherCount"

    /**
     * Un limite inferiore del peso di ogni percorso non ancora trovato: la chiave minima della ricerca.
     * Con potenziali coerenti (≤ il peso dagli ingressi degli altri) ogni incontro futuro pesa almeno tanto.
     */
    private fun frontier(): Double {
        val h = core.heap
        while (!h.isEmpty && core.settled.contains(h.peekNode())) h.pop()
        return if (h.isEmpty) Double.POSITIVE_INFINITY else h.peekKey()
    }

    private fun phase1(s: State, reverse: Boolean) {
        while (!s.heap.isEmpty) {
            val node = s.heap.pop()
            if (!s.settled.add(node)) continue
            visitedNodes++
            if (inCore(node)) continue // punto d'ingresso: non si espande in fase 1
            val w = s.weightOf(node)
            it1.setBaseNode(node, reverse)
            while (it1.next()) {
                val adj = it1.adjNode
                if (s.settled.contains(adj)) continue
                s.relax(adj, w + it1.weight, node, it1.chEdge)
            }
        }
    }

    /** La chiave dell'heap: il peso più il limite inferiore del resto verso gli ingressi degli altri. */
    private fun heapKey(w: Double, node: Int): Double {
        val p = potentials ?: return w
        return w + if (rootIsTarget) p.fromSources(node) else p.toTargets(node)
    }

    private fun relaxCore(key: Int, w: Double, from: Int, edge: Int) {
        val node = if (edgeBased) keyNode(key, rootReverse) else key
        if (!core.relax(key, w, from, edge, heapKey(w, node))) return
        val seeds = seedsAt.get(node) ?: return
        // un incontro con l'ingresso di un altro punto nello stesso nodo (col costo della svolta fra i due archi)
        for (s in seeds) {
            var total = w + s.weight
            if (edgeBased) {
                val fwdKey = if (rootReverse) s.key else key
                val bwdKey = if (rootReverse) key else s.key
                if (fwdKey >= 0 && bwdKey >= 0) {
                    val turn = turnCosts!!.turnWeight(fwdKey ushr 1, node, bwdKey ushr 1)
                    if (turn.isInfinite()) continue
                    total += turn
                }
            }
            // la stessa chiave d'arco nelle due ricerche non capita: l'arco d'ingresso di un altro punto
            // ha l'altro capo fuori dal core, la ricerca della radice percorre solo archi fra nodi del core
            if (total < tentative[s.other]) tentative[s.other] = total
        }
    }

    /** Come [CoreDijkstra] in fase 2: archi base fra nodi del core col filtro della richiesta, shortcut fra nodi del core. */
    private fun settleCore() {
        val key = core.heap.pop()
        if (!core.settled.add(key)) return
        visitedNodes++
        val w = core.weightOf(key)
        val node = if (edgeBased) keyNode(key, rootReverse) else key
        val inEdge = if (edgeBased && key >= 0) key ushr 1 else -1
        coreIt.setBaseNode(node, rootReverse)
        while (coreIt.next()) {
            val adj = coreIt.adjNode
            if (!inCore(adj)) continue
            val chEdge = coreIt.chEdge
            var ew = coreIt.weight
            var firstBase = chEdge; var lastBase = chEdge
            if (coreIt.isShortcut) {
                if (edgeBased) {
                    val sc = storage.shortcutIndex(chEdge)
                    if (storage.nodeA(sc) == node) { firstBase = storage.origFirst(sc); lastBase = storage.origLast(sc) }
                    else { firstBase = storage.origLast(sc); lastBase = storage.origFirst(sc) }
                }
            } else {
                baseIt.setEdge(chEdge, node)
                if (!filter.accept(baseIt)) continue
                ew = weighting.edgeWeight(baseIt, rootReverse)
                if (ew.isInfinite()) continue
            }
            var turn = 0.0
            if (edgeBased && inEdge >= 0) {
                turn = if (!rootReverse) turnCosts!!.turnWeight(inEdge, node, firstBase) else turnCosts!!.turnWeight(firstBase, node, inEdge)
                if (turn.isInfinite()) continue
            }
            val nk = if (!edgeBased) adj else edgeKeyArriving(lastBase, adj, rootReverse)
            if (core.settled.contains(nk)) continue
            relaxCore(nk, w + ew + turn, key, chEdge)
        }
    }

    // le chiavi d'arco come in CoreDijkstra

    private fun virtualKey(node: Int) = -(node + 2)

    private fun lastBaseEdge(s: State, node: Int): Int {
        val chEdge = s.parentEdge.getOrDefault(node, -1)
        if (chEdge < 0) return -1
        if (!storage.isShortcut(chEdge)) return chEdge
        val sc = storage.shortcutIndex(chEdge)
        val parent = s.parentNode.get(node)
        return if (storage.nodeA(sc) == parent) storage.origLast(sc) else storage.origFirst(sc)
    }

    private fun edgeKeyArriving(edge: Int, node: Int, reverse: Boolean): Int {
        val forwardAB = if (!reverse) base.nodeB(edge) == node else base.nodeA(edge) == node
        return (edge shl 1) or (if (forwardAB) 0 else 1)
    }

    private fun keyNode(key: Int, reverse: Boolean): Int {
        if (key < 0) return -key - 2
        val e = key ushr 1
        val ab = key and 1 == 0
        return if (!reverse) (if (ab) base.nodeB(e) else base.nodeA(e)) else (if (ab) base.nodeA(e) else base.nodeB(e))
    }
}
