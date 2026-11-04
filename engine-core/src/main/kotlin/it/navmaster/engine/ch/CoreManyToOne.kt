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
    private val lm = landmarks?.takeIf { it.landmarkCount > 0 }
    private val potentials: CorePotentials? = lm?.let { CorePotentials(it) }
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

    /**
     * Un ingresso al core di un altro punto: la chiave (nodo o arco d'arrivo/partenza), il peso fin lì
     * e l'arco base con cui il viaggio lo tocca (per la svolta con la ricerca della radice; -1 senza).
     */
    private class Seed(val other: Int, val key: Int, val weight: Double, val edge: Int)

    private val root1 = State(1024)
    private val others1 = Array(otherCount) { State(64) }
    /** I capi degli archi agganciati della radice e degli altri punti, col loro arco (vedi [SnapEnds]). */
    private val rootEnds = SnapEnds()
    private val otherEnds = Array(otherCount) { SnapEnds() }
    private val core = State(4096)
    private val seedsAt = IntObjectHashMap<ArrayList<Seed>>()
    private val tentative = DoubleArray(otherCount) { Double.POSITIVE_INFINITY }
    private val final = BooleanArray(otherCount)
    private val entersCore = BooleanArray(otherCount)
    /** Il limite inferiore dei landmark per ogni altro punto, senza ricerca (0 senza landmark). */
    private val landmarkBound = DoubleArray(otherCount)
    private val it1 = ch.iterator()
    private val coreIt = ch.iterator()
    private val baseIt = base.createEdgeIterator()
    private var prepared = false
    private var exhausted = false

    var visitedNodes = 0; private set
    private var phase1Nodes = 0
    private var phase1Ms = 0L
    private var coreNanos = 0L

    /**
     * Un capo dell'arco della radice, col peso del tratto fino al punto. [edge]: l'arco agganciato
     * (per la svolta col primo o l'ultimo arco del percorso); -1 per un aggancio su una torre.
     */
    fun addRoot(node: Int, w: Double, edge: Int = -1) { if (rootEnds.add(node, w, edge)) root1.relax(node, w, -1, -1) }

    /** Un capo dell'arco dell'altro punto [other] (con [edge] come in [addRoot]). */
    fun addOther(other: Int, node: Int, w: Double, edge: Int = -1) { if (otherEnds[other].add(node, w, edge)) others1[other].relax(node, w, -1, -1) }

    /** Un percorso diretto già noto fra la radice e [other] (lo stesso arco). */
    fun addDirect(other: Int, w: Double) { if (w < tentative[other]) tentative[other] = w }

    /** Fase 1 di tutti i punti e semina della ricerca nel core. */
    fun prepare() {
        check(!prepared) { "già preparata" }
        prepared = true
        val t0 = System.nanoTime()
        phase1(root1, rootReverse, rootEnds)
        for ((i, s) in others1.withIndex()) phase1(s, otherReverse, otherEnds[i])
        phase1Nodes = visitedNodes
        // incontri in fase 1 (come CoreDijkstra: un nodo raggiunto da entrambi, anche del core; edge-based
        // con la svolta fra l'arco d'arrivo e quello di partenza)
        for ((i, s) in others1.withIndex()) for (c in s.weight) {
            val node = c.key
            if (!root1.weight.containsKey(node)) continue
            phase1States(s, otherEnds[i], node) { wo, eo ->
                phase1States(root1, rootEnds, node) { wr, er ->
                    val turn = if (!edgeBased) 0.0 else if (rootIsTarget) turnCosts!!.turnOrZero(eo, node, er) else turnCosts!!.turnOrZero(er, node, eo)
                    if (turn.isFinite() && wo + wr + turn < tentative[i]) tentative[i] = wo + wr + turn
                }
            }
        }
        // gli ingressi al core degli altri: dove la ricerca della radice li incontra; e i potenziali verso di loro
        for ((i, s) in others1.withIndex()) for (c in s.weight) {
            val node = c.key
            if (!inCore(node) || !s.settled.contains(node)) continue
            potentials?.let { p -> if (rootIsTarget) p.addSource(node, c.value) else p.addTarget(node, c.value) }
            entersCore[i] = true
            if (!edgeBased) { addSeed(node, Seed(i, node, c.value, -1)); continue }
            // edge-based: la chiave è l'ultimo arco base fra il nodo e la fase 1; il capo diretto a parte (sotto)
            if (s.parentEdge.getOrDefault(node, -1) < 0) continue
            val e = lastBaseEdge(s, node)
            addSeed(node, Seed(i, edgeKeyArriving(e, node, otherReverse), c.value, e))
        }
        // edge-based: i capi degli archi agganciati nel core entrano col tratto diretto e il loro arco
        if (edgeBased) for ((i, ends) in otherEnds.withIndex()) for (c in ends.weight) {
            if (inCore(c.key)) addSeed(c.key, Seed(i, virtualKey(c.key), c.value, ends.edgeAt(c.key)))
        }
        // senza ingressi al core un punto ha solo gli incontri di fase 1: già definitivo
        for (i in 0 until otherCount) if (!entersCore[i]) final[i] = true
        // gli ingressi al core della radice: i semi della ricerca (i capi diretti con una chiave fittizia)
        for (c in root1.weight) {
            val node = c.key
            if (!inCore(node) || !root1.settled.contains(node)) continue
            if (!edgeBased) { relaxCore(node, c.value, -1, -1); continue }
            if (root1.parentEdge.getOrDefault(node, -1) >= 0) relaxCore(edgeKeyArriving(lastBaseEdge(root1, node), node, rootReverse), c.value, -1, -1)
        }
        if (edgeBased) for (c in rootEnds.weight) if (inCore(c.key)) relaxCore(virtualKey(c.key), c.value, -1, -1)
        // senza ricerca, dai landmark: attraverso il core un percorso va da un ingresso della radice a
        // uno dell'altro punto e pesa almeno il limite fra i due; altrimenti è un incontro di fase 1
        lm?.let { l ->
            val rp = CorePotentials(l)
            var rootEntries = 0
            for (c in root1.weight) if (inCore(c.key) && root1.settled.contains(c.key)) {
                rootEntries++
                if (rootIsTarget) rp.addTarget(c.key, c.value) else rp.addSource(c.key, c.value)
            }
            val viaCore = DoubleArray(otherCount) { Double.POSITIVE_INFINITY }
            if (rootEntries > 0) for (c in seedsAt) {
                val h = if (rootIsTarget) rp.toTargets(c.key) else rp.fromSources(c.key)
                for (seed in c.value) if (seed.weight + h < viaCore[seed.other]) viaCore[seed.other] = seed.weight + h
            }
            for (i in 0 until otherCount) landmarkBound[i] = minOf(tentative[i], viaCore[i])
        }
        phase1Ms = (System.nanoTime() - t0) / 1_000_000
    }

    /** Diagnostica (prove): da dove viene il limite dei landmark di [other]. */
    internal fun boundDebug(other: Int): String {
        val l = lm ?: return "senza landmark"
        val k = l.landmarkCount
        val entries = ArrayList<Pair<Int, Double>>()
        for (c in root1.weight) if (inCore(c.key) && root1.settled.contains(c.key)) entries += c.key to c.value
        val sb = StringBuilder()
        sb.append("radice: ${entries.size} ingressi, pesi ${entries.map { it.second }.sorted().take(5).joinToString { "%.0f".format(it) }}")
        sb.append(", landmark finiti agli ingressi ${entries.take(3).joinToString { (n, _) -> (0 until k).count { l.fromLandmark(it, n).isFinite() }.toString() + "/" + (0 until k).count { l.toLandmark(it, n).isFinite() } }}")
        sb.append("; tentativo fase 1 ${tentative[other]}, limite ${landmarkBound[other]}")
        val seeds = ArrayList<Seed>()
        for (c in seedsAt) for (sd in c.value) if (sd.other == other) seeds += sd
        val nodes = seedsAt.keys().toArray().filter { n -> seedsAt.get(n).any { it.other == other } }
        sb.append("; semi ${seeds.size} in ${nodes.size} nodi")
        for (n in nodes.take(4)) {
            val w = seedsAt.get(n).filter { it.other == other }.minOf { it.weight }
            var best = 0.0; var bestL = -1; var bestKind = ""
            for (li in 0 until k) {
                val dLc = l.fromLandmark(li, n); val dcL = l.toLandmark(li, n)
                var sC1 = Double.NEGATIVE_INFINITY; var sC2 = Double.POSITIVE_INFINITY
                var tC1 = Double.NEGATIVE_INFINITY; var tC2 = Double.POSITIVE_INFINITY
                for ((e, f) in entries) {
                    val fL = l.fromLandmark(li, e); val tL = l.toLandmark(li, e)
                    if (fL.isFinite()) { sC1 = maxOf(sC1, fL - f); tC2 = minOf(tC2, fL + f) }
                    if (tL.isFinite()) { sC2 = minOf(sC2, tL + f); tC1 = maxOf(tC1, tL - f) }
                }
                val t1 = if (rootIsTarget) dcL - tC1 else dLc - sC1
                val t2 = if (rootIsTarget) tC2 - dLc else sC2 - dcL
                if (t1.isFinite() && t1 > best) { best = t1; bestL = li; bestKind = "a" }
                if (t2.isFinite() && t2 > best) { best = t2; bestL = li; bestKind = "b" }
            }
            sb.append("\n        seme nodo $n peso %.0f: landmark finiti %d/%d, miglior termine %.0f (landmark $bestL $bestKind)".format(w,
                (0 until k).count { l.fromLandmark(it, n).isFinite() }, (0 until k).count { l.toLandmark(it, n).isFinite() }, best))
        }
        return sb.toString()
    }

    /** Vero quando il peso di [other] è definitivo ([weight]). */
    fun isFinal(other: Int): Boolean = final[other]

    /** Vero quando almeno un punto ha il suo peso definitivo ed è raggiungibile. */
    val anyReached: Boolean get() = (0 until otherCount).any { final[it] && tentative[it].isFinite() }

    /**
     * Il peso di [other] (+∞ se irraggiungibile) quando è definitivo, altrimenti un limite inferiore:
     * il migliore fra quello dei landmark (subito, senza ricerca) e quello della ricerca fin qui.
     */
    fun lowerBound(other: Int): Double = if (final[other]) tentative[other] else maxOf(landmarkBound[other], minOf(tentative[other], frontier()))

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

    /** Un ingresso al core di un altro punto nel nodo [node]. */
    private fun addSeed(node: Int, seed: Seed) {
        (seedsAt.get(node) ?: ArrayList<Seed>(2).also { seedsAt.put(node, it) }).add(seed)
    }

    /** Come la fase 1 di [CoreDijkstra]: dal capo di un arco agganciato la prima mossa svolta dal tratto agganciato. */
    private fun phase1(s: State, reverse: Boolean, ends: SnapEnds) {
        while (!s.heap.isEmpty) {
            val node = s.heap.pop()
            if (!s.settled.add(node)) continue
            visitedNodes++
            if (inCore(node)) continue // punto d'ingresso: non si espande in fase 1
            val w = s.weightOf(node)
            val snapEdge = if (edgeBased && s.parentEdge.getOrDefault(node, -1) < 0) ends.edgeAt(node) else -1
            it1.setBaseNode(node, reverse)
            while (it1.next()) {
                val adj = it1.adjNode
                if (s.settled.contains(adj)) continue
                var turn = 0.0
                if (snapEdge >= 0) {
                    val b = storage.baseEdgeAt(it1.chEdge, node)
                    turn = if (!reverse) turnCosts!!.turnWeight(snapEdge, node, b) else turnCosts!!.turnWeight(b, node, snapEdge)
                    if (turn.isInfinite()) continue
                }
                s.relax(adj, w + it1.weight + turn, node, it1.chEdge)
            }
        }
    }

    /**
     * Gli stati di fase 1 di [s] in [node] come in [CoreDijkstra]: (peso, arco base con cui il viaggio
     * tocca il nodo); edge-based anche il capo diretto quando la fase 1 lo ha superato da un altro capo.
     */
    private inline fun phase1States(s: State, ends: SnapEnds, node: Int, f: (Double, Int) -> Unit) {
        if (s.parentEdge.getOrDefault(node, -1) < 0) { f(s.weightOf(node), if (edgeBased) ends.edgeAt(node) else -1); return }
        f(s.weightOf(node), if (edgeBased) lastBaseEdge(s, node) else -1)
        if (edgeBased && ends.has(node)) f(ends.weightAt(node), ends.edgeAt(node))
    }

    /** L'arco della chiave della ricerca della radice in [node]: per una chiave fittizia l'arco agganciato della radice (o -1). */
    private fun keyEdge(key: Int, node: Int): Int = if (key >= 0) key ushr 1 else rootEnds.edgeAt(node)

    /** La chiave dell'heap: il peso più il limite inferiore del resto verso gli ingressi degli altri. */
    private fun heapKey(w: Double, node: Int): Double {
        val p = potentials ?: return w
        return w + if (rootIsTarget) p.fromSources(node) else p.toTargets(node)
    }

    private fun relaxCore(key: Int, w: Double, from: Int, edge: Int) {
        val node = if (edgeBased) keyNode(key, rootReverse) else key
        // i landmark dicono che di qui non si incontra nessun altro punto
        val hk = heapKey(w, node)
        if (hk.isInfinite()) return
        if (!core.relax(key, w, from, edge, hk)) return
        val seeds = seedsAt.get(node) ?: return
        // un incontro con l'ingresso di un altro punto nello stesso nodo (col costo della svolta fra i due archi)
        val rootEdge = if (edgeBased) keyEdge(key, node) else -1
        for (s in seeds) {
            var total = w + s.weight
            if (edgeBased) {
                val turn = if (rootReverse) turnCosts!!.turnOrZero(s.edge, node, rootEdge) else turnCosts!!.turnOrZero(rootEdge, node, s.edge)
                if (turn.isInfinite()) continue
                total += turn
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
        val inEdge = if (edgeBased) keyEdge(key, node) else -1
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
