package it.navmaster.engine.ch

import com.carrotsearch.hppc.IntArrayList
import it.navmaster.engine.routing.NodeHeap
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.Weighting
import java.util.Random

/**
 * Grafo di preparazione: archi **orientati** ("arc") in liste concatenate per nodo (uscenti ed entranti).
 * Ogni arco base dà fino a due arc (A→B id `2e`, B→A id `2e+1`); gli shortcut sono arc successivi.
 */
class PrepareGraph(val nodes: Int, val baseEdges: Int) {
    private var cap = baseEdges * 2 + 1024
    var from = IntArray(cap); private set
    var to = IntArray(cap); private set
    var weight = DoubleArray(cap); private set
    var skip1 = IntArray(cap); private set   // arc saltati (per gli shortcut), -1 per gli archi base
    var skip2 = IntArray(cap); private set
    var origCount = IntArray(cap); private set
    /** Id di arco CH dello shortcut già scritto nello storage, -1 se non ancora. */
    var storedId = IntArray(cap); private set
    private var nextOut = IntArray(cap)
    private var nextIn = IntArray(cap)
    private var prevOut = IntArray(cap)
    private var prevIn = IntArray(cap)
    val headOut = IntArray(nodes) { -1 }
    val headIn = IntArray(nodes) { -1 }
    val degree = IntArray(nodes)
    var arcCount = 0; private set

    val isBaseArc: (Int) -> Boolean = { it < baseEdges * 2 }
    fun baseEdgeOf(arc: Int) = arc ushr 1
    /** Vero se l'arc base è percorso nel verso B→A. */
    fun baseArcReverse(arc: Int) = arc and 1 == 1

    fun addArc(f: Int, t: Int, w: Double, s1: Int, s2: Int, orig: Int): Int {
        if (arcCount == cap) grow()
        val id = arcCount++
        from[id] = f; to[id] = t; weight[id] = w; skip1[id] = s1; skip2[id] = s2; origCount[id] = orig; storedId[id] = -1
        // inserimento in testa nelle due liste
        nextOut[id] = headOut[f]; prevOut[id] = -1; if (headOut[f] >= 0) prevOut[headOut[f]] = id; headOut[f] = id
        nextIn[id] = headIn[t]; prevIn[id] = -1; if (headIn[t] >= 0) prevIn[headIn[t]] = id; headIn[t] = id
        degree[f]++; if (t != f) degree[t]++
        return id
    }

    /** Riserva un id di arc base specifico (gli arc base hanno id fissi 2e / 2e+1). */
    fun addBaseArc(id: Int, f: Int, t: Int, w: Double) {
        while (arcCount <= id) { if (arcCount == cap) grow(); from[arcCount] = -1; to[arcCount] = -1; weight[arcCount] = Double.POSITIVE_INFINITY; nextOut[arcCount] = -1; nextIn[arcCount] = -1; arcCount++ }
        from[id] = f; to[id] = t; weight[id] = w; skip1[id] = -1; skip2[id] = -1; origCount[id] = 1; storedId[id] = -1
        nextOut[id] = headOut[f]; prevOut[id] = -1; if (headOut[f] >= 0) prevOut[headOut[f]] = id; headOut[f] = id
        nextIn[id] = headIn[t]; prevIn[id] = -1; if (headIn[t] >= 0) prevIn[headIn[t]] = id; headIn[t] = id
        degree[f]++; if (t != f) degree[t]++
    }

    fun nextOut(arc: Int) = nextOut[arc]
    fun nextIn(arc: Int) = nextIn[arc]

    /** Garantisce che gli id degli shortcut partano da [count], oltre lo spazio degli arc base. */
    fun padTo(count: Int) {
        while (arcCount < count) {
            if (arcCount == cap) grow()
            from[arcCount] = -1; to[arcCount] = -1; weight[arcCount] = Double.POSITIVE_INFINITY
            nextOut[arcCount] = -1; nextIn[arcCount] = -1; storedId[arcCount] = -1
            arcCount++
        }
    }

    private fun removeArc(arc: Int) {
        val f = from[arc]; val t = to[arc]
        if (prevOut[arc] >= 0) nextOut[prevOut[arc]] = nextOut[arc] else headOut[f] = nextOut[arc]
        if (nextOut[arc] >= 0) prevOut[nextOut[arc]] = prevOut[arc]
        if (prevIn[arc] >= 0) nextIn[prevIn[arc]] = nextIn[arc] else headIn[t] = nextIn[arc]
        if (nextIn[arc] >= 0) prevIn[nextIn[arc]] = prevIn[arc]
        degree[f]--; if (t != f) degree[t]--
    }

    /** Stacca il nodo dal grafo di preparazione; ritorna i vicini distinti. */
    fun disconnect(node: Int, neighbours: IntArrayList) {
        neighbours.clear()
        var a = headOut[node]
        while (a >= 0) { val n = nextOut[a]; if (to[a] != node && !neighbours.contains(to[a])) neighbours.add(to[a]); removeArc(a); a = n }
        a = headIn[node]
        while (a >= 0) { val n = nextIn[a]; if (from[a] != node && !neighbours.contains(from[a])) neighbours.add(from[a]); removeArc(a); a = n }
    }

    /** Shortcut esistente f→t, o -1. */
    fun findArc(f: Int, t: Int): Int {
        var a = headOut[f]
        while (a >= 0) { if (to[a] == t && !isBaseArc(a)) return a; a = nextOut[a] }
        return -1
    }

    private fun grow() {
        cap *= 2
        from = from.copyOf(cap); to = to.copyOf(cap); weight = weight.copyOf(cap); skip1 = skip1.copyOf(cap); skip2 = skip2.copyOf(cap)
        origCount = origCount.copyOf(cap); storedId = storedId.copyOf(cap)
        nextOut = nextOut.copyOf(cap); nextIn = nextIn.copyOf(cap); prevOut = prevOut.copyOf(cap); prevIn = prevIn.copyOf(cap)
    }
}

/** Dijkstra locale limitato per la ricerca dei testimoni (GraphHopper `NodeBasedWitnessPathSearcher`). */
class WitnessPathSearcher(private val g: PrepareGraph) {
    private val weights = DoubleArray(g.nodes) { Double.POSITIVE_INFINITY }
    private val changed = IntArrayList()
    private val heap = NodeHeap(1024)
    private var ignoreNode = -1
    private var settledNodes = 0

    fun init(source: Int, ignore: Int) {
        reset()
        ignoreNode = ignore
        weights[source] = 0.0; changed.add(source)
        heap.push(source, 0.0)
        settledNodes = 0
    }

    private fun reset() {
        for (i in 0 until changed.size()) weights[changed.get(i)] = Double.POSITIVE_INFINITY
        changed.clear(); heap.clear()
    }

    /**
     * Limite superiore del peso da `source` a [target]: esplora finché la testa dell'heap non supera
     * [acceptedWeight] o si sono sistemati [maxSettled] nodi. Riutilizzabile per più target dalla stessa sorgente.
     */
    fun findUpperBound(target: Int, acceptedWeight: Double, maxSettled: Int): Double {
        if (weights[target] <= acceptedWeight) return weights[target]
        while (!heap.isEmpty && settledNodes < maxSettled && heap.peekKey() <= acceptedWeight) {
            val node = heap.pop()
            val w = weights[node]
            if (heap.size > 0 && w > heap.peekKey() + 1e-12) { /* voce pigra */ }
            settledNodes++
            var a = g.headOut[node]
            while (a >= 0) {
                val adj = g.to[a]
                if (adj != ignoreNode) {
                    val nw = w + g.weight[a]
                    if (nw < weights[adj]) {
                        if (weights[adj].isInfinite()) changed.add(adj)
                        weights[adj] = nw
                        heap.push(adj, nw)
                    }
                }
                a = g.nextOut(a)
            }
            if (node == target) return weights[target]
        }
        return weights[target]
    }
}

/**
 * Preparazione di una Contraction Hierarchy node-based (GraphHopper `PrepareContractionHierarchies`
 * + `NodeBasedNodeContractor`): ordinamento euristico (differenza archi ×10 + archi originali ×1),
 * aggiornamenti periodici (20 %), pigri (ultimo 10 %) e dei vicini (20 %, `Random(123)`).
 */
class CHPreparation(
    private val graph: BaseGraph,
    private val weighting: Weighting,
    private val storage: CHStorage,
    private val filter: EdgeFilter = EdgeFilter.ALL,
    private val params: Params = Params(),
) {
    data class Params(
        val periodicUpdatesPercent: Int = 20,
        val lazyUpdatesPercent: Int = 10,
        val neighborUpdatesPercent: Int = 20,
        val edgeDifferenceWeight: Double = 10.0,
        val originalEdgesCountWeight: Double = 1.0,
        /** Frazione di nodi da NON contrarre (il "core"); 0 = CH completa. */
        val uncontractedPercent: Double = 0.0,
        val log: ((String) -> Unit)? = null,
    )

    private val n = graph.nodeCount
    private val pg = PrepareGraph(n, graph.edgeCount)
    private val witness = WitnessPathSearcher(pg)
    private val priorities = DoubleArray(n)
    private val neighbours = IntArrayList()
    private var meanDegree = 0.0
    private val random = Random(123)
    var shortcutsAdded = 0; private set
    var contracted = 0; private set

    /** Nodi che non devono essere contratti (per il Core di ORS); default nessuno. */
    var doNotContract: (Int) -> Boolean = { false }

    fun prepare() {
        buildPrepareGraph()
        storage.initLevels()
        val candidate = BooleanArray(n) { !doNotContract(it) }
        val heap = NodeHeap(maxOf(16, n))
        var init = 0
        // come GraphHopper, le priorità iniziali sono calcolate con meanDegree = 0: ogni candidato conta come shortcut
        for (v in 0 until n) if (candidate[v]) { priorities[v] = calculatePriority(v); heap.push(v, priorities[v]); init++ }
        meanDegree = (graph.edgeCount / maxOf(1, n)).toDouble() // divisione intera come GraphHopper
        val p = params
        val periodicCount = if (p.periodicUpdatesPercent == 0) Long.MAX_VALUE else Math.round(maxOf(10.0, init * p.periodicUpdatesPercent / 100.0))
        val lazyThreshold = Math.round(init * p.lazyUpdatesPercent / 100.0)
        val nodesToAvoid = Math.round(init * p.uncontractedPercent / 100.0)
        val contractedFlag = BooleanArray(n)
        var level = 0
        var checkCounter = 0L
        var remaining = init
        val t0 = System.currentTimeMillis()
        while (!heap.isEmpty && remaining > 0) {
            if (checkCounter > 0 && checkCounter % periodicCount == 0L) {
                heap.clear()
                for (v in 0 until n) if (candidate[v] && !contractedFlag[v]) { priorities[v] = calculatePriority(v); heap.push(v, priorities[v]) }
            }
            checkCounter++
            val key = heap.peekKey()
            val node = heap.pop()
            if (contractedFlag[node] || key != priorities[node]) continue // voce pigra superata
            if (remaining > 1 && remaining < lazyThreshold) {
                val prio = calculatePriority(node)
                priorities[node] = prio
                if (!heap.isEmpty && prio > heap.peekKey()) { heap.push(node, prio); continue }
            }
            contractNode(node, level++)
            contractedFlag[node] = true
            remaining--
            if (remaining <= nodesToAvoid) break
            for (i in 0 until neighbours.size()) {
                val nb = neighbours.get(i)
                if (contractedFlag[nb] || !candidate[nb]) continue
                if (random.nextInt(100) < p.neighborUpdatesPercent) {
                    priorities[nb] = calculatePriority(nb)
                    heap.push(nb, priorities[nb]) // la voce vecchia resta nell'heap e verrà scartata
                }
            }
            p.log?.let { if (contracted % 100_000 == 0) it("contratti $contracted/$init, shortcut $shortcutsAdded, ${(System.currentTimeMillis() - t0) / 1000}s") }
        }
        // nodi non contratti: livello massimo (core); i loro shortcut incidenti vanno comunque scritti
        for (v in 0 until n) if (!contractedFlag[v]) storage.setLevel(v, n)
        storage.coreNodeCount = n - contracted
        writeRemainingShortcuts(contractedFlag)
    }

    private fun buildPrepareGraph() {
        val it = graph.createEdgeIterator()
        for (e in 0 until graph.edgeCount) {
            it.setEdge(e, graph.nodeA(e))
            if (!filter.accept(it)) continue
            val wf = weighting.edgeWeight(it, false)
            val wb = weighting.edgeWeight(it, true)
            if (!wf.isInfinite()) pg.addBaseArc(2 * e, graph.nodeA(e), graph.nodeB(e), wf)
            if (!wb.isInfinite() && graph.nodeA(e) != graph.nodeB(e)) pg.addBaseArc(2 * e + 1, graph.nodeB(e), graph.nodeA(e), wb)
        }
        pg.padTo(2 * graph.edgeCount)
    }

    private var shortcutsCount = 0
    private var origEdgesCount = 0

    private fun calculatePriority(v: Int): Double {
        shortcutsCount = 0; origEdgesCount = 0
        findAndHandleShortcuts(v, countOnly = true)
        return params.edgeDifferenceWeight * (shortcutsCount - pg.degree[v]) + params.originalEdgesCountWeight * origEdgesCount
    }

    private fun contractNode(v: Int, level: Int) {
        storage.setLevel(v, level)
        writeShortcutsIncidentTo(v)
        shortcutsCount = 0; origEdgesCount = 0
        findAndHandleShortcuts(v, countOnly = false)
        pg.disconnect(v, neighbours)
        contracted++
    }

    /** Cerca gli shortcut necessari attorno a [v]; se [countOnly] li conta soltanto. */
    private fun findAndHandleShortcuts(v: Int, countOnly: Boolean) {
        val maxSettled = (meanDegree.toInt() * 100)
        var degree = 0
        var inArc = pg.headIn[v]
        while (inArc >= 0) {
            val u = pg.from[inArc]
            val wIn = pg.weight[inArc]
            if (u == v || wIn.isInfinite()) { inArc = pg.nextIn(inArc); continue }
            degree++
            witness.init(u, v)
            var outArc = pg.headOut[v]
            while (outArc >= 0) {
                val w = pg.to[outArc]
                val wOut = pg.weight[outArc]
                if (w != v && w != u && !wOut.isInfinite()) {
                    val direct = wIn + wOut
                    val bound = witness.findUpperBound(w, direct, maxSettled)
                    if (bound > direct) {
                        val orig = pg.origCount[inArc] + pg.origCount[outArc]
                        if (countOnly) { shortcutsCount++; origEdgesCount += orig }
                        else addOrUpdateShortcut(u, w, direct, inArc, outArc, orig)
                    }
                }
                outArc = pg.nextOut(outArc)
            }
            inArc = pg.nextIn(inArc)
        }
        if (!countOnly) meanDegree = (meanDegree * 2 + degree) / 3
    }

    private fun addOrUpdateShortcut(u: Int, w: Int, weight: Double, inArc: Int, outArc: Int, orig: Int) {
        val existing = pg.findArc(u, w)
        if (existing >= 0) {
            if (weight < pg.weight[existing]) { pg.weight[existing] = weight; pg.skip1[existing] = inArc; pg.skip2[existing] = outArc; pg.origCount[existing] = orig }
            return
        }
        pg.addArc(u, w, weight, inArc, outArc, orig)
        shortcutsAdded++
    }

    /** Id di arco CH di un arc: arco base, oppure lo shortcut già scritto. */
    private fun chEdgeOf(arc: Int): Int {
        if (pg.isBaseArc(arc)) return pg.baseEdgeOf(arc)
        val s = pg.storedId[arc]
        check(s >= 0) { "shortcut saltato non ancora scritto" }
        return s
    }

    /** Primo arco base dell'arc percorso nel suo verso (from→to). */
    private fun firstBaseOf(arc: Int): Int {
        if (pg.isBaseArc(arc)) return pg.baseEdgeOf(arc)
        val sc = storage.shortcutIndex(pg.storedId[arc])
        // il record è orientato nodeA→nodeB: se l'arc parte da nodeA il primo è origFirst, altrimenti origLast
        return if (storage.nodeA(sc) == pg.from[arc]) storage.origFirst(sc) else storage.origLast(sc)
    }

    private fun lastBaseOf(arc: Int): Int {
        if (pg.isBaseArc(arc)) return pg.baseEdgeOf(arc)
        val sc = storage.shortcutIndex(pg.storedId[arc])
        return if (storage.nodeA(sc) == pg.from[arc]) storage.origLast(sc) else storage.origFirst(sc)
    }

    /** Scrive un arc di shortcut u→w registrato su [nodeA] (= u se out, = w se in), con l'orientamento del record. */
    private fun writeArc(arc: Int, nodeA: Int, fwd: Boolean, bwd: Boolean): Int {
        val from = pg.from[arc]; val to = pg.to[arc]
        val s1 = chEdgeOf(pg.skip1[arc]); val s2 = chEdgeOf(pg.skip2[arc])
        val first = firstBaseOf(pg.skip1[arc]); val last = lastBaseOf(pg.skip2[arc])
        return if (nodeA == from) storage.addShortcut(from, to, fwd, bwd, pg.weight[arc], s1, s2, first, last)
        else storage.addShortcut(to, from, fwd, bwd, pg.weight[arc], s2, s1, last, first) // orientamento invertito
    }

    /** Scrive nello storage gli shortcut (non base) incidenti a [v], unendo le coppie uguali nei due versi. */
    private fun writeShortcutsIncidentTo(v: Int) {
        // uscenti v→w: fwd
        var a = pg.headOut[v]
        while (a >= 0) {
            if (!pg.isBaseArc(a) && pg.storedId[a] < 0) {
                val w = pg.to[a]
                // esiste il gemello w→v con stesso peso e stessi archi saltati (in ordine inverso)?
                val twin = pg.findArc(w, v)
                if (twin >= 0 && pg.storedId[twin] < 0 && pg.weight[twin] == pg.weight[a] && sameSkipped(a, twin)) {
                    val id = writeArc(a, v, fwd = true, bwd = true)
                    pg.storedId[a] = id; pg.storedId[twin] = id
                } else {
                    pg.storedId[a] = writeArc(a, v, fwd = true, bwd = false)
                }
            }
            a = pg.nextOut(a)
        }
        // entranti u→v non ancora scritti: bwd rispetto a nodeA = v (orientamento del record invertito)
        a = pg.headIn[v]
        while (a >= 0) {
            if (!pg.isBaseArc(a) && pg.storedId[a] < 0) pg.storedId[a] = writeArc(a, v, fwd = false, bwd = true)
            a = pg.nextIn(a)
        }
    }

    /** I due arc u→w e w→u saltano gli stessi archi base/shortcut (in ordine opposto). */
    private fun sameSkipped(a: Int, twin: Int): Boolean =
        chEdgeOf(pg.skip1[a]) == chEdgeOf(pg.skip2[twin]) && chEdgeOf(pg.skip2[a]) == chEdgeOf(pg.skip1[twin])

    /**
     * Shortcut fra nodi del core, mai scritti perché nessuno dei due estremi è stato contratto.
     * Nel core tutti i nodi hanno lo stesso livello e la ricerca di fase 2 li percorre da entrambi
     * i lati, quindi ogni shortcut viene registrato su **entrambi** gli estremi (nessun altro shortcut
     * li salta, perciò il loro id non serve a nessuno).
     */
    private fun writeRemainingShortcuts(contractedFlag: BooleanArray) {
        val handled = HashSet<Int>()
        for (v in 0 until n) {
            if (contractedFlag[v]) continue
            handled.clear()
            var a = pg.headOut[v]
            while (a >= 0) {
                if (!pg.isBaseArc(a)) {
                    val w = pg.to[a]
                    val twin = pg.findArc(w, v)
                    if (twin >= 0 && pg.weight[twin] == pg.weight[a] && sameSkipped(a, twin)) {
                        writeArc(a, v, fwd = true, bwd = true)
                        handled.add(twin)
                    } else {
                        writeArc(a, v, fwd = true, bwd = false)
                    }
                }
                a = pg.nextOut(a)
            }
            a = pg.headIn[v]
            while (a >= 0) {
                if (!pg.isBaseArc(a) && a !in handled) writeArc(a, v, fwd = false, bwd = true)
                a = pg.nextIn(a)
            }
        }
    }
}
