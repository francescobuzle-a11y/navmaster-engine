package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntHashSet
import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.weighting.TurnCostProvider

/**
 * Il percorso migliore visto dalle varianti: per ogni tratto le posizioni p = 0..size-2, cioè i nodi
 * di giunzione `edges[p].adjNode` fra l'arco `edges[p]` ([inEdge]) e `edges[p+1]` ([outEdge]), con i
 * pesi, i tempi e i metri cumulati nel tratto fino a quel nodo (compreso `edges[p]`). Gli indici
 * degli array sono globali: il tratto l occupa [legStart][l] until [legStart][l+1].
 */
class BestTrace(legs: List<RoutePath>) {
    val legStart: IntArray = IntArray(legs.size + 1)
    val node: IntArray
    val inEdge: IntArray
    val outEdge: IntArray
    val pre: DoubleArray
    val preMs: LongArray
    val preM: DoubleArray
    /** Il peso di ogni tratto. */
    val legW: DoubleArray = DoubleArray(legs.size) { legs[it].weight }
    /** Gli archi del migliore. */
    val edgeSet: IntHashSet = IntHashSet()
    /** nodo → la sua prima posizione (globale). */
    val posOf: IntIntHashMap = IntIntHashMap()

    init {
        var total = 0
        for ((l, leg) in legs.withIndex()) { legStart[l] = total; total += maxOf(0, leg.edges.size - 1) }
        legStart[legs.size] = total
        node = IntArray(total); inEdge = IntArray(total); outEdge = IntArray(total)
        pre = DoubleArray(total); preMs = LongArray(total); preM = DoubleArray(total)
        for ((l, leg) in legs.withIndex()) {
            var w = 0.0; var ms = 0L; var m = 0.0
            val es = leg.edges
            for (e in es) edgeSet.add(e.edge)
            for (p in 0 until es.size - 1) {
                val g = legStart[l] + p
                w += es[p].weight; ms += es[p].millis; m += es[p].distance
                node[g] = es[p].adjNode; inEdge[g] = es[p].edge; outEdge[g] = es[p + 1].edge
                pre[g] = w; preMs[g] = ms; preM[g] = m
                if (!posOf.containsKey(node[g])) posOf.put(node[g], g)
            }
        }
    }

    /** Il tratto della posizione globale [g]. */
    fun legOf(g: Int): Int { var l = 0; while (l + 1 < legStart.size - 1 && legStart[l + 1] <= g) l++; return l }
}

/**
 * Una deviazione dal migliore nel tratto [leg]: lascia il migliore alla posizione [fromPos] e vi
 * rientra a [toPos] (posizioni nel tratto, vedi [BestTrace]) per gli archi [arcs]; costa [dW] di
 * peso, [dMs] di tempo e [dM] metri più del migliore fra le due posizioni. Fonte M (passo 6).
 */
class Detour(val leg: Int, val fromPos: Int, val toPos: Int, val arcs: IntArray, val dW: Double, val dMs: Long, val dM: Double,
             val plateauW: Double, val detourMs: Long, val bestMsBetween: Long,
             /** I metri della deviazione e il punteggio con cui si è scelta (più basso = meglio). */
             val detourM: Double = 0.0, val score: Double = 0.0) {
    /** Vero se le due deviazioni si possono fare insieme: tratti diversi, o una rientra prima che l'altra esca. */
    fun disjoint(o: Detour): Boolean = leg != o.leg || toPos < o.fromPos || o.toPos < fromPos
}

/** Una svolta vietata per le deviazioni: i divieti del profilo ([TurnCostProvider]), nelle prove un divieto finto. */
fun interface TurnCheck { fun forbidden(inEdge: Int, node: Int, outEdge: Int): Boolean }

/**
 * Le deviazioni dal migliore ([trace]) sulla rete autostradale in RAM ([net]): per tratto una
 * ricerca in avanti dalle posizioni del migliore (etichette [f] con l'origine) e una all'indietro
 * verso di esse (etichette [g] con la destinazione), entrambe sugli archi diretti della rete, con i
 * nodi del migliore come barriera (non si espandono) e senza i suoi archi. Una deviazione è una
 * catena di archi su cui le due etichette coincidono (plateau, come le alternative «via» di ORS),
 * col rientro più avanti dell'uscita. Le uniche letture del grafo sono le svolte nei nodi con
 * divieti ([MotorwayNet.restricted]). [allowed]: i nodi usabili ([MotorwayNet.mask]; null tutti),
 * gli archi con un capo fuori non si usano mai. Gli array per arco si riusano fra i tratti: un
 * oggetto per richiesta, su un thread.
 */
class DetourSearch internal constructor(private val net: MotorwayNet, private val trace: BestTrace, private val turns: TurnCheck?,
                                        allowed: BooleanArray? = null) {
    constructor(net: MotorwayNet, trace: BestTrace, tcp: TurnCostProvider?, allowed: BooleanArray? = null) :
        this(net, trace, tcp?.let { t -> TurnCheck { i, n, o -> t.turnWeight(i, n, o).isInfinite() } }, allowed)

    private val nA = net.arcCount
    private val f = DoubleArray(nA); private val g = DoubleArray(nA)
    private val fOrig = IntArray(nA); private val fPar = IntArray(nA)
    private val gDest = IntArray(nA); private val gNext = IntArray(nA)
    /** I metri delle etichette: dall'uscita dal migliore fino all'arco compreso ([fM]), dall'arco compreso al rientro ([gM]). */
    private val fM = FloatArray(nA); private val gM = FloatArray(nA)
    /** I nodi della rete che sono nodi del migliore (barriera). */
    private val onB = BooleanArray(net.nodeCount).also { b -> for (n in trace.node) { val k = net.netNode(n); if (k >= 0) b[k] = true } }
    /** Gli archi mai usati: quelli del migliore (in un verso o nell'altro) e quelli con un capo fuori da [allowed]. */
    private val arcOnB = BooleanArray(nA) {
        trace.edgeSet.contains(net.arcEdge[it]) || (allowed != null && (!allowed[net.arcTail[it]] || !allowed[net.arcHead[it]]))
    }
    private val heap = ArcHeap()

    private fun forbidden(inEdge: Int, node: Int, outEdge: Int): Boolean = turns?.forbidden(inEdge, node, outEdge) == true

    /**
     * Le deviazioni del tratto [leg], al più [perLeg], dal punteggio migliore (Δ peso relativo meno
     * metà della quota di plateau): solo percorsi via con peso ≤ [maxFactor] volte il tratto, quota
     * di plateau ≥ [minPlateauShare] (ottimalità locale), almeno [minDetourM] metri fuori dal
     * migliore, nessun nodo ripetuto; due con la stessa uscita e lo stesso rientro e l'80 % dei km
     * in comune contano per una.
     */
    fun detours(leg: Int, maxFactor: Double, minPlateauShare: Double = 0.25, perLeg: Int = 12, cancel: CancelToken = CancelToken.NEVER,
                minDetourM: Double = 0.0): List<Detour> {
        val s = trace.legStart[leg]; val e = trace.legStart[leg + 1]
        if (e - s < 2 || nA == 0) return emptyList()
        val legW = trace.legW[leg]
        val limit = maxFactor * legW
        java.util.Arrays.fill(f, INF); java.util.Arrays.fill(g, INF)
        java.util.Arrays.fill(fOrig, -1); java.util.Arrays.fill(fPar, -1); java.util.Arrays.fill(gDest, -1); java.util.Arrays.fill(gNext, -1)
        var steps = 0
        // 1. avanti: dagli archi che lasciano il migliore in una sua posizione
        heap.clear()
        for (p in s until e) {
            val n = net.netNode(trace.node[p]); if (n < 0) continue
            for (k in net.outStart[n] until net.outStart[n + 1]) {
                val a = net.outArcs[k]
                if (arcOnB[a] || net.arcEdge[a] == trace.inEdge[p]) continue
                if (net.restricted[n] && forbidden(trace.inEdge[p], trace.node[p], net.arcEdge[a])) continue
                val v = trace.pre[p] + net.arcW[a]
                if (v <= limit && v < f[a]) { f[a] = v; fOrig[a] = p; fPar[a] = -1; fM[a] = net.arcM[a]; heap.push(v, a) }
            }
        }
        while (heap.size > 0) {
            if ((++steps and 255) == 0) cancel.check()
            val key = heap.topKey(); val a = heap.pop()
            if (key > f[a]) continue
            val h = net.arcHead[a]
            if (onB[h]) continue
            for (k in net.outStart[h] until net.outStart[h + 1]) {
                val b = net.outArcs[k]
                if (arcOnB[b] || net.arcEdge[b] == net.arcEdge[a]) continue
                if (net.restricted[h] && forbidden(net.arcEdge[a], net.nodeGraph[h], net.arcEdge[b])) continue
                val v = key + net.arcW[b]
                if (v <= limit && v < f[b]) { f[b] = v; fOrig[b] = fOrig[a]; fPar[b] = a; fM[b] = fM[a] + net.arcM[b]; heap.push(v, b) }
            }
        }
        // 2. indietro: dagli archi che rientrano nel migliore in una sua posizione
        heap.clear()
        for (q in s until e) {
            val n = net.netNode(trace.node[q]); if (n < 0) continue
            for (k in net.inStart[n] until net.inStart[n + 1]) {
                val a = net.inArcs[k]
                if (arcOnB[a] || net.arcEdge[a] == trace.outEdge[q]) continue
                if (net.restricted[n] && forbidden(net.arcEdge[a], trace.node[q], trace.outEdge[q])) continue
                val v = legW - trace.pre[q] + net.arcW[a]
                if (v <= limit && v < g[a]) { g[a] = v; gDest[a] = q; gNext[a] = -1; gM[a] = net.arcM[a]; heap.push(v, a) }
            }
        }
        while (heap.size > 0) {
            if ((++steps and 255) == 0) cancel.check()
            val key = heap.topKey(); val a = heap.pop()
            if (key > g[a]) continue
            val t = net.arcTail[a]
            if (onB[t]) continue
            for (k in net.inStart[t] until net.inStart[t + 1]) {
                val b = net.inArcs[k]
                if (arcOnB[b] || net.arcEdge[b] == net.arcEdge[a]) continue
                if (net.restricted[t] && forbidden(net.arcEdge[b], net.nodeGraph[t], net.arcEdge[a])) continue
                val v = key + net.arcW[b]
                if (v <= limit && v < g[b]) { g[b] = v; gDest[b] = gDest[a]; gNext[b] = a; gM[b] = gM[a] + net.arcM[b]; heap.push(v, b) }
            }
        }
        cancel.check()
        // 3-4. le catene di plateau (a → n legati se gNext[a] == n e fPar[n] == a), dalle teste
        class Chain(val head: Int, val tail: Int, val p: Int, val q: Int, val plateauW: Double, val detourW: Double, val score: Double)
        val chains = ArrayList<Chain>()
        for (a in 0 until nA) {
            if (f[a] == INF || g[a] == INF || gDest[a] <= fOrig[a]) continue
            val x = fPar[a]
            if (x >= 0 && gNext[x] == a) continue // non è una testa
            var plateau = net.arcW[a]; var cur = a
            while (true) { val nx = gNext[cur]; if (nx < 0 || fPar[nx] != cur) break; cur = nx; plateau += net.arcW[cur] }
            val tot = f[a] + g[a] - net.arcW[a]
            if (tot > limit) continue
            // corta (un giro di svincolo): fuori prima del tetto delle ricostruite (1 m di tolleranza sulle somme in float)
            if (fM[a] + gM[a] - net.arcM[a] < minDetourM - 1.0) continue
            val p = fOrig[a]; val q = gDest[a]
            val detourW = tot - legW + (trace.pre[q] - trace.pre[p])
            if (detourW <= 0 || plateau < minPlateauShare * detourW) continue
            chains += Chain(a, cur, p, q, plateau, detourW, score(tot - legW, plateau, detourW, legW))
        }
        chains.sortWith(compareBy<Chain>({ it.score }, { it.head }))
        // 5. le prime perLeg, ricostruite (risalita di fPar, discesa di gNext)
        val out = ArrayList<Detour>()
        val seen = com.carrotsearch.hppc.IntHashSet()
        var built = 0
        for (c in chains) {
            if (out.size >= perLeg || built >= perLeg * MAX_BUILT_PER_KEPT) break
            built++
            val up = com.carrotsearch.hppc.IntArrayList()
            var cur = c.tail
            while (cur >= 0 && up.size() <= nA) { up.add(cur); cur = fPar[cur] }
            val arcs = com.carrotsearch.hppc.IntArrayList(up.size() + 16)
            for (i in up.size() - 1 downTo 0) arcs.add(up.get(i))
            cur = gNext[c.tail]
            while (cur >= 0 && arcs.size() <= nA) { arcs.add(cur); cur = gNext[cur] }
            val ids = arcs.toArray()
            seen.clear()
            var repeated = !seen.add(net.arcTail[ids[0]])
            var m = 0.0; var ms = 0L
            for (a in ids) { if (!seen.add(net.arcHead[a])) repeated = true; m += net.arcM[a]; ms += net.arcMs[a] }
            if (repeated || m < minDetourM) continue
            val p = c.p - s; val q = c.q - s
            if (out.any { o -> o.fromPos == p && o.toPos == q && commonMeters(o.arcs, ids) >= UNIFY_SHARE * minOf(m, o.detourM) }) continue
            val bestMs = trace.preMs[c.q] - trace.preMs[c.p]
            out += Detour(leg, p, q, ids, c.detourW - (trace.pre[c.q] - trace.pre[c.p]), ms - bestMs, m - (trace.preM[c.q] - trace.preM[c.p]),
                c.plateauW, ms, bestMs, m, c.score)
        }
        return out
    }

    /**
     * Il punteggio di [d] sul viaggio, per confrontare deviazioni di tratti diversi: come
     * [Detour.score] ma col Δ di peso riferito a [tripW] (il peso di tutti i tratti) invece che al
     * suo tratto. Con [Detour.score] una deviazione di un tratto corto valeva tante volte peggio di
     * una di un tratto lungo con gli stessi minuti in più (il rapporto fra i due tratti), anche se
     * sul viaggio pesava meno; con un tratto solo i due punteggi coincidono.
     */
    fun tripScore(d: Detour, tripW: Double): Double {
        val s = trace.legStart[d.leg]
        return score(d.dW, d.plateauW, d.dW + (trace.pre[s + d.toPos] - trace.pre[s + d.fromPos]), tripW)
    }

    private fun commonMeters(a: IntArray, b: IntArray): Double {
        val set = com.carrotsearch.hppc.IntHashSet(a.size * 2)
        for (x in a) set.add(x)
        var m = 0.0
        for (x in b) if (set.contains(x)) m += net.arcM[x]
        return m
    }

    /**
     * Le combinazioni di 2 (e, con [maxParts] 3, di 3) deviazioni compatibili ([Detour.disjoint])
     * fra le prime 8 di [ds] per punteggio ([key]: [Detour.score], sul tratto, o [tripScore], sul
     * viaggio), dalla più leggera, al più [limit]. Il Δ di una
     * combinazione è la somma: fra le deviazioni resta il migliore, e le svolte di raccordo sono già controllate.
     */
    fun combos(ds: List<Detour>, maxParts: Int = 3, limit: Int = 40, key: (Detour) -> Double = { it.score }): List<List<Detour>> {
        val top = ds.sortedBy(key).take(COMBO_TOP)
        val out = ArrayList<List<Detour>>()
        for (i in top.indices) for (j in i + 1 until top.size) {
            if (!top[i].disjoint(top[j])) continue
            out += sortedParts(top[i], top[j])
            if (maxParts >= 3) for (k in j + 1 until top.size) if (top[k].disjoint(top[i]) && top[k].disjoint(top[j])) out += sortedParts(top[i], top[j], top[k])
        }
        return out.sortedBy { c -> c.sumOf { it.dW } }.take(limit)
    }

    private fun sortedParts(vararg d: Detour): List<Detour> = d.sortedWith(compareBy({ it.leg }, { it.fromPos }))

    /** Una coda di priorità di archi (chiave double) con le chiavi vecchie lasciate dentro (scartate all'uscita). */
    private class ArcHeap {
        private var keys = DoubleArray(256); private var vals = IntArray(256)
        var size = 0; private set
        fun clear() { size = 0 }
        fun topKey(): Double = keys[0]
        fun push(k: Double, v: Int) {
            if (size == keys.size) { keys = keys.copyOf(size * 2); vals = vals.copyOf(size * 2) }
            var i = size++
            while (i > 0) { val p = (i - 1) / 2; if (keys[p] <= k) break; keys[i] = keys[p]; vals[i] = vals[p]; i = p }
            keys[i] = k; vals[i] = v
        }
        fun pop(): Int {
            val top = vals[0]
            val k = keys[--size]; val v = vals[size]
            var i = 0
            while (true) {
                var c = 2 * i + 1
                if (c >= size) break
                if (c + 1 < size && keys[c + 1] < keys[c]) c++
                if (keys[c] >= k) break
                keys[i] = keys[c]; vals[i] = vals[c]; i = c
            }
            if (size > 0) { keys[i] = k; vals[i] = v }
            return top
        }
    }

    companion object {
        private const val INF = Double.POSITIVE_INFINITY
        /** Due deviazioni con la stessa uscita e lo stesso rientro e questa quota di km in comune contano per una. */
        const val UNIFY_SHARE = 0.8
        /** Le singole da cui si combinano le combinazioni. */
        const val COMBO_TOP = 8
        /** Quante catene si ricostruiscono al più per deviazione tenuta (le altre cadono per nodi ripetuti o doppioni). */
        private const val MAX_BUILT_PER_KEPT = 20

        /**
         * Il punteggio di una deviazione (più basso = meglio): il Δ di peso [dW] riferito a [refW]
         * (il tratto in [detours], il viaggio in [tripScore]) meno metà della quota di plateau
         * ([plateauW] sul peso [detourW] della deviazione fra uscita e rientro).
         */
        internal fun score(dW: Double, plateauW: Double, detourW: Double, refW: Double): Double = dW / refW - 0.5 * plateauW / detourW
    }
}
