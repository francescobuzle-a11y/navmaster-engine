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
    /**
     * Il gruppo nell'ordine dei posti di [DetourSearch.detours]: [RANK_LONG] lunga di un corridoio
     * nuovo, [RANK_SAME_CORRIDOR] lunga rimandata (stesso corridoio di una presa), [RANK_SHORT] corta
     * (sotto la deviazione minima del giudice). Le singole offerte e le prime per le combinazioni
     * ([DetourSearch.comboTop]) seguono quest'ordine prima del punteggio.
     */
    var rankClass: Int = RANK_LONG
        internal set

    companion object {
        const val RANK_LONG = 0
        const val RANK_SAME_CORRIDOR = 1
        const val RANK_SHORT = 2
    }

    /** Vero se le due deviazioni si possono fare insieme: tratti diversi, o una rientra prima che l'altra esca. */
    fun disjoint(o: Detour): Boolean = leg != o.leg || toPos < o.fromPos || o.toPos < fromPos
}

/**
 * La traccia di [DetourSearch.detours] (solo le prove diagnostiche delle varianti): le righe vanno a
 * [log]; [ref]: la strada di un arco del grafo (letta solo qui, fuori dai cicli caldi); [watch]: i
 * percorsi da sorvegliare (le sonde: archi in ordine col nodo d'arrivo e i metri); le deviazioni
 * oltre `perLeg` fino a [maxKept] per tratto finiscono in [extra] (mai restituite), col rango della
 * loro catena in [rank] (anche le tenute).
 */
class DetourTrace(val log: (String) -> Unit, val ref: (Int) -> String?, val watch: List<Watch> = emptyList(), val maxKept: Int = 40) {
    class Watch(val name: String, val edges: IntArray, val adjNodes: IntArray, val meters: DoubleArray)
    val extra = ArrayList<Detour>()
    val rank = java.util.IdentityHashMap<Detour, Int>()
}

/** Una svolta vietata per le deviazioni: i divieti del profilo ([TurnCostProvider]), nelle prove un divieto finto. */
fun interface TurnCheck { fun forbidden(inEdge: Int, node: Int, outEdge: Int): Boolean }

/**
 * Le deviazioni dal migliore ([trace]) sulla rete autostradale in RAM ([net]): per tratto una
 * ricerca in avanti dalle posizioni del migliore (etichette [f] con l'origine) e una all'indietro
 * verso di esse (etichette [g] con la destinazione), entrambe sugli archi diretti della rete, con i
 * nodi del migliore come barriera (non si espandono) e senza i suoi archi. Una deviazione è una
 * catena di archi su cui le due etichette coincidono (plateau, come le alternative «via» di ORS),
 * col rientro più avanti dell'uscita. Le svolte nei nodi con divieti ([MotorwayNet.restricted]):
 * fra due archi della rete dalle coppie vietate della rete ([MotorwayNet.turnBans], in RAM, lette
 * una volta alla costruzione), se ci sono; se no (una rete giocattolo) e all'uscita dal migliore e
 * al rientro (un arco del migliore) da [turns]. [allowed]: i nodi usabili ([MotorwayNet.mask]; null
 * tutti), gli archi con un capo fuori non si usano mai. Gli array per arco si riusano fra i tratti:
 * un oggetto per richiesta, su un thread.
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
    private val bans = net.turnBans

    /** La svolta dall'arco [a] all'arco [b] della rete nel nodo [node] (della rete, con divieti) è vietata: dalle coppie in RAM se ci sono. */
    private fun banned(a: Int, node: Int, b: Int): Boolean =
        if (bans != null) bans.contains(MotorwayNet.pairKey(a, b)) else forbidden(net.arcEdge[a], net.nodeGraph[node], net.arcEdge[b])

    /**
     * Le deviazioni del tratto [leg], al più [perLeg]: solo percorsi via con peso ≤ [maxFactor]
     * volte il tratto, quota di plateau ≥ [minPlateauShare] (ottimalità locale), almeno
     * [minDetourM] metri fuori dal migliore, nessun nodo ripetuto; due con la stessa uscita e lo
     * stesso rientro e l'80 % dei km in comune contano per una. L'ordine per i posti di [perLeg]:
     * 1. le catene «lunghe» (deviazione di almeno [rankMinM] metri: la deviazione minima del
     *    giudice, [VariantRules.dMin1] del viaggio) per punteggio (Δ peso relativo meno metà della
     *    quota di plateau), saltando quelle che hanno almeno [CORRIDOR_SHARE] dei propri km in
     *    comune con una lunga già presa (stesso corridoio: rimandate);
     * 2. le lunghe rimandate, nel loro ordine;
     * 3. le corte (sotto [rankMinM]: giri di svincolo, scorciatoie di pochi km che il giudice scarta
     *    da sole, ma che servono alle combinazioni con una lunga), per punteggio.
     * Prima tutte stavano in un solo ordine per punteggio, che premia le deviazioni minuscole con
     * plateau ~1: le vie lunghe su un'altra autostrada finivano oltre il tetto. Una catena è lunga
     * se anche il pezzo del migliore che sostituisce è almeno [LONG_BEST_SHARE] di [rankMinM] (un
     * anello di uscita e rientro è corto). Le lunghe tengono al più perLeg meno [SHORT_RESERVE]
     * posti (un quarto di perLeg al più) se ci sono corte: le corte servono alle combinazioni.
     * Scartate anche: le deviazioni con più di [maxAccessShare] dei km su strade d'accesso
     * ([MotorwayNet.KIND_MAIN], [MotorwayNet.KIND_TERTIARY]: un giro urbano), e quelle che il
     * giudice non accetterebbe mai, nemmeno da sole (tempo oltre [saneA] volte il migliore fra
     * uscita e rientro più [saneBMs] ms: la sanità locale più larga). [diag]: la traccia
     * delle varianti (solo le prove; null: nessun conto in più): righe di registro e le deviazioni
     * oltre [perLeg] in [DetourTrace.extra], mai restituite (le prime [perLeg] sono le stesse senza traccia).
     */
    fun detours(leg: Int, maxFactor: Double, minPlateauShare: Double = 0.25, perLeg: Int = PER_LEG, cancel: CancelToken = CancelToken.NEVER,
                minDetourM: Double = 0.0, diag: DetourTrace? = null, rankMinM: Double = 0.0, maxAccessShare: Double = 1.0,
                saneA: Double = Double.POSITIVE_INFINITY, saneBMs: Double = Double.POSITIVE_INFINITY): List<Detour> {
        val s = trace.legStart[leg]; val e = trace.legStart[leg + 1]
        if (e - s < 2 || nA == 0) { diag?.log?.invoke("varianti: traccia M tratto $leg: ${e - s} posizioni, rete di $nA archi: nessuna ricerca"); return emptyList() }
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
                if (net.restricted[h] && banned(a, h, b)) continue
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
                if (net.restricted[t] && banned(b, t, a)) continue
                val v = key + net.arcW[b]
                if (v <= limit && v < g[b]) { g[b] = v; gDest[b] = gDest[a]; gNext[b] = a; gM[b] = gM[a] + net.arcM[b]; heap.push(v, b) }
            }
        }
        cancel.check()
        // 3-4. le catene di plateau (a → n legati se gNext[a] == n e fPar[n] == a), dalle teste
        // (con la traccia, i conti per motivo: solo incrementi, le condizioni sono le stesse)
        val cnt = if (diag != null) IntArray(5) else null
        val chains = ArrayList<Chain>()
        for (a in 0 until nA) {
            if (f[a] == INF || g[a] == INF) continue
            val x = fPar[a]
            if (x >= 0 && gNext[x] == a) continue // non è una testa
            if (gDest[a] <= fOrig[a]) { if (cnt != null) cnt[0]++; continue }
            var plateau = net.arcW[a]; var cur = a
            while (true) { val nx = gNext[cur]; if (nx < 0 || fPar[nx] != cur) break; cur = nx; plateau += net.arcW[cur] }
            val tot = f[a] + g[a] - net.arcW[a]
            if (tot > limit) { if (cnt != null) cnt[1]++; continue }
            // corta (un giro di svincolo): fuori prima del tetto delle ricostruite (1 m di tolleranza sulle somme in float)
            if (fM[a] + gM[a] - net.arcM[a] < minDetourM - 1.0) { if (cnt != null) cnt[2]++; continue }
            val p = fOrig[a]; val q = gDest[a]
            val detourW = tot - legW + (trace.pre[q] - trace.pre[p])
            if (detourW <= 0 || plateau < minPlateauShare * detourW) { if (cnt != null) cnt[if (detourW <= 0) 3 else 4]++; continue }
            val m = (fM[a] + gM[a] - net.arcM[a]).toDouble()
            // lunga: la deviazione e il pezzo del migliore che sostituisce (un anello di uscita e rientro
            // di 40 km al posto di 400 m è un giro, non una via lunga)
            val long = m >= rankMinM - 1.0 && trace.preM[q] - trace.preM[p] >= LONG_BEST_SHARE * rankMinM - 1.0
            chains += Chain(a, cur, p, q, plateau, detourW, score(tot - legW, plateau, detourW, legW), long)
        }
        // le lunghe prima delle corte, dentro ognuna per punteggio
        chains.sortWith(compareBy<Chain>({ if (it.long) 0 else 1 }, { it.score }, { it.head }))
        val longCount = chains.count { it.long }
        // 5. ricostruite (risalita di fPar, discesa di gNext) in due giri: le lunghe (con quelle dello
        // stesso corridoio di una presa da parte), poi le corte. Le tenute: le lunghe fino a perLeg
        // meno i posti delle corte ([SHORT_RESERVE], se ce ne sono), poi le corte. Con la traccia si
        // continua oltre il tetto (fino a DetourTrace.maxKept) mettendo le altre da parte: le tenute
        // sono quelle che si avrebbero senza traccia (i conti fino al punto in cui ci si sarebbe fermati).
        val seen = com.carrotsearch.hppc.IntHashSet()
        val fate = if (diag != null) HashMap<Int, String>() else null
        var stopAt = -1; var stopWhy = ""; var looked = 0
        val longs = ArrayList<Built>(); val deferred = ArrayList<Built>(); val shorts = ArrayList<Built>()
        // gli archi delle lunghe prese (per il corridoio): un insieme per lunga, fatto una volta
        val longSets = ArrayList<com.carrotsearch.hppc.IntHashSet>()
        var cutLongs = -1; var cutDeferred = -1; var cutShorts = -1
        val maxBuilt = perLeg * MAX_BUILT_PER_KEPT
        val diagBuilt = if (diag != null) maxOf(diag.maxKept, perLeg) * MAX_BUILT_PER_KEPT else 0
        var builtL = 0; var builtS = 0
        fun stop(ci: Int, why: String) { if (stopAt < 0) { stopAt = ci; stopWhy = why } }
        // una catena ricostruita, o null (scartata, col motivo nella traccia)
        fun build(ci: Int, c: Chain): Built? {
            if ((ci and 15) == 0) cancel.check()
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
            var m = 0.0; var ms = 0L; var accM = 0.0
            for (a in ids) {
                if (!seen.add(net.arcHead[a])) repeated = true
                m += net.arcM[a]; ms += net.arcMs[a]
                if (net.kind(a) != MotorwayNet.KIND_MAJOR) accM += net.arcM[a]
            }
            if (repeated || m < minDetourM) { fate?.put(c.head, if (repeated) "nodo ripetuto" else "corta (%.1f km)".format(m / 1000)); return null }
            // quasi tutta su strade d'accesso: un giro urbano, non un'altra autostrada
            if (accM > maxAccessShare * m + 1.0) { fate?.put(c.head, "strade d'accesso (%.0f %%)".format(100 * accM / m)); return null }
            val bestMs = trace.preMs[c.q] - trace.preMs[c.p]
            // mai accettata dal giudice, nemmeno da sola (la sanità più larga): fuori prima dei posti
            if (saneA.isFinite() && ms > saneA * bestMs + saneBMs) { fate?.put(c.head, "locale (%.0f s, migliore %.0f s)".format(ms / 1000.0, bestMs / 1000.0)); return null }
            val p = c.p - s; val q = c.q - s
            val isTwin = { o: Built -> o.d.fromPos == p && o.d.toPos == q && commonMeters(o.d.arcs, ids) >= UNIFY_SHARE * minOf(m, o.d.detourM) }
            if (longs.any(isTwin) || deferred.any(isTwin) || shorts.any(isTwin)) { fate?.put(c.head, "doppione (stesse posizioni, 80 % in comune)"); return null }
            val d = Detour(leg, p, q, ids, c.detourW - (trace.pre[c.q] - trace.pre[c.p]), ms - bestMs, m - (trace.preM[c.q] - trace.preM[c.p]),
                c.plateauW, ms, bestMs, m, c.score)
            diag?.rank?.put(d, ci)
            return Built(d, c.head, m)
        }
        // le lunghe
        for (ci in 0 until longCount) {
            val c = chains[ci]
            if (cutLongs < 0 && (longs.size >= perLeg || builtL >= maxBuilt)) {
                cutLongs = longs.size; cutDeferred = deferred.size
                stop(ci, if (longs.size >= perLeg) "$perLeg tenute (perLeg)" else "$maxBuilt ricostruite")
            }
            if (cutLongs >= 0 && (diag == null || longs.size + deferred.size >= diag.maxKept || builtL >= diagBuilt)) break
            looked = ci + 1
            builtL++
            val b = build(ci, c) ?: continue
            // stesso corridoio di una lunga già presa: rimandata finché ci sono lunghe di altri corridoi
            if (cutLongs < 0 && longSets.any { set -> commonMeters(set, b.d.arcs) >= CORRIDOR_SHARE * b.m }) {
                b.d.rankClass = Detour.RANK_SAME_CORRIDOR; deferred += b
                continue
            }
            longs += b
            if (cutLongs < 0) longSets += com.carrotsearch.hppc.IntHashSet(b.d.arcs.size * 2).also { set -> for (a in b.d.arcs) set.add(a) }
        }
        if (cutLongs < 0) { cutLongs = longs.size; cutDeferred = deferred.size }
        val lCount = cutLongs + cutDeferred
        // le corte: quante ne servono (i posti tenuti per loro, o quelli lasciati dalle lunghe)
        val reserve = minOf(SHORT_RESERVE, perLeg / 4)
        val needShorts = minOf(perLeg, maxOf(reserve, perLeg - lCount))
        for (ci in longCount until chains.size) {
            val c = chains[ci]
            if (cutShorts < 0 && (shorts.size >= needShorts || builtS >= maxBuilt)) {
                cutShorts = shorts.size
                if (shorts.size < needShorts) stop(ci, "$maxBuilt ricostruite")
            }
            if (cutShorts >= 0 && (diag == null || shorts.size >= diag.maxKept || builtS >= diagBuilt)) break
            looked = maxOf(looked, ci + 1)
            builtS++
            val b = build(ci, c) ?: continue
            b.d.rankClass = Detour.RANK_SHORT
            shorts += b
        }
        if (cutShorts < 0) cutShorts = shorts.size
        // le tenute: le lunghe (poi le rimandate) meno i posti delle corte, poi le corte
        val nS = minOf(cutShorts, maxOf(reserve, perLeg - lCount))
        val nL = minOf(lCount, perLeg - nS)
        val longPart = longs.take(cutLongs) + deferred.take(cutDeferred)
        val kept = longPart.take(nL) + shorts.take(nS)
        val out = ArrayList<Detour>(kept.size).also { o -> for (b in kept) o += b.d }
        if (diag != null && cnt != null && fate != null) {
            val keptSet = java.util.IdentityHashMap<Built, Boolean>().also { m -> for (b in kept) m[b] = true }
            val extraBuilt = (longs + deferred + shorts).filter { it !in keptSet.keys }.let { rest ->
                // l'ordine della traccia: le lunghe oltre il tetto, le rimandate, le corte
                rest.sortedBy { if (it.d.rankClass == Detour.RANK_LONG) 0 else if (it.d.rankClass == Detour.RANK_SAME_CORRIDOR) 1 else 2 }
            }.take(maxOf(0, diag.maxKept - out.size))
            for (b in kept) fate[b.head] = if (b.d.rankClass == Detour.RANK_SAME_CORRIDOR) "tenuta (stesso corridoio, rimandata)" else "tenuta"
            for (b in (longs + deferred + shorts)) if (b !in keptSet.keys)
                fate[b.head] = if (b.d.rankClass == Detour.RANK_SAME_CORRIDOR) "oltre il tetto (stesso corridoio)" else "oltre il tetto"
            val extra = extraBuilt.map { it.d }
            diag.extra += extra
            report(diag, leg, s, e, legW, maxFactor, minPlateauShare, minDetourM, cnt, chains, out, extra, fate, builtL + builtS, stopAt, stopWhy, looked)
        }
        return out
    }

    /** Una deviazione ricostruita, con la testa della sua catena e i suoi metri. */
    private class Built(val d: Detour, val head: Int, val m: Double)

    /** Una catena di plateau (passo 3-4): testa, coda, uscita, rientro, plateau, peso fra uscita e rientro, punteggio. */
    private class Chain(val head: Int, val tail: Int, val p: Int, val q: Int, val plateauW: Double, val detourW: Double, val score: Double,
                        /** La deviazione è lunga almeno quanto la deviazione minima del giudice (passa prima delle corte). */
                        val long: Boolean = true)

    /** Le righe della traccia di un tratto ([detours] con [DetourTrace]): fuori dai cicli caldi, solo con la traccia. */
    private fun report(diag: DetourTrace, leg: Int, s: Int, e: Int, legW: Double, maxFactor: Double, share: Double, minDetourM: Double, cnt: IntArray,
                       chains: List<Chain>, out: List<Detour>, extra: List<Detour>, fate: Map<Int, String>, built: Int, stopAt: Int, stopWhy: String, looked: Int) {
        val log = diag.log
        var inNet = 0; var firstP = -1; var lastP = -1
        for (p in s until e) if (net.netNode(trace.node[p]) >= 0) { inNet++; if (firstP < 0) firstP = p; lastP = p }
        var nf = 0; var ng = 0; var nfg = 0
        for (a in 0 until nA) { val hf = f[a] != INF; val hg = g[a] != INF; if (hf) nf++; if (hg) ng++; if (hf && hg) nfg++ }
        fun km(p: Int) = if (p < 0) "-" else "%.1f".format(trace.preM[p] / 1000)
        log("varianti: traccia M tratto $leg: peso %.0f, tetto ×%.3f, %d posizioni del migliore, %d nella rete (km %s–%s del migliore, posizioni %d–%d), archi con f %d, con g %d, con entrambe %d".format(
            legW, maxFactor, e - s, inNet, km(firstP), km(lastP), if (firstP < 0) -1 else firstP - s, if (lastP < 0) -1 else lastP - s, nf, ng, nfg))
        log("varianti: traccia M tratto $leg: teste scartate: rientro non dopo l'uscita ${cnt[0]}, oltre il tetto ${cnt[1]}, corte (< %.1f km) ${cnt[2]}, Δ ≤ 0 ${cnt[3]}, plateau < %.2f ${cnt[4]}; catene valide ${chains.size}".format(minDetourM / 1000, share))
        val fates = fate.values.groupingBy { it.substringBefore(" (") }.eachCount()
        log("varianti: traccia M tratto $leg: ricostruite $built (guardate $looked di ${chains.size}): tenute ${out.size}, oltre il tetto ${extra.size}; " +
            fates.filterKeys { it != "tenuta" && it != "oltre il tetto" }.entries.joinToString(", ") { "${it.key} ${it.value}" }.ifEmpty { "nessuna scartata" } +
            (if (stopAt >= 0) "; tetto: $stopWhy alla catena $stopAt (punteggio %.3f)".format(chains[stopAt].score) else "; tetto non raggiunto") +
            "; catene non guardate ${chains.size - looked}")
        if (diag.watch.isEmpty()) return
        // le sonde: lungo il percorso forzato, gli archi nella rete e le loro etichette
        val byEdge = com.carrotsearch.hppc.IntIntHashMap(nA * 2)
        val byEdge2 = com.carrotsearch.hppc.IntIntHashMap()
        for (a in 0 until nA) { val ed = net.arcEdge[a]; if (byEdge.containsKey(ed)) byEdge2.put(ed, a) else byEdge.put(ed, a) }
        val rankOf = com.carrotsearch.hppc.IntIntHashMap(chains.size * 2)
        for ((i, c) in chains.withIndex()) rankOf.put(c.head, i)
        for (w in diag.watch) {
            val n = w.edges.size
            val arc = IntArray(n) { i ->
                val ed = w.edges[i]
                val a1 = byEdge.getOrDefault(ed, -1); val a2 = byEdge2.getOrDefault(ed, -1)
                when {
                    a1 >= 0 && net.nodeGraph[net.arcHead[a1]] == w.adjNodes[i] -> a1
                    a2 >= 0 && net.nodeGraph[net.arcHead[a2]] == w.adjNodes[i] -> a2
                    else -> -1
                }
            }
            fun key(i: Int): String {
                val ed = w.edges[i]; val a = arc[i]
                return when {
                    trace.edgeSet.contains(ed) -> "migliore"
                    a < 0 -> "fuori rete"
                    arcOnB[a] -> "esclusa (maschera della rete)"
                    else -> "f:${if (f[a] == INF) -1 else fOrig[a]} g:${if (g[a] == INF) -1 else gDest[a]}"
                }
            }
            var offM = 0.0; var labM = 0.0; var outM = 0.0
            for (i in 0 until n) {
                if (trace.edgeSet.contains(w.edges[i])) continue
                offM += w.meters[i]
                val a = arc[i]
                if (a < 0 || arcOnB[a]) outM += w.meters[i] else if (f[a] != INF && g[a] != INF) labM += w.meters[i]
            }
            log("varianti: traccia sonda ${w.name} (tratto $leg): %.1f km fuori dal migliore, %.1f km fuori dalla rete, %.1f km con le due etichette".format(offM / 1000, outM / 1000, labM / 1000))
            var i = 0; var at = 0.0; var groups = 0
            while (i < n) {
                val k = key(i)
                var j = i; var m = 0.0
                val refs = LinkedHashMap<String, Double>()
                var fFollow = 0; var gFollow = 0; var links = 0
                var minTot = INF
                val heads = LinkedHashSet<Int>()
                while (j < n && key(j) == k) {
                    m += w.meters[j]
                    diag.ref(w.edges[j])?.let { refs.merge(it, w.meters[j], Double::plus) }
                    val a = arc[j]
                    if (a >= 0 && !arcOnB[a] && !trace.edgeSet.contains(w.edges[j])) {
                        // (lo stesso arco due volte di fila: le due metà al punto di passaggio della sonda)
                        if (j > i && arc[j - 1] >= 0 && arc[j - 1] != a) { links++; if (fPar[a] == arc[j - 1]) fFollow++; if (gNext[arc[j - 1]] == a) gFollow++ }
                        if (f[a] != INF && g[a] != INF) {
                            minTot = minOf(minTot, f[a] + g[a] - net.arcW[a])
                            var h = a; var guard = 0
                            while (guard++ < nA) { val x = fPar[h]; if (x < 0 || gNext[x] != h) break; h = x }
                            heads += h
                        }
                    }
                    j++
                }
                val head = "  km %.1f–%.1f (%.1f km, %s)".format(at / 1000, (at + m) / 1000, m / 1000,
                    refs.entries.sortedByDescending { it.value }.take(3).joinToString(" ") { "${it.key} %.0f".format(it.value / 1000) }.ifEmpty { "senza ref" })
                val detail = if (k.startsWith("f:")) {
                    val a = arc[i]
                    val fo = if (f[a] == INF) "f infinita" else "esce dal migliore al km ${km(fOrig[a])} (pos ${fOrig[a] - s})"
                    val gd = if (g[a] == INF) "g infinita" else "rientra al km ${km(gDest[a])} (pos ${gDest[a] - s})"
                    val tot = if (minTot == INF) "" else ", via ×%.3f del tratto (tetto ×%.3f)".format(minTot / legW, maxFactor)
                    val hs = heads.take(3).joinToString("; ") { h ->
                        val r = rankOf.getOrDefault(h, -1)
                        "catena con testa $h: ${why(h, legW, maxFactor * legW, minDetourM, share)}" + (if (r >= 0) ", rango $r di ${chains.size}, ${fate[h] ?: "non guardata"}" else "")
                    }
                    "$fo, $gd$tot; archi legati f $fFollow/$links, g $gFollow/$links" + (if (hs.isNotEmpty()) "; $hs" else "")
                } else k
                if (groups < MAX_WATCH_GROUPS) log("varianti: traccia sonda ${w.name}:$head: $detail")
                groups++
                at += m; i = j
            }
            if (groups > MAX_WATCH_GROUPS) log("varianti: traccia sonda ${w.name}: altri ${groups - MAX_WATCH_GROUPS} gruppi non scritti")
        }
    }

    /** Perché la catena con testa [a] è o non è valida: le stesse condizioni del passo 3-4 (solo la traccia). */
    private fun why(a: Int, legW: Double, limit: Double, minDetourM: Double, share: Double): String {
        if (f[a] == INF || g[a] == INF) return "senza le due etichette"
        if (gDest[a] <= fOrig[a]) return "rientro (pos ${gDest[a]}) non dopo l'uscita (pos ${fOrig[a]})"
        var plateau = net.arcW[a]; var cur = a
        while (true) { val nx = gNext[cur]; if (nx < 0 || fPar[nx] != cur) break; cur = nx; plateau += net.arcW[cur] }
        val tot = f[a] + g[a] - net.arcW[a]
        if (tot > limit) return "oltre il tetto (×%.3f > ×%.3f)".format(tot / legW, limit / legW)
        val m = fM[a] + gM[a] - net.arcM[a]
        if (m < minDetourM - 1.0) return "corta (%.1f km < %.1f)".format(m / 1000, minDetourM / 1000)
        val detourW = tot - legW + (trace.pre[gDest[a]] - trace.pre[fOrig[a]])
        if (detourW <= 0) return "Δ ≤ 0"
        if (plateau < share * detourW) return "plateau %.2f < %.2f (×%.3f)".format(plateau / detourW, share, tot / legW)
        return "valida (plateau %.2f, ×%.3f, %.1f km)".format(plateau / detourW, tot / legW, m / 1000)
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

    /**
     * Le singole da cui si combinano le combinazioni: le prime [COMBO_TOP] nell'ordine dei posti
     * ([Detour.rankClass]: le lunghe di corridoi diversi, le rimandate, le corte), poi per [key],
     * con almeno [COMBO_SHORT_RESERVE] posti alle corte se ce ne sono.
     * Prima erano le prime per solo punteggio: i giri di svincolo minuscoli prendevano tutti gli 8
     * posti e le combinazioni non contenevano mai una deviazione lunga.
     */
    fun comboTop(ds: List<Detour>, key: (Detour) -> Double = { it.score }): List<Detour> {
        val sorted = ds.sortedWith(compareBy<Detour>({ it.rankClass }, key))
        // le corte tengono almeno COMBO_SHORT_RESERVE posti (i cambi in più punti: una lunga e una corta)
        val shorts = sorted.filter { it.rankClass == Detour.RANK_SHORT }
        val nS = minOf(shorts.size, maxOf(COMBO_SHORT_RESERVE, COMBO_TOP - (sorted.size - shorts.size)))
        return sorted.filter { it.rankClass != Detour.RANK_SHORT }.take(COMBO_TOP - nS) + shorts.take(nS)
    }

    private fun commonMeters(a: IntArray, b: IntArray): Double {
        val set = com.carrotsearch.hppc.IntHashSet(a.size * 2)
        for (x in a) set.add(x)
        return commonMeters(set, b)
    }

    private fun commonMeters(set: com.carrotsearch.hppc.IntHashSet, b: IntArray): Double {
        var m = 0.0
        for (x in b) if (set.contains(x)) m += net.arcM[x]
        return m
    }

    /**
     * Le combinazioni di 2 (e, con [maxParts] 3, di 3) deviazioni compatibili ([Detour.disjoint], e
     * nello stesso tratto senza nodi in comune: con le strade d'accesso due deviazioni disgiunte
     * sul migliore possono passare per lo stesso incrocio, e il tratto cucito ripeterebbe un nodo)
     * fra le prime 8 di [ds] per punteggio ([key]: [Detour.score], sul tratto, o [tripScore], sul
     * viaggio), dalla più leggera, al più [limit]. Il Δ di una
     * combinazione è la somma: fra le deviazioni resta il migliore, e le svolte di raccordo sono già controllate.
     */
    fun combos(ds: List<Detour>, maxParts: Int = 3, limit: Int = 40, key: (Detour) -> Double = { it.score }): List<List<Detour>> {
        val top = comboTop(ds, key)
        // i nodi della rete di ogni deviazione (solo RAM, al più 8 insiemi)
        val nodes = top.map { d -> com.carrotsearch.hppc.IntHashSet(d.arcs.size * 2 + 2).also { s -> for (a in d.arcs) { s.add(net.arcTail[a]); s.add(net.arcHead[a]) } } }
        fun ok(i: Int, j: Int): Boolean {
            if (!top[i].disjoint(top[j])) return false
            if (top[i].leg != top[j].leg) return true
            val (small, big) = if (nodes[i].size() <= nodes[j].size()) nodes[i] to nodes[j] else nodes[j] to nodes[i]
            for (c in small) if (big.contains(c.value)) return false
            return true
        }
        val out = ArrayList<List<Detour>>()
        for (i in top.indices) for (j in i + 1 until top.size) {
            if (!ok(i, j)) continue
            out += sortedParts(top[i], top[j])
            if (maxParts >= 3) for (k in j + 1 until top.size) if (ok(i, k) && ok(j, k)) out += sortedParts(top[i], top[j], top[k])
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
        /**
         * Le deviazioni tenute al più per tratto. Era 12: con le lunghe prima delle corte e la
         * varietà di corridoio 16 lascia posto anche ai giri di svincolo per le combinazioni; la
         * memoria resta sotto il tetto degli insiemi offerti per richiesta (`MAX_M_SETS` = 64).
         */
        const val PER_LEG = 16
        /**
         * Una deviazione lunga con almeno questa quota dei propri km in comune con una lunga già
         * presa è dello stesso corridoio: non occupa un posto finché ce ne sono di corridoi diversi.
         */
        const val CORRIDOR_SHARE = 0.6
        /** Una catena è lunga se anche il pezzo del migliore che sostituisce è almeno questa quota della soglia. */
        const val LONG_BEST_SHARE = 0.5
        /** I posti per tratto tenuti alle corte se ce ne sono (al più un quarto di perLeg). */
        const val SHORT_RESERVE = 4
        /** I posti delle prime per le combinazioni tenuti alle corte se ce ne sono. */
        const val COMBO_SHORT_RESERVE = 2
        /** Quante catene si ricostruiscono al più per deviazione tenuta, per giro (le altre cadono per nodi ripetuti, doppioni, giri urbani). */
        private const val MAX_BUILT_PER_KEPT = 20
        /** I gruppi di archi scritti al più per sonda nella traccia. */
        private const val MAX_WATCH_GROUPS = 60

        /**
         * Il punteggio di una deviazione (più basso = meglio): il Δ di peso [dW] riferito a [refW]
         * (il tratto in [detours], il viaggio in [tripScore]) meno metà della quota di plateau
         * ([plateauW] sul peso [detourW] della deviazione fra uscita e rientro).
         */
        internal fun score(dW: Double, plateauW: Double, detourW: Double, refW: Double): Double = dW / refW - 0.5 * plateauW / detourW
    }
}
