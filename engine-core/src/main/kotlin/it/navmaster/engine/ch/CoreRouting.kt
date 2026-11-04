package it.navmaster.engine.ch

import com.carrotsearch.hppc.IntDoubleHashMap
import com.carrotsearch.hppc.IntHashSet
import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.WayType
import it.navmaster.engine.routing.NodeHeap
import it.navmaster.engine.routing.SearchPath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting

/**
 * Decide quali archi finiscono nel "core": un arco è nel core se potrà essere escluso da una
 * richiesta con le misure del mezzo (limiti, merci pericolose, tipi di mezzo pesante) o se ha un
 * divieto di svolta a un estremo. Entrambi i suoi nodi non vengono contratti.
 *
 * A differenza del core di ORS, autostrade, pedaggi, traghetti, guadi e le strade oltre gli 80 km/h
 * restano fuori: metterli nel core (che nella preparazione non può usarli come testimoni) fa
 * esplodere il numero di shortcut e i tempi di costruzione sui Paesi grandi. Le richieste con
 * `avoid_features` o `maximum_speed` usano la ricerca piena (stessi risultati, più lenta).
 */
class CoreEdgeSelector(
    private val encoding: GraphEncoding,
    private val profileName: String,
    private val turnCosts: TurnCostStorage?,
    /** ORS `maximum_speed_lower_bound`: archi più veloci di così possono ricevere un `maximum_speed`. */
    private val maximumSpeedLowerBound: Double = 80.0,
) {
    private val sh = encoding.shared
    private val pe = encoding.profile(profileName)
    private val isHgv = profileName == "heavyvehicle"

    fun isCoreEdge(it: BaseGraph.EdgeIterator): Boolean {
        // restrizioni per mezzi pesanti
        if (isHgv) {
            if (it.get(sh.hgvBlocked) != 0 || it.get(sh.hazmatForbidden)) return true
            if (it.get(sh.maxHeight).isFinite() || it.get(sh.maxWidth).isFinite() || it.get(sh.maxLength).isFinite() ||
                it.get(sh.maxWeight).isFinite() || it.get(sh.maxAxleLoad).isFinite()) return true
        }
        // divieti di svolta a uno dei due estremi
        if (turnCosts != null && (turnCosts.hasEntries(it.baseNode) || turnCosts.hasEntries(it.adjNode))) return true
        return false
    }

    /** Nodi da non contrarre: estremi di ogni arco del core. */
    fun coreNodes(graph: BaseGraph): BooleanArray {
        val core = BooleanArray(graph.nodeCount)
        val it = graph.createEdgeIterator()
        for (e in 0 until graph.edgeCount) {
            it.setEdge(e, graph.nodeA(e))
            if (isCoreEdge(it)) { core[graph.nodeA(e)] = true; core[graph.nodeB(e)] = true }
        }
        return core
    }
}

/**
 * Ricerca "core-ALT" semplificata (ORS `CoreDijkstra`): fase 1, ricerca CH verso l'alto da entrambi
 * i lati senza espandere i nodi del core (che diventano punti d'ingresso); fase 2, Dijkstra
 * bidirezionale sui soli archi del core con il filtro della richiesta, node-based o edge-based.
 * Il risultato è il migliore fra un incontro in fase 1 e uno attraverso il core.
 */
class CoreDijkstra(
    private val ch: CHGraph,
    private val weighting: Weighting,
    private val filter: EdgeFilter,
    private val turnCosts: TurnCostProvider?,
    /** Se presenti, la fase 2 è un A* bidirezionale con potenziali ALT ("core-ALT"). */
    landmarks: CoreLandmarks? = null,
) {
    private val base = ch.base
    private val storage = ch.storage
    private val coreLevel = base.nodeCount
    private fun inCore(node: Int) = storage.level(node) >= coreLevel
    private val potentials: CorePotentials? = landmarks?.takeIf { it.landmarkCount > 0 }?.let { CorePotentials(it) }

    private class State {
        val weight = IntDoubleHashMap(1024)
        val parentNode = IntIntHashMap(1024)
        val parentEdge = IntIntHashMap(1024)
        val settled = IntHashSet(1024)
        val heap = NodeHeap(256)
        fun weightOf(n: Int) = weight.getOrDefault(n, Double.POSITIVE_INFINITY)
        /** [key] è la chiave dell'heap (peso più potenziale); [w] il peso reale. */
        fun relax(n: Int, w: Double, from: Int, edge: Int, key: Double = w): Boolean {
            if (w >= weightOf(n)) return false
            weight.put(n, w); parentNode.put(n, from); parentEdge.put(n, edge); heap.push(n, key); return true
        }
    }

    private val fwd = State(); private val bwd = State()
    /** Fase 2: chiavi = nodi (node-based) oppure chiavi d'arco (edge-based). */
    private val cFwd = State(); private val cBwd = State()
    /** Edge-based: chiavi di fase 2 presenti in ogni nodo (in avanti: arrivate lì; all'indietro: in partenza da lì). */
    private val cFwdAtNode = com.carrotsearch.hppc.IntObjectHashMap<com.carrotsearch.hppc.IntArrayList>()
    private val cBwdAtNode = com.carrotsearch.hppc.IntObjectHashMap<com.carrotsearch.hppc.IntArrayList>()
    private val fwdIt = ch.iterator(); private val bwdIt = ch.iterator()
    private val baseIt = base.createEdgeIterator()
    var bestWeight = Double.POSITIVE_INFINITY; internal set
    private var meetingNode = -1          // incontro in fase 1
    private var coreMeetFwd = Int.MIN_VALUE   // chiave in avanti dell'incontro in fase 2
    private var coreMeetBwd = Int.MIN_VALUE   // chiave all'indietro dell'incontro in fase 2 (uguale a coreMeetFwd se sullo stesso arco)
    private val coreMet get() = coreMeetFwd != Int.MIN_VALUE
    var visitedNodes = 0; private set
    private val edgeBased get() = turnCosts != null

    fun addSource(node: Int, w: Double) { if (fwd.relax(node, w, -1, -1)) checkMeeting(node) }
    fun addTarget(node: Int, w: Double) { if (bwd.relax(node, w, -1, -1)) checkMeeting(node) }

    /** Tempi e nodi delle due fasi dell'ultima ricerca (diagnostica). */
    var stats = ""; private set

    fun run(): Boolean {
        val t0 = System.nanoTime()
        // fase 1: entrambe le direzioni fino a esaurimento (le ricerche CH verso l'alto sono piccole)
        while (!fwd.heap.isEmpty || !bwd.heap.isEmpty) {
            if (!fwd.heap.isEmpty) settlePhase1(fwd, fwdIt, false)
            if (!bwd.heap.isEmpty) settlePhase1(bwd, bwdIt, true)
        }
        val v1 = visitedNodes; val t1 = System.nanoTime()
        // fase 2: semina gli ingressi al core e cerca dentro con i filtri (A* bilanciato se ci sono i landmark)
        potentials?.let { p ->
            for (c in fwd.weight) if (inCore(c.key) && fwd.settled.contains(c.key)) p.addSource(c.key, c.value)
            for (c in bwd.weight) if (inCore(c.key) && bwd.settled.contains(c.key)) p.addTarget(c.key, c.value)
        }
        seedCore(fwd, cFwd, reverse = false)
        seedCore(bwd, cBwd, reverse = true)
        while (!cFwd.heap.isEmpty || !cBwd.heap.isEmpty) {
            val fk = if (cFwd.heap.isEmpty) Double.POSITIVE_INFINITY else cFwd.heap.peekKey()
            val bk = if (cBwd.heap.isEmpty) Double.POSITIVE_INFINITY else cBwd.heap.peekKey()
            // con p_b = −p_f le chiavi di un incontro sommano al peso reale: la condizione resta esatta
            if (fk + bk >= bestWeight) break
            if (fk <= bk) settleCore(cFwd, false) else settleCore(cBwd, true)
        }
        stats = "fase1 $v1 nodi ${(t1 - t0) / 1_000_000} ms, core ${visitedNodes - v1} nodi ${(System.nanoTime() - t1) / 1_000_000} ms" + (if (potentials != null) " ALT" else "")
        return meetingNode >= 0 || coreMet
    }

    /** Chiave dell'heap per un nodo: peso più potenziale (segno opposto per la ricerca all'indietro). */
    private fun heapKey(w: Double, node: Int, reverse: Boolean): Double {
        val p = potentials ?: return w
        val pf = p.forward(node)
        return if (reverse) w - pf else w + pf
    }

    /** Registra una chiave di fase 2 nel suo nodo (una sola volta) e verifica gli incontri. */
    private fun relaxCore(s: State, key: Int, w: Double, from: Int, edge: Int, reverse: Boolean) {
        val isNew = !s.weight.containsKey(key)
        val node = if (edgeBased) keyNode(key, reverse) else key
        if (!s.relax(key, w, from, edge, heapKey(w, node, reverse))) return
        if (edgeBased && isNew) {
            val map = if (reverse) cBwdAtNode else cFwdAtNode
            (map.get(node) ?: com.carrotsearch.hppc.IntArrayList().also { map.put(node, it) }).add(key)
        }
        checkCoreMeeting(key, reverse)
    }

    private fun settlePhase1(s: State, it: CHGraph.Iterator, reverse: Boolean) {
        val node = s.heap.pop()
        if (!s.settled.add(node)) return
        visitedNodes++
        if (inCore(node)) return // punto d'ingresso: non si espande in fase 1
        val w = s.weightOf(node)
        it.setBaseNode(node, reverse)
        while (it.next()) {
            val adj = it.adjNode
            if (s.settled.contains(adj)) continue
            if (s.relax(adj, w + it.weight, node, it.chEdge)) checkMeeting(adj)
        }
    }

    private fun checkMeeting(node: Int) {
        val f = fwd.weightOf(node); val b = bwd.weightOf(node)
        if (f.isInfinite() || b.isInfinite()) return
        if (f + b < bestWeight) { bestWeight = f + b; meetingNode = node; coreMeetFwd = Int.MIN_VALUE; coreMeetBwd = Int.MIN_VALUE }
    }

    /** Ogni nodo del core raggiunto in fase 1 diventa una sorgente della fase 2. */
    private fun seedCore(from: State, to: State, reverse: Boolean) {
        for (c in from.weight) {
            val node = c.key
            if (!inCore(node) || !from.settled.contains(node)) continue
            if (!edgeBased) { relaxCore(to, node, c.value, -1, -1, reverse); continue }
            // edge-based: la chiave è l'ultimo arco base con cui si arriva al nodo (per il costo di svolta successivo)
            val lastBase = lastBaseEdge(from, node)
            val key = if (lastBase < 0) virtualKey(node) else edgeKeyArriving(lastBase, node, reverse)
            relaxCore(to, key, c.value, -1, -1, reverse)
        }
    }

    /** Chiave fittizia per un nodo del core che è sorgente diretta (nessun arco di arrivo): negativa, mai confusa con una chiave d'arco. */
    private fun virtualKey(node: Int) = -(node + 2)

    /** Arco base con cui la ricerca di fase 1 è arrivata a [node], o -1. */
    private fun lastBaseEdge(s: State, node: Int): Int {
        val chEdge = s.parentEdge.getOrDefault(node, -1)
        if (chEdge < 0) return -1
        if (!storage.isShortcut(chEdge)) return chEdge
        val sc = storage.shortcutIndex(chEdge)
        val parent = s.parentNode.get(node)
        return if (storage.nodeA(sc) == parent) storage.origLast(sc) else storage.origFirst(sc)
    }

    /** Chiave d'arco di [edge] nel verso di percorrenza del viaggio che arriva a (fwd) o parte da (bwd) [node]. */
    private fun edgeKeyArriving(edge: Int, node: Int, reverse: Boolean): Int {
        // fwd: il viaggio percorre edge e arriva in node → verso A→B se node == B
        // bwd: il viaggio parte da node lungo edge → verso A→B se node == A
        val forwardAB = if (!reverse) base.nodeB(edge) == node else base.nodeA(edge) == node
        return (edge shl 1) or (if (forwardAB) 0 else 1)
    }

    private fun keyNode(key: Int, reverse: Boolean): Int {
        if (key < 0) return -key - 2
        val e = key ushr 1
        val ab = key and 1 == 0
        // fwd: nodo raggiunto = estremo finale del verso; bwd: nodo da cui si parte = estremo iniziale del verso
        return if (!reverse) (if (ab) base.nodeB(e) else base.nodeA(e)) else (if (ab) base.nodeA(e) else base.nodeB(e))
    }

    private val coreIt = ch.iterator()

    /**
     * Fase 2: dal nodo del core espande gli archi base verso altri nodi del core (con il filtro della
     * richiesta) e gli shortcut fra nodi del core (percorsi attraverso nodi contratti, mai ristretti per costruzione).
     */
    private fun settleCore(s: State, reverse: Boolean) {
        val key = s.heap.pop()
        if (!s.settled.add(key)) return
        visitedNodes++
        val w = s.weightOf(key)
        val node = if (edgeBased) keyNode(key, reverse) else key
        val inEdge = if (edgeBased && key >= 0) key ushr 1 else -1
        coreIt.setBaseNode(node, reverse)
        while (coreIt.next()) {
            val adj = coreIt.adjNode
            if (!inCore(adj)) continue
            val chEdge = coreIt.chEdge
            var ew = coreIt.weight
            var firstBase = chEdge; var lastBase = chEdge
            if (coreIt.isShortcut) {
                if (edgeBased) {
                    // primo/ultimo arco base memorizzati nell'orientamento nodeA→nodeB del record
                    val sc = storage.shortcutIndex(chEdge)
                    if (storage.nodeA(sc) == node) { firstBase = storage.origFirst(sc); lastBase = storage.origLast(sc) }
                    else { firstBase = storage.origLast(sc); lastBase = storage.origFirst(sc) }
                }
            } else {
                baseIt.setEdge(chEdge, node)
                if (!filter.accept(baseIt)) continue
                ew = weighting.edgeWeight(baseIt, reverse)
                if (ew.isInfinite()) continue
            }
            var turn = 0.0
            if (edgeBased && inEdge >= 0) {
                turn = if (!reverse) turnCosts!!.turnWeight(inEdge, node, firstBase) else turnCosts!!.turnWeight(firstBase, node, inEdge)
                if (turn.isInfinite()) continue
            }
            val nk = if (!edgeBased) adj else edgeKeyArriving(lastBase, adj, reverse)
            if (s.settled.contains(nk)) continue
            relaxCore(s, nk, w + ew + turn, key, chEdge, reverse)
        }
    }


    /** Chiave d'arco nel verso del viaggio: in avanti coincide con l'iteratore, all'indietro è l'opposto. */
    private fun travelKey(it: BaseGraph.EdgeIterator, reverse: Boolean): Int = (it.edge shl 1) or (if (it.reverse != reverse) 1 else 0)

    /**
     * Incontro in fase 2. Node-based: stesso nodo. Edge-based: stessa chiave d'arco (l'arco è contato
     * due volte e va sottratto) oppure stesso nodo con chiavi diverse, aggiungendo il costo della svolta
     * fra l'arco con cui si arriva e quello con cui si riparte (inversione = +∞).
     */
    private fun checkCoreMeeting(key: Int, reverse: Boolean) {
        if (!edgeBased) {
            val f = cFwd.weightOf(key); val b = cBwd.weightOf(key)
            if (f.isInfinite() || b.isInfinite()) return
            if (f + b < bestWeight) { bestWeight = f + b; coreMeetFwd = key; coreMeetBwd = key; meetingNode = -1 }
            return
        }
        // stessa chiave d'arco
        if (key >= 0) {
            val f = cFwd.weightOf(key); val b = cBwd.weightOf(key)
            if (!f.isInfinite() && !b.isInfinite()) {
                baseIt.setEdge(key ushr 1, base.nodeA(key ushr 1))
                val total = f + b - weighting.edgeWeight(baseIt, key and 1 == 1)
                if (total < bestWeight) { bestWeight = total; coreMeetFwd = key; coreMeetBwd = key; meetingNode = -1 }
            }
        }
        // stesso nodo, chiavi diverse
        val node = keyNode(key, reverse)
        val others = (if (reverse) cFwdAtNode else cBwdAtNode).get(node) ?: return
        val myW = (if (reverse) cBwd else cFwd).weightOf(key)
        for (i in 0 until others.size()) {
            val other = others.get(i)
            val otherW = (if (reverse) cFwd else cBwd).weightOf(other)
            val fwdKey = if (reverse) other else key
            val bwdKey = if (reverse) key else other
            var total = myW + otherW
            if (fwdKey >= 0 && bwdKey >= 0) {
                val turn = turnCosts!!.turnWeight(fwdKey ushr 1, node, bwdKey ushr 1)
                if (turn.isInfinite()) continue
                total += turn
            }
            if (total < bestWeight) { bestWeight = total; coreMeetFwd = fwdKey; coreMeetBwd = bwdKey; meetingNode = -1 }
        }
    }

    fun extractPath(): SearchPath? {
        if (meetingNode < 0 && !coreMet) return null
        val edges = ArrayList<Int>()
        val startNode: Int
        if (meetingNode >= 0) {
            startNode = unpackPhase1(fwd, meetingNode, edges, forward = true)
            unpackPhase1(bwd, meetingNode, edges, forward = false)
        } else {
            // catena di fase 2 in avanti: (arco CH, nodo di partenza) fino al seme, poi fase 1 dal nodo d'ingresso
            val chain = ArrayList<Long>(); var k = coreMeetFwd
            var entryFwd: Int
            while (true) {
                val pe = cFwd.parentEdge.getOrDefault(k, -1)
                if (pe < 0) { entryFwd = if (edgeBased) keyNode(k, false) else k; break }
                val pk = cFwd.parentNode.get(k)
                val fromNode = if (edgeBased) keyNode(pk, false) else pk
                chain.add((pe.toLong() shl 32) or (fromNode.toLong() and 0xFFFFFFFFL)); k = pk
            }
            chain.reverse()
            startNode = unpackPhase1(fwd, entryFwd, edges, forward = true)
            for (c in chain) ch.unpack((c ushr 32).toInt(), c.toInt(), edges)
            // catena di fase 2 all'indietro: dal punto d'incontro verso la destinazione
            val bwdEdges = ArrayList<Int>()
            k = coreMeetBwd
            var entryBwd: Int
            while (true) {
                val pe = cBwd.parentEdge.getOrDefault(k, -1)
                if (pe < 0) { entryBwd = if (edgeBased) keyNode(k, true) else k; break }
                val fromNode = if (edgeBased) keyNode(k, true) else k
                ch.unpack(pe, fromNode, bwdEdges); k = cBwd.parentNode.get(k)
            }
            unpackPhase1(bwd, entryBwd, bwdEdges, forward = false)
            // edge-based con incontro sullo stesso arco: l'arco è incluso in entrambe le catene, la parte all'indietro inizia con esso
            if (edgeBased && coreMeetFwd == coreMeetBwd && coreMeetFwd >= 0 && bwdEdges.isNotEmpty()) {
                check(bwdEdges.first() == coreMeetFwd ushr 1) { "la catena all'indietro non inizia con l'arco d'incontro" }
                bwdEdges.removeAt(0)
            }
            edges.addAll(bwdEdges)
        }
        val nodes = IntArray(edges.size + 1)
        nodes[0] = startNode
        for (i in edges.indices) { val e = edges[i]; nodes[i + 1] = if (base.nodeA(e) == nodes[i]) base.nodeB(e) else base.nodeA(e) }
        return SearchPath(nodes, edges.toIntArray(), bestWeight)
    }

    /** Scompatta la catena di fase 1 di [s] da [node] fino alla sorgente; ritorna il nodo sorgente/destinazione. */
    private fun unpackPhase1(s: State, node: Int, out: MutableList<Int>, forward: Boolean): Int {
        val chEdges = ArrayList<Int>(); val nodes = ArrayList<Int>()
        var n = node
        while (true) {
            nodes.add(n)
            val e = s.parentEdge.getOrDefault(n, -1)
            if (e < 0) break
            chEdges.add(e); n = s.parentNode.get(n)
        }
        if (forward) {
            chEdges.reverse(); nodes.reverse()
            for (i in chEdges.indices) ch.unpack(chEdges[i], nodes[i], out)
            return nodes.first()
        } else {
            for (i in chEdges.indices) ch.unpack(chEdges[i], nodes[i], out)
            return nodes.last()
        }
    }
}
