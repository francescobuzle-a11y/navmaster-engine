package it.navmaster.engine.routing

import com.carrotsearch.hppc.DoubleArrayList
import com.carrotsearch.hppc.FloatArrayList
import com.carrotsearch.hppc.IntArrayList
import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.RouteOptions
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.SharedEncoding
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/** Le regole di un router per la rete autostradale delle varianti ([Router.access]): peso, tempi, filtro, svolte. */
class DetourAccess internal constructor(
    val graph: BaseGraph, val weighting: Weighting, val timeWeighting: Weighting, val filter: EdgeFilter,
    val turnCosts: TurnCostProvider?, val shared: SharedEncoding,
)

/** I punti la cui somma delle distanze dai fuochi A e B non supera [sumM] metri. */
class Ellipse(val aLat: Double, val aLon: Double, val bLat: Double, val bLon: Double, sumM: Double) {
    /** Mai meno della distanza fra i fuochi (l'ellisse vuota non esiste). */
    val sumM: Double = maxOf(sumM, DistanceCalc.haversine(aLat, aLon, bLat, bLon))

    fun contains(lat: Double, lon: Double, slack: Double = 0.0): Boolean =
        DistanceCalc.haversine(lat, lon, aLat, aLon) + DistanceCalc.haversine(lat, lon, bLat, bLon) <= sumM + slack

    /**
     * [n] punti del bordo, nelle direzioni a passo costante dal centro: ognuno trovato per
     * bisezione con la stessa distanza di [contains] (nessuna proiezione piana da cui scostarsi).
     */
    fun boundary(n: Int = 16): List<DoubleArray> {
        val cLat = (aLat + bLat) / 2; val cLon = (aLon + bLon) / 2
        val k = Math.cos(Math.toRadians(cLat)).coerceAtLeast(0.01)
        return List(n) { i ->
            val th = 2 * Math.PI * i / n
            val dLat = Math.cos(th) / DistanceCalc.METERS_PER_DEGREE; val dLon = Math.sin(th) / (DistanceCalc.METERS_PER_DEGREE * k)
            var lo = 0.0; var hi = sumM
            repeat(40) { val mid = (lo + hi) / 2; if (contains(cLat + dLat * mid, cLon + dLon * mid)) lo = mid else hi = mid }
            doubleArrayOf(cLat + dLat * lo, cLon + dLon * lo)
        }
    }

    internal fun sameFoci(o: Ellipse): Boolean = Math.abs(aLat - o.aLat) < 1e-7 && Math.abs(aLon - o.aLon) < 1e-7 && Math.abs(bLat - o.bLat) < 1e-7 && Math.abs(bLon - o.bLon) < 1e-7

    internal fun same(o: Ellipse): Boolean = sameFoci(o) && Math.abs(sumM - o.sumM) < 1e-3
}

/**
 * Un cerchio d'accesso attorno a un punto del viaggio (partenza o arrivo di un tratto): lì la rete
 * delle varianti ([MotorwayNet.build]) prende anche le strade principali, primarie e secondarie
 * entro [radiusM], terziarie entro [TERTIARY_SHARE] del raggio, così una deviazione può lasciare il
 * migliore nella parte urbana e raggiungere un'altra autostrada.
 */
class Circle(val lat: Double, val lon: Double, val radiusM: Double) {
    /** Il riquadro del cerchio in gradi (1 % in più): fuori, nessuna haversine. */
    private val dLat = 1.01 * radiusM / DistanceCalc.METERS_PER_DEGREE
    private val dLon = 1.01 * radiusM / (DistanceCalc.METERS_PER_DEGREE * Math.cos(Math.toRadians(lat)).coerceAtLeast(0.01))

    /** Il punto (lat, lon) sta nel cerchio per una strada di classe [kind] ([MotorwayNet.KIND_MAIN] o [MotorwayNet.KIND_TERTIARY]). */
    fun contains(lat: Double, lon: Double, kind: Byte): Boolean {
        if (Math.abs(lat - this.lat) > dLat || Math.abs(lon - this.lon) > dLon) return false
        return DistanceCalc.haversine(lat, lon, this.lat, this.lon) <= (if (kind == MotorwayNet.KIND_TERTIARY) TERTIARY_SHARE * radiusM else radiusM)
    }

    /**
     * Vero se [o] sta dentro questo cerchio per ogni classe (1 m di tolleranza): il raggio pieno
     * (primarie, secondarie) e quello delle terziarie ([TERTIARY_SHARE]); ogni sua strada d'accesso
     * è anche di questo.
     */
    fun covers(o: Circle): Boolean {
        val d = DistanceCalc.haversine(lat, lon, o.lat, o.lon)
        return d + o.radiusM <= radiusM + 1.0 && d + TERTIARY_SHARE * o.radiusM <= TERTIARY_SHARE * radiusM + 1.0
    }

    internal fun same(o: Circle): Boolean = Math.abs(lat - o.lat) < 1e-7 && Math.abs(lon - o.lon) < 1e-7 && Math.abs(radiusM - o.radiusM) < 1e-3

    companion object {
        /** Le terziarie solo vicino al punto (le prime vie della partenza): 0,4 del raggio. */
        const val TERTIARY_SHARE = 0.4

        /** Vero se [a] e [b] sono gli stessi cerchi, nello stesso ordine. */
        fun sameAll(a: List<Circle>, b: List<Circle>): Boolean = a.size == b.size && a.indices.all { a[it].same(b[it]) }

        /** Il punto (lat, lon) sta in uno dei cerchi per la classe [kind]. */
        fun inAny(cs: List<Circle>, lat: Double, lon: Double, kind: Byte): Boolean = cs.any { it.contains(lat, lon, kind) }
    }
}

/** L'unione di più ellissi (una per tratto del viaggio): dove si cerca la rete autostradale. */
class Region(val ellipses: List<Ellipse>) {
    fun contains(lat: Double, lon: Double): Boolean = ellipses.any { it.contains(lat, lon) }

    /** Vero se [o] sta dentro questa regione: per ogni sua ellisse una uguale ma non più piccola, o i 16 punti del bordo dentro (1 m di tolleranza). */
    fun covers(o: Region): Boolean = o.ellipses.all { e ->
        ellipses.any { it.sameFoci(e) && it.sumM >= e.sumM } || e.boundary(16).all { p -> ellipses.any { it.contains(p[0], p[1], 1.0) } }
    }

    /** Vero se [o] è questa stessa regione (le stesse ellissi, nello stesso ordine). */
    fun sameAs(o: Region): Boolean = ellipses.size == o.ellipses.size && ellipses.indices.all { ellipses[it].same(o.ellipses[it]) }
}

/**
 * La rete autostradale (autostrade e superstrade, rampe comprese) attorno a un viaggio, in RAM:
 * archi diretti (un verso percorribile = un arco) con peso [arcW], tempo [arcMs] e metri [arcM]
 * della richiesta, coda e testa (nodi della rete), arco del grafo [arcEdge] e verso [arcRev];
 * adiacenze uscenti ed entranti compatte. [restricted]: il nodo ha divieti di svolta (lì soltanto
 * le deviazioni chiedono la svolta al grafo). [truncated]: oltre il tetto di archi, incompleta.
 * [nodeLat]/[nodeLon]: le coordinate dei nodi (vuote in una rete giocattolo), per [mask].
 * [access]: i cerchi d'accesso ([Circle]) con cui è costruita; [arcKind]: per arco [KIND_MAJOR]
 * (autostrada, superstrada), [KIND_MAIN] (primaria, secondaria) o [KIND_TERTIARY] (vuoto: tutti
 * [KIND_MAJOR]); [accessCapped]: le strade d'accesso si sono fermate al tetto ([MAX_ACCESS_ARCS]).
 * [turnBans]: le svolte vietate fra due archi della rete nei nodi con divieti ([pairKey]), lette una
 * volta alla costruzione (null: non lette, una rete giocattolo). Le ricerche delle deviazioni
 * ([DetourSearch]) leggono il grafo solo per le svolte all'uscita dal migliore e al rientro.
 */
class MotorwayNet internal constructor(
    val nodeGraph: IntArray, val restricted: BooleanArray,
    val arcTail: IntArray, val arcHead: IntArray, val arcEdge: IntArray, val arcRev: BooleanArray,
    val arcW: DoubleArray, val arcMs: IntArray, val arcM: FloatArray,
    val outStart: IntArray, val outArcs: IntArray, val inStart: IntArray, val inArcs: IntArray,
    val region: Region, val truncated: Boolean,
    val nodeLat: DoubleArray = DoubleArray(0), val nodeLon: DoubleArray = DoubleArray(0),
    val access: List<Circle> = emptyList(), val arcKind: ByteArray = ByteArray(0), val accessCapped: Boolean = false,
) {
    val nodeCount: Int get() = nodeGraph.size
    val arcCount: Int get() = arcTail.size
    /** La classe dell'arco [a] per le strade d'accesso ([KIND_MAJOR] se la rete non ne ha). */
    fun kind(a: Int): Byte = if (arcKind.size == arcTail.size) arcKind[a] else KIND_MAJOR
    /** Gli archi delle strade d'accesso (primarie, secondarie, terziarie). */
    val accessArcs: Int by lazy { arcKind.count { it != KIND_MAJOR } }
    /**
     * I nodi toccati solo da strade d'accesso: le letture in più della costruzione (per nodo la
     * lista delle adiacenze, le coordinate dei vicini, i divieti di svolta), per il registro.
     */
    val accessNodes: Int by lazy {
        if (accessArcs == 0) 0 else {
            val major = BooleanArray(nodeCount)
            for (a in 0 until arcCount) if (kind(a) == KIND_MAJOR) { major[arcTail[a]] = true; major[arcHead[a]] = true }
            major.count { !it }
        }
    }

    /** Le svolte vietate fra archi della rete ([pairKey]); null: non lette (le chiede [DetourSearch]). */
    var turnBans: com.carrotsearch.hppc.LongHashSet? = null
        internal set

    /**
     * Vero se questa rete autostradale (senza strade d'accesso) vale per una richiesta con la
     * regione [need]: la regione la copre; una rete troncata solo per la sua stessa regione. Le
     * strade d'accesso si aggiungono per richiesta ([withAccess], [MotorwayNetCache.overlay]).
     */
    fun serves(need: Region): Boolean = access.isEmpty() && region.covers(need) && (!truncated || region.sameAs(need))
    private val index = IntIntHashMap(nodeGraph.size * 2).also { m -> for (i in nodeGraph.indices) m.put(nodeGraph[i], i) }

    /** Il nodo della rete del nodo [graphNode] del grafo, -1 se assente. */
    fun netNode(graphNode: Int): Int = index.getOrDefault(graphNode, -1)

    /**
     * I nodi che una rete costruita per [need] dai semi [seeds] avrebbe ([build]: i semi e i nodi
     * dentro [need] raggiunti da loro per archi in un verso o nell'altro): una rete della cache
     * costruita per una regione più grande dà così le stesse deviazioni di una nuova. Null se tutti
     * (o rete senza coordinate). Solo RAM.
     */
    fun mask(need: Region, seeds: IntArray): BooleanArray? {
        if (nodeLat.size != nodeCount) return null
        val inNeed = BooleanArray(nodeCount) { need.contains(nodeLat[it], nodeLon[it]) }
        val m = BooleanArray(nodeCount)
        val queue = IntArray(nodeCount); var qh = 0; var qt = 0
        for (s in seeds) { val k = netNode(s); if (k >= 0 && !m[k]) { m[k] = true; queue[qt++] = k } }
        while (qh < qt) {
            val n = queue[qh++]
            for (k in outStart[n] until outStart[n + 1]) { val h = arcHead[outArcs[k]]; if (!m[h] && inNeed[h]) { m[h] = true; queue[qt++] = h } }
            for (k in inStart[n] until inStart[n + 1]) { val t = arcTail[inArcs[k]]; if (!m[t] && inNeed[t]) { m[t] = true; queue[qt++] = t } }
        }
        return if (qt == nodeCount) null else m
    }

    /** Il pezzo da cucire nel tratto ([Router.splice]) per la deviazione [d]. */
    fun cut(d: Detour): Router.Cut = Router.Cut(d.fromPos, d.toPos, IntArray(d.arcs.size) { arcEdge[d.arcs[it]] }, IntArray(d.arcs.size) { nodeGraph[arcTail[d.arcs[it]]] })

    companion object {
        /**
         * La rete raggiunta dai nodi [seeds] (i nodi del migliore su un'autostrada) dentro [region]:
         * visita in ampiezza sugli archi `motorway`/`trunk` (rampe comprese) che il filtro accetta;
         * un verso con peso finito diventa un arco diretto, registrato solo dalla sua coda; si visita
         * anche il nodo raggiunto solo all'indietro (l'inizio di una rampa d'entrata). Per nodo una
         * lettura dei divieti di svolta. Oltre [maxArcs] la visita si ferma ([truncated]). [counts]
         * (solo la traccia delle varianti): i versi lasciati fuori, per motivo: [0] filtro del
         * veicolo, [1] fuori regione (o, per una strada d'accesso, fuori dai cerchi), [2] peso
         * infinito nei due versi, [3] peso infinito in avanti (un verso visto da ogni capo: un arco
         * può contare due volte).
         *
         * Con i cerchi d'accesso [access] ([Circle]): anche le primarie e le secondarie (rampe
         * comprese) coi due capi in un cerchio, le terziarie coi due capi entro
         * [Circle.TERTIARY_SHARE] del raggio, col filtro del veicolo, i sensi unici e i divieti di
         * svolta come le autostrade; al più [maxAccessArcs] ([accessCapped]: oltre, solo autostrade).
         * Le letture in più sono solo le adiacenze dei nodi nei cerchi: fuori la classe dell'arco
         * (già letta) basta a scartarle. I cerchi stanno dentro le ellissi della regione (un fuoco
         * ciascuno, raggio ≤ un quarto del tratto). Nel servizio la rete si costruisce senza cerchi
         * (in cache) e le strade d'accesso si aggiungono per richiesta ([withAccess]): stessi archi.
         * Alla fine, con le svolte del profilo, le svolte vietate fra archi della rete ([turnBans]).
         */
        fun build(a: DetourAccess, seeds: IntArray, region: Region, maxArcs: Int = 250_000, cancel: CancelToken = CancelToken.NEVER,
                  counts: IntArray? = null, access: List<Circle> = emptyList(), maxAccessArcs: Int = MAX_ACCESS_ARCS): MotorwayNet =
            grow(a, null, seeds, region, maxArcs, cancel, counts, access, maxAccessArcs)

        /**
         * La rete autostradale [base] (senza strade d'accesso, con le regole di [a]: della cache)
         * più le strade d'accesso dei cerchi [access] e quello che si raggiunge da loro e dai semi
         * [seeds] dentro [need]: come [build] con i cerchi, ma le letture sono solo dei nodi nei
         * cerchi e di quelli nuovi (la parte autostradale si copia in RAM, gli archi e i nodi di
         * [base] con gli stessi indici, le svolte vietate di [base] riprese). [base] troncata: resta
         * com'è (nessuna strada d'accesso).
         */
        fun withAccess(a: DetourAccess, base: MotorwayNet, seeds: IntArray, need: Region, access: List<Circle>, maxArcs: Int = 250_000,
                       cancel: CancelToken = CancelToken.NEVER, counts: IntArray? = null, maxAccessArcs: Int = MAX_ACCESS_ARCS): MotorwayNet {
            require(base.access.isEmpty()) { "rete di base con strade d'accesso" }
            if (access.isEmpty() || base.truncated || base.nodeLat.size != base.nodeCount) return base
            return grow(a, base, seeds, need, maxArcs, cancel, counts, access, maxAccessArcs)
        }

        /** La chiave di una svolta dall'arco [inArc] all'arco [outArc] della rete ([turnBans]). */
        fun pairKey(inArc: Int, outArc: Int): Long = (inArc.toLong() shl 32) or (outArc.toLong() and 0xffffffffL)

        private fun grow(a: DetourAccess, base: MotorwayNet?, seeds: IntArray, region: Region, maxArcs: Int, cancel: CancelToken,
                         counts: IntArray?, access: List<Circle>, maxAccessArcs: Int): MotorwayNet {
            val g = a.graph; val sh = a.shared; val w = a.weighting
            val index = IntIntHashMap(4096)
            val nodes = IntArrayList(); val restricted = IntArrayList(); val lats = DoubleArrayList(); val lons = DoubleArrayList()
            val tails = IntArrayList(); val heads = IntArrayList(); val edges = IntArrayList(); val revs = IntArrayList()
            val ws = DoubleArrayList(); val mss = IntArrayList(); val ms = FloatArrayList()
            val kinds = com.carrotsearch.hppc.ByteArrayList()
            // per nodo: dentro un cerchio d'accesso per le primarie/secondarie (bit 1) e per le terziarie (bit 2)
            val near = com.carrotsearch.hppc.ByteArrayList()
            fun nearBits(lat: Double, lon: Double): Byte {
                if (access.isEmpty()) return 0
                var b = 0
                if (Circle.inAny(access, lat, lon, KIND_MAIN)) b = b or 1
                if (b != 0 && Circle.inAny(access, lat, lon, KIND_TERTIARY)) b = b or 2
                return b.toByte()
            }
            fun visit(n: Int, lat: Double, lon: Double, bits: Byte = nearBits(lat, lon)): Int {
                val i = nodes.size()
                index.put(n, i); nodes.add(n); lats.add(lat); lons.add(lon); near.add(bits)
                restricted.add(if (a.turnCosts?.restrictedAt(n) == true) 1 else 0)
                return i
            }
            // la base (solo RAM): i suoi nodi e archi con gli stessi indici; i suoi nodi sono già
            // espansi per le autostrade, si rileggono solo quelli nei cerchi (per le strade d'accesso)
            val baseNodes = base?.nodeCount ?: 0
            val baseArcs = base?.arcCount ?: 0
            if (base != null) {
                for (i in 0 until baseNodes) {
                    val n = base.nodeGraph[i]
                    index.put(n, i); nodes.add(n); lats.add(base.nodeLat[i]); lons.add(base.nodeLon[i]); near.add(nearBits(base.nodeLat[i], base.nodeLon[i]))
                    restricted.add(if (base.restricted[i]) 1 else 0)
                }
                for (k in 0 until baseArcs) {
                    tails.add(base.arcTail[k]); heads.add(base.arcHead[k]); edges.add(base.arcEdge[k]); revs.add(if (base.arcRev[k]) 1 else 0); kinds.add(KIND_MAJOR)
                    ws.add(base.arcW[k]); mss.add(base.arcMs[k]); ms.add(base.arcM[k])
                }
            }
            for (s in seeds) if (!index.containsKey(s)) visit(s, g.lat(s), g.lon(s))
            val it = g.createEdgeIterator()
            var qi = 0
            var truncated = false
            var accessArcs = 0; var capped = false
            while (qi < nodes.size() && !truncated) {
                if ((qi and 255) == 0) cancel.check()
                val n = nodes.get(qi); val tail = qi; qi++
                val tailNear = near.get(tail).toInt()
                // un nodo della base: solo le strade d'accesso, e solo se è in un cerchio
                val accessOnly = tail < baseNodes
                if (accessOnly && (tailNear == 0 || capped)) continue
                it.setBaseNode(n)
                while (it.next()) {
                    val adj = it.adjNode
                    if (adj == n) continue
                    val rc = it.get(sh.roadClass)
                    // la classe: autostrada e superstrada ovunque; primaria/secondaria (terziaria) solo fra due nodi nei cerchi d'accesso
                    val kind = when (rc) {
                        RoadClass.MOTORWAY, RoadClass.TRUNK -> if (accessOnly) continue else KIND_MAJOR
                        RoadClass.PRIMARY, RoadClass.SECONDARY -> if (tailNear and 1 != 0) KIND_MAIN else continue
                        RoadClass.TERTIARY -> if (tailNear and 2 != 0) KIND_TERTIARY else continue
                        else -> continue
                    }
                    if (kind != KIND_MAJOR && capped) continue
                    if (!a.filter.accept(it)) { if (counts != null) counts[0]++; continue }
                    var head = index.getOrDefault(adj, -1)
                    var lat = 0.0; var lon = 0.0
                    var headNear: Int
                    if (head < 0) {
                        lat = g.lat(adj); lon = g.lon(adj)
                        if (!region.contains(lat, lon)) { if (counts != null) counts[1]++; continue }
                        headNear = nearBits(lat, lon).toInt()
                    } else headNear = near.get(head).toInt()
                    if (kind == KIND_MAIN && headNear and 1 == 0 || kind == KIND_TERTIARY && headNear and 2 == 0) { if (counts != null) counts[1]++; continue }
                    val fw = w.edgeWeight(it, false)
                    if (head < 0) {
                        if (fw.isInfinite() && w.edgeWeight(it, true).isInfinite()) { if (counts != null) counts[2]++; continue }
                        head = visit(adj, lat, lon, headNear.toByte())
                    }
                    if (fw.isInfinite()) { if (counts != null) counts[3]++; continue }
                    tails.add(tail); heads.add(head); edges.add(it.edge); revs.add(if (it.reverse) 1 else 0); kinds.add(kind)
                    ws.add(fw); mss.add(minOf(Int.MAX_VALUE.toLong(), a.timeWeighting.edgeMillis(it, false)).toInt()); ms.add(it.distance.toFloat())
                    if (kind != KIND_MAJOR && ++accessArcs >= maxAccessArcs) capped = true
                    if (tails.size() >= maxArcs) truncated = true
                }
            }
            val net = of(nodes.toArray(), BooleanArray(nodes.size()) { restricted.get(it) != 0 }, tails.toArray(), heads.toArray(), edges.toArray(),
                BooleanArray(revs.size()) { revs.get(it) != 0 }, ws.toArray(), mss.toArray(), ms.toArray(), base?.region ?: region, truncated, lats.toArray(), lons.toArray(),
                access, if (access.isEmpty()) ByteArray(0) else kinds.toArray(), capped)
            val tcp = a.turnCosts
            if (tcp != null) net.turnBans = bans(net, tcp, base?.turnBans, if (base?.turnBans != null) baseArcs else 0, cancel)
            return net
        }

        /**
         * Le svolte vietate fra due archi della rete nei nodi con divieti (una lettura per coppia,
         * una volta per rete invece che a ogni tratto di ogni richiesta): quelle di [baseBans] per
         * le coppie di archi sotto [baseArcs] (le stesse della base), le altre lette.
         */
        private fun bans(net: MotorwayNet, tcp: TurnCostProvider, baseBans: com.carrotsearch.hppc.LongHashSet?, baseArcs: Int,
                         cancel: CancelToken): com.carrotsearch.hppc.LongHashSet {
            val out = if (baseBans != null) com.carrotsearch.hppc.LongHashSet(baseBans) else com.carrotsearch.hppc.LongHashSet()
            var asked = 0
            for (h in 0 until net.nodeCount) {
                if (!net.restricted[h]) continue
                val via = net.nodeGraph[h]
                for (i in net.inStart[h] until net.inStart[h + 1]) {
                    val x = net.inArcs[i]
                    for (k in net.outStart[h] until net.outStart[h + 1]) {
                        val y = net.outArcs[k]
                        if (x < baseArcs && y < baseArcs) continue
                        if (net.arcEdge[x] == net.arcEdge[y]) continue
                        if ((++asked and 1023) == 0) cancel.check()
                        if (tcp.turnWeight(net.arcEdge[x], via, net.arcEdge[y]).isInfinite()) out.add(pairKey(x, y))
                    }
                }
            }
            return out
        }

        /** La rete con le adiacenze compatte calcolate dagli archi (anche per le prove: una rete giocattolo). */
        /** La classe di un arco della rete: autostrada o superstrada (rampe comprese). */
        const val KIND_MAJOR: Byte = 0
        /** Una strada d'accesso primaria o secondaria (rampe comprese), dentro un cerchio d'accesso. */
        const val KIND_MAIN: Byte = 1
        /** Una strada d'accesso terziaria, dentro [Circle.TERTIARY_SHARE] del raggio di un cerchio. */
        const val KIND_TERTIARY: Byte = 2
        /**
         * Gli archi d'accesso (primarie, secondarie, terziarie nei cerchi) al più per rete: oltre, la
         * visita non ne aggiunge altri ([accessCapped]) e la rete autostradale resta completa. Una
         * città grande con i cerchi da 10 km ne ha qualche decina di migliaia: ~40 byte per arco nella
         * rete e ~50 nella ricerca, sul tablet al più ~5 MB.
         */
        const val MAX_ACCESS_ARCS = 60_000

        internal fun of(nodeGraph: IntArray, restricted: BooleanArray, tails: IntArray, heads: IntArray, edges: IntArray, revs: BooleanArray,
                        w: DoubleArray, ms: IntArray, m: FloatArray, region: Region, truncated: Boolean = false,
                        lat: DoubleArray = DoubleArray(0), lon: DoubleArray = DoubleArray(0),
                        access: List<Circle> = emptyList(), kinds: ByteArray = ByteArray(0), accessCapped: Boolean = false): MotorwayNet {
            val n = nodeGraph.size
            fun csr(key: IntArray): Pair<IntArray, IntArray> {
                val start = IntArray(n + 1)
                for (k in key) start[k + 1]++
                for (i in 0 until n) start[i + 1] += start[i]
                val pos = start.copyOf()
                val arcs = IntArray(key.size)
                for (a in key.indices) arcs[pos[key[a]]++] = a
                return start to arcs
            }
            val (outStart, outArcs) = csr(tails)
            val (inStart, inArcs) = csr(heads)
            return MotorwayNet(nodeGraph, restricted, tails, heads, edges, revs, w, ms, m, outStart, outArcs, inStart, inArcs, region, truncated, lat, lon,
                access, kinds, accessCapped)
        }
    }
}

/**
 * Le ultime reti autostradali costruite ([capacity]), senza strade d'accesso, per opzioni del
 * router ([RoutingGraph.routerKey]): una rete vale per una richiesta con la stessa chiave se la sua
 * regione copre quella chiesta ([MotorwayNet.serves]) e [fits] (i nodi del nuovo migliore ci sono:
 * la rete è la parte raggiunta dai semi); una rete troncata solo per la sua stessa regione. La
 * costruzione è fuori dal lucchetto: chi chiede la stessa chiave e una regione dentro quella in
 * costruzione la aspetta (fermandosi col suo [CancelToken]), poi riprova. Vive quanto il servizio.
 *
 * Le strade d'accesso dipendono dai punti del viaggio: si aggiungono per richiesta sopra una rete
 * di qui ([MotorwayNet.withAccess]), così un viaggio con altri punti (o un ricalcolo da una
 * posizione nuova) riusa la parte autostradale e rilegge solo i nodi dei suoi cerchi. Le ultime
 * [overlayCapacity] reti con le strade d'accesso restano per la stessa base, la stessa regione e
 * gli stessi cerchi ([overlay]: la stessa richiesta ripetuta).
 */
class MotorwayNetCache(private val capacity: Int = 2, private val overlayCapacity: Int = 1) {
    private val entries = ArrayList<Pair<RouteOptions, MotorwayNet>>()
    private class Building(val key: RouteOptions, val region: Region) { val done = CountDownLatch(1) }
    private val building = ArrayList<Building>()
    private class Overlay(val base: MotorwayNet, val need: Region, val access: List<Circle>, val net: MotorwayNet)
    private val overlays = ArrayList<Overlay>()

    fun get(key: RouteOptions, need: Region, fits: (MotorwayNet) -> Boolean = { true }, cancel: CancelToken = CancelToken.NEVER,
            build: () -> MotorwayNet): MotorwayNet {
        while (true) {
            var wait: Building? = null
            var mine: Building? = null
            synchronized(this) {
                val i = entries.indexOfFirst { (k, n) -> k == key && n.serves(need) && fits(n) }
                if (i >= 0) { val e = entries.removeAt(i); entries.add(0, e); return e.second }
                wait = building.firstOrNull { it.key == key && it.region.covers(need) }
                if (wait == null) mine = Building(key, need).also { building += it }
            }
            val w = wait
            if (w != null) {
                // la stessa rete in costruzione per un'altra richiesta: si aspetta, poi la si cerca di nuovo
                while (!w.done.await(WAIT_MS, TimeUnit.MILLISECONDS)) cancel.check()
                continue
            }
            val b = mine!!
            try {
                val net = build()
                synchronized(this) {
                    entries.add(0, key to net)
                    while (entries.size > capacity) entries.removeAt(entries.size - 1)
                    // le reti con le strade d'accesso di una base uscita dalla cache non servono più
                    overlays.removeAll { o -> entries.none { it.second === o.base } }
                }
                return net
            } finally {
                synchronized(this) { building.remove(b) }
                b.done.countDown()
            }
        }
    }

    /**
     * La rete con le strade d'accesso [access] sopra la rete [base] (di questa cache) per la regione
     * [need]: l'ultima fatta per la stessa base, la stessa regione e gli stessi cerchi se [fits], se
     * no [build] (fuori dal lucchetto, tenuta se la base è ancora in cache).
     */
    fun overlay(base: MotorwayNet, need: Region, access: List<Circle>, fits: (MotorwayNet) -> Boolean = { true }, build: () -> MotorwayNet): MotorwayNet {
        synchronized(this) {
            val i = overlays.indexOfFirst { o -> o.base === base && o.need.sameAs(need) && Circle.sameAll(o.access, access) && fits(o.net) }
            if (i >= 0) { val o = overlays.removeAt(i); overlays.add(0, o); return o.net }
        }
        val net = build()
        synchronized(this) {
            if (net !== base && entries.any { it.second === base }) {
                overlays.add(0, Overlay(base, need, access, net))
                while (overlays.size > overlayCapacity) overlays.removeAt(overlays.size - 1)
            }
        }
        return net
    }

    @Synchronized fun clear() { entries.clear(); overlays.clear() }
    val size: Int @Synchronized get() = entries.size
    /** Le reti con le strade d'accesso tenute. */
    val overlaySize: Int @Synchronized get() = overlays.size

    private companion object { const val WAIT_MS = 20L }
}
