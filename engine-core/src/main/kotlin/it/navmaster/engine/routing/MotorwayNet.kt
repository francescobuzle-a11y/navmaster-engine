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
 * Le ricerche delle deviazioni ([DetourSearch]) non leggono più il grafo.
 */
class MotorwayNet internal constructor(
    val nodeGraph: IntArray, val restricted: BooleanArray,
    val arcTail: IntArray, val arcHead: IntArray, val arcEdge: IntArray, val arcRev: BooleanArray,
    val arcW: DoubleArray, val arcMs: IntArray, val arcM: FloatArray,
    val outStart: IntArray, val outArcs: IntArray, val inStart: IntArray, val inArcs: IntArray,
    val region: Region, val truncated: Boolean,
    val nodeLat: DoubleArray = DoubleArray(0), val nodeLon: DoubleArray = DoubleArray(0),
) {
    val nodeCount: Int get() = nodeGraph.size
    val arcCount: Int get() = arcTail.size
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
         * (solo la traccia delle varianti): i versi `motorway`/`trunk` lasciati fuori, per motivo:
         * [0] filtro del veicolo, [1] fuori regione, [2] peso infinito nei due versi, [3] peso
         * infinito in avanti (un verso visto da ogni capo: un arco può contare due volte).
         */
        fun build(a: DetourAccess, seeds: IntArray, region: Region, maxArcs: Int = 250_000, cancel: CancelToken = CancelToken.NEVER,
                  counts: IntArray? = null): MotorwayNet {
            val g = a.graph; val sh = a.shared; val w = a.weighting
            val index = IntIntHashMap(4096)
            val nodes = IntArrayList(); val restricted = IntArrayList(); val lats = DoubleArrayList(); val lons = DoubleArrayList()
            val tails = IntArrayList(); val heads = IntArrayList(); val edges = IntArrayList(); val revs = IntArrayList()
            val ws = DoubleArrayList(); val mss = IntArrayList(); val ms = FloatArrayList()
            fun visit(n: Int, lat: Double, lon: Double): Int {
                val i = nodes.size()
                index.put(n, i); nodes.add(n); lats.add(lat); lons.add(lon)
                restricted.add(if (a.turnCosts?.restrictedAt(n) == true) 1 else 0)
                return i
            }
            for (s in seeds) if (!index.containsKey(s)) visit(s, g.lat(s), g.lon(s))
            val it = g.createEdgeIterator()
            var qi = 0
            var truncated = false
            while (qi < nodes.size() && !truncated) {
                if ((qi and 255) == 0) cancel.check()
                val n = nodes.get(qi); val tail = qi; qi++
                it.setBaseNode(n)
                while (it.next()) {
                    val adj = it.adjNode
                    if (adj == n) continue
                    val rc = it.get(sh.roadClass)
                    if (rc != RoadClass.MOTORWAY && rc != RoadClass.TRUNK) continue
                    if (!a.filter.accept(it)) { if (counts != null) counts[0]++; continue }
                    var head = index.getOrDefault(adj, -1)
                    var lat = 0.0; var lon = 0.0
                    if (head < 0) { lat = g.lat(adj); lon = g.lon(adj); if (!region.contains(lat, lon)) { if (counts != null) counts[1]++; continue } }
                    val fw = w.edgeWeight(it, false)
                    if (head < 0) {
                        if (fw.isInfinite() && w.edgeWeight(it, true).isInfinite()) { if (counts != null) counts[2]++; continue }
                        head = visit(adj, lat, lon)
                    }
                    if (fw.isInfinite()) { if (counts != null) counts[3]++; continue }
                    tails.add(tail); heads.add(head); edges.add(it.edge); revs.add(if (it.reverse) 1 else 0)
                    ws.add(fw); mss.add(minOf(Int.MAX_VALUE.toLong(), a.timeWeighting.edgeMillis(it, false)).toInt()); ms.add(it.distance.toFloat())
                    if (tails.size() >= maxArcs) truncated = true
                }
            }
            return of(nodes.toArray(), BooleanArray(nodes.size()) { restricted.get(it) != 0 }, tails.toArray(), heads.toArray(), edges.toArray(),
                BooleanArray(revs.size()) { revs.get(it) != 0 }, ws.toArray(), mss.toArray(), ms.toArray(), region, truncated, lats.toArray(), lons.toArray())
        }

        /** La rete con le adiacenze compatte calcolate dagli archi (anche per le prove: una rete giocattolo). */
        internal fun of(nodeGraph: IntArray, restricted: BooleanArray, tails: IntArray, heads: IntArray, edges: IntArray, revs: BooleanArray,
                        w: DoubleArray, ms: IntArray, m: FloatArray, region: Region, truncated: Boolean = false,
                        lat: DoubleArray = DoubleArray(0), lon: DoubleArray = DoubleArray(0)): MotorwayNet {
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
            return MotorwayNet(nodeGraph, restricted, tails, heads, edges, revs, w, ms, m, outStart, outArcs, inStart, inArcs, region, truncated, lat, lon)
        }
    }
}

/**
 * Le ultime reti costruite ([capacity]), per opzioni del router ([RoutingGraph.routerKey]): una
 * rete vale per una richiesta con la stessa chiave se la sua regione copre quella chiesta
 * ([Region.covers]) e [fits] (i nodi del nuovo migliore ci sono: la rete è la parte raggiunta dai
 * semi); una rete troncata solo per la sua stessa regione (altrove una nuova sarebbe completa).
 * La costruzione è fuori dal lucchetto: chi chiede la stessa chiave e una regione dentro quella in
 * costruzione la aspetta (fermandosi col suo [CancelToken]), poi riprova. Vive quanto il servizio.
 */
class MotorwayNetCache(private val capacity: Int = 2) {
    private val entries = ArrayList<Pair<RouteOptions, MotorwayNet>>()
    private class Building(val key: RouteOptions, val region: Region) { val done = CountDownLatch(1) }
    private val building = ArrayList<Building>()

    fun get(key: RouteOptions, need: Region, fits: (MotorwayNet) -> Boolean = { true }, cancel: CancelToken = CancelToken.NEVER,
            build: () -> MotorwayNet): MotorwayNet {
        while (true) {
            var wait: Building? = null
            var mine: Building? = null
            synchronized(this) {
                val i = entries.indexOfFirst { (k, n) -> k == key && n.region.covers(need) && (!n.truncated || n.region.sameAs(need)) && fits(n) }
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
                }
                return net
            } finally {
                synchronized(this) { building.remove(b) }
                b.done.countDown()
            }
        }
    }

    @Synchronized fun clear() = entries.clear()
    val size: Int @Synchronized get() = entries.size

    private companion object { const val WAIT_MS = 20L }
}
