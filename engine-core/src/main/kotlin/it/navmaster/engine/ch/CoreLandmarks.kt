package it.navmaster.engine.ch

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import it.navmaster.engine.routing.NodeHeap
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.storage.IntSource
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.Weighting
import java.util.Random

/**
 * Landmark per il core (ORS `CoreLandmarkStorage`): per ogni nodo del core e per ogni landmark L
 * le distanze d(L→v) e d(v→L) sul grafo del core (archi base e shortcut fra nodi del core, pesi
 * senza restrizioni). Servono come euristica A* nella fase 2 del [CoreDijkstra]: ogni restrizione
 * o rallentamento della richiesta può solo aumentare i pesi, quindi i limiti inferiori restano validi.
 *
 * Un insieme può essere preparato con delle strade escluse (ORS `lmsets`, per esempio `tollways`):
 * le distanze sono quelle del core senza quelle strade, e i limiti valgono per le richieste che le
 * evitano tutte (e magari altre). Per «evita pedaggi» i landmark dell'insieme completo presumono le
 * autostrade a pedaggio percorribili e danno limiti deboli: la ricerca esplorava un terzo del core.
 *
 * File `landmarks_<nome>`: header [0] versione, [1] numero landmark, [2] nodi del core, [3] nodi del grafo,
 * [4] strade escluse ([AvoidFeatureFlags]), [5] 1 se l'indice del core non è nel file (quello dell'insieme
 * completo dello stesso core); poi per ogni nodo del grafo un int (indice nel core o -1, se presente), poi
 * per ogni nodo del core `2·k` float (d(L→v), d(v→L) per landmark, +∞ se irraggiungibile), poi i nodi landmark.
 */
class CoreLandmarks private constructor(
    val landmarkCount: Int,
    private val coreIndex: IntSource,
    /** Le distanze come bit dei float (letti dal file mappato senza copiarli nell'heap). */
    private val distanceBits: IntSource,
    val landmarkNodes: IntArray,
    /** Le strade escluse dal calcolo delle distanze ([AvoidFeatureFlags]); 0: nessuna, l'insieme completo. */
    val avoidFeatures: Int = 0,
    /** Falso quando [coreIndex] è quello di un altro insieme (non va chiuso due volte). */
    private val ownsIndex: Boolean = true,
) {
    private constructor(k: Int, coreIndex: IntArray, distances: FloatArray, lms: IntArray, avoid: Int) :
        this(k, IntSource.OfArray(coreIndex), IntSource.OfArray(IntArray(distances.size) { java.lang.Float.floatToRawIntBits(distances[it]) }), lms, avoid)

    /** Il nome dell'insieme (`allow_all`, `tollways`, `highways_tollways`…), come in ORS. */
    val setName: String get() = setName(avoidFeatures)

    /** Vero se i limiti di questo insieme valgono per una richiesta che evita [requestAvoid]: le strade escluse qui sono fra quelle evitate. */
    fun admissibleFor(requestAvoid: Int) = avoidFeatures and requestAvoid.inv() == 0

    fun isCoreNode(node: Int) = coreIndex[node] >= 0

    /**
     * The distances kept in memory as 16-bit steps of [step] (as ORS and GraphHopper keep their
     * landmarks): read from the mapped file one at a time they cost a native call each on Android,
     * 32 per node of the search (3 s for a long lorry trip on the tablet). Made at the first search.
     */
    @Volatile private var quantized: ShortArray? = null
    /** The size of a step: each distance is within half a step (the potentials allow for it). */
    @Volatile var step = 0.0; private set

    /**
     * The distances in memory (once; the file mapped stays for the rest). Not used: rounded to
     * 16 bits the potentials are no longer exactly consistent and the path could be a little off
     * the best one (seen on Canneto → Innsbruck); the gain was a fifth of the time only.
     */
    fun warm() {
        if (!QUANTIZE) return
        if (quantized != null || distanceBits !is IntSource.OfData) return
        synchronized(this) {
            if (quantized != null) return
            val n = distanceBits.size
            val buf = IntArray(minOf(n, 1 shl 20))
            var max = 0f
            var i = 0
            while (i < n) {
                val c = minOf(buf.size, n - i)
                distanceBits.read(i, buf, c)
                for (j in 0 until c) { val f = java.lang.Float.intBitsToFloat(buf[j]); if (f.isFinite() && f > max) max = f }
                i += c
            }
            val s = if (max > 0f) max.toDouble() / (INF_STEP - 1) else 1.0
            val q = ShortArray(n)
            i = 0
            while (i < n) {
                val c = minOf(buf.size, n - i)
                distanceBits.read(i, buf, c)
                for (j in 0 until c) {
                    val f = java.lang.Float.intBitsToFloat(buf[j])
                    q[i + j] = (if (f.isFinite()) Math.round(f / s).toInt().coerceIn(0, INF_STEP - 1) else INF_STEP).toShort()
                }
                i += c
            }
            step = s
            quantized = q
        }
    }

    private fun distance(index: Int): Double {
        val q = quantized
        if (q != null) {
            val v = q[index].toInt() and 0xFFFF
            return if (v == INF_STEP) Double.POSITIVE_INFINITY else v * step
        }
        return java.lang.Float.intBitsToFloat(distanceBits[index]).toDouble()
    }

    /** d(L→v); +∞ se v non è nel core o irraggiungibile. */
    fun fromLandmark(l: Int, node: Int): Double {
        val ci = coreIndex[node]; if (ci < 0) return Double.POSITIVE_INFINITY
        return distance(ci * 2 * landmarkCount + 2 * l)
    }

    /** d(v→L). */
    fun toLandmark(l: Int, node: Int): Double {
        val ci = coreIndex[node]; if (ci < 0) return Double.POSITIVE_INFINITY
        return distance(ci * 2 * landmarkCount + 2 * l + 1)
    }

    fun close() { if (ownsIndex) coreIndex.close(); distanceBits.close() }

    /** Riscaldamento (vedi [DataAccess.touch]): il file delle distanze (che contiene anche l'indice del core, quando è suo). Byte toccati. */
    fun touch(progress: ((Long) -> Unit)? = null): Long = distanceBits.touch(progress)
    /** I byte che [touch] leggerebbe. */
    val bytes: Long get() = distanceBits.bytes

    /**
     * Salva nel file `landmarks_<name>`; [withIndex] falso per un insieme con strade escluse, che al
     * caricamento prende l'indice del core dall'insieme completo (28 MB in meno per l'Italia).
     */
    fun save(dir: GraphDirectory, name: String, withIndex: Boolean = true) {
        val da = dir.create("landmarks_$name", 1 shl 20)
        val coreCount = if (landmarkCount == 0) 0 else distanceBits.size / (2 * landmarkCount)
        da.setHeader(0, GRAPH_FORMAT_VERSION); da.setHeader(1, landmarkCount); da.setHeader(2, coreCount); da.setHeader(3, coreIndex.size)
        da.setHeader(4, avoidFeatures); da.setHeader(5, if (withIndex) 0 else 1)
        val idxBytes = if (withIndex) coreIndex.size.toLong() * 4 else 0L
        da.ensureCapacity(idxBytes + distanceBits.size.toLong() * 4 + landmarkNodes.size.toLong() * 4 + 4)
        if (withIndex) for (i in 0 until coreIndex.size) da.setInt(i.toLong() * 4, coreIndex[i])
        for (i in 0 until distanceBits.size) da.setInt(idxBytes + i.toLong() * 4, distanceBits[i])
        val lmBytes = idxBytes + distanceBits.size.toLong() * 4
        for (i in landmarkNodes.indices) da.setInt(lmBytes + i.toLong() * 4, landmarkNodes[i])
        da.flush(); da.close()
    }

    companion object {
        /** The 16-bit value of an unreachable landmark. */
        private const val INF_STEP = 0xFFFF
        /** Landmarks rounded to 16 bits in memory (see [warm]): off, the search stays exact. */
        private const val QUANTIZE = false

        /** Il nome di un insieme dalle strade escluse: `allow_all`, `tollways`, `highways_tollways`… */
        fun setName(avoid: Int): String {
            if (avoid == 0) return "allow_all"
            val names = ArrayList<String>()
            if (avoid and AvoidFeatureFlags.HIGHWAYS != 0) names += "highways"
            if (avoid and AvoidFeatureFlags.TOLLWAYS != 0) names += "tollways"
            if (avoid and AvoidFeatureFlags.STEPS != 0) names += "steps"
            if (avoid and AvoidFeatureFlags.FERRIES != 0) names += "ferries"
            if (avoid and AvoidFeatureFlags.FORDS != 0) names += "fords"
            return names.joinToString("_")
        }

        /** Le strade escluse dal nome di un insieme (`highways_tollways`, anche `highways+tollways`); 0 per `allow_all` o un nome sconosciuto. */
        fun parseSetName(name: String): Int = name.split('_', '+').fold(0) { acc, part -> acc or AvoidFeatureFlags.fromString(part.trim()) }

        /** Il nome del file (dopo `landmarks_`) dell'insieme di un core: `<coreKey>` per quello completo, `<coreKey>_<insieme>` per gli altri. */
        fun fileName(coreKey: String, avoid: Int): String = if (avoid == 0) coreKey else "${coreKey}_${setName(avoid)}"

        /**
         * Carica un insieme; [base] è l'insieme completo dello stesso core, da cui un insieme salvato
         * senza indice ([save] con `withIndex = false`) prende l'indice dei nodi del core.
         */
        fun load(dir: GraphDirectory, name: String, nodeCount: Int, base: CoreLandmarks? = null): CoreLandmarks? {
            val da = dir.open("landmarks_$name", 1 shl 20) ?: return null
            var keep = false
            try {
                if (da.getHeader(0) != GRAPH_FORMAT_VERSION || da.getHeader(3) != nodeCount) return null
                val k = da.getHeader(1); val coreCount = da.getHeader(2)
                val avoid = da.getHeader(4); val sharedIndex = da.getHeader(5) == 1
                if (sharedIndex && (base == null || base.coreIndex.size != nodeCount)) return null
                val idxBytes = if (sharedIndex) 0L else nodeCount.toLong() * 4
                val nDist = coreCount * 2 * k
                val lmBytes = idxBytes + nDist.toLong() * 4
                val lms = IntArray(k) { da.getInt(lmBytes + it.toLong() * 4) }
                // grafo mappato: letti dal file (niente copia nell'heap); in RAM copiati negli array
                if (dir.mmap) {
                    keep = true
                    val index = if (sharedIndex) base!!.coreIndex else IntSource.OfData(da, 0, nodeCount)
                    return CoreLandmarks(k, index, IntSource.OfData(da, idxBytes, nDist), lms, avoid, ownsIndex = !sharedIndex)
                }
                val index = if (sharedIndex) base!!.coreIndex else IntSource.OfArray(IntArray(nodeCount) { da.getInt(it.toLong() * 4) })
                return CoreLandmarks(k, index, IntSource.OfArray(IntArray(nDist) { da.getInt(idxBytes + it.toLong() * 4) }), lms, avoid, ownsIndex = !sharedIndex)
            } finally { if (!keep) da.close() }
        }

        /**
         * Prepara [count] landmark scegliendo ogni volta il nodo del core più lontano da quelli già scelti
         * (selezione "farthest" di GraphHopper), poi calcola le distanze in entrambi i versi.
         */
        fun prepare(ch: CHGraph, weighting: Weighting, count: Int = 16, log: ((String) -> Unit)? = null, seed: Long = 123,
                    filter: EdgeFilter? = null, avoidFeatures: Int = 0): CoreLandmarks {
            val base = ch.base; val storage = ch.storage
            val n = base.nodeCount
            val coreLevel = n
            val coreIndex = IntArray(n) { -1 }
            var coreCount = 0
            for (v in 0 until n) if (storage.level(v) >= coreLevel) coreIndex[v] = coreCount++
            if (coreCount == 0) return CoreLandmarks(0, coreIndex, FloatArray(0), IntArray(0), avoidFeatures)
            val k = minOf(count, coreCount)
            val lms = IntArray(k) { -1 }
            val minDist = DoubleArray(n) { Double.POSITIVE_INFINITY }
            val dist = DoubleArray(n)
            val it = ch.iterator()
            fun explore(from: Int, reverse: Boolean, out: DoubleArray) {
                java.util.Arrays.fill(out, Double.POSITIVE_INFINITY)
                val heap = NodeHeap(1024); val settled = java.util.BitSet(n)
                out[from] = 0.0; heap.push(from, 0.0)
                while (!heap.isEmpty) {
                    val v = heap.pop()
                    if (settled.get(v)) continue
                    settled.set(v)
                    val w = out[v]
                    it.setBaseNode(v, reverse)
                    while (it.next()) {
                        val adj = it.adjNode
                        if (storage.level(adj) < coreLevel) continue
                        if (filter != null && !it.accepts(filter)) continue
                        val nw = w + it.weight
                        if (nw < out[adj]) { out[adj] = nw; heap.push(adj, nw) }
                    }
                }
            }
            // la parte principale del core: da un nodo casuale si può finire in una zona chiusa (sensi
            // unici, restrizioni) da cui si raggiunge poco; allora tutti i landmark starebbero lì e le
            // distanze sarebbero infinite per il resto (limiti nulli). Fra alcuni nodi a caso quello che
            // raggiunge ed è raggiunto dalla parte più grande; i landmark solo in quella parte
            val rnd = Random(seed)
            val coreNodes = IntArray(coreCount).also { a -> var c = 0; for (v in 0 until n) if (coreIndex[v] >= 0) a[c++] = v }
            var start = coreNodes[0]; var main = java.util.BitSet(n); var mainSize = -1
            for (attempt in 0 until 12) {
                val s = coreNodes[rnd.nextInt(coreCount)]
                explore(s, false, dist)
                val reached = java.util.BitSet(n)
                for (v in coreNodes) if (dist[v].isFinite()) reached.set(v)
                explore(s, true, dist)
                val both = java.util.BitSet(n)
                for (v in coreNodes) if (reached.get(v) && dist[v].isFinite()) both.set(v)
                val size = both.cardinality()
                if (size > mainSize) { mainSize = size; main = both; start = s }
                if (mainSize * 2 >= coreCount) break
            }
            log?.invoke("landmark ${setName(avoidFeatures)}: parte principale del core ${mainSize} nodi su $coreCount")
            // primo landmark: il nodo della parte principale più lontano dal nodo di partenza
            explore(start, false, dist)
            var farthest = start; var best = -1.0
            for (v in coreNodes) if (main.get(v) && dist[v].isFinite() && dist[v] > best) { best = dist[v]; farthest = v }
            lms[0] = farthest
            return compute(ch, coreIndex, coreCount, lms, minDist, dist, main, coreNodes, log, filter, avoidFeatures)
        }

        /** Landmark nei nodi dati (prove: per esempio in una zona chiusa del core). */
        internal fun prepareAt(ch: CHGraph, nodes: IntArray): CoreLandmarks {
            val n = ch.base.nodeCount
            val coreIndex = IntArray(n) { -1 }
            var coreCount = 0
            for (v in 0 until n) if (ch.storage.level(v) >= n) coreIndex[v] = coreCount++
            val coreNodes = IntArray(coreCount).also { a -> var c = 0; for (v in 0 until n) if (coreIndex[v] >= 0) a[c++] = v }
            return compute(ch, coreIndex, coreCount, nodes.copyOf(), DoubleArray(n) { Double.POSITIVE_INFINITY }, DoubleArray(n), null, coreNodes, null, null, 0)
        }

        /**
         * Le distanze dai e verso i landmark [lms] (scelti man mano, il più lontano dai precedenti fra i
         * nodi di [main], quando un elemento è -1).
         */
        private fun compute(ch: CHGraph, coreIndex: IntArray, coreCount: Int, lms: IntArray, minDist: DoubleArray, dist: DoubleArray,
                            main: java.util.BitSet?, coreNodes: IntArray, log: ((String) -> Unit)?, filter: EdgeFilter?, avoidFeatures: Int): CoreLandmarks {
            val n = ch.base.nodeCount; val storage = ch.storage; val coreLevel = n
            val k = lms.size
            val it = ch.iterator()
            fun explore(from: Int, reverse: Boolean, out: DoubleArray) {
                java.util.Arrays.fill(out, Double.POSITIVE_INFINITY)
                val heap = NodeHeap(1024); val settled = java.util.BitSet(n)
                out[from] = 0.0; heap.push(from, 0.0)
                while (!heap.isEmpty) {
                    val v = heap.pop()
                    if (settled.get(v)) continue
                    settled.set(v)
                    val w = out[v]
                    it.setBaseNode(v, reverse)
                    while (it.next()) {
                        val adj = it.adjNode
                        if (storage.level(adj) < coreLevel) continue
                        if (filter != null && !it.accepts(filter)) continue
                        val nw = w + it.weight
                        if (nw < out[adj]) { out[adj] = nw; heap.push(adj, nw) }
                    }
                }
            }
            val distances = FloatArray(coreCount * 2 * k)
            for (l in 0 until k) {
                if (lms[l] < 0) {
                    var cand = -1; var cbest = -1.0
                    for (v in coreNodes) if ((main == null || main.get(v)) && minDist[v].isFinite() && minDist[v] > cbest) { cbest = minDist[v]; cand = v }
                    lms[l] = if (cand < 0) lms[0] else cand
                }
                val lmNode = lms[l]
                explore(lmNode, false, dist) // L→v
                for (v in 0 until n) { val ci = coreIndex[v]; if (ci >= 0) { distances[ci * 2 * k + 2 * l] = dist[v].toFloat(); if (dist[v] < minDist[v]) minDist[v] = dist[v] } }
                explore(lmNode, true, dist) // v→L
                for (v in 0 until n) { val ci = coreIndex[v]; if (ci >= 0) distances[ci * 2 * k + 2 * l + 1] = dist[v].toFloat() }
                log?.invoke("landmark ${l + 1}/$k: nodo $lmNode")
            }
            return CoreLandmarks(k, coreIndex, distances, lms, avoidFeatures)
        }
    }
}

/**
 * Potenziali A* bilanciati per la fase 2 del core (approccio simmetrico, Goldberg & Harrelson):
 * p_f(v) = ½ (h_t(v) − h_s(v)), p_b = −p_f. Le chiavi dell'heap sono g + p; con costi ridotti
 * non negativi la ricerca bidirezionale può fermarsi quando min_key_f + min_key_b ≥ migliore.
 *
 * h_t(v) è un limite inferiore del costo da v a una qualunque destinazione di fase 2 (seme e con costo b(e)):
 * d(v→e)+b(e) ≥ d(v→L) − max_e(d(e→L) − b(e))  e  ≥ min_e(d(L→e) + b(e)) − d(L→v); analogamente h_s.
 */
class CorePotentials(private val lm: CoreLandmarks) {
    private val k = lm.landmarkCount
    init { lm.warm() }
    /** Each bound is a difference of two distances, each within half a step: a step off at most. */
    private val margin = EPS + 2 * lm.step
    private val tC1 = DoubleArray(k) { Double.NEGATIVE_INFINITY }  // max_e (d(e→L) − b(e))
    private val tC2 = DoubleArray(k) { Double.POSITIVE_INFINITY }  // min_e (d(L→e) + b(e))
    private val sC1 = DoubleArray(k) { Double.NEGATIVE_INFINITY }  // max_e (d(L→e) − f(e))
    private val sC2 = DoubleArray(k) { Double.POSITIVE_INFINITY }  // min_e (d(e→L) + f(e))
    /**
     * Il primo termine di un landmark vale solo se tutte le destinazioni (sorgenti) lo raggiungono
     * (ne sono raggiunte): con una destinazione e da cui L è irraggiungibile, d(v→L) − d(e→L) non è un
     * limite per e, e saltare il termine nodo per nodo rompe la coerenza dei potenziali (la ricerca A*
     * può fermarsi prima del percorso migliore: visto coi landmark in una zona chiusa). Quando vale, un
     * nodo che non raggiunge L non raggiunge nessuna destinazione: potenziale infinito.
     */
    private val tAll = BooleanArray(k) { true }
    private val sAll = BooleanArray(k) { true }
    private var hasTargets = false; private var hasSources = false

    fun addTarget(node: Int, b: Double) {
        if (!lm.isCoreNode(node)) return
        hasTargets = true
        for (l in 0 until k) {
            val toL = lm.toLandmark(l, node); val fromL = lm.fromLandmark(l, node)
            if (toL.isFinite()) tC1[l] = maxOf(tC1[l], toL - b) else tAll[l] = false
            if (fromL.isFinite()) tC2[l] = minOf(tC2[l], fromL + b)
        }
    }

    fun addSource(node: Int, f: Double) {
        if (!lm.isCoreNode(node)) return
        hasSources = true
        for (l in 0 until k) {
            val fromL = lm.fromLandmark(l, node); val toL = lm.toLandmark(l, node)
            if (fromL.isFinite()) sC1[l] = maxOf(sC1[l], fromL - f) else sAll[l] = false
            if (toL.isFinite()) sC2[l] = minOf(sC2[l], toL + f)
        }
    }

    /**
     * Limite inferiore del costo residuo da [node] alle destinazioni; +∞ quando i landmark dicono che
     * da [node] non se ne raggiunge nessuna. Il secondo termine vale anche con destinazioni che L non
     * raggiunge: L raggiunge [node], quindi [node] non raggiunge quelle.
     */
    fun toTargets(node: Int): Double {
        if (!hasTargets || !lm.isCoreNode(node)) return 0.0
        var h = 0.0
        for (l in 0 until k) {
            if (tAll[l] && tC1[l].isFinite()) {
                val toL = lm.toLandmark(l, node)
                if (toL.isInfinite()) return Double.POSITIVE_INFINITY
                h = maxOf(h, toL - tC1[l])
            }
            val fromL = lm.fromLandmark(l, node)
            if (fromL.isFinite() && tC2[l].isFinite()) h = maxOf(h, tC2[l] - fromL)
        }
        return maxOf(0.0, h - margin)
    }

    /** Limite inferiore del costo dalle sorgenti a [node]; +∞ quando nessuna sorgente lo raggiunge (come sopra). */
    fun fromSources(node: Int): Double {
        if (!hasSources || !lm.isCoreNode(node)) return 0.0
        var h = 0.0
        for (l in 0 until k) {
            if (sAll[l] && sC1[l].isFinite()) {
                val fromL = lm.fromLandmark(l, node)
                if (fromL.isInfinite()) return Double.POSITIVE_INFINITY
                h = maxOf(h, fromL - sC1[l])
            }
            val toL = lm.toLandmark(l, node)
            if (toL.isFinite() && sC2[l].isFinite()) h = maxOf(h, sC2[l] - toL)
        }
        return maxOf(0.0, h - margin)
    }

    /** p_f(v); la ricerca all'indietro usa −p_f. */
    fun forward(node: Int): Double = 0.5 * (toTargets(node) - fromSources(node))

    companion object { private const val EPS = 1e-3 }
}
