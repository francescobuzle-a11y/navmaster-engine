package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.routing.RoutePath

/**
 * Un punto agganciato come lo vede la ricerca di un tratto: l'arco, il segmento, la torre e il
 * punto esatto sull'arco (non la coordinata chiesta: due coordinate vicine che si agganciano allo
 * stesso punto danno lo stesso tratto).
 */
internal data class SnapKey(val edge: Int, val wayIndex: Int, val tower: Boolean, val node: Int, val latBits: Long, val lonBits: Long) {
    companion object {
        fun of(s: Snap): SnapKey = SnapKey(s.edge, s.wayIndex, s.position == SnapPosition.TOWER, s.closestNode, s.snappedLat.toBits(), s.snappedLon.toBits())
    }
}

/**
 * Un tratto: le opzioni del router (`RoutingGraph.routerKey`), i due punti, il verso d'arrivo che
 * conta (`Router.arrivalKey`: -1, 0, 1) e le direzioni chieste (`bearings`, bit del Double; NaN se nessuna).
 */
internal data class LegKey(val options: RouteOptions, val from: SnapKey, val to: SnapKey, val arrival: Int, val fromHeading: Long, val toHeading: Long)

/**
 * Il tetto di archi comune a più cache dei tratti ([LegCache]): quelle dei servizi di un processo
 * (uno per Paese caricato; un viaggio tra Paesi le riempie tutte). Oltre [maxEdges] si esclude il
 * tratto usato meno di recente fra tutte. Un lucchetto solo per tutte le sue cache.
 */
internal class LegBudget(val maxEdges: Int = 60_000) {
    internal val caches = ArrayList<java.lang.ref.WeakReference<LegCache>>()
    /** L'orologio degli usi: chi è stato usato meno di recente fra le cache. */
    internal var tick = 0L

    companion object {
        /** Quello del processo: le cache dei servizi (sul tablet tutte insieme ~20 MB al più). */
        val PROCESS = LegBudget()
    }
}

/**
 * I tratti già calcolati di un servizio, i meno usati di recente esclusi oltre [maxEntries] tratti o
 * [maxEdges] archi in tutto (sul tablet un arco con la sua geometria pesa qualche centinaio di byte)
 * e oltre il tetto comune [budget] (null: nessuno). Solo tratti esatti: cercati senza limite di peso,
 * con il router della richiesta (mai penalità), finiti (un annullamento esce prima). I [RoutePath]
 * sono condivisi fra le risposte: nessuno li modifica.
 */
internal class LegCache(val maxEdges: Int = 60_000, val maxEntries: Int = 256, private val budget: LegBudget? = LegBudget.PROCESS) {
    private class Entry(val path: RoutePath, var used: Long)
    private val map = LinkedHashMap<LegKey, Entry>(64, 0.75f, true)
    private val lock: Any = budget ?: this
    private var edges = 0
    private var hits = 0L
    private var misses = 0L
    private var tick = 0L

    init { if (budget != null) synchronized(lock) { budget.caches += java.lang.ref.WeakReference(this) } }

    private fun now(): Long = if (budget != null) ++budget.tick else ++tick

    fun get(k: LegKey): RoutePath? = synchronized(lock) {
        val e = map[k]
        if (e != null) { hits++; e.used = now() } else misses++
        e?.path
    }

    /** Vero se c'è un tratto per [k], senza contarlo né rinfrescarlo. */
    fun contains(k: LegKey): Boolean = synchronized(lock) { map.containsKey(k) }

    fun put(k: LegKey, p: RoutePath) {
        synchronized(lock) {
            if (p.edges.size > maxEdges || (budget != null && p.edges.size > budget.maxEdges)) return
            map.put(k, Entry(p, now()))?.let { edges -= it.path.edges.size }
            edges += p.edges.size
            // il più vecchio per primo; l'ultimo messo è in fondo e ci sta da solo
            while (edges > maxEdges || map.size > maxEntries) if (!evictEldest(k)) break
            // oltre il tetto comune: il meno usato di recente fra tutte le cache
            if (budget != null) while (true) {
                var total = 0
                var oldest: LegCache? = null
                var oldestUsed = Long.MAX_VALUE
                val i = budget.caches.iterator()
                while (i.hasNext()) {
                    val c = i.next().get()
                    if (c == null) { i.remove(); continue }
                    total += c.edges
                    val e = c.map.entries.firstOrNull() ?: continue
                    if (c === this && e.key == k) continue
                    if (e.value.used < oldestUsed) { oldest = c; oldestUsed = e.value.used }
                }
                if (total <= budget.maxEdges || oldest == null || !oldest.evictEldest(if (oldest === this) k else null)) break
            }
        }
    }

    /** Esclude il tratto usato meno di recente, se non è [keep]; falso se non ce n'è un altro. */
    private fun evictEldest(keep: LegKey?): Boolean {
        val i = map.entries.iterator()
        if (!i.hasNext()) return false
        val old = i.next()
        if (old.key == keep) return false
        edges -= old.value.path.edges.size
        i.remove()
        return true
    }

    fun clear() = synchronized(lock) { map.clear(); edges = 0 }

    /** Per le prove: quanti tratti ci sono ora. */
    val size: Int get() = synchronized(lock) { map.size }

    /** Diagnostica: "12 tratti, 3400 archi, colpi 30/42". */
    val info: String get() = synchronized(lock) { "${map.size} tratti, $edges archi, colpi $hits/${hits + misses}" }
}

/**
 * Una ricerca rapida della scelta dei valichi (`CrossBorderDirections.Ctx.weight`): le opzioni
 * intere del router del grafo (avoid, penalità, velocità massima… decidono il peso), i due capi
 * (bit dei Double) coi loro raggi di aggancio.
 */
internal data class QuickKey(val options: RouteOptions, val aLat: Long, val aLon: Long, val ra: Long, val bLat: Long, val bLon: Long, val rb: Long)

/**
 * I pesi delle ricerche rapide di un servizio, con il limite sotto cui sono stati cercati: un peso
 * finito è esatto e si riusa sotto qualunque limite (+∞ se non sta sotto quello chiesto); «nessun
 * percorso sotto l» vale solo per i limiti ≤ l. Vive quanto il servizio (grafo riaperto = memoria
 * vuota), i meno usati di recente escono oltre [maxEntries] (≈ 80 B l'uno). Un lucchetto solo.
 */
internal class QuickWeightCache(val maxEntries: Int = 4096) {
    private class Entry(var w: Double, var limit: Double)
    private val map = object : LinkedHashMap<QuickKey, Entry>(64, 0.75f, true) {
        override fun removeEldestEntry(eldest: MutableMap.MutableEntry<QuickKey, Entry>?) = size > maxEntries
    }
    private var hits = 0L
    private var misses = 0L

    /** Il peso sotto [limit] se la memoria lo sa (+∞: nessun percorso sotto [limit]); null se va cercato. */
    @Synchronized fun get(key: QuickKey, limit: Double): Double? {
        val e = map[key]
        if (e == null || !(e.w.isFinite() || e.limit >= limit)) { misses++; return null }
        hits++
        return if (e.w < limit) e.w else Double.POSITIVE_INFINITY
    }

    /** Ricorda [w] cercato sotto [limit]: una voce peggiore (∞ con un limite più basso) non sostituisce mai una migliore. */
    @Synchronized fun put(key: QuickKey, w: Double, limit: Double) {
        val e = map[key]
        if (e == null) { map[key] = Entry(w, limit); return }
        if (e.w.isFinite()) return
        if (w.isFinite() || limit > e.limit) { e.w = w; e.limit = limit }
    }

    /**
     * La voce di [key] così com'è: il peso e il limite sotto cui è stato cercato (∞: nessun percorso più
     * leggero del limite); null se non c'è. Per i percorsi mirati della scelta dei valichi, che col peso
     * esatto o col limite dimostrato fanno a meno della ricerca.
     */
    @Synchronized fun entry(key: QuickKey): Pair<Double, Double>? {
        val e = map[key]
        if (e == null) { misses++; return null }
        hits++
        return e.w to e.limit
    }

    @Synchronized fun clear() { map.clear() }

    /** Per le prove: quante voci ci sono ora. */
    val size: Int @Synchronized get() = map.size

    /** Diagnostica: "118 pesi, colpi 118/236". */
    val info: String @Synchronized get() = "${map.size} pesi, colpi $hits/${hits + misses}"
}

/**
 * Le scelte dei due valichi di un transito (`CrossBorderDirections` chooseTwo) di un servizio: per
 * chiave i due valichi scelti e il peso del viaggio, col limite sotto cui è stata fatta la scelta. Come
 * [QuickWeightCache]: un peso finito è il minimo esatto fra tutte le coppie di valichi e vale sotto
 * qualunque limite; «nessuna coppia sotto l» (peso ∞) vale solo per i limiti ≤ l; una voce peggiore
 * non sostituisce una migliore. La chiave (grafi coi loro servizi, opzioni, punti, elenchi dei valichi)
 * la costruisce chi chiama. Così la richiesta delle varianti, subito dopo quella del migliore, non rifà
 * la scelta (in CI fino a 20 s, sul tablet minuti). Pochi KB: due riferimenti e due numeri per voce
 * più la chiave, i meno usati di recente escono oltre [maxEntries]. Un lucchetto solo.
 */
internal class TransitCache(val maxEntries: Int = 64) {
    /** [seq]: per la memoria del passo 1 di una scelta per confine, l'indice della sequenza di Paesi del migliore (-1 altrove). */
    class Entry(val x: Transfer?, val y: Transfer?, val w: Double, val limit: Double, val seq: Int = -1)
    private val map = object : LinkedHashMap<Any, Entry>(16, 0.75f, true) {
        override fun removeEldestEntry(eldest: MutableMap.MutableEntry<Any, Entry>?) = size > maxEntries
    }

    @Synchronized fun get(key: Any): Entry? = map[key]

    /** Ricorda la scelta ([x], [y], [w]) fatta sotto [limit]; [w] ∞: nessuna coppia più leggera di [limit]. */
    @Synchronized fun put(key: Any, x: Transfer?, y: Transfer?, w: Double, limit: Double, seq: Int = -1) {
        val e = map[key]
        if (e != null && (e.w.isFinite() || (!w.isFinite() && limit <= e.limit))) return
        map[key] = if (w.isFinite()) Entry(x, y, w, limit, seq) else Entry(null, null, Double.POSITIVE_INFINITY, limit)
    }

    /** Toglie la voce di [key] (una memoria del passo 1 che non torna: [put] non sostituirebbe il suo peso finito). */
    @Synchronized fun remove(key: Any) { map.remove(key) }

    @Synchronized fun clear() { map.clear() }

    /** Per le prove: quante voci ci sono ora. */
    val size: Int @Synchronized get() = map.size

    /** Per le prove: le chiavi di ora. */
    internal val keys: List<Any> @Synchronized get() = map.keys.toList()
}

/** Un aggancio: le opzioni del router (`RoutingGraph.routerKey`), la coordinata, il raggio, il controllo delle reti isolate, la direzione. */
internal data class SnapCacheKey(val options: RouteOptions, val latBits: Long, val lonBits: Long, val radius: Double, val checkNetwork: Boolean, val headingBits: Long)

/**
 * Gli agganci già fatti di un servizio (i [max] più recenti). Un punto non trovato non resta
 * (l'eccezione esce da [get]); i [Snap] restituiti sono condivisi e nessuno li modifica.
 */
internal class SnapCache(val max: Int = 512) {
    private val map = object : LinkedHashMap<SnapCacheKey, Snap>(64, 0.75f, true) {
        override fun removeEldestEntry(eldest: MutableMap.MutableEntry<SnapCacheKey, Snap>?) = size > max
    }

    /** Il valore in cache, o null. */
    @Synchronized fun peek(k: SnapCacheKey): Snap? = map[k]

    /** Il valore in cache o quello di [compute] (calcolato fuori dal lucchetto: due thread non si aspettano). */
    fun get(k: SnapCacheKey, compute: () -> Snap): Snap {
        peek(k)?.let { return it }
        val s = compute()
        synchronized(this) { map[k] = s }
        return s
    }

    @Synchronized fun clear() = map.clear()
}
