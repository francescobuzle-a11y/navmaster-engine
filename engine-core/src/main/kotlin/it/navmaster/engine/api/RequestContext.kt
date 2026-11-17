package it.navmaster.engine.api

import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.SearchCancelledException
import java.util.concurrent.TimeUnit
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock

/**
 * Le misure di una richiesta (diagnostica): aggancio, ogni tratto con la sua fonte, la costruzione
 * della risposta. [line] è la riga `stats …` del registro dell'app; non comincia con nessuno dei
 * prefissi che RouteProgress interpreta.
 */
class RequestStats {
    /** Il profilo e il numero di punti della richiesta, per la riga. */
    @Volatile var profile = ""
    @Volatile var points = 0
    @Volatile var snapMs = 0L
    @Volatile var snapsNew = 0
    @Volatile var snapsCached = 0
    val legs: MutableList<LegStats> = java.util.Collections.synchronizedList(ArrayList())
    /** Il tempo delle ricerche dei tratti (orologio, anche quando vanno in parallelo). */
    @Volatile var searchWallMs = 0L
    /** La risposta: tutto [buildMs], di cui istruzioni, strade (`navmaster.roads`) e punti lungo il percorso. */
    @Volatile var buildMs = 0L
    @Volatile var instructionsMs = 0L
    @Volatile var roadsMs = 0L
    @Volatile var pointsMs = 0L
    @Volatile var totalMs = 0L
    /** Riepilogo delle varianti (vedi `navmaster.alternatives`). */
    @Volatile var variants: String = ""
    /** Un viaggio tra Paesi: la scelta dei valichi (ricerche, ms, pesi dei pezzi), prima dei pezzi finali. */
    @Volatile var crossings: String = ""
    val children: MutableList<Pair<String, RequestStats>> = java.util.Collections.synchronizedList(ArrayList())

    /** Le misure di una parte della richiesta (un pezzo di un viaggio tra Paesi, una variante). */
    fun child(name: String): RequestStats = RequestStats().also { children += name to it }

    /** Aggiunge [text] a [crossings] (un viaggio può sceglierli più volte: il divieto, poi la penalità). */
    @Synchronized fun addCrossings(text: String) { crossings = if (crossings.isEmpty()) text else "$crossings; $text" }

    /**
     * Mette da parte, come parte [name], le misure raccolte finora: un tentativo scartato (il divieto
     * prima di «evita» come penalità) resta nella riga, separato dal calcolo che dà la risposta.
     */
    fun setAside(name: String) {
        val c = RequestStats()
        c.profile = profile; c.points = points
        c.snapMs = snapMs; c.snapsNew = snapsNew; c.snapsCached = snapsCached; c.searchWallMs = searchWallMs
        c.buildMs = buildMs; c.instructionsMs = instructionsMs; c.roadsMs = roadsMs; c.pointsMs = pointsMs; c.totalMs = totalMs
        c.variants = variants; c.crossings = crossings
        synchronized(legs) { c.legs += legs; legs.clear() }
        synchronized(children) { c.children += children; children.clear() }
        snapMs = 0; snapsNew = 0; snapsCached = 0; searchWallMs = 0
        buildMs = 0; instructionsMs = 0; roadsMs = 0; pointsMs = 0; totalMs = 0
        variants = ""; crossings = ""
        children += name to c
    }

    /** "stats hgv 4 punti: aggancio … | tratti … | risposta … | varianti …". */
    fun line(): String = "stats " + body()

    private fun body(): String {
        val sb = StringBuilder()
        sb.append(profile.removePrefix("driving-").ifEmpty { "?" }).append(' ').append(points).append(" punti: ")
        sb.append("aggancio ").append(snapMs).append(" ms (").append(snapsNew).append(" nuovi, ").append(snapsCached).append(" in cache)")
        val ls = synchronized(legs) { legs.toList() }
        sb.append(" | tratti ").append(searchWallMs).append(" ms")
        if (ls.isNotEmpty()) sb.append(": ").append(ls.joinToString("; ") { it.text() })
        sb.append(" | risposta ").append(buildMs).append(" ms (istruzioni ").append(instructionsMs).append(", strade ").append(roadsMs).append(", punti ").append(pointsMs).append(')')
        if (variants.isNotEmpty()) sb.append(" | varianti ").append(variants)
        if (crossings.isNotEmpty()) sb.append(" | valichi ").append(crossings)
        sb.append(" | totale ").append(totalMs).append(" ms")
        val cs = synchronized(children) { children.toList() }
        for ((name, c) in cs) sb.append(" [").append(name).append(": ").append(c.body()).append(']')
        return sb.toString()
    }
}

/**
 * Un tratto di una richiesta: da dove viene ([Source]), quanto è costato, con quale ricerca
 * ([algorithm]: `Router.lastAlgorithm`, e [search]: le fasi della ricerca, `Router.lastSearch`), su quale thread.
 */
class LegStats(val index: Int, val source: Source, val ms: Long, val algorithm: String, val search: String, val edges: Int, val meters: Double, val thread: String) {
    /**
     * CACHE: dalla cache dei tratti; CALC: cercato; SPEC_OK: cercato in anticipo senza il verso
     * d'arrivo, ed è valso; RECALC: ricercato col verso d'arrivo; REUSED: ripreso dal percorso
     * migliore; PENALIZED: cercato con le penalità di una variante.
     */
    enum class Source { CACHE, CALC, SPEC_OK, RECALC, REUSED, PENALIZED }

    fun text(): String = "${index} ${source.name.lowercase()} $algorithm $ms ms $edges archi %.1f km".format(meters / 1000) +
        (if (search.isNotEmpty()) " ($search)" else "") + (if (thread.isNotEmpty()) " @$thread" else "")
}

/**
 * Il contesto di una richiesta al motore: il suo annullamento, la sua priorità (quella del percorso
 * a schermo, [MAIN], o del lavoro in sottofondo, [BACKGROUND]), il thread aiutante e i posti di
 * ricerca condivisi dal processo ([SearchSlots]), il registro e le misure.
 */
class RequestContext(val cancel: CancelToken = CancelToken.NEVER, val priority: Int = MAIN,
                     val executor: java.util.concurrent.ExecutorService? = null, val slots: SearchSlots? = null,
                     val log: ((String) -> Unit)? = null, val stats: RequestStats = RequestStats(), val useCache: Boolean = true) {
    /** Lo stesso contesto per una parte della richiesta, con le misure sue ([RequestStats.child]). */
    fun child(name: String): RequestContext = RequestContext(cancel, priority, executor, slots, log, stats.child(name), useCache)

    /** [block] (le ricerche di una richiesta) con un posto primario di [slots], se ci sono. */
    fun <T> searching(block: () -> T): T {
        val s = slots
        return if (s == null) block() else s.primary(priority, cancel, block)
    }

    companion object {
        /**
         * Nessun annullamento, nessun limite: un contesto nuovo a ogni lettura, così le misure di una
         * richiesta non si accumulano in un oggetto condiviso (e si scartano).
         */
        val NONE: RequestContext get() = RequestContext()
        const val MAIN = 0
        const val BACKGROUND = 1
    }
}

/**
 * I posti delle ricerche sul core dell'intero processo: al più [permits] insieme (sul tablet ogni
 * ricerca core tiene 20-80 MB). Un posto «primario» è quello di una richiesta ([primary]: si
 * aspetta); un «aiutante» è un thread in più della stessa richiesta ([tryHelper]: mai attesa). Il
 * lavoro [RequestContext.BACKGROUND] tiene al più [backgroundPrimary] posti primari e i suoi
 * aiutanti cedono il posto (PREEMPTED) a una richiesta [RequestContext.MAIN] che aspetta: così una
 * richiesta MAIN trova sempre un posto in breve.
 */
class SearchSlots(val permits: Int = 2, val backgroundPrimary: Int = 1) {
    init { require(permits >= 1 && backgroundPrimary in 1..permits) }
    private val lock = ReentrantLock()
    private val changed = lock.newCondition()
    private var used = 0
    private var backgroundPrimaries = 0
    private var mainWaiting = 0
    private val helpers = ArrayList<Helper>()
    /** Quanti posti tiene il thread corrente (primari o aiutante in [Helper.run]): le richieste annidate non ne prendono altri. */
    private val held = ThreadLocal<IntArray>()

    /** Per le prove e la diagnostica: i posti occupati ora. */
    val inUse: Int get() = lock.withLock { used }

    private fun depth(): IntArray = held.get() ?: IntArray(1).also { held.set(it) }

    /**
     * [block] con un posto primario: si aspetta un posto libero, controllando [cancel] ogni 50 ms
     * (un annullamento durante l'attesa lancia [SearchCancelledException]). Rientrante: un thread che
     * già tiene un posto (anche da aiutante) non ne prende un altro.
     */
    fun <T> primary(priority: Int, cancel: CancelToken, block: () -> T): T {
        val d = depth()
        if (d[0] > 0) { d[0]++; try { return block() } finally { d[0]-- } }
        acquirePrimary(priority, cancel)
        d[0]++
        try { return block() } finally {
            d[0]--
            lock.withLock {
                used--
                if (priority != RequestContext.MAIN) backgroundPrimaries--
                changed.signalAll()
            }
        }
    }

    private fun acquirePrimary(priority: Int, cancel: CancelToken) {
        val main = priority == RequestContext.MAIN
        lock.withLock {
            if (main) mainWaiting++
            try {
                while (true) {
                    cancel.check()
                    val free = used < permits
                    if (main && free) { used++; return }
                    if (!main && free && mainWaiting == 0 && backgroundPrimaries < backgroundPrimary) { used++; backgroundPrimaries++; return }
                    // una richiesta MAIN senza posto: gli aiutanti del lavoro in sottofondo lo lasciano
                    if (main) for (h in helpers) if (h.preemptible) h.token.cancel(SearchCancelledException.Reason.PREEMPTED)
                    changed.await(WAIT_MS, TimeUnit.MILLISECONDS)
                }
            } finally { if (main) mainWaiting-- }
        }
    }

    /**
     * Un posto da aiutante, se ce n'è uno libero e nessuna richiesta MAIN aspetta; null altrimenti
     * (senza attesa). Il suo [Helper.token] è figlio di [parent]; quello di un aiutante
     * [RequestContext.BACKGROUND] viene annullato con PREEMPTED quando una richiesta MAIN cerca un posto.
     */
    fun tryHelper(priority: Int, parent: CancelToken): Helper? = lock.withLock {
        if (used >= permits || mainWaiting > 0 || parent.isCancelled) return null
        used++
        Helper(parent.child(), priority != RequestContext.MAIN).also { helpers += it }
    }

    /** Un posto da aiutante: va chiuso ([close]) appena il thread ha finito. */
    inner class Helper internal constructor(val token: CancelToken, internal val preemptible: Boolean) : AutoCloseable {
        private var closed = false

        /** [block] sul thread aiutante, che conta come titolare del posto (le ricerche annidate non ne chiedono altri). */
        fun <T> run(block: () -> T): T {
            val d = depth()
            d[0]++
            try { return block() } finally { d[0]-- }
        }

        override fun close() {
            lock.withLock {
                if (closed) return
                closed = true
                helpers -= this
                used--
                changed.signalAll()
            }
        }
    }

    private companion object {
        /** Ogni quanto un'attesa controlla l'annullamento. */
        const val WAIT_MS = 50L
    }
}
