package it.navmaster.engine.routing

/**
 * Una ricerca fermata a metà: annullata da chi l'ha chiesta ([Reason.CANCELLED]), oltre la sua
 * scadenza ([Reason.DEADLINE]) o lasciata a una richiesta più urgente ([Reason.PREEMPTED]). Senza
 * traccia dello stack: è un'uscita normale, non un errore.
 */
class SearchCancelledException(val reason: Reason) : RuntimeException("calcolo annullato: $reason", null, false, false) {
    enum class Reason { CANCELLED, DEADLINE, PREEMPTED }
}

/**
 * Il segnale di annullamento di un calcolo, controllato dai cicli delle ricerche ogni 256 passi
 * ([check]). Niente `Thread.interrupt`: le letture dei file mappati non lo guardano, e interrompere
 * un `channel.map` chiuderebbe il file. Un token figlio ([child]) si ferma anche col genitore e può
 * avere una scadenza sua (le varianti dentro una richiesta, un aiutante prelazionabile).
 */
open class CancelToken(private val deadlineNanos: Long = Long.MAX_VALUE, private val parent: CancelToken? = null) {
    @Volatile private var reason: SearchCancelledException.Reason? = null

    /** Ferma il calcolo alla prossima verifica; il primo motivo resta. */
    open fun cancel(why: SearchCancelledException.Reason = SearchCancelledException.Reason.CANCELLED) { if (reason == null) reason = why }

    val isCancelled: Boolean get() = reason != null || parent?.isCancelled == true || System.nanoTime() > deadlineNanos

    /**
     * Lancia [SearchCancelledException] se il calcolo va fermato: prima il motivo proprio, poi quello del genitore, poi la scadenza.
     * Aperta per le prove (un token che si annulla dopo un certo numero di verifiche).
     */
    open fun check() {
        reason?.let { throw SearchCancelledException(it) }
        parent?.check()
        if (deadlineNanos != Long.MAX_VALUE && System.nanoTime() > deadlineNanos) throw SearchCancelledException(SearchCancelledException.Reason.DEADLINE)
    }

    /** Un token che si ferma con questo e, in più, alla scadenza [deadlineNanos] (`System.nanoTime`). */
    fun child(deadlineNanos: Long = Long.MAX_VALUE): CancelToken = CancelToken(deadlineNanos, this)

    companion object {
        /** Mai annullato: il valore predefinito, con un costo di una lettura per verifica. */
        @JvmField val NEVER: CancelToken = object : CancelToken() { override fun cancel(why: SearchCancelledException.Reason) {} }
    }
}
