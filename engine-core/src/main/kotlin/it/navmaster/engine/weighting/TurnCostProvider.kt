package it.navmaster.engine.weighting

import it.navmaster.engine.storage.TurnCostStorage

/**
 * Costo delle svolte per un profilo (GraphHopper `DefaultTurnCostProvider`): +∞ per le svolte
 * vietate e per le inversioni a U quando [uTurnCostSeconds] è negativo (default: vietate).
 */
class TurnCostProvider(
    private val storage: TurnCostStorage,
    private val profileName: String,
    /** Costo di un'inversione a U in secondi; < 0 significa vietata (default ORS/GraphHopper). */
    private val uTurnCostSeconds: Double = -1.0,
) {
    val uTurnsAllowed get() = uTurnCostSeconds >= 0

    /** L'ultimo nodo chiesto e se ha divieti: una ricerca chiede la svolta per ogni arco che esce dallo stesso nodo (sul tablet ogni lettura è una chiamata nativa). */
    private var lastVia = -1
    private var lastHas = false

    fun turnWeight(inEdge: Int, viaNode: Int, outEdge: Int): Double {
        if (inEdge == outEdge) return if (uTurnsAllowed) uTurnCostSeconds else Double.POSITIVE_INFINITY
        if (viaNode != lastVia) { lastVia = viaNode; lastHas = storage.hasEntries(viaNode) }
        if (!lastHas) return 0.0
        return if (storage.encoding.isForbidden(storage.flags(inEdge, viaNode, outEdge), profileName)) Double.POSITIVE_INFINITY else 0.0
    }

    fun turnMillis(inEdge: Int, viaNode: Int, outEdge: Int): Long {
        val w = turnWeight(inEdge, viaNode, outEdge)
        return if (w.isInfinite()) Long.MAX_VALUE else (w * 1000).toLong()
    }
}
