package it.navmaster.engine.weighting

import it.navmaster.engine.profile.PriorityCode
import it.navmaster.engine.profile.RoadAccess
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.ProfileEncoding
import kotlin.math.pow

/**
 * Costo di percorrenza di un arco. [reverse] = true calcola il costo percorrendo l'arco
 * nella direzione opposta a quella dell'iteratore (ricerca all'indietro).
 * +∞ significa non percorribile.
 */
interface Weighting {
    val name: String
    /** Limite inferiore del peso per una distanza in linea d'aria (euristica A*). */
    fun minWeight(distanceMeters: Double): Double
    fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double
    fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean): Long
}

/**
 * Tempo di percorrenza in secondi (ORS `ORSFastestWeighting`, senza penalità di direzione).
 * [maximumSpeed] riproduce `MaximumSpeedCalculator`: le velocità superiori vengono ridotte a quel valore.
 */
class FastestWeighting(private val p: ProfileEncoding, private val maximumSpeed: Double = Double.POSITIVE_INFINITY) : Weighting {
    override val name = "fastest"
    private val maxSpeed = minOf(p.profile.maxPossibleSpeed.toDouble(), maximumSpeed)

    override fun minWeight(distanceMeters: Double) = distanceMeters / maxSpeed * SPEED_CONV

    private fun speed(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
        val s = if (reverse) edge.getReverse(p.speed) else edge.get(p.speed)
        return if (s > maximumSpeed) maximumSpeed else s
    }

    override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
        val access = if (reverse) edge.getReverse(p.access) else edge.get(p.access)
        if (!access) return Double.POSITIVE_INFINITY
        val speed = speed(edge, reverse)
        if (speed == 0.0) return Double.POSITIVE_INFINITY
        return edge.distance / speed * SPEED_CONV
    }

    override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean): Long {
        val speed = speed(edge, reverse)
        if (speed == 0.0) return Long.MAX_VALUE
        return (edge.distance / speed * 3600).toLong()
    }

    companion object { const val SPEED_CONV = 3.6 }
}

/** Distanza in metri. */
class ShortestWeighting(private val p: ProfileEncoding) : Weighting {
    override val name = "shortest"
    private val fastest = FastestWeighting(p)
    override fun minWeight(distanceMeters: Double) = distanceMeters
    override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
        val access = if (reverse) edge.getReverse(p.access) else edge.get(p.access)
        return if (access) edge.distance else Double.POSITIVE_INFINITY
    }
    override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean) = fastest.edgeMillis(edge, reverse)
}

/**
 * "recommended" (ORS `ORSPriorityWeighting`): il peso del [base] viene moltiplicato per
 * 2^((priorità·7 − 4) / (4 − 7)), così le strade preferite costano meno (scala di ORS 10).
 * Come in ORS, `minWeight` è 0: l'euristica A* non è informativa con questo peso.
 */
class PriorityWeighting(private val p: ProfileEncoding, private val base: Weighting) : Weighting {
    override val name = "recommended"
    override fun minWeight(distanceMeters: Double) = 0.0
    override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
        val w = base.edgeWeight(edge, reverse)
        if (w.isInfinite()) return w
        val priority = edge.get(p.priority)
        val normalized = priority * BEST - UNCHANGED
        return w * 2.0.pow(normalized / (UNCHANGED - BEST))
    }
    override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean) = base.edgeMillis(edge, reverse)

    companion object {
        private val BEST = PriorityCode.BEST.value.toDouble()
        private val UNCHANGED = PriorityCode.UNCHANGED.value.toDouble()
    }
}

/**
 * Penalità per strade ad accesso limitato (ORS `LimitedAccessWeighting`): per i veicoli a motore
 * ×10 su `destination` e `private`, ×1.5 su `customers`, ×1.2 su `highway=service`.
 */
class LimitedAccessWeighting(
    private val shared: GraphEncoding,
    private val base: Weighting,
    private val destinationFactor: Double = 10.0,
    private val privateFactor: Double = 10.0,
    private val customersFactor: Double = 1.5,
    private val serviceFactor: Double = 1.2,
) : Weighting {
    override val name get() = base.name

    init {
        for (f in listOf(destinationFactor, privateFactor, customersFactor, serviceFactor)) require(f in 1.0..10.0) { "fattore fuori da [1,10]: $f" }
    }

    override fun minWeight(distanceMeters: Double) = base.minWeight(distanceMeters)

    override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
        var w = base.edgeWeight(edge, reverse)
        if (w.isInfinite()) return w
        when (edge.get(shared.shared.roadAccess)) {
            RoadAccess.DESTINATION -> w *= destinationFactor
            RoadAccess.PRIVATE -> w *= privateFactor
            RoadAccess.CUSTOMERS -> w *= customersFactor
            else -> {}
        }
        if (edge.get(shared.shared.roadClass) == RoadClass.SERVICE) w *= serviceFactor
        return w
    }

    override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean) = base.edgeMillis(edge, reverse)
}

enum class WeightingMethod { FASTEST, SHORTEST, RECOMMENDED }

object WeightingFactory {
    /**
     * Costruisce la catena di pesi come ORS: per i profili auto "recommended" equivale a "fastest";
     * per il camion "fastest" diventa "recommended". Sempre avvolto in [LimitedAccessWeighting].
     */
    fun create(encoding: GraphEncoding, profile: ProfileEncoding, method: WeightingMethod, maximumSpeed: Double = Double.POSITIVE_INFINITY): Weighting {
        val effective = when (profile.profile.name) {
            "car-ors" -> if (method == WeightingMethod.RECOMMENDED) WeightingMethod.FASTEST else method
            else -> if (method == WeightingMethod.FASTEST) WeightingMethod.RECOMMENDED else method
        }
        val base: Weighting = when (effective) {
            WeightingMethod.FASTEST -> FastestWeighting(profile, maximumSpeed)
            WeightingMethod.SHORTEST -> ShortestWeighting(profile)
            WeightingMethod.RECOMMENDED -> PriorityWeighting(profile, FastestWeighting(profile, maximumSpeed))
        }
        return LimitedAccessWeighting(encoding, base)
    }
}
