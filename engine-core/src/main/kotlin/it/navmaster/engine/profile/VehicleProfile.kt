package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.roundToInt

/** Risultato dell'analisi di una way per un profilo: accesso per direzione e velocità. */
data class WaySpeedAccess(
    val access: Access,
    val forward: Boolean,
    val backward: Boolean,
    /** km/h, già limitata a [minSpeed, maxSpeed] del profilo. */
    val speedKmh: Double,
    /** Fattore di priorità [PriorityCode.factor], usato dal peso "recommended". */
    val priority: Double,
)

/** Opzioni di costruzione del profilo (`encoder_options` di ORS). */
data class VehicleProfileOptions(
    val blockFords: Boolean = true,
    val blockBarriers: Boolean = true,
    val useAcceleration: Boolean = false,
    val maximumGradeLevel: Int = 3,
    /** Passo della velocità memorizzata (km/h); è anche la velocità minima. */
    val speedFactor: Double = 5.0,
)

/**
 * Logica comune ai profili a motore (ORS `VehicleFlagEncoder`): accesso, senso unico,
 * velocità da tipo di strada, limiti, superficie, rotatorie, accelerazione.
 */
abstract class VehicleProfile(
    val options: VehicleProfileOptions,
    protected val highwaySpeeds: Map<String, Int>,
    protected val surfaceSpeeds: Map<String, Int>,
    protected val trackTypeSpeeds: Map<String, Int>,
    protected val countryMaxSpeeds: Map<String, Int>,
) {
    abstract val name: String
    abstract val category: ProfileCategory
    abstract val maxPossibleSpeed: Int
    /** Velocità media usata per il reach factor delle isocrone. */
    abstract val meanSpeed: Int

    /** Chiavi di accesso in ordine di specificità. */
    abstract val restrictions: List<String>
    protected val restrictedValues: MutableSet<String> = mutableSetOf("private", "no", "restricted", "military")
    protected val intendedValues: MutableSet<String> = mutableSetOf("yes", "permissive", "destination", "permit")
    protected val ferries = setOf("shuttle_train", "ferry")
    protected val oneways = setOf("yes", "true", "1", "-1")
    protected val passByDefaultBarriers: MutableSet<String> = mutableSetOf("gate", "lift_gate", "kissing_gate", "swing_gate")
    protected val blockByDefaultBarriers: MutableSet<String> =
        mutableSetOf("bollard", "stile", "turnstile", "cycle_barrier", "motorcycle_barrier", "block")

    val minSpeed: Double get() = options.speedFactor

    abstract fun access(way: OsmWay): Access

    /** Priorità della way per questo profilo; default neutro. */
    open fun priority(way: OsmWay): PriorityCode = PriorityCode.UNCHANGED

    /**
     * Calcola accesso per direzione, velocità e priorità.
     * @param estimatedDistance lunghezza stimata della way in metri (per accelerazione e penalità residenziale), o NaN
     * @param ferryDurationSeconds valore di `duration` già convertito, o -1
     */
    fun analyse(way: OsmWay, access: Access, estimatedDistance: Double = Double.NaN, ferryDurationSeconds: Long = -1): WaySpeedAccess? {
        if (access.canSkip) return null
        if (access.isFerry) {
            val speed = ferrySpeed(way, estimatedDistance, ferryDurationSeconds)
            return WaySpeedAccess(access, forward = true, backward = true, speedKmh = clampSpeed(speed), priority = priority(way).factor)
        }
        val highway = highwayValue(way) ?: return null
        var speed = baseSpeed(way, highway).toDouble()
        speed = applyMaxSpeed(way, speed)
        speed = applySurface(way, speed)
        if (!estimatedDistance.isNaN()) {
            if (highway == "residential") speed = residentialPenalty(speed, way, estimatedDistance)
            else if (options.useAcceleration && highway != "motorway" && highway != "motorroad") {
                speed = adjustSpeedForAcceleration(estimatedDistance, speed)
            }
        }
        val roundabout = way.hasTag("junction", "roundabout")
        if (roundabout) {
            if (way.hasTag("highway", "mini_roundabout")) speed = min(speed, 25.0)
            way.tag("lanes")?.toIntOrNull()?.let { lanes -> speed = min(speed, if (lanes >= 2) 40.0 else 35.0) }
        }
        val fwd: Boolean
        val bwd: Boolean
        if (isOneway(way) || roundabout) {
            fwd = isForwardOneway(way)
            bwd = isBackwardOneway(way)
        } else {
            fwd = true; bwd = true
        }
        return WaySpeedAccess(access, fwd, bwd, clampSpeed(speed), priority(way).factor)
    }

    /** `highway=*`, ma `motorroad=yes` su strade non autostradali viene trattato come "motorroad". */
    fun highwayValue(way: OsmWay): String? {
        val h = way.tag("highway") ?: return null
        if (h.isEmpty()) return null
        if (way.hasTag("motorroad", "yes") && h != "motorway" && h != "motorway_link") return "motorroad"
        return h
    }

    /** `maxspeed` (anche per direzione): il più basso tra i valori presenti. */
    open fun maxSpeed(way: OsmWay): Double = lowestOf(way, "maxspeed", "maxspeed:forward", "maxspeed:backward")

    protected fun lowestOf(way: OsmWay, vararg keys: String): Double {
        var result = Double.NaN
        for (k in keys) {
            val v = OsmValues.parseKmh(way.tag(k))
            if (!v.isNaN() && (result.isNaN() || v < result)) result = v
        }
        return result
    }

    /** Velocità del tipo di strada, sovrascritta dal limite esplicito o di zona, poi dal tracktype. */
    protected fun baseSpeed(way: OsmWay, highway: String): Int {
        var speed = highwaySpeeds[highway] ?: error("$name: nessuna velocità per highway=$highway")
        var limit = maxSpeed(way).let { if (it.isNaN()) 0 else it.roundToInt() }
        if (limit <= 0) limit = zoneMaxSpeed(way)
        if (limit > 0) speed = limit
        if (highway == "track") {
            way.tag("tracktype")?.let { tt -> trackTypeSpeeds[tt]?.takeIf { it != -1 }?.let { speed = it } }
        }
        return speed
    }

    /** Limite implicito da `zone:maxspeed` / `zone:traffic` (es. `IT:urban`), solo se manca `maxspeed`. */
    protected fun zoneMaxSpeed(way: OsmWay): Int {
        if (way.hasTag("maxspeed")) return -1
        val key = way.tag("zone:maxspeed") ?: way.tag("zone:traffic") ?: return -1
        return countryMaxSpeeds[key.lowercase()] ?: -1
    }

    /** Con un limite esplicito la velocità di percorrenza è il 90% del limite. */
    protected fun applyMaxSpeed(way: OsmWay, speed: Double): Double {
        val ms = maxSpeed(way)
        return if (!ms.isNaN()) ms * MAX_SPEED_FACTOR else speed
    }

    protected fun applySurface(way: OsmWay, speed: Double): Double {
        val surface = way.tag("surface") ?: return speed
        val s = surfaceSpeeds[surface] ?: -1
        return if (s != -1 && speed > s) s.toDouble() else speed
    }

    /** Strade residenziali con nodi molto ravvicinati (<100 m): velocità dimezzata. */
    protected fun residentialPenalty(speed: Double, way: OsmWay, estimatedDistance: Double): Double {
        if (speed == 0.0) return 0.0
        val interimNodes = way.nodeIds.size - 2
        val interimDistance = if (interimNodes > 0) estimatedDistance / (interimNodes + 1) else estimatedDistance
        return if (interimDistance < 100) speed * 0.5 else speed
    }

    protected fun clampSpeed(speed: Double): Double {
        require(speed >= 0 && !speed.isNaN()) { "velocità non valida: $speed" }
        return speed.coerceIn(minSpeed, maxPossibleSpeed.toDouble())
    }

    // --- senso unico -------------------------------------------------------------------

    protected fun isOneway(way: OsmWay): Boolean =
        way.hasTag("oneway", oneways) || way.hasTag("vehicle:backward") || way.hasTag("vehicle:forward") ||
            way.hasTag("motor_vehicle:backward") || way.hasTag("motor_vehicle:forward")

    protected fun isBackwardOneway(way: OsmWay): Boolean =
        way.hasTag("oneway", "-1") || way.hasTag("vehicle:forward", "no") || way.hasTag("motor_vehicle:forward", "no")

    protected fun isForwardOneway(way: OsmWay): Boolean = !isBackwardOneway(way)

    // --- barriere sui nodi ---------------------------------------------------------------

    /** Vero se il nodo blocca il passaggio (ORS `ORSAbstractFlagEncoder.isBarrier`). */
    fun isBarrier(nodeTags: Map<String, String>): Boolean {
        val barrier = nodeTags["barrier"]
        val blockByDefault = barrier in blockByDefaultBarriers
        if (blockByDefault || barrier in passByDefaultBarriers) {
            val locked = nodeTags["locked"] == "yes"
            for (res in restrictions) {
                val v = nodeTags[res] ?: continue
                if (!locked && v in intendedValues) return false
                if (v in restrictedValues) return true
            }
            return blockByDefault || options.blockBarriers
        }
        if (nodeTags["highway"] == "ford" || nodeTags["ford"] == "yes") {
            val intended = restrictions.any { nodeTags[it] in intendedValues }
            val restricted = restrictions.any { nodeTags[it] in restrictedValues }
            if (options.blockFords && !intended || restricted) return true
        }
        return false
    }

    // --- traghetti ------------------------------------------------------------------------

    /** GraphHopper `FerrySpeedCalculator(speedFactor, maxSpeed, long=30, short=20, unknown=5)`. */
    protected fun ferrySpeed(way: OsmWay, estimatedDistance: Double, durationSeconds: Long): Double {
        val sf = options.speedFactor
        var hours = if (durationSeconds > 0) durationSeconds / 3600.0 else 0.0
        if (hours > 0 && !estimatedDistance.isNaN()) {
            val tripSpeed = estimatedDistance / 1000 / hours / 1.4 // attese comprese
            if (tripSpeed > 0.01) {
                if (tripSpeed > maxPossibleSpeed) return maxPossibleSpeed.toDouble()
                if (Math.round(tripSpeed) < sf / 2) return sf / 2
                return Math.round(tripSpeed).toDouble()
            }
            hours = 0.0 // durata inverosimile: ignorata
        }
        return when {
            hours == 0.0 -> {
                if (!estimatedDistance.isNaN() && estimatedDistance <= 300) return sf / 2
                way.tag("maxspeed")?.let { ms ->
                    val avg = OsmValues.parseKmh(ms) / 1.4
                    if (avg < sf / 2) return sf / 2
                    if (avg > maxPossibleSpeed) return maxPossibleSpeed.toDouble()
                    if (avg.isFinite()) return avg
                }
                5.0
            }
            hours > 1 -> 30.0
            else -> 20.0
        }
    }

    // --- modello di accelerazione (ORS `use_acceleration`) --------------------------------

    private var accelerationModifier = 0.0

    private fun secondsTo100Kmh() = AVERAGE_SECS_TO_100_KMPH + accelerationModifier * AVERAGE_SECS_TO_100_KMPH
    private fun accelerationKmhPerS() = 100.0 / secondsTo100Kmh()

    /** Velocità media su un tratto tenendo conto di accelerazione e decelerazione agli estremi. */
    fun adjustSpeedForAcceleration(distance: Double, maxSpeedKmh: Double): Double {
        if (maxSpeedKmh >= ACCELERATION_SPEED_CUTOFF_MAX) return maxSpeedKmh
        if (distance <= 0) return maxSpeedKmh
        var normalised = max(maxSpeedKmh, ACCELERATION_SPEED_CUTOFF_MIN)
        normalised = (normalised - ACCELERATION_SPEED_CUTOFF_MIN) / (ACCELERATION_SPEED_CUTOFF_MAX - ACCELERATION_SPEED_CUTOFF_MIN)
        accelerationModifier = 0.01.pow(normalised)
        val timeToMax = maxSpeedKmh / accelerationKmhPerS()
        val accelerationDistance = distanceTravelledInDuration(maxSpeedKmh, timeToMax)
        val distanceAtMax = distance - accelerationDistance * 2
        return if (distanceAtMax < 0) {
            var duration = durationToTravelDistance(maxSpeedKmh, distance / 2)
            if (duration == 0.0) duration = 1.0
            mpsToKmh(distance / (duration * 2))
        } else {
            val timeAtMax = distanceAtMax / kmhToMps(maxSpeedKmh)
            mpsToKmh(distance / (timeToMax * 2 + timeAtMax))
        }
    }

    private fun durationToTravelDistance(maxSpeedKmh: Double, distanceM: Double): Double {
        var seconds = 0.0
        var travelled = 0.0
        var speed = 0.0
        while (speed < maxSpeedKmh && travelled < distanceM) {
            speed += accelerationKmhPerS()
            seconds += 1
            travelled += kmhToMps(speed)
        }
        val remaining = distanceM - travelled
        if (remaining > 0) seconds += remaining / kmhToMps(maxSpeedKmh)
        return seconds
    }

    private fun distanceTravelledInDuration(maxSpeedKmh: Double, durationS: Double): Double {
        var seconds = 0.0
        var travelled = 0.0
        var speed = 0.0
        while (seconds < durationS) {
            speed = min(speed + accelerationKmhPerS(), maxSpeedKmh)
            travelled += kmhToMps(speed)
            seconds += 1
        }
        return travelled
    }

    private fun kmhToMps(kmh: Double) = kmh / 3.6
    private fun mpsToKmh(mps: Double) = mps * 3.6

    companion object {
        const val MAX_SPEED_FACTOR = 0.9
        const val AVERAGE_SECS_TO_100_KMPH = 10
        private const val ACCELERATION_SPEED_CUTOFF_MAX = 80.0
        private const val ACCELERATION_SPEED_CUTOFF_MIN = 20.0

        /** `tracktype=grade1..5`; valori multipli ("grade2;grade3", "grade1-grade3") danno il peggiore; sconosciuto = 0. */
        fun trackGradeLevel(grade: String?): Int {
            if (grade == null || grade == "unknown") return 0
            var maxGrade = 0
            for (part in grade.split(';', '-')) {
                val n = part.replace("grade", "").trim().toIntOrNull() ?: return 0
                if (n > maxGrade) maxGrade = n
            }
            return minOf(5, maxGrade)
        }
    }
}
