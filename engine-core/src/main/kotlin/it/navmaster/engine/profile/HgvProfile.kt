package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import kotlin.math.min

/**
 * Profilo mezzi pesanti (ORS `HeavyVehicleFlagEncoder`, encoder `heavyvehicle`, profilo API `driving-hgv`).
 *
 * Differenze rispetto all'auto: velocità più basse (80 in autostrada, massimo 90), i tag
 * `maxspeed:hgv*` hanno la precedenza, i valori `hgv/goods/bus/agricultural/forestry/delivery`
 * aprono l'accesso, e la priorità favorisce le grandi arterie.
 */
class HgvProfile(options: VehicleProfileOptions = VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = true, blockFords = false)) : VehicleProfile(
    options,
    HgvSpeedTables.highway, HgvSpeedTables.surface, HgvSpeedTables.trackType, HgvSpeedTables.countryMaxSpeeds,
) {
    override val name = "heavyvehicle"
    override val category = ProfileCategory.DRIVING
    override val maxPossibleSpeed = 90
    override val meanSpeed = 70
    /**
     * Ordine effettivo in ORS: `AbstractFlagEncoder` aggiunge le chiavi del modo HGV (`hgv, motor_vehicle,
     * vehicle, access`) e `VehicleFlagEncoder` accoda quelle dell'auto. La prima chiave presente decide:
     * `hgv=yes` prevale su `motor_vehicle=no`, mentre `access=no` prevale su `motorcar=yes`.
     */
    override val restrictions = listOf("hgv", "motor_vehicle", "vehicle", "access", "motorcar")

    private val hgvAccess = listOf("hgv", "goods", "bus", "agricultural", "forestry", "delivery")

    // modello NAVMASTER: un autoarticolato da 40 t accelera e frena meno dell'auto, rotatorie e residenziali più piano
    override val acceleration = 0.5
    override val deceleration = 1.0
    override val roundaboutCap = 25.0
    override val roundaboutCapMultiLane = 30.0
    override val miniRoundaboutCap = 15.0
    override val residentialCap = 30.0

    init {
        intendedValues += listOf("designated", "agricultural", "forestry", "delivery", "bus", "hgv", "goods")
    }

    /** `maxspeed:hgv[:forward|:backward]` ha la precedenza; altrimenti il `maxspeed` generico, ma solo se è sotto la velocità di default del tipo di strada. */
    override fun maxSpeed(way: OsmWay): Double {
        val hgv = lowestOf(way, "maxspeed:hgv", "maxspeed:hgv:forward", "maxspeed:hgv:backward")
        if (!hgv.isNaN()) return hgv
        val generic = super.maxSpeed(way)
        if (generic.isNaN()) return generic
        val highway = highwayValue(way) ?: return generic
        val default = highwaySpeeds[highway] ?: return generic
        return if (default < generic) Double.NaN else generic
    }

    /**
     * Il cartello che vale per il camion (modello NAVMASTER), nel primo caso che vale:
     * `maxspeed:hgv[:forward|:backward]`; `maxspeed:hgv:conditional` per un mezzo pesante da 40 t con
     * rimorchio (`V @ (weight>N)` con N < 40, `V @ (trailer)`, `V @ (articulated)`; gli orari si
     * ignorano); il `maxspeed` generico (`XX:valore` vale il limite generale del Paese) — non più
     * scartato sopra il default del tipo di strada: lo limita la legge; il limite implicito della zona
     * se manca il `maxspeed`; altrimenti NaN.
     */
    fun hgvSigned(way: OsmWay, zone: ZoneInfo? = null): Double {
        val hgv = lowestSigned(way, "maxspeed:hgv", "maxspeed:hgv:forward", "maxspeed:hgv:backward")
        if (!hgv.isNaN()) return hgv
        val cond = conditionalHgvSpeed(way.tag("maxspeed:hgv:conditional"))
        if (!cond.isNaN()) return cond
        val generic = lowestSigned(way, "maxspeed", "maxspeed:forward", "maxspeed:backward")
        if (!generic.isNaN()) return generic
        if (!way.hasTag("maxspeed")) zone?.implicitKmh?.takeIf { it > 0 }?.let { return it.toDouble() }
        return Double.NaN
    }

    /**
     * Velocità di marcia del camion nel modello NAVMASTER: min(cartello × 0,9, limite legale del Paese per
     * il contesto × fattore), oppure min(default del tipo di strada, legale × fattore); poi il `tracktype`.
     */
    override fun navmasterSpeed(way: OsmWay, highway: String): Double {
        val z = ZoneInfo.of(way, highway)
        val signed = hgvSigned(way, z)
        val ctx = RoadContext.of(highway, z?.context, signed)
        val legal = HgvLegalSpeeds.limit(z?.country ?: options.country, ctx) * HgvLegalSpeeds.factor(ctx)
        val default = (highwaySpeeds[highway] ?: error("$name: nessuna velocità per highway=$highway")).toDouble()
        var v = if (!signed.isNaN()) min(signed * MAX_SPEED_FACTOR, legal) else min(default, legal)
        v = trackTypeSpeed(way, highway, v)
        return v
    }

    override fun access(way: OsmWay): Access {
        val highway = way.tag("highway")
        val restrictionValues = way.firstPriorityTagValues(restrictions)
        if (highway == null) {
            if (way.hasTag("route", ferries)) {
                for (v in restrictionValues) {
                    if (v in restrictedValues) return Access.CAN_SKIP
                    if (v in intendedValues) return Access.FERRY
                }
                if (restrictionValues.isEmpty() && !way.hasTag("foot") && !way.hasTag("bicycle")) return Access.FERRY
            }
            return Access.CAN_SKIP
        }
        if (highway == "track" && trackGradeLevel(way.tag("tracktype")) > options.maximumGradeLevel) return Access.CAN_SKIP
        if (!highwaySpeeds.containsKey(highwayValue(way))) return Access.CAN_SKIP
        if (way.hasTag("impassable", "yes") || way.hasTag("status", "impassable") || way.hasTag("smoothness", "impassable")) return Access.CAN_SKIP

        for (v in restrictionValues) {
            if (v.isEmpty()) continue
            // TODO restrizioni condizionali (access:conditional): per ora trattate come assolute
            if (v in restrictedValues) return Access.CAN_SKIP
            if (v in intendedValues) return Access.WAY
        }
        val carsAllowed = way.hasTag(restrictions, intendedValues)
        if (options.blockFords && (highway == "ford" || way.hasTag("ford")) && !carsAllowed) return Access.CAN_SKIP
        // accesso negato ai veicoli generici ma non ai tipi pesanti citati (es. hgv=yes)
        if (way.hasTag(restrictions, restrictedValues) && !carsAllowed && !way.hasTag(hgvAccess, intendedValues)) return Access.CAN_SKIP
        way.tag("maxwidth")?.toDoubleOrNull()?.let { if (it < 2.0) return Access.CAN_SKIP }
        return Access.WAY
    }

    override fun priority(way: OsmWay): PriorityCode {
        if (way.hasTag("hgv", "designated") ||
            (way.hasTag("access", "designated") && (way.hasTag("goods", "yes") || way.hasTag("hgv", "yes") ||
                way.hasTag("bus", "yes") || way.hasTag("agricultural", "yes") || way.hasTag("forestry", "yes")))
        ) return PriorityCode.BEST

        // (ordine, codice): vince la voce con ordine più alto
        val candidates = sortedMapOf<Double, PriorityCode>()
        val highway = highwayValue(way)
        val maxSpeed = maxSpeed(way)
        if (!highway.isNullOrEmpty()) {
            when (highway) {
                "motorway", "motorway_link", "motorroad", "trunk", "trunk_link" -> candidates[100.0] = PriorityCode.VERY_NICE
                "primary", "primary_link", "secondary", "secondary_link" -> candidates[100.0] = PriorityCode.PREFER
                "tertiary", "tertiary_link" -> candidates[100.0] = PriorityCode.UNCHANGED
                "residential", "service", "road", "unclassified" ->
                    if (!maxSpeed.isNaN() && maxSpeed <= 30) candidates[120.0] = PriorityCode.REACH_DEST
                    else candidates[100.0] = PriorityCode.AVOID_IF_POSSIBLE
                "living_street" -> candidates[100.0] = PriorityCode.AVOID_AT_ALL_COSTS
                "track" -> candidates[100.0] = PriorityCode.WORST
                else -> candidates[40.0] = PriorityCode.AVOID_IF_POSSIBLE
            }
        } else {
            candidates[100.0] = PriorityCode.UNCHANGED
        }
        if (!maxSpeed.isNaN()) {
            // un limite basso indica un centro abitato
            if (maxSpeed <= 40) candidates[110.0] = PriorityCode.AVOID_IF_POSSIBLE
            else if (maxSpeed <= 50) candidates[110.0] = PriorityCode.UNCHANGED
        }
        return candidates[candidates.lastKey()]!!
    }

    companion object {
        private val CONDITION = Regex("""^\s*([^@]+?)\s*@\s*\(?\s*(.+?)\s*\)?\s*$""")
        private val WEIGHT = Regex("""^(weight|weightrating|maxweight)\s*>=?\s*([0-9]+(?:\.[0-9]+)?)\s*t?$""")

        /** Il limite più basso di `maxspeed:hgv:conditional` che vale per un autoarticolato da 40 t, NaN se nessuno. */
        fun conditionalHgvSpeed(raw: String?): Double {
            if (raw.isNullOrBlank()) return Double.NaN
            var best = Double.NaN
            for (part in raw.split(';')) {
                val m = CONDITION.find(part) ?: continue
                val v = ZoneInfo.parseSigned(m.groupValues[1])
                if (v.isNaN()) continue
                val conditions = m.groupValues[2].lowercase().split(" and ").map { it.trim() }
                val applies = conditions.all { c ->
                    when {
                        c == "trailer" || c == "articulated" -> true
                        else -> WEIGHT.find(c)?.groupValues?.get(2)?.toDoubleOrNull()?.let { it < 40.0 } ?: false
                    }
                }
                if (applies && (best.isNaN() || v < best)) best = v
            }
            return best
        }
    }

    /** Info di destinazione sulle rampe (`destination=A;B`), come ORS `getWayInfo`. */
    fun wayInfo(way: OsmWay): String {
        if (!way.hasTag("highway", "motorway_link")) return ""
        val parts = way.tag("destination")?.split(';')?.map { it.trim() }?.filter { it.isNotEmpty() } ?: return ""
        if (parts.isEmpty()) return ""
        return (if (parts.size > 1) "destinations: " else "destination: ") + parts.joinToString(", ")
    }

}
