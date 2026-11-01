package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay

/** Profilo automobile (ORS `CarFlagEncoder`, encoder `car-ors`, profilo API `driving-car`). */
class CarProfile(options: VehicleProfileOptions = VehicleProfileOptions()) : VehicleProfile(
    options,
    CarSpeedTables.highway, CarSpeedTables.surface, CarSpeedTables.trackType, CarSpeedTables.countryMaxSpeeds,
) {
    override val name = "car-ors"
    override val category = ProfileCategory.DRIVING
    override val maxPossibleSpeed = 140
    override val meanSpeed = 100
    override val restrictions = listOf("motorcar", "motor_vehicle", "vehicle", "access")

    init {
        restrictedValues += listOf("agricultural", "forestry", "delivery", "emergency")
        blockByDefaultBarriers += listOf("bus_trap", "sump_buster")
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
        if (highway == "track") {
            val tt = way.tag("tracktype")
            if (tt != null && trackGradeLevel(tt) > options.maximumGradeLevel) return Access.CAN_SKIP
        }
        if (!highwaySpeeds.containsKey(highwayValue(way))) return Access.CAN_SKIP
        if (way.hasTag("impassable", "yes") || way.hasTag("status", "impassable") || way.hasTag("smoothness", "impassable")) return Access.CAN_SKIP
        for (v in restrictionValues) {
            if (v.isEmpty()) continue
            // TODO restrizioni condizionali: per ora assolute
            if (v in restrictedValues) return Access.CAN_SKIP
            if (v in intendedValues) return Access.WAY
        }
        if (options.blockFords && (highway == "ford" || way.hasTag("ford"))) return Access.CAN_SKIP
        way.tag("maxwidth")?.toDoubleOrNull()?.let { if (it < 2.0) return Access.CAN_SKIP }
        return Access.WAY
    }
}
