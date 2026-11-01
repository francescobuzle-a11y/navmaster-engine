package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay

/**
 * Estrae da una way gli [WayAttributes] indipendenti dal profilo.
 * Ogni funzione riproduce il corrispondente TagParser di GraphHopper/ORS.
 */
object WayAttributeParser {
    private val motorVehicleRestrictions = listOf("motorcar", "motor_vehicle", "vehicle", "access")
    private val motorVehicleRestrictedValues = setOf("private", "no", "restricted", "military")
    private val heavyVehicleValues = setOf("hgv", "goods", "bus", "agricultural", "forestry", "delivery")
    private val noValues = setOf("no", "private")
    private val yesValues = setOf("yes", "designated")
    private val hazmatKey = Regex("^hazmat(:[B-E])?$")
    private val hgvTollTags = listOf("toll:hgv", "toll:N2", "toll:N3")
    private val heightTags = listOf("maxheight", "maxheight:physical")
    private val widthTags = listOf("maxwidth", "maxwidth:physical", "width")
    private val widthTagsStrict = listOf("maxwidth", "maxwidth:physical")
    private val weightTags = listOf("maxweight", "maxgcweight")

    /** Chiavi di accesso in ordine di specificità per ciascun modo di trasporto (`OSMRoadAccessParser.toOSMRestrictions`). */
    val carRestrictions = listOf("motorcar", "motor_vehicle", "vehicle", "access")
    val hgvRestrictions = listOf("hgv", "motor_vehicle", "vehicle", "access")

    /**
     * @param widthFromCarriageway come GraphHopper/ORS, usa anche il tag `width` (larghezza della carreggiata)
     *   come limite di larghezza; con `false` contano solo `maxwidth` e `maxwidth:physical`.
     */
    fun parse(way: OsmWay, isFerry: Boolean, roadAccessRestrictions: List<String> = carRestrictions, widthFromCarriageway: Boolean = true): WayAttributes = WayAttributes(
        osmWayId = way.id,
        blockedHeavyVehicleTypes = blockedHeavyVehicleTypes(way),
        hazmatForbidden = hazmatForbidden(way),
        maxHeight = meter(way, heightTags),
        maxWidth = meter(way, if (widthFromCarriageway) widthTags else widthTagsStrict),
        maxLength = meter(way, listOf("maxlength")),
        maxWeight = tons(way, weightTags),
        maxAxleLoad = tons(way, listOf("maxaxleload")),
        toll = toll(way),
        roadAccess = roadAccess(way, roadAccessRestrictions),
        roadClass = if (isFerry) RoadClass.OTHER else roadClass(way),
        isLink = !isFerry && (way.tag("highway")?.endsWith("_link") ?: false),
        wayType = wayType(way),
        waySurface = waySurface(way.tag("surface")),
        isMotorway = way.hasTag("highway", "motorway", "motorway_link"),
        isFord = way.hasTag("ford", "yes"),
        isRoundabout = way.hasTag("junction", "roundabout", "circular"),
    )

    /**
     * Tipi di veicolo pesante bloccati (ORS `VehicleAccessParser`).
     * Prima le restrizioni generiche (`access=no` blocca tutti, salvo i tipi citati esplicitamente),
     * poi i tag specifici: `hgv=no`, `hgv=delivery` (consente delivery e blocca gli altri hgv), ecc.
     */
    fun blockedHeavyVehicleTypes(way: OsmWay): Int {
        if (!way.hasTag("highway")) return 0
        var blocked = 0
        if (way.hasTag(motorVehicleRestrictions, motorVehicleRestrictedValues)) blocked = HeavyVehicleAttributes.ANY
        if (way.hasTag(motorVehicleRestrictions, heavyVehicleValues)) {
            var allowed = 0
            for (key in motorVehicleRestrictions) for (v in way.tagValues(key)) {
                if (v in heavyVehicleValues) allowed = allowed or HeavyVehicleAttributes.fromString(v)
            }
            blocked = HeavyVehicleAttributes.ANY and allowed.inv()
        }
        for ((key, value) in way.tags) {
            if (key !in heavyVehicleValues) continue
            val vehicleType = if (value in heavyVehicleValues) value else key
            val access = when {
                vehicleType == value || value in yesValues -> "yes"
                value in noValues -> "no"
                else -> null
            }
            blocked = setAccess(blocked, vehicleType, access)
            // es. hgv=delivery: i mezzi pesanti diversi dalle consegne sono bloccati
            if (vehicleType == value) blocked = setAccess(blocked, key, "no")
        }
        return blocked
    }

    private fun setAccess(blocked: Int, vehicle: String, access: String?): Int {
        val flag = HeavyVehicleAttributes.fromString(vehicle)
        return when (access) {
            "no" -> blocked or flag
            "yes" -> blocked and flag.inv()
            else -> blocked
        }
    }

    fun hazmatForbidden(way: OsmWay): Boolean =
        way.tags.any { (k, v) -> hazmatKey.matches(k) && v == "no" }

    private fun meter(way: OsmWay, keys: List<String>): Double {
        val v = OsmValues.parseMeter(way.firstPriorityTag(keys))
        return if (v.isNaN()) Double.POSITIVE_INFINITY else v
    }

    private fun tons(way: OsmWay, keys: List<String>): Double {
        val v = OsmValues.parseTons(way.firstPriorityTag(keys))
        return if (v.isNaN()) Double.POSITIVE_INFINITY else v
    }

    fun toll(way: OsmWay): Toll = when {
        way.hasTag("toll", "yes") -> Toll.ALL
        way.hasTag(hgvTollTags, listOf("yes")) -> Toll.HGV
        way.hasTag("toll", "no") -> Toll.NO
        else -> Toll.MISSING
    }

    /** Il valore più restrittivo tra le chiavi di accesso (ordinale maggiore = più restrittivo). */
    fun roadAccess(way: OsmWay, restrictions: List<String>): RoadAccess {
        var result = RoadAccess.YES
        for (key in restrictions) {
            val v = RoadAccess.find(way.tag(key, "yes")) ?: continue
            if (v.ordinal > result.ordinal) result = v
        }
        return result
    }

    fun roadClass(way: OsmWay): RoadClass {
        val highway = way.tag("highway") ?: return RoadClass.OTHER
        var rc = RoadClass.find(highway)
        if (rc == RoadClass.OTHER && highway.endsWith("_link")) rc = RoadClass.find(highway.dropLast(5))
        return rc
    }

    fun wayType(way: OsmWay): WayType {
        if (way.hasTag("route", "ferry", "shuttle_train")) return WayType.FERRY
        return wayTypeOf(way.tag("highway", ""))
    }

    fun wayTypeOf(highway: String): WayType = when (highway.lowercase()) {
        "primary", "primary_link", "motorway", "motorway_link", "trunk", "trunk_link" -> WayType.STATE_ROAD
        "secondary", "secondary_link", "tertiary", "tertiary_link", "road", "unclassified" -> WayType.ROAD
        "residential", "service", "living_street" -> WayType.STREET
        "path" -> WayType.PATH
        "track" -> WayType.TRACK
        "cycleway" -> WayType.CYCLEWAY
        "footway", "pedestrian", "crossing" -> WayType.FOOTWAY
        "steps" -> WayType.STEPS
        "construction" -> WayType.CONSTRUCTION
        else -> WayType.UNKNOWN
    }

    fun waySurface(raw: String?): WaySurface {
        var surface = raw ?: return WaySurface.UNKNOWN
        if (';' in surface) surface = surface.substringBefore(';')
        if (':' in surface) surface = surface.substringBefore(':')
        return when (surface.lowercase()) {
            "paved" -> WaySurface.PAVED
            "unpaved", "woodchips", "rock", "rocks", "stone", "shells", "salt" -> WaySurface.UNPAVED
            "asphalt", "chipseal", "bitmac", "tarmac" -> WaySurface.ASPHALT
            "concrete", "cement" -> WaySurface.CONCRETE
            "paving_stones", "paved_stones", "sett", "cobblestone", "unhewn_cobblestone", "bricks", "brick" -> WaySurface.PAVING_STONES
            "metal" -> WaySurface.METAL
            "wood" -> WaySurface.WOOD
            "compacted", "pebblestone" -> WaySurface.COMPACTED_GRAVEL
            "gravel", "fine_gravel" -> WaySurface.GRAVEL
            "dirt", "earth", "soil" -> WaySurface.DIRT
            "ground", "mud" -> WaySurface.GROUND
            "ice", "snow" -> WaySurface.ICE
            "sand" -> WaySurface.SAND
            "grass" -> WaySurface.GRASS
            "grass_paver" -> WaySurface.GRASS_PAVER
            else -> WaySurface.UNKNOWN
        }
    }
}
