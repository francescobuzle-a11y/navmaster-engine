package it.navmaster.engine.profile

/** Accesso al veicolo su una way (GraphHopper `EncodingManager.Access`). */
enum class Access {
    /** La way non è percorribile da questo profilo: non diventa un arco. */
    CAN_SKIP,
    WAY,
    FERRY;

    val canSkip: Boolean get() = this == CAN_SKIP
    val isFerry: Boolean get() = this == FERRY
}

enum class RoadAccess(val osm: String) {
    YES("yes"), DESTINATION("destination"), CUSTOMERS("customers"), DELIVERY("delivery"),
    FORESTRY("forestry"), AGRICULTURAL("agricultural"), PRIVATE("private"), OTHER("other"), NO("no");

    companion object {
        /** Come `RoadAccess.find`: valori sconosciuti diventano OTHER, null se vuoto. */
        fun find(value: String?): RoadAccess? {
            if (value.isNullOrEmpty()) return null
            return entries.firstOrNull { it.osm == value } ?: OTHER
        }
    }
}

enum class RoadClass(val osm: String) {
    OTHER("other"), MOTORWAY("motorway"), TRUNK("trunk"), PRIMARY("primary"), SECONDARY("secondary"),
    TERTIARY("tertiary"), RESIDENTIAL("residential"), UNCLASSIFIED("unclassified"), SERVICE("service"),
    ROAD("road"), TRACK("track"), BRIDLEWAY("bridleway"), STEPS("steps"), CYCLEWAY("cycleway"), PATH("path"),
    LIVING_STREET("living_street"), FOOTWAY("footway"), PEDESTRIAN("pedestrian"), PLATFORM("platform"),
    CORRIDOR("corridor");

    companion object {
        fun find(value: String?): RoadClass = entries.firstOrNull { it.osm == value } ?: OTHER
    }
}

enum class Toll { MISSING, NO, HGV, ALL }

/** Categoria ORS della way (`extra_info=waytype`). I numeri sono quelli dell'API ORS. */
enum class WayType(val code: Int) {
    UNKNOWN(0), STATE_ROAD(1), ROAD(2), STREET(3), PATH(4), TRACK(5),
    CYCLEWAY(6), FOOTWAY(7), STEPS(8), FERRY(9), CONSTRUCTION(10)
}

/** Superficie ORS (`extra_info=surface`). I numeri sono quelli dell'API ORS. */
enum class WaySurface(val code: Int) {
    UNKNOWN(0), PAVED(1), UNPAVED(2), ASPHALT(3), CONCRETE(4), METAL(6), WOOD(7),
    COMPACTED_GRAVEL(8), GRAVEL(10), DIRT(11), GROUND(12), ICE(13), PAVING_STONES(14),
    SAND(15), GRASS(17), GRASS_PAVER(18)
}

/**
 * Attributi di una way derivati dai tag, indipendenti dal profilo. Vengono poi
 * compattati nei bit dell'arco dal livello di storage.
 *
 * Le dimensioni massime valgono [Double.POSITIVE_INFINITY] quando il tag manca:
 * così "nessun limite" non blocca mai e si distingue da un limite reale.
 */
data class WayAttributes(
    val osmWayId: Long,
    /** Bit di [HeavyVehicleAttributes] dei tipi di veicolo pesante **bloccati**. */
    val blockedHeavyVehicleTypes: Int,
    /** Vero se la way vieta le merci pericolose (`hazmat=no`, `hazmat:B..E=no`). */
    val hazmatForbidden: Boolean,
    val maxHeight: Double,
    val maxWidth: Double,
    val maxLength: Double,
    val maxWeight: Double,
    val maxAxleLoad: Double,
    val toll: Toll,
    val roadAccess: RoadAccess,
    val roadClass: RoadClass,
    /** `highway=*_link` (rampe e raccordi), GraphHopper `road_class_link`. */
    val isLink: Boolean,
    val wayType: WayType,
    val waySurface: WaySurface,
    /** `highway=motorway|motorway_link` (categoria "highways" di avoid_features). */
    val isMotorway: Boolean,
    val isFord: Boolean,
    val isRoundabout: Boolean,
)
