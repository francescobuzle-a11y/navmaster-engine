package it.navmaster.engine.profile

/** Tipi di veicolo pesante e caratteristiche del carico, come maschera di bit (ORS `HeavyVehicleAttributes`). */
object HeavyVehicleAttributes {
    const val UNKNOWN = 0
    const val GOODS = 1
    const val HGV = 2
    const val BUS = 4
    const val AGRICULTURE = 8
    const val FORESTRY = 16
    const val DELIVERY = 32
    const val ANY = GOODS or HGV or BUS or AGRICULTURE or FORESTRY or DELIVERY

    /** Caratteristica del carico, non un tipo di veicolo. */
    const val HAZMAT = 128

    fun fromString(value: String): Int = when (value.lowercase()) {
        "goods" -> GOODS
        "hgv" -> HGV
        "bus" -> BUS
        "agricultural" -> AGRICULTURE
        "forestry" -> FORESTRY
        "delivery" -> DELIVERY
        else -> UNKNOWN
    }

    val vehicleTypes: List<Int> = listOf(GOODS, HGV, BUS, AGRICULTURE, FORESTRY, DELIVERY)
}

/** Caratteristiche stradali evitabili su richiesta (ORS `AvoidFeatureFlags`). */
object AvoidFeatureFlags {
    const val NONE = 0
    const val HIGHWAYS = 1
    const val TOLLWAYS = 2
    const val STEPS = 4
    const val FERRIES = 8
    const val FORDS = 16

    const val DRIVING_FEATURES = HIGHWAYS or TOLLWAYS or FERRIES or FORDS
    const val CYCLING_FEATURES = STEPS or FERRIES or FORDS
    const val WALKING_FEATURES = STEPS or FERRIES or FORDS

    fun fromString(value: String): Int = when (value.lowercase()) {
        "highways" -> HIGHWAYS
        "tollways" -> TOLLWAYS
        "ferries" -> FERRIES
        "steps" -> STEPS
        "fords" -> FORDS
        else -> NONE
    }

    fun forCategory(category: ProfileCategory): Int = when (category) {
        ProfileCategory.DRIVING -> DRIVING_FEATURES
        ProfileCategory.CYCLING -> CYCLING_FEATURES
        ProfileCategory.WALKING, ProfileCategory.WHEELCHAIR -> WALKING_FEATURES
        ProfileCategory.UNKNOWN -> NONE
    }
}

enum class ProfileCategory { UNKNOWN, DRIVING, CYCLING, WALKING, WHEELCHAIR }

/**
 * Codici di priorità di GraphHopper: 0 = escludi … 10 = neutro … 15 = ottimo.
 * Il fattore memorizzato è `value / 10`.
 */
enum class PriorityCode(val value: Int) {
    EXCLUDE(0),
    REACH_DESTINATION(1),
    VERY_BAD(3),
    BAD(5),
    AVOID_MORE(6),
    AVOID(8),
    SLIGHT_AVOID(9),
    UNCHANGED(10),
    SLIGHT_PREFER(11),
    PREFER(12),
    VERY_NICE(13),
    BEST(15);

    val factor: Double get() = value / 10.0

    companion object {
        /** Nomi storici usati dal codice ORS. */
        val REACH_DEST = REACH_DESTINATION
        val AVOID_AT_ALL_COSTS = VERY_BAD
        val AVOID_IF_POSSIBLE = AVOID
        val WORST = EXCLUDE
    }
}
