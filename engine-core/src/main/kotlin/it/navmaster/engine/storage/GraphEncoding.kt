package it.navmaster.engine.storage

import it.navmaster.engine.profile.RoadAccess
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.profile.WayAttributes
import it.navmaster.engine.profile.WaySpeedAccess
import it.navmaster.engine.profile.WaySurface
import it.navmaster.engine.profile.WayType

/** Valori codificati di un profilo veicolo: accesso e velocità per direzione, priorità. */
class ProfileEncoding(val profile: VehicleProfile) {
    val access = BooleanEncodedValue("${profile.name}_access", twoDirections = true)
    /** 5 bit × 5 km/h come ORS (`speed_bits=5`, `speed_factor=5`): massimo 155 km/h. */
    val speed = DecimalEncodedValue("${profile.name}_speed", 5, 5.0, twoDirections = true)
    /** Fattore di priorità 0.0..1.5 a passi di 0.1 (GraphHopper `PriorityCode`). */
    /** Il fattore di priorità (ORS: 4 bit a passi di 1/7, i valori di PriorityCode esatti). */
    val priority = DecimalEncodedValue("${profile.name}_priority", 4, 1.0 / 7)

    internal fun register(b: EncodedValueRegistry.Builder) { b.add(access).add(speed).add(priority) }

    fun write(flags: IntArray, r: WaySpeedAccess?) {
        if (r == null) {
            access.set(flags, false, false); access.set(flags, true, false)
            return
        }
        access.set(flags, false, r.forward); access.set(flags, true, r.backward)
        speed.set(flags, false, r.speedKmh); speed.set(flags, true, r.speedKmh)
        priority.set(flags, r.priority)
    }
}

/** Valori codificati condivisi fra tutti i profili (attributi della way). */
class SharedEncoding {
    val roundabout = BooleanEncodedValue("roundabout")
    val roadClass = EnumEncodedValue("road_class", RoadClass.entries.toTypedArray())
    val roadClassLink = BooleanEncodedValue("road_class_link")
    val roadAccess = EnumEncodedValue("road_access", RoadAccess.entries.toTypedArray())
    val toll = EnumEncodedValue("toll", Toll.entries.toTypedArray())
    val maxHeight = DecimalEncodedValue("max_height", 11, 0.01, Double.POSITIVE_INFINITY)
    val maxWidth = DecimalEncodedValue("max_width", 10, 0.01, Double.POSITIVE_INFINITY)
    val maxLength = DecimalEncodedValue("max_length", 11, 0.01, Double.POSITIVE_INFINITY)
    val maxWeight = DecimalEncodedValue("max_weight", 11, 0.1, Double.POSITIVE_INFINITY)
    val maxAxleLoad = DecimalEncodedValue("max_axle_load", 9, 0.1, Double.POSITIVE_INFINITY)
    /** Bit di `HeavyVehicleAttributes` dei tipi di mezzo pesante bloccati. */
    val hgvBlocked = IntEncodedValue("hgv_blocked", 6)
    val hazmatForbidden = BooleanEncodedValue("hazmat_forbidden")
    val wayType = EnumEncodedValue("way_type", WayType.entries.toTypedArray())
    val waySurface = EnumEncodedValue("way_surface", WaySurface.entries.toTypedArray())
    val motorway = BooleanEncodedValue("motorway")
    val ford = BooleanEncodedValue("ford")
    val osmWayId = IntEncodedValue("osm_way_id", 31)

    internal fun register(b: EncodedValueRegistry.Builder) {
        b.add(roundabout).add(roadClass).add(roadClassLink).add(roadAccess).add(toll)
            .add(maxHeight).add(maxWidth).add(maxLength).add(maxWeight).add(maxAxleLoad)
            .add(hgvBlocked).add(hazmatForbidden).add(wayType).add(waySurface).add(motorway).add(ford).add(osmWayId)
    }

    fun write(flags: IntArray, a: WayAttributes) {
        roundabout.set(flags, a.isRoundabout)
        roadClass.set(flags, a.roadClass)
        roadClassLink.set(flags, a.isLink)
        roadAccess.set(flags, a.roadAccess)
        toll.set(flags, a.toll)
        maxHeight.set(flags, a.maxHeight); maxWidth.set(flags, a.maxWidth); maxLength.set(flags, a.maxLength)
        maxWeight.set(flags, a.maxWeight); maxAxleLoad.set(flags, a.maxAxleLoad)
        hgvBlocked.set(flags, a.blockedHeavyVehicleTypes)
        hazmatForbidden.set(flags, a.hazmatForbidden)
        wayType.set(flags, a.wayType); waySurface.set(flags, a.waySurface)
        motorway.set(flags, a.isMotorway); ford.set(flags, a.isFord)
        require(a.osmWayId <= osmWayId.maxRaw) { "id way OSM troppo grande: ${a.osmWayId}" }
        osmWayId.set(flags, a.osmWayId.toInt())
    }
}

/**
 * Schema completo dei flag di un grafo: attributi condivisi più un blocco per ogni profilo.
 * L'ordine di registrazione è fisso e fa parte del formato su disco.
 */
class GraphEncoding(profiles: List<VehicleProfile>) {
    val shared = SharedEncoding()
    val profiles: List<ProfileEncoding> = profiles.map { ProfileEncoding(it) }
    val registry: EncodedValueRegistry

    init {
        require(profiles.isNotEmpty()) { "serve almeno un profilo" }
        require(profiles.map { it.name }.toSet().size == profiles.size) { "profili con lo stesso nome" }
        val b = EncodedValueRegistry.Builder()
        shared.register(b)
        this.profiles.forEach { it.register(b) }
        registry = b.build()
    }

    fun profile(name: String): ProfileEncoding = profiles.firstOrNull { it.profile.name == name } ?: error("profilo non presente nel grafo: $name")
}
