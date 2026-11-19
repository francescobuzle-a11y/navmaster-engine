package it.navmaster.engine.storage

import it.navmaster.engine.profile.HgvTransit
import it.navmaster.engine.profile.NodeDelays
import it.navmaster.engine.profile.PriorityModel
import it.navmaster.engine.profile.RoadAccess
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.profile.WayAttributes
import it.navmaster.engine.profile.WaySpeedAccess
import it.navmaster.engine.profile.WaySurface
import it.navmaster.engine.profile.WayType

/** Valori codificati di un profilo veicolo: accesso e velocità per direzione, priorità. */
class ProfileEncoding(val profile: VehicleProfile, withDelay: Boolean = true) {
    val access = BooleanEncodedValue("${profile.name}_access", twoDirections = true)
    /** 5 bit × 5 km/h come ORS (`speed_bits=5`, `speed_factor=5`): massimo 155 km/h. */
    val speed = DecimalEncodedValue("${profile.name}_speed", 5, 5.0, twoDirections = true)
    /** Fattore di priorità 0.0..1.5 a passi di 0.1 (GraphHopper `PriorityCode`). */
    /** Il fattore di priorità (ORS: 4 bit a passi di 1/7, i valori di PriorityCode esatti). */
    val priority = DecimalEncodedValue("${profile.name}_priority", 4, 1.0 / 7)

    /**
     * I secondi persi nei nodi (modello NAVMASTER, [it.navmaster.engine.profile.NodeDelays]): 0–63 s a passi
     * di 1 s, per senso; sommati al tempo dell'arco da [it.navmaster.engine.weighting.FastestWeighting].
     * Null nei grafi costruiti prima (novembre 2026); 0 nei grafi del modello ORS.
     */
    val delay: DecimalEncodedValue? = if (withDelay) DecimalEncodedValue("${profile.name}_delay", 6, 1.0, twoDirections = true) else null

    internal fun register(b: EncodedValueRegistry.Builder) { b.add(access).add(speed).add(priority) }

    /** Registrato dopo i valori ADR: così nei grafi vecchi tutti gli altri bit restano dove erano. */
    internal fun registerDelay(b: EncodedValueRegistry.Builder) { delay?.let { b.add(it) } }

    /** Scrive i secondi persi nei due sensi (arrotondati all'intero, al massimo 63). */
    fun writeDelay(flags: IntArray, fwdSec: Double, bwdSec: Double) {
        val d = delay ?: return
        d.set(flags, false, clampDelay(fwdSec)); d.set(flags, true, clampDelay(bwdSec))
    }

    private fun clampDelay(s: Double): Double = if (s.isNaN() || s <= 0) 0.0 else Math.round(s).toDouble().coerceAtMost(NodeDelays.MAX_SECONDS.toDouble())

    fun write(flags: IntArray, r: WaySpeedAccess?) {
        if (r == null) {
            access.set(flags, false, false); access.set(flags, true, false)
            return
        }
        access.set(flags, false, r.forward); access.set(flags, true, r.backward)
        speed.set(flags, false, r.speedKmh); speed.set(flags, true, r.speedBackwardKmh)
        priority.set(flags, r.priority)
    }
}

/**
 * Valori codificati condivisi fra tutti i profili (attributi della way).
 *
 * @param withAdr i valori delle merci pericolose aggiunti a novembre 2026 ([adrTunnel], [hazmatGeneral],
 *   [hazmatWater]); `false` per aprire i grafi costruiti prima, che non li hanno (vedi [GraphEncoding.forExisting]).
 * @param withTransit i divieti di transito dei mezzi pesanti ([hgvDestination], [hgvWeightExempt]), solo
 *   nei grafi del modello NAVMASTER costruiti da novembre 2026.
 */
class SharedEncoding(val withAdr: Boolean = true, val withTransit: Boolean = false) {
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
    /** Categoria ADR della galleria: 0 nessuna, 2 = B … 5 = E ([WayAttributes.adrTunnel]); null nei grafi vecchi. */
    val adrTunnel: IntEncodedValue? = if (withAdr) IntEncodedValue("adr_tunnel", 3) else null
    /** `hazmat=no`; null nei grafi vecchi (lì vale solo [hazmatForbidden]). */
    val hazmatGeneral: BooleanEncodedValue? = if (withAdr) BooleanEncodedValue("hazmat_general") else null
    /** `hazmat:water=no`; null nei grafi vecchi. */
    val hazmatWater: BooleanEncodedValue? = if (withAdr) BooleanEncodedValue("hazmat_water") else null
    /**
     * Tratto vietato al transito dei mezzi pesanti, per senso ([HgvTransit]): il camion lo usa solo per
     * arrivare o partire (peso ×10 nel profilo heavyvehicle, LimitedAccessWeighting). Null nei grafi ORS
     * e in quelli di prima.
     */
    val hgvDestination: BooleanEncodedValue? = if (withTransit) BooleanEncodedValue("hgv_destination", twoDirections = true) else null
    /** Il limite di peso ha la deroga per destinazione: il filtro del camion non blocca sul peso. Null come [hgvDestination]. */
    val hgvWeightExempt: BooleanEncodedValue? = if (withTransit) BooleanEncodedValue("hgv_weight_exempt") else null

    internal fun register(b: EncodedValueRegistry.Builder) {
        b.add(roundabout).add(roadClass).add(roadClassLink).add(roadAccess).add(toll)
            .add(maxHeight).add(maxWidth).add(maxLength).add(maxWeight).add(maxAxleLoad)
            .add(hgvBlocked).add(hazmatForbidden).add(wayType).add(waySurface).add(motorway).add(ford).add(osmWayId)
    }

    /** Registrati dopo i profili: così nei grafi vecchi tutti gli altri bit restano dove erano. */
    internal fun registerAdr(b: EncodedValueRegistry.Builder) {
        if (!withAdr) return
        b.add(adrTunnel!!).add(hazmatGeneral!!).add(hazmatWater!!)
    }

    /**
     * Registrati per ultimi, dopo i ritardi ai nodi: gli altri bit restano dove erano. Con due profili
     * (car-ors e heavyvehicle, come grafo-nm) i 3 bit occupano gli ultimi liberi del sesto int: 192 bit
     * su 192, nessun bit libero; un valore in più vorrebbe dire 7 int per arco.
     */
    internal fun registerTransit(b: EncodedValueRegistry.Builder) {
        if (!withTransit) return
        b.add(hgvDestination!!).add(hgvWeightExempt!!)
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
        adrTunnel?.set(flags, a.adrTunnel)
        hazmatGeneral?.set(flags, a.hazmatGeneral)
        hazmatWater?.set(flags, a.hazmatWater)
        hgvDestination?.let { ev ->
            ev.set(flags, false, a.hgvTransit and HgvTransit.FWD != 0); ev.set(flags, true, a.hgvTransit and HgvTransit.BWD != 0)
        }
        hgvWeightExempt?.set(flags, a.hgvTransit and HgvTransit.EXEMPT != 0)
    }
}

/**
 * Schema completo dei flag di un grafo: attributi condivisi più un blocco per ogni profilo.
 * L'ordine di registrazione è fisso e fa parte del formato su disco.
 *
 * @param withDelay i ritardi ai nodi (`<profilo>_delay`, novembre 2026); di default presenti solo con i valori
 *   ADR, che sono più vecchi: uno schema senza ADR è quello dei grafi di prima di entrambi.
 * @param withTransit i divieti di transito dei mezzi pesanti (novembre 2026, [SharedEncoding.hgvDestination]):
 *   di default solo per i profili del modello NAVMASTER in uno schema coi ritardi; mai nei grafi ORS, che
 *   restano identici. Un motore di prima non apre i grafi che li hanno (valori codificati diversi, e la
 *   variante di formato `navmaster-transito`, vedi RoutingGraph): prima l'app, poi i grafi.
 */
class GraphEncoding(
    profiles: List<VehicleProfile>, withAdr: Boolean = true, val withDelay: Boolean = withAdr,
    withTransit: Boolean = withDelay && profiles.any { it.options.speedModel == SpeedModel.NAVMASTER },
) {
    val shared = SharedEncoding(withAdr, withTransit)
    val profiles: List<ProfileEncoding> = profiles.map { ProfileEncoding(it, withDelay) }
    val registry: EncodedValueRegistry

    /**
     * Come il peso «recommended» usa la priorità ([PriorityModel]): per un grafo nuovo dal modello di
     * velocità dei profili; per uno letto dal disco quello con cui è stato preparato (RoutingGraph.load),
     * perché CH, core e landmark hanno quel peso. Da fissare prima di preparare o interrogare il grafo.
     */
    @Volatile var priorityModel: PriorityModel = PriorityModel.forProfiles(profiles)

    init {
        require(profiles.isNotEmpty()) { "serve almeno un profilo" }
        require(profiles.map { it.name }.toSet().size == profiles.size) { "profili con lo stesso nome" }
        val b = EncodedValueRegistry.Builder()
        shared.register(b)
        this.profiles.forEach { it.register(b) }
        shared.registerAdr(b)
        this.profiles.forEach { it.registerDelay(b) }
        shared.registerTransit(b)
        registry = b.build()
    }

    companion object {
        /**
         * Lo schema di un grafo già costruito, dalla descrizione salvata nelle sue properties
         * (`graph.encoded_values`): quello dei profili dati (ADR e ritardi ai nodi; i profili letti dal disco
         * sono del modello ORS, quindi senza i divieti di transito), quello coi divieti di transito (grafi
         * NAVMASTER da novembre 2026) o, per i grafi di prima, quello senza i ritardi (ottobre–novembre
         * 2026) o senza ADR né ritardi. Se nessuno coincide restituisce il primo (il caricamento poi
         * segnala la differenza).
         */
        fun forExisting(profiles: List<VehicleProfile>, described: String?): GraphEncoding {
            val current = GraphEncoding(profiles)
            if (described == null || described == current.registry.describe()) return current
            if (!current.shared.withTransit) {
                val transit = GraphEncoding(profiles, withTransit = true)
                if (described == transit.registry.describe()) return transit
            }
            val noTransit = GraphEncoding(profiles, withTransit = false)
            if (described == noTransit.registry.describe()) return noTransit
            val noDelay = GraphEncoding(profiles, withAdr = true, withDelay = false, withTransit = false)
            if (described == noDelay.registry.describe()) return noDelay
            val old = GraphEncoding(profiles, withAdr = false, withDelay = false, withTransit = false)
            return if (described == old.registry.describe()) old else current
        }
    }

    fun profile(name: String): ProfileEncoding = profiles.firstOrNull { it.profile.name == name } ?: error("profilo non presente nel grafo: $name")
}
