package it.navmaster.engine.weighting

import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.profile.ProfileCategory
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.WayType
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.SharedEncoding

/** Decide se un arco può essere considerato dalla ricerca (indipendentemente dalla direzione). */
fun interface EdgeFilter {
    fun accept(edge: BaseGraph.EdgeIterator): Boolean

    companion object {
        val ALL = EdgeFilter { true }
    }
}

class EdgeFilterSequence(private val filters: List<EdgeFilter>) : EdgeFilter {
    override fun accept(edge: BaseGraph.EdgeIterator): Boolean {
        for (f in filters) if (!f.accept(edge)) return false
        return true
    }
}

/** Parametri del veicolo pesante inviati con la richiesta (ORS `profile_params.restrictions`). */
data class VehicleParameters(
    val length: Double = 0.0,
    val height: Double = 0.0,
    val width: Double = 0.0,
    val weight: Double = 0.0,
    val axleLoad: Double = 0.0,
    val hazmat: Boolean = false,
    /**
     * Estensione NavMaster: il codice galleria ADR del carico, 0 se non dato, 2 = B … 5 = E. Con un
     * codice le gallerie si confrontano per categoria (codice C: vietate le gallerie C, D, E); senza,
     * come ORS, ogni galleria con restrizioni ADR è vietata a chi trasporta merci pericolose.
     */
    val adrTunnelCode: Int = 0,
    /** Estensione NavMaster: merci inquinanti per le acque (strade con `hazmat:water=no` vietate). */
    val hazmatWater: Boolean = false,
) {
    init { require(adrTunnelCode == 0 || adrTunnelCode in 2..5) { "codice galleria ADR non valido: $adrTunnelCode" } }

    val hasAttributes get() = length > 0 || height > 0 || width > 0 || weight > 0 || axleLoad > 0 || hazmat || hazmatWater
    /** Vero se il carico è di merci pericolose (anche solo inquinanti per le acque). */
    val dangerousGoods get() = hazmat || hazmatWater || adrTunnelCode != 0
    /** Vero se la richiesta usa le estensioni ADR (codice galleria o acque). */
    val usesAdrDetails get() = adrTunnelCode != 0 || hazmatWater
    /** Le misure fisiche (senza le merci pericolose): valgono per ogni veicolo, anche l'auto (camper, furgoni). */
    val hasDimensions get() = length > 0 || height > 0 || width > 0 || weight > 0 || axleLoad > 0

    /** Vero se un arco coi limiti dati (0 = nessuno) non è percorribile con queste misure. */
    fun blockedBy(maxHeight: Double, maxWidth: Double, maxLength: Double, maxWeight: Double, maxAxleLoad: Double): Boolean =
        (height > 0 && blocks(maxHeight, height)) || (width > 0 && blocks(maxWidth, width)) || (length > 0 && blocks(maxLength, length)) ||
            (weight > 0 && blocks(maxWeight, weight)) || (axleLoad > 0 && blocks(maxAxleLoad, axleLoad))

    private fun blocks(limit: Double, vehicleValue: Double) = limit > 0.0 && limit < vehicleValue
}

/**
 * Estensione NavMaster, non in ORS: le misure fisiche del veicolo applicate ai limiti delle strade
 * (altezza, larghezza, lunghezza, peso, carico per asse) per il profilo auto, cioè camper e furgoni,
 * senza i divieti riservati ai mezzi pesanti (`hgv=no`, che valgono sopra 3,5 t di merci) e senza
 * le merci pericolose.
 */
class DimensionsEdgeFilter(private val shared: SharedEncoding, private val params: VehicleParameters) : EdgeFilter {
    override fun accept(edge: BaseGraph.EdgeIterator): Boolean =
        !params.blockedBy(edge.get(shared.maxHeight), edge.get(shared.maxWidth), edge.get(shared.maxLength), edge.get(shared.maxWeight), edge.get(shared.maxAxleLoad))
}

/**
 * Filtro camion (ORS `HeavyVehicleEdgeFilter`): accesso del tipo di veicolo, merci pericolose e
 * limiti dimensionali. Un limite blocca se è presente (finito) e inferiore al valore del veicolo.
 */
class HeavyVehicleEdgeFilter(
    private val shared: SharedEncoding,
    /** Uno dei bit di [HeavyVehicleAttributes] (HGV, BUS, DELIVERY, …). */
    private val vehicleType: Int,
    private val params: VehicleParameters?,
    /** I dati di guida: nei grafi senza i valori ADR danno la categoria delle gallerie (vedi [hazmatBlocked]). */
    private val guidance: it.navmaster.engine.storage.GuidanceStore? = null,
) : EdgeFilter {
    init { require(vehicleType in HeavyVehicleAttributes.vehicleTypes) { "tipo di veicolo non valido: $vehicleType" } }

    override fun accept(edge: BaseGraph.EdgeIterator): Boolean {
        if (edge.get(shared.hgvBlocked) and vehicleType != 0) return false
        if (params == null || !params.hasAttributes) return true
        if (params.dangerousGoods && hazmatBlocked(edge, shared, params, guidance)) return false
        // limite di peso con la deroga per destinazione (grafi NAVMASTER): non blocca, il tratto pesa ×10 (HgvTransit)
        val maxWeight = if (shared.hgvWeightExempt?.let { edge.get(it) } == true) Double.POSITIVE_INFINITY else edge.get(shared.maxWeight)
        return !params.blockedBy(edge.get(shared.maxHeight), edge.get(shared.maxWidth), edge.get(shared.maxLength), maxWeight, edge.get(shared.maxAxleLoad))
    }
}

/**
 * Merci pericolose. Con i valori ADR del grafo: `hazmat=no` vieta sempre; `hazmat:water=no` vieta le
 * merci inquinanti per le acque; una galleria di categoria X vieta i codici ≤ X (B è il più severo:
 * un carico B non passa nelle gallerie B, C, D, E; un carico E solo nelle E). Senza codice galleria,
 * come ORS: vietata ogni way con `hazmat=no` o `hazmat:B..E=no`, e in più quelle col solo
 * `hazmat:adr_tunnel_category` (più prudente di ORS, che non le legge).
 *
 * Nei grafi di prima dei valori ADR c'è solo [SharedEncoding.hazmatForbidden]: con un codice galleria
 * si usa la categoria salvata nei dati di guida per lasciar passare le gallerie meno severe del codice
 * (una way con insieme `hazmat=no` e `hazmat:X=no`, rara, verrebbe aperta per errore).
 */
internal fun hazmatBlocked(edge: BaseGraph.EdgeIterator, shared: SharedEncoding, params: VehicleParameters, guidance: it.navmaster.engine.storage.GuidanceStore?): Boolean {
    if (shared.withAdr) {
        if (edge.get(shared.hazmatGeneral!!)) return true
        if (params.hazmatWater && edge.get(shared.hazmatWater!!)) return true
        val cat = edge.get(shared.adrTunnel!!)
        if (params.adrTunnelCode != 0) return cat >= params.adrTunnelCode
        // senza codice: le merci solo inquinanti per le acque non sono vincolate dalle gallerie
        if (!params.hazmat) return false
        return cat != 0 || edge.get(shared.hazmatForbidden)
    }
    if (!edge.get(shared.hazmatForbidden)) return false
    if (params.adrTunnelCode != 0 && guidance != null) {
        val cat = guidance.adrTunnel(edge.edge)
        if (cat != 0 && cat < params.adrTunnelCode) return false
    }
    return params.hazmat || params.adrTunnelCode != 0
}

/**
 * `avoid_features` (ORS `AvoidFeaturesEdgeFilter` + `FormerWayCategory` + `TollwayExtractor`):
 * autostrade, pedaggi (per l'auto solo `toll=yes`, per il camion anche `toll:hgv`), traghetti, guadi, scale.
 */
class AvoidFeaturesEdgeFilter(private val shared: SharedEncoding, avoidFeatures: Int, private val category: ProfileCategory, private val isHgv: Boolean) : EdgeFilter {
    private val flags = avoidFeatures and AvoidFeatureFlags.forCategory(category)

    override fun accept(edge: BaseGraph.EdgeIterator): Boolean {
        if (flags == 0) return true
        if (flags and AvoidFeatureFlags.HIGHWAYS != 0 && edge.get(shared.motorway)) return false
        if (flags and AvoidFeatureFlags.FORDS != 0 && edge.get(shared.ford)) return false
        if (flags and (AvoidFeatureFlags.FERRIES or AvoidFeatureFlags.STEPS) != 0) {
            val wt = edge.get(shared.wayType)
            if (flags and AvoidFeatureFlags.FERRIES != 0 && wt == WayType.FERRY) return false
            if (flags and AvoidFeatureFlags.STEPS != 0 && wt == WayType.STEPS) return false
        }
        if (flags and AvoidFeatureFlags.TOLLWAYS != 0) {
            val toll = edge.get(shared.toll)
            if (toll == Toll.ALL) return false
            if (isHgv && toll == Toll.HGV) return false
        }
        return true
    }
}

/**
 * `avoid_polygons` (ORS `AvoidAreasEdgeFilter`): un arco è escluso se la sua geometria è contenuta in
 * un poligono o lo attraversa. Un controllo sull'inviluppo evita i calcoli JTS per gli archi lontani.
 */
class AvoidPolygonsEdgeFilter(polygons: List<org.locationtech.jts.geom.Polygon>) : EdgeFilter {
    private val polys = polygons
    private val env: org.locationtech.jts.geom.Envelope? = if (polygons.isEmpty()) null else org.locationtech.jts.geom.Envelope().also { e -> polygons.forEach { e.expandToInclude(it.envelopeInternal) } }
    private val gf = org.locationtech.jts.geom.GeometryFactory()

    override fun accept(edge: BaseGraph.EdgeIterator): Boolean {
        val e = env ?: return true
        val pl = edge.fetchWayGeometry(it.navmaster.engine.storage.FetchMode.ALL)
        var minX = Double.MAX_VALUE; var minY = Double.MAX_VALUE; var maxX = -Double.MAX_VALUE; var maxY = -Double.MAX_VALUE
        for (i in 0 until pl.size) {
            val x = pl.lon(i); val y = pl.lat(i)
            if (x < minX) minX = x; if (x > maxX) maxX = x; if (y < minY) minY = y; if (y > maxY) maxY = y
        }
        if (minX > e.maxX || maxX < e.minX || minY > e.maxY || maxY < e.minY) return true
        if (pl.size < 2) return false
        val coords = Array(pl.size) { org.locationtech.jts.geom.Coordinate(pl.lon(it), pl.lat(it)) }
        val ls = gf.createLineString(coords)
        for (p in polys) if (p.contains(ls) || ls.crosses(p)) return false
        return true
    }
}

/** Costruisce la catena di filtri per una richiesta. */
object EdgeFilterFactory {
    fun create(
        encoding: GraphEncoding, profileName: String, avoidFeatures: Int = 0, vehicleType: Int = HeavyVehicleAttributes.HGV,
        params: VehicleParameters? = null, avoidPolygons: List<org.locationtech.jts.geom.Polygon> = emptyList(),
        guidance: it.navmaster.engine.storage.GuidanceStore? = null,
    ): EdgeFilter {
        val pe = encoding.profile(profileName)
        val isHgv = pe.profile.name == "heavyvehicle"
        val filters = ArrayList<EdgeFilter>(4)
        if (avoidFeatures != 0) filters += AvoidFeaturesEdgeFilter(encoding.shared, avoidFeatures, pe.profile.category, isHgv)
        if (isHgv) filters += HeavyVehicleEdgeFilter(encoding.shared, vehicleType, params, guidance)
        else if (params != null && params.hasDimensions) filters += DimensionsEdgeFilter(encoding.shared, params)
        if (avoidPolygons.isNotEmpty()) filters += AvoidPolygonsEdgeFilter(avoidPolygons)
        return when (filters.size) {
            0 -> EdgeFilter.ALL
            1 -> filters[0]
            else -> EdgeFilterSequence(filters)
        }
    }
}
