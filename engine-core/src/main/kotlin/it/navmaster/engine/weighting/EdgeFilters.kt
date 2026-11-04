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
) {
    val hasAttributes get() = length > 0 || height > 0 || width > 0 || weight > 0 || axleLoad > 0 || hazmat
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
) : EdgeFilter {
    init { require(vehicleType in HeavyVehicleAttributes.vehicleTypes) { "tipo di veicolo non valido: $vehicleType" } }

    override fun accept(edge: BaseGraph.EdgeIterator): Boolean {
        if (edge.get(shared.hgvBlocked) and vehicleType != 0) return false
        if (params == null || !params.hasAttributes) return true
        if (params.hazmat && edge.get(shared.hazmatForbidden)) return false
        return !params.blockedBy(edge.get(shared.maxHeight), edge.get(shared.maxWidth), edge.get(shared.maxLength), edge.get(shared.maxWeight), edge.get(shared.maxAxleLoad))
    }
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
    ): EdgeFilter {
        val pe = encoding.profile(profileName)
        val isHgv = pe.profile.name == "heavyvehicle"
        val filters = ArrayList<EdgeFilter>(4)
        if (avoidFeatures != 0) filters += AvoidFeaturesEdgeFilter(encoding.shared, avoidFeatures, pe.profile.category, isHgv)
        if (isHgv) filters += HeavyVehicleEdgeFilter(encoding.shared, vehicleType, params)
        else if (params != null && params.hasDimensions) filters += DimensionsEdgeFilter(encoding.shared, params)
        if (avoidPolygons.isNotEmpty()) filters += AvoidPolygonsEdgeFilter(avoidPolygons)
        return when (filters.size) {
            0 -> EdgeFilter.ALL
            1 -> filters[0]
            else -> EdgeFilterSequence(filters)
        }
    }
}
