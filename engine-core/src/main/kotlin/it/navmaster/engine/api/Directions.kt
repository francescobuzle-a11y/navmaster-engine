package it.navmaster.engine.api

import it.navmaster.engine.Engine
import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.instructions.DistanceUnit
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.instructions.InstructionsBuilder
import it.navmaster.engine.instructions.OrsStepBuilder
import it.navmaster.engine.instructions.RouteStep
import it.navmaster.engine.instructions.StepOptions
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.index.Snap
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.AvoidFeaturesEdgeFilter
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingMethod
import java.time.Instant
import java.util.concurrent.CountDownLatch
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicReferenceArray

/** Codici d'errore dell'endpoint directions di ORS (`RoutingErrorCodes`). */
object RoutingErrorCodes {
    const val INVALID_JSON_FORMAT = 2000
    const val MISSING_PARAMETER = 2001
    const val INVALID_PARAMETER_FORMAT = 2002
    const val INVALID_PARAMETER_VALUE = 2003
    const val REQUEST_EXCEEDS_SERVER_LIMIT = 2004
    const val EXPORT_HANDLER_ERROR = 2006
    const val UNSUPPORTED_EXPORT_FORMAT = 2007
    const val EMPTY_ELEMENT = 2008
    const val ROUTE_NOT_FOUND = 2009
    const val POINT_NOT_FOUND = 2010
    const val INCOMPATIBLE_PARAMETERS = 2011
    const val UNKNOWN_PARAMETER = 2012
    const val UNSUPPORTED_REQUEST_OPTION = 2018
    const val UNKNOWN = 2099
}

/** Errore con codice ORS e stato HTTP; il corpo JSON è prodotto da [DirectionsService.errorBody]. */
class ApiException(val httpStatus: Int, val code: Int, message: String) : RuntimeException(message)

enum class ResponseFormat { JSON, GEOJSON }

/** ORS `alternative_routes`: quanti percorsi in tutto, quanto possono pesare di più, quanto possono condividere. */
class AlternativeOptions(val targetCount: Int, val weightFactor: Double, val shareFactor: Double)

/** Richiesta directions nel vocabolario ORS, già validata. */
class DirectionsRequest(
    val profile: String,
    /** Coppie (lat, lon) in ordine di visita. */
    val coordinates: List<Pair<Double, Double>>,
    val preference: WeightingMethod,
    val units: DistanceUnit,
    val language: String,
    val instructions: Boolean,
    val htmlFormat: Boolean,
    val geometry: Boolean,
    val maneuvers: Boolean,
    val avoidFeatures: Int,
    val vehicleType: Int,
    val vehicleParameters: VehicleParameters?,
    val avoidPolygons: List<org.locationtech.jts.geom.Polygon>,
    val maximumSpeed: Double,
    /** ORS `continue_straight`: ai via-point si prosegue nel verso d'arrivo (niente inversione), ricerca non "static". */
    val continueStraight: Boolean,
    /** Estensione NavMaster `{"navmaster": {"roads": true}}`: i fatti sulle strade del percorso ([RoadFactsBuilder]). */
    val navmasterRoads: Boolean,
    /** ORS `alternative_routes` (target_count, weight_factor, share_factor), null se non richieste. */
    val alternatives: AlternativeOptions?,
    /** Raggio di aggancio per punto in metri (-1 = illimitato → raggio massimo del profilo). */
    val radiuses: DoubleArray?,
    val id: String?,
    val attributes: Set<String>,
    val extraInfo: List<String>,
    /** Richiesta originale, rispecchiata in `metadata.query`. */
    val echo: Map<String, Any?>,
    /**
     * Punti agganciati così come sono, senza il controllo delle piccole reti isolate: i passaggi di
     * confine di un viaggio tra Paesi (dove l'estratto di un Paese finisce, la strada è un moncone).
     */
    val exactPoints: BooleanArray? = null,
    /**
     * ORS `bearings`: per punto la direzione (gradi) con cui partire da lì (e, per l'ultimo, con
     * cui arrivarci); null dove non è data.
     */
    val bearings: List<Double?>? = null,
    /**
     * Estensione NavMaster `{"navmaster": {"avoid_penalty": true}}`: `avoid_features` come penalità
     * ([AVOID_PENALTY_FACTOR]) invece che come divieto, come gli "evita" di TomTom.
     */
    val avoidPenalty: Boolean = false,
    /**
     * Estensione NavMaster `{"navmaster": {"pass_through": [false, true, false]}}`: per punto, vero
     * dove si passa senza fermarsi (un «passa per di qui»): lì, come con `continue_straight`, il
     * tratto seguente prosegue nel verso d'arrivo. Le soste (false) lasciano invertire.
     */
    val passThrough: BooleanArray? = null,
    /**
     * Estensione NavMaster `{"navmaster": {"alternatives": {…}}}`: le varianti del migliore con
     * qualunque numero di punti ([NavAlternativesEngine]); null se non chieste. Non insieme ad `alternative_routes`.
     */
    val navAlternatives: NavAlternativesOptions? = null,
) {
    /** Vero se al punto [i] si prosegue nel verso d'arrivo: `continue_straight`, o `pass_through` di quel punto. */
    fun passThroughAt(i: Int): Boolean = continueStraight || passThrough?.getOrNull(i) == true

    /** La stessa richiesta con o senza gli "evita" come penalità ([avoidPenalty]). */
    fun withAvoidPenalty(on: Boolean): DirectionsRequest = if (on == avoidPenalty) this else DirectionsRequest(
        profile, coordinates, preference, units, language, instructions, htmlFormat, geometry, maneuvers, avoidFeatures, vehicleType,
        vehicleParameters, avoidPolygons, maximumSpeed, continueStraight, navmasterRoads, alternatives, radiuses, id, attributes, extraInfo, echo, exactPoints,
        bearings, on, passThrough, navAlternatives)

    /** La stessa richiesta senza `navmaster.alternatives`: quella del percorso migliore. */
    fun withoutNavAlternatives(): DirectionsRequest = withNavAlternatives(null)

    /** La stessa richiesta con le varianti [opt] (anche con le opzioni interne, per le prove). */
    internal fun withNavAlternatives(opt: NavAlternativesOptions?): DirectionsRequest = if (navAlternatives === opt) this else DirectionsRequest(
        profile, coordinates, preference, units, language, instructions, htmlFormat, geometry, maneuvers, avoidFeatures, vehicleType,
        vehicleParameters, avoidPolygons, maximumSpeed, continueStraight, navmasterRoads, alternatives, radiuses, id, attributes, extraInfo, echo, exactPoints,
        bearings, avoidPenalty, passThrough, opt)

    /**
     * La stessa richiesta su altri punti (un tratto di un viaggio tra Paesi), senza alternative (né varianti);
     * [pointPassThrough]: il `pass_through` dei nuovi punti (null: nessuno).
     */
    fun withPoints(points: List<Pair<Double, Double>>, pointRadiuses: DoubleArray?, exact: BooleanArray? = null, pointBearings: List<Double?>? = null,
                   pointPassThrough: BooleanArray? = null): DirectionsRequest = DirectionsRequest(
        profile, points, preference, units, language, instructions, htmlFormat, geometry, maneuvers, avoidFeatures, vehicleType,
        vehicleParameters, avoidPolygons, maximumSpeed, continueStraight, navmasterRoads, null, pointRadiuses, id, attributes, extraInfo, echo, exact,
        pointBearings?.takeIf { b -> b.any { it != null } }, avoidPenalty, pointPassThrough?.takeIf { p -> p.any { it } })

    companion object {
        /**
         * Il fattore degli "evita" come penalità ([avoidPenalty]): una strada da evitare pesa 30 volte
         * tanto, quindi si usa solo se ogni altra via costa più di 30 volte quel tratto (in pratica
         * quando non c'è altra via: partenza in autostrada, isole, valichi solo a pedaggio).
         */
        const val AVOID_PENALTY_FACTOR = 30.0
        private val KNOWN = setOf("coordinates", "preference", "units", "language", "instructions", "instructions_format", "geometry", "maneuvers",
            "options", "radiuses", "id", "attributes", "elevation", "extra_info", "continue_straight", "roundabout_exits", "geometry_simplify",
            "suppress_warnings", "skip_segments", "alternative_routes", "bearings", "maximum_speed", "schedule", "custom_model", "navmaster")

        /** Costruisce la richiesta dal JSON del corpo (già analizzato in mappa). */
        fun fromJson(profile: String, body: Map<String, Any?>): DirectionsRequest {
            for (k in body.keys) if (k !in KNOWN) throw ApiException(400, RoutingErrorCodes.UNKNOWN_PARAMETER, "Unknown parameter '$k'.")
            val coordsRaw = body["coordinates"] as? List<*> ?: throw ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter 'coordinates' is missing.")
            if (coordsRaw.size < 2) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'coordinates' needs at least two points.")
            val coordinates = coordsRaw.map { c ->
                val p = c as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'coordinates' has an invalid format.")
                if (p.size < 2) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'coordinates' has an invalid format.")
                val lon = num(p[0], "coordinates"); val lat = num(p[1], "coordinates")
                if (lat !in -90.0..90.0 || lon !in -180.0..180.0) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'coordinates' has an invalid value.")
                lat to lon
            }
            val preference = when (val p = body["preference"]?.toString() ?: "recommended") {
                "fastest" -> WeightingMethod.FASTEST; "shortest" -> WeightingMethod.SHORTEST; "recommended" -> WeightingMethod.RECOMMENDED
                else -> throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'preference' has incorrect value of '$p'.")
            }
            val units = when (val u = body["units"]?.toString() ?: "m") {
                "m" -> DistanceUnit.METERS; "km" -> DistanceUnit.KILOMETERS; "mi" -> DistanceUnit.MILES
                else -> throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'units' has incorrect value of '$u'.")
            }
            val htmlFormat = when (val f = body["instructions_format"]?.toString() ?: "text") {
                "text" -> false; "html" -> true
                else -> throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'instructions_format' has incorrect value of '$f'.")
            }
            if (body["elevation"] == true) throw ApiException(400, RoutingErrorCodes.UNSUPPORTED_REQUEST_OPTION, "Parameter 'elevation' is not supported yet.")
            var avoid = 0; var vehicleType = HeavyVehicleAttributes.HGV; var params: VehicleParameters? = null
            var polygons: List<org.locationtech.jts.geom.Polygon> = emptyList()
            (body["options"] as? Map<*, *>)?.let { o ->
                (o["avoid_polygons"] as? Map<*, *>)?.let { polygons = parsePolygons(it) }
                (o["avoid_features"] as? List<*>)?.forEach { f ->
                    val flag = AvoidFeatureFlags.fromString(f.toString())
                    if (flag == 0) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'avoid_features' has incorrect value of '$f'.")
                    avoid = avoid or flag
                }
                o["vehicle_type"]?.let { v ->
                    vehicleType = HeavyVehicleAttributes.fromString(v.toString())
                    if (vehicleType == HeavyVehicleAttributes.UNKNOWN) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'vehicle_type' has incorrect value of '$v'.")
                }
                ((o["profile_params"] as? Map<*, *>)?.get("restrictions") as? Map<*, *>)?.let { r ->
                    params = VehicleParameters(
                        length = num(r["length"], "length", 0.0), height = num(r["height"], "height", 0.0), width = num(r["width"], "width", 0.0),
                        weight = num(r["weight"], "weight", 0.0), axleLoad = num(r["axleload"], "axleload", 0.0), hazmat = r["hazmat"] == true,
                        adrTunnelCode = adrTunnelCode(r["adr_tunnel_code"]), hazmatWater = r["hazmat_water"] == true,
                    )
                }
            }
            val radiuses = when (val r = body["radiuses"]) {
                null -> null
                is List<*> -> { if (r.size != coordinates.size) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'radiuses' must have one entry per coordinate."); DoubleArray(r.size) { num(r[it], "radiuses") } }
                else -> DoubleArray(coordinates.size) { num(r, "radiuses") }
            }
            val attributes = (body["attributes"] as? List<*>)?.map { it.toString() }?.toSet() ?: emptySet()
            for (a in attributes) if (a !in setOf("avgspeed", "detourfactor", "percentage")) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'attributes' has incorrect value of '$a'.")
            val extraInfo = (body["extra_info"] as? List<*>)?.map { it.toString() } ?: emptyList()
            for (x in extraInfo) if (x !in SUPPORTED_EXTRA_INFO) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'extra_info' has incorrect value of '$x'.")
            val maximumSpeed = body["maximum_speed"]?.let { num(it, "maximum_speed") } ?: Double.POSITIVE_INFINITY
            // ORS `bearings`: [[gradi, deviazione], …] per punto (o per tutti salvo l'ultimo); [] per nessuna
            val bearings = (body["bearings"] as? List<*>)?.let { list ->
                if (list.size != coordinates.size && list.size != coordinates.size - 1) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'bearings' must have one entry per coordinate (or one less).")
                List(coordinates.size) { i ->
                    val b = list.getOrNull(i) as? List<*>
                    if (b == null || b.isEmpty()) null else num(b[0], "bearings").also { v ->
                        if (v < 0 || v > 360) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'bearings' has an invalid value.")
                    }
                }
            }?.takeIf { b -> b.any { it != null } }
            // NavMaster `pass_through`: un valore per punto
            val passThrough = ((body["navmaster"] as? Map<*, *>)?.get("pass_through"))?.let { v ->
                val list = v as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'navmaster.pass_through' must be a list with one entry per coordinate.")
                if (list.size != coordinates.size) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'navmaster.pass_through' must have one entry per coordinate.")
                // solo vero/falso: 1, "true" o null non diventano in silenzio una sosta
                BooleanArray(list.size) { list[it] as? Boolean ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'navmaster.pass_through' must contain only true or false.") }
            }?.takeIf { p -> p.any { it } }
            // NavMaster `alternatives`: le varianti del migliore, non insieme alle alternative ORS
            val navAlternatives = (body["navmaster"] as? Map<*, *>)?.get("alternatives")?.let { NavAlternativesOptions.fromJson(it) }
            if (navAlternatives != null && body["alternative_routes"] != null) throw ApiException(400, RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, "Parameters 'alternative_routes' and 'navmaster.alternatives' cannot be used together.")
            if (maximumSpeed < MAXIMUM_SPEED_LOWER_BOUND) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'maximum_speed' must be at least ${MAXIMUM_SPEED_LOWER_BOUND.toInt()} km/h.")
            return DirectionsRequest(
                profile, coordinates, preference, units, body["language"]?.toString() ?: "en",
                instructions = body["instructions"] as? Boolean ?: true, htmlFormat = htmlFormat,
                geometry = body["geometry"] as? Boolean ?: true, maneuvers = body["maneuvers"] as? Boolean ?: false,
                avoidFeatures = avoid, vehicleType = vehicleType, vehicleParameters = params?.takeIf { it.hasAttributes },
                avoidPolygons = polygons, maximumSpeed = maximumSpeed,
                continueStraight = body["continue_straight"] as? Boolean ?: false,
                navmasterRoads = (body["navmaster"] as? Map<*, *>)?.get("roads") == true,
                avoidPenalty = (body["navmaster"] as? Map<*, *>)?.get("avoid_penalty") == true,
                alternatives = (body["alternative_routes"] as? Map<*, *>)?.let { a ->
                    val count = (a["target_count"] as? Number)?.toInt() ?: 2
                    if (count !in 2..3) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'alternative_routes.target_count' has incorrect value of '$count' (2 or 3).")
                    AlternativeOptions(count, (a["weight_factor"] as? Number)?.toDouble() ?: 1.4, (a["share_factor"] as? Number)?.toDouble() ?: 0.6)
                },
                radiuses = radiuses, id = body["id"]?.toString(), attributes = attributes, extraInfo = extraInfo, echo = body,
                bearings = bearings, passThrough = passThrough, navAlternatives = navAlternatives,
            )
        }

        /** ORS `maximum_speed_lower_bound`: sotto questa soglia gli archi lenti non sono nel core. */
        const val MAXIMUM_SPEED_LOWER_BOUND = 80.0
        val SUPPORTED_EXTRA_INFO = setOf("tollways", "waycategory", "waytype", "surface", "roadaccessrestrictions", "osmid", "avgspeed")

        /** GeoJSON `Polygon` o `MultiPolygon` (coordinate lon/lat) → poligoni JTS. */
        private fun parsePolygons(geo: Map<*, *>): List<org.locationtech.jts.geom.Polygon> {
            val gf = org.locationtech.jts.geom.GeometryFactory()
            fun ring(r: Any?): org.locationtech.jts.geom.LinearRing {
                val pts = r as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'avoid_polygons' has an invalid format.")
                val coords = pts.map { p -> val c = p as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'avoid_polygons' has an invalid format."); org.locationtech.jts.geom.Coordinate(num(c[0], "avoid_polygons"), num(c[1], "avoid_polygons")) }.toMutableList()
                if (coords.size < 3) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'avoid_polygons' needs at least three points.")
                if (coords.first() != coords.last()) coords.add(coords.first())
                return gf.createLinearRing(coords.toTypedArray())
            }
            fun polygon(rings: Any?): org.locationtech.jts.geom.Polygon {
                val rs = rings as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'avoid_polygons' has an invalid format.")
                if (rs.isEmpty()) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'avoid_polygons' has an invalid format.")
                return gf.createPolygon(ring(rs[0]), rs.drop(1).map { ring(it) }.toTypedArray())
            }
            return when (geo["type"]) {
                "Polygon" -> listOf(polygon(geo["coordinates"]))
                "MultiPolygon" -> (geo["coordinates"] as? List<*> ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter 'avoid_polygons' has an invalid format.")).map { polygon(it) }
                else -> throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'avoid_polygons' must be a Polygon or MultiPolygon.")
            }
        }

        /**
         * Estensione NavMaster `restrictions.adr_tunnel_code`: il codice galleria ADR del carico, "B"…"E"
         * (anche nella forma dei documenti di trasporto, per esempio "(C/D)" o "B1000C": conta la prima
         * lettera). 0 se assente o "A"/"none" (nessuna restrizione in galleria).
         */
        internal fun adrTunnelCode(v: Any?): Int {
            val s = v?.toString()?.trim()?.uppercase() ?: return 0
            if (s.isEmpty() || s == "NONE" || s == "-" || s == "A") return 0
            val letter = s.firstOrNull { it in 'A'..'E' } ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'adr_tunnel_code' has incorrect value of '$v'.")
            return if (letter == 'A') 0 else letter - 'B' + 2
        }

        private fun num(v: Any?, name: String, default: Double? = null): Double = when (v) {
            null -> default ?: throw ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter '$name' is missing.")
            is Number -> v.toDouble()
            is String -> v.toDoubleOrNull() ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter '$name' has an invalid format.")
            else -> throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter '$name' has an invalid format.")
        }
    }
}

/** Limiti del servizio (ORS `endpoints.routing` e `profile.service`). */
data class DirectionsLimits(
    val maximumWaypoints: Int = 50,
    val maximumDistance: Double = 100_000.0,
    val maximumDistanceDynamicWeights: Double = 100_000.0,
    val maximumSnappingRadius: Double = 400.0,
    /** ORS `maximum_distance_alternative_routes`: oltre questa distanza in linea d'aria niente alternative. */
    val maximumDistanceAlternativeRoutes: Double = 100_000.0,
)

/**
 * Endpoint `directions`: convalida, calcola i tratti fra via-point, costruisce la risposta ORS
 * (JSON o GeoJSON) come struttura di mappe/liste serializzabile con [Json].
 */
class DirectionsService(private val graphs: RoutingGraph, private val limits: DirectionsLimits = DirectionsLimits(), private val version: String = Engine.VERSION) {
    val attribution = "navmaster-engine | OpenStreetMap contributors"

    /** Le opzioni di calcolo di una richiesta. */
    fun routeOptions(request: DirectionsRequest) = RouteOptions(
        profile = request.profile, weighting = request.preference, avoidFeatures = request.avoidFeatures,
        vehicleType = request.vehicleType, vehicleParameters = request.vehicleParameters,
        avoidPolygons = request.avoidPolygons, maximumSpeed = request.maximumSpeed,
        continueStraight = request.continueStraight || request.passThrough?.any { it } == true,
        hasBearings = request.bearings != null,
        avoidPenalty = if (request.avoidPenalty) DirectionsRequest.AVOID_PENALTY_FACTOR else 0.0,
    )

    /** maximum_speed ≥ 80 km/h su un viaggio oltre [FAST_MAXIMUM_SPEED_BEELINE] in linea d'aria: solo sui tempi. */
    fun fastMaximumSpeed(o: RouteOptions, beeline: Double): Boolean =
        !graphs.coreWide && o.maximumSpeed.isFinite() && o.maximumSpeed >= DirectionsRequest.MAXIMUM_SPEED_LOWER_BOUND && beeline > FAST_MAXIMUM_SPEED_BEELINE &&
            o.avoidPolygons.isEmpty()

    /** Registro delle alternative dei viaggi lunghi ([LongAlternatives]), per la diagnostica dell'app. */
    @Volatile var alternativesLog: ((String) -> Unit)? = null

    /**
     * Vero se il punto [i] della richiesta sta su una strada da evitare (il mezzo già in autostrada):
     * l'aggancio senza gli evitamenti cade su una di quelle strade e quello con il divieto cade
     * altrove, più lontano di [AVOIDED_ROAD_MARGIN] m (o non trova nulla). Falso se il punto non è
     * in questo grafo. Si guarda al più entro i 400 m di ORS ([DirectionsLimits.maximumSnappingRadius])
     * anche con un raggio chiesto più largo: "il mezzo sta su quella strada" vale per la strada sotto
     * il punto, non per un'autostrada a pedaggio a 2 km da una tappa di campagna (col raggio largo
     * l'aggancio libero la trovava, quello col divieto no, e il viaggio intero passava alla penalità
     * che agganciava la tappa sulla carreggiata dell'autostrada; ORS col divieto rifiuta il punto o
     * lo aggancia alla strada ordinaria più lontana).
     */
    fun onAvoidedRoad(request: DirectionsRequest, i: Int): Boolean {
        if (request.avoidFeatures == 0) return false
        val (lat, lon) = request.coordinates[i]
        if (!graphs.graph.bounds.contains(lat, lon)) return false
        val o = routeOptions(request)
        val radius = minOf(request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: limits.maximumSnappingRadius, limits.maximumSnappingRadius)
        val heading = request.bearings?.getOrNull(i) ?: Double.NaN
        val freeOptions = o.copy(avoidFeatures = 0, avoidPenalty = 0.0)
        val free = try { snap(graphs.router(freeOptions), freeOptions, i, lat, lon, radius, true, heading) } catch (e: Exception) { return false }
        val name = try { graphs.encoderName(request.profile) } catch (e: IllegalArgumentException) { return false }
        val pe = graphs.encoding.profile(name)
        val avoid = AvoidFeaturesEdgeFilter(graphs.encoding.shared, request.avoidFeatures, pe.profile.category, name == "heavyvehicle")
        val it = graphs.graph.createEdgeIterator().setEdge(free.edge, graphs.graph.nodeA(free.edge))
        if (avoid.accept(it)) return false
        val bannedOptions = o.copy(avoidPenalty = 0.0)
        val banned = try { snap(graphs.router(bannedOptions), bannedOptions, i, lat, lon, radius, true, heading) } catch (e: Exception) { return true }
        return banned.distance > free.distance + AVOIDED_ROAD_MARGIN
    }

    private fun beeline(request: DirectionsRequest): Double {
        var d = 0.0
        for (i in 1 until request.coordinates.size) {
            val (aLat, aLon) = request.coordinates[i - 1]; val (bLat, bLon) = request.coordinates[i]
            d += DistanceCalc.haversine(aLat, aLon, bLat, bLon)
        }
        return d
    }

    /** Il raggio di aggancio massimo del servizio (metri). */
    val maximumSnappingRadius: Double get() = limits.maximumSnappingRadius

    /** Il grafo del servizio. */
    val routingGraph: RoutingGraph get() = graphs

    /**
     * La risposta ORS a [request]. [rc]: l'annullamento (una ricerca annullata lancia
     * [SearchCancelledException][it.navmaster.engine.routing.SearchCancelledException], non
     * [ApiException]), i posti di ricerca e le misure ([RequestStats]).
     */
    fun handle(request: DirectionsRequest, format: ResponseFormat, rc: RequestContext = RequestContext.NONE): Map<String, Any?> {
        val started = System.nanoTime()
        try {
            // "avoid" as a penalty: first as a ban (faster: the roads to avoid are left out of the search,
            // and with a route free of them it is the same route); the penalty only when the vehicle is on
            // a road to avoid or when the ban finds no route
            if (request.avoidPenalty && request.avoidFeatures != 0 && request.coordinates.indices.none { onAvoidedRoad(request, it) }) {
                try { return handle(request.withAvoidPenalty(false), format, rc) } catch (e: ApiException) {
                    if (e.code != RoutingErrorCodes.ROUTE_NOT_FOUND && e.code != RoutingErrorCodes.POINT_NOT_FOUND) throw e
                    rc.stats.setAside("divieto scartato")
                }
            }
            // NavMaster variants (any number of points), with the "avoid" mode of the best route
            request.navAlternatives?.let { opt ->
                if (format != ResponseFormat.GEOJSON) throw ApiException(400, RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, "Parameter 'navmaster.alternatives' needs the GeoJSON format.")
                return variantsResponse(request, opt, rc)
            }
            // alternatives on a trip too long for the plateau method: the via points method (GeoJSON only)
            if (request.alternatives != null && format == ResponseFormat.GEOJSON && request.coordinates.size == 2 && beeline(request) > limits.maximumDistanceAlternativeRoutes) {
                encoder(request)
                return LongAlternatives(listOf(graphs), log = alternativesLog, cancel = rc.cancel) { r, _ -> handle(r, format, rc) }.handle(request, listOf(null, null))
            }
            val encoder = encoder(request)
            val routes = routes(request, rc)
            val docs = routes.map { buildRoute(request, it, encoder, rc.stats) }
            val all = BBox.empty()
            for (d in docs) { all.include(d.bbox[1], d.bbox[0]); all.include(d.bbox[3], d.bbox[2]) }
            val bboxArr = doubleArrayOf(r6(all.minLon), r6(all.minLat), r6(all.maxLon), r6(all.maxLat))
            val metadata = metadata(request)
            return when (format) {
                ResponseFormat.JSON -> linkedMapOf("bbox" to bboxArr, "routes" to docs.map { it.json }, "metadata" to metadata)
                ResponseFormat.GEOJSON -> linkedMapOf("type" to "FeatureCollection", "bbox" to bboxArr, "features" to docs.map { it.feature }, "metadata" to metadata)
            }
        } finally {
            // la chiamata più esterna finisce per ultima: le sue misure restano
            rc.stats.profile = request.profile; rc.stats.points = request.coordinates.size
            rc.stats.totalMs = (System.nanoTime() - started) / 1_000_000
        }
    }

    /** L'encoder del profilo della richiesta (errore ORS se sconosciuto o assente da questo server). */
    internal fun encoder(request: DirectionsRequest): String {
        val encoder = try { graphs.encoderName(request.profile) } catch (e: IllegalArgumentException) {
            throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'profile' has incorrect value of '${request.profile}'.")
        }
        if (encoder !in graphs.profileNames) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Profile '${request.profile}' is not available on this server.")
        return encoder
    }

    /**
     * I percorsi di una richiesta, quelli che [handle] mette nella risposta: convalida, aggancio dei
     * punti, un tratto fra ogni coppia di punti ([solve]) o le alternative ORS. Errori come ORS ([ApiException]).
     * [rc]: le ricerche con un posto di [RequestContext.slots], annullabili; le misure in [RequestContext.stats].
     */
    fun routes(request: DirectionsRequest, rc: RequestContext = RequestContext.NONE): List<List<RoutePath>> {
        val alt = request.alternatives ?: return listOf(solve(request, rc).legs)
        val (routeOptions, beeline) = checked(request)
        if (beeline > limits.maximumDistanceAlternativeRoutes) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The approximated route distance must not be greater than ${limits.maximumDistanceAlternativeRoutes.toLong()} meters for alternative routes.")

        val st = rc.stats
        if (st.points == 0) { st.profile = request.profile; st.points = request.coordinates.size }
        return rc.searching {
            // a long trip with a limited speed: the speed on the times only (see RouteOptions.maximumSpeedTimeOnly)
            val opts = if (fastMaximumSpeed(routeOptions, beeline)) routeOptions.copy(maximumSpeedTimeOnly = true) else routeOptions
            val router = graphs.router(opts)
            router.cancel = rc.cancel
            val snaps = snapAll(request, opts, router, rc, null)
            val tSearch = System.nanoTime()
            val routes = ArrayList<List<RoutePath>>()
            // come ORS: alternative solo fra due punti, in modalità flessibile
            if (snaps.size != 2) throw ApiException(400, RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, "Alternative routes algorithm does not support more than two way points.")
            try { for (p in router.alternatives(snaps[0], snaps[1], alt.targetCount, alt.weightFactor, alt.shareFactor)) routes += listOf(p) } catch (e: RouteNotFoundException) {
                throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points 0 (%.7f %.7f) and 1 (%.7f %.7f).".format(
                    request.coordinates[0].second, request.coordinates[0].first, request.coordinates[1].second, request.coordinates[1].first))
            }
            st.legs += LegStats(0, LegStats.Source.CALC, ms(tSearch), router.lastAlgorithm + " alternative ${routes.size}", router.lastSearch,
                routes.sumOf { r -> r.sumOf { it.edges.size } }, routes.firstOrNull()?.sumOf { it.distance } ?: 0.0, Thread.currentThread().name)
            st.searchWallMs += ms(tSearch)
            routes
        }
    }

    /** Le opzioni di [request] e la sua linea d'aria, dopo i controlli del profilo e dei limiti del servizio. */
    private fun checked(request: DirectionsRequest): Pair<RouteOptions, Double> {
        val routeOptions = routeOptions(request)
        encoder(request)
        if (request.coordinates.size > limits.maximumWaypoints) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The specified number of waypoints must not be greater than ${limits.maximumWaypoints}.")
        var beeline = 0.0
        for (i in 1 until request.coordinates.size) {
            val (aLat, aLon) = request.coordinates[i - 1]; val (bLat, bLon) = request.coordinates[i]
            beeline += DistanceCalc.haversine(aLat, aLon, bLat, bLon)
        }
        val maxDist = if (routeOptions.isStatic) limits.maximumDistance else limits.maximumDistanceDynamicWeights
        if (beeline > maxDist) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The approximated route distance must not be greater than ${maxDist.toLong()} meters.")
        return routeOptions to beeline
    }

    /**
     * Una richiesta risolta: le opzioni del router ([options], come le ha usate la ricerca), i punti
     * agganciati e un tratto fra ogni coppia di punti; la risposta si costruisce a parte ([feature], [handle]).
     */
    class Solved(val request: DirectionsRequest, val options: RouteOptions, val snaps: List<Snap>, val legs: List<RoutePath>)

    /** I tratti già calcolati ([solve]) e gli agganci: vivono quanto il servizio. */
    private val legCache = LegCache()
    private val snapCache = SnapCache()
    /** Le reti autostradali delle varianti (fonte M), per opzioni del router e regione. */
    internal val netCache = it.navmaster.engine.routing.MotorwayNetCache()

    /** Svuota le cache dei tratti, degli agganci e delle reti autostradali (poca memoria: `onTrimMemory` dell'app). */
    fun clearCaches() { legCache.clear(); snapCache.clear(); netCache.clear() }

    /** Diagnostica e prove: lo stato della cache dei tratti. */
    internal val legCacheInfo: String get() = legCache.info
    internal val legCacheSize: Int get() = legCache.size

    /**
     * Aggancia i punti e cerca i tratti di [request] (senza alternative ORS), con le cache: un
     * tratto già calcolato con le stesse opzioni, gli stessi punti agganciati e lo stesso verso
     * d'arrivo (dove conta, [Router.arrivalKey]) non si ricalcola. Con un aiutante libero
     * ([SearchSlots.tryHelper] e [RequestContext.executor]) agganci e tratti vanno su due thread: un
     * tratto che dipende dal precedente (`pass_through` a metà arco) mentre l'altro thread cerca quello
     * si cerca senza il verso d'arrivo e poi si controlla ([LegStats.Source.SPEC_OK] o [LegStats.Source.RECALC]).
     * Errori come ORS ([ApiException]): del punto, o del tratto, d'indice minimo.
     */
    fun solve(request: DirectionsRequest, rc: RequestContext = RequestContext.NONE): Solved {
        val (routeOptions, beeline) = checked(request)
        val st = rc.stats
        if (st.points == 0) { st.profile = request.profile; st.points = request.coordinates.size }
        return rc.searching {
            // anche una richiesta tutta in cache si ferma se annullata
            rc.cancel.check()
            // a long trip with a limited speed: the speed on the times only (see RouteOptions.maximumSpeedTimeOnly)
            val opts = if (fastMaximumSpeed(routeOptions, beeline)) routeOptions.copy(maximumSpeedTimeOnly = true) else routeOptions
            val router = graphs.router(opts)
            router.cancel = rc.cancel
            val help = Help(rc, opts)
            val snaps = snapAll(request, opts, router, rc, help)
            Solved(request, opts, snaps, legsOf(request, opts, router, snaps, rc, help))
        }
    }

    /**
     * [solve] con il modo «evita» di [handle]: gli «evita» come penalità prima come divieto, se
     * nessun punto sta su una strada da evitare e il divieto trova il percorso. [Solved.request] è la
     * richiesta usata (quella da dare a [feature]). I pezzi di un viaggio tra Paesi.
     */
    internal fun solveAsHandled(request: DirectionsRequest, rc: RequestContext = RequestContext.NONE): Solved {
        if (request.avoidPenalty && request.avoidFeatures != 0 && request.coordinates.indices.none { onAvoidedRoad(request, it) }) {
            try { return solve(request.withAvoidPenalty(false), rc) } catch (e: ApiException) {
                if (e.code != RoutingErrorCodes.ROUTE_NOT_FOUND && e.code != RoutingErrorCodes.POINT_NOT_FOUND) throw e
                rc.stats.setAside("divieto scartato")
            }
        }
        return solve(request, rc)
    }

    /**
     * L'aggancio di un punto ([Router.snap]) dalla cache degli agganci, se c'è: stesse opzioni del
     * router ([RoutingGraph.routerKey]), stessa coordinata, raggio, controllo delle reti isolate e
     * direzione. [i]: l'indice del punto, per il messaggio di [PointNotFoundException] (mai in cache).
     */
    fun snap(router: Router, options: RouteOptions, i: Int, lat: Double, lon: Double, radius: Double, checkNetwork: Boolean, heading: Double, useCache: Boolean = true): Snap =
        if (!useCache) router.snap(lat, lon, radius, i, checkNetwork, heading)
        else snapCache.get(SnapCacheKey(graphs.routerKey(options), lat.toBits(), lon.toBits(), radius, checkNetwork, heading.toBits())) { router.snap(lat, lon, radius, i, checkNetwork, heading) }

    /** Il risultato di una voce di [inParallel] finita con un errore (da lanciare nell'ordine delle voci). */
    private class Failure(val e: RuntimeException)

    /**
     * Il thread aiutante di una richiesta: il posto da aiutante ([SearchSlots.tryHelper]) si chiede a
     * ogni [inParallel] che ne ha bisogno e si lascia appena l'aiutante ha finito (un posto fermo non
     * fa aspettare una richiesta MAIN); il router dell'aiutante, creato e usato solo sul thread
     * dell'aiutante (un [inParallel] alla volta), resta per il successivo.
     */
    private inner class Help(private val rc: RequestContext, private val options: RouteOptions) {
        private var router: Router? = null

        /** Un posto da aiutante, se c'è l'executor e un posto libero (senza attesa); va chiuso. */
        fun slot(): SearchSlots.Helper? = if (rc.executor != null) rc.slots?.tryHelper(rc.priority, rc.cancel) else null

        /** Il router dell'aiutante (sul suo thread soltanto), col token del posto [token]. */
        fun router(token: CancelToken): Router = (router ?: graphs.router(options).also { router = it }).also { it.cancel = token }
    }

    /**
     * Le voci [order] (indici in [out]), nell'ordine dato, su questo thread e, con il posto da
     * aiutante [slot], anche sull'aiutante: ogni thread prende la prima voce non ancora presa la cui
     * voce [after] (d'indice minore, -1 se nessuna) è finita o, se non ce n'è, in corso sull'altro
     * thread; mai una voce prima della sua [after] ancora da prendere (un thread solo le fa in ordine
     * di dipendenza). [work] la fa col router del thread. In [out] il risultato di ogni voce, o il suo
     * errore ([Failure]); l'annullamento invece esce. Un aiutante annullato (prelazionato) si ferma e
     * lascia le sue voci a questo thread. Ritorna (o esce) sempre con l'aiutante fermo e [slot] chiuso.
     */
    private fun inParallel(order: List<Int>, out: AtomicReferenceArray<Any?>, router: Router, help: Help?, slot: SearchSlots.Helper?, rc: RequestContext,
                           after: (Int) -> Int = { -1 }, work: (Router, Int) -> Any) {
        // lo stato delle voci: fatta (o fuori da [order]), da prendere, in corso
        val state = IntArray(out.length())
        for (i in order) state[i] = ITEM_FREE
        fun take(): Int = synchronized(state) {
            var behind = -1
            for (i in order) {
                if (state[i] != ITEM_FREE) continue
                val a = after(i)
                val s = if (a < 0) ITEM_DONE else state[a]
                if (s == ITEM_DONE) { state[i] = ITEM_RUNNING; return i }
                if (s == ITEM_RUNNING && behind < 0) behind = i
            }
            if (behind >= 0) state[behind] = ITEM_RUNNING
            behind
        }
        fun one(r: Router, i: Int) {
            try {
                out.set(i, try { work(r, i) } catch (e: SearchCancelledException) { throw e } catch (e: RuntimeException) { Failure(e) })
                synchronized(state) { state[i] = ITEM_DONE }
            } catch (e: SearchCancelledException) { synchronized(state) { state[i] = ITEM_FREE }; throw e }
        }
        // [stop]: l'aiutante, prelazionato fra una voce e l'altra (anche prima di partire), non ne prende altre
        fun loop(r: Router, stop: CancelToken? = null) { while (true) { stop?.check(); val i = take(); if (i < 0) return; one(r, i) } }
        val executor = rc.executor
        // l'aiutante parte solo se nessuno l'ha fermato prima (in coda dietro altro lavoro: allora non serve più)
        val claimed = AtomicBoolean(false)
        val done = CountDownLatch(1)
        var helperError: Throwable? = null
        var submitted = false
        try {
            if (slot != null && executor != null) submitted = try {
                executor.execute {
                    if (claimed.compareAndSet(false, true)) try {
                        slot.run { loop(help!!.router(slot.token), slot.token) }
                    } catch (e: SearchCancelledException) {
                        // prelazionato o annullato: le voci restano a questo thread (che vede da sé un annullamento)
                    } catch (e: Throwable) { helperError = e } finally { slot.close(); done.countDown() }
                }
                true
            } catch (e: java.util.concurrent.RejectedExecutionException) { false }
            // senza aiutante il posto non serve
            if (!submitted) slot?.close()
            try {
                loop(router)
            } catch (e: Throwable) {
                slot?.token?.cancel()
                throw e
            } finally {
                if (submitted && !claimed.compareAndSet(false, true)) awaitQuietly(done)
            }
        } finally { slot?.close() }
        helperError?.let { throw it }
        // le voci lasciate da un aiutante fermato, in ordine d'indice (dopo la loro [after])
        for (i in order.sorted()) if (out.get(i) == null) { rc.cancel.check(); one(router, i) }
    }

    private fun awaitQuietly(l: CountDownLatch) {
        var interrupted = false
        while (true) { try { l.await(); break } catch (e: InterruptedException) { interrupted = true } }
        if (interrupted) Thread.currentThread().interrupt()
    }

    /** I punti agganciati di [request] con [router] (dalla cache dove ci sono), il primo non trovato come errore ORS. */
    private fun snapAll(request: DirectionsRequest, opts: RouteOptions, router: Router, rc: RequestContext, help: Help?): List<Snap> {
        val st = rc.stats
        val tSnap = System.nanoTime()
        val n = request.coordinates.size
        val key = if (rc.useCache) graphs.routerKey(opts) else null
        fun radius(i: Int) = request.radiuses?.get(i)?.takeIf { it > 0 } ?: limits.maximumSnappingRadius
        fun cacheKey(k: RouteOptions, i: Int) = SnapCacheKey(k, request.coordinates[i].first.toBits(), request.coordinates[i].second.toBits(), radius(i),
            request.exactPoints?.getOrNull(i) != true, (request.bearings?.getOrNull(i) ?: Double.NaN).toBits())
        val out = AtomicReferenceArray<Any?>(n)
        val todo = ArrayList<Int>()
        for (i in 0 until n) { val s = key?.let { snapCache.peek(cacheKey(it, i)) }; if (s != null) out.set(i, s) else todo += i }
        inParallel(todo, out, router, help, if (todo.size >= 2) help?.slot() else null, rc) { r, i ->
            val (lat, lon) = request.coordinates[i]
            try {
                val s = r.snap(lat, lon, radius(i), i, checkNetwork = request.exactPoints?.getOrNull(i) != true, heading = request.bearings?.getOrNull(i) ?: Double.NaN)
                if (key != null) snapCache.get(cacheKey(key, i)) { s } else s
            } catch (e: PointNotFoundException) {
                throw ApiException(404, RoutingErrorCodes.POINT_NOT_FOUND, "Could not find routable point within a radius of ${radius(i).toLong()} meters of specified coordinate ${e.pointIndex}: %.7f %.7f.".format(lon, lat))
            }
        }
        st.snapMs += ms(tSnap); st.snapsNew += todo.size; st.snapsCached += n - todo.size
        return List(n) { i -> when (val v = out.get(i)) { is Failure -> throw v.e; else -> v as Snap } }
    }

    /**
     * Un tratto di [legsOf]: il percorso, il verso d'arrivo con cui è stato cercato ([arrival]:
     * `Router.arrivalKey`, -1 senza vincolo), da dove viene, le sue misure.
     */
    private class Leg(val path: RoutePath, val arrival: Int, val stats: LegStats)

    /**
     * I tratti fra i punti [snaps] (vedi [solve]): prima dalla cache, in ordine; poi i mancanti, in
     * ordine con un thread solo, dal più lungo con due (un dipendente che la cache conosce dopo il
     * suo precedente, di nuovo dalla cache se può); infine, in ordine, la verifica dei tratti
     * dipendenti cercati con un verso d'arrivo diverso da quello del tratto precedente finale (senza,
     * o da un precedente poi ricalcolato).
     */
    private fun legsOf(request: DirectionsRequest, opts: RouteOptions, router: Router, snaps: List<Snap>, rc: RequestContext, help: Help): List<RoutePath> {
        val st = rc.stats
        val tSearch = System.nanoTime()
        val m = snaps.size - 1
        val key = if (rc.useCache) graphs.routerKey(opts) else null
        val sk = if (key != null) snaps.map { SnapKey.of(it) } else emptyList()
        // ORS `bearings`: la direzione di partenza dal punto i e d'arrivo all'ultimo
        fun fromHeading(i: Int) = request.bearings?.getOrNull(i) ?: Double.NaN
        fun toHeading(i: Int) = if (i == m - 1) request.bearings?.getOrNull(i + 1) ?: Double.NaN else Double.NaN
        fun legKey(i: Int, arrival: Int) = LegKey(key!!, sk[i], sk[i + 1], arrival, fromHeading(i).toBits(), toHeading(i).toBits())
        // continue_straight / pass_through a metà arco percorribile nei due versi: il tratto i prosegue nel verso d'arrivo del tratto i−1
        val dependent = BooleanArray(m) { i -> i > 0 && request.passThroughAt(i) && !router.directionFree(snaps[i]) }
        val legs = AtomicReferenceArray<Any?>(m)
        fun pathAt(i: Int): RoutePath? = (legs.get(i) as? Leg)?.path
        fun cached(i: Int, arrival: Int, source: LegStats.Source = LegStats.Source.CACHE): Leg? =
            legCache.get(legKey(i, arrival))?.let { p -> Leg(p, arrival, LegStats(i, source, 0, "cache", "", p.edges.size, p.distance, "")) }
        /** Il tratto [i] cercato con [r] (col verso d'arrivo [arrival], se c'è), messo in cache. */
        fun search(r: Router, i: Int, arrival: PathEdge?, source: LegStats.Source): Leg {
            rc.cancel.check()
            val t0 = System.nanoTime()
            val leg = try { r.route(snaps[i], snaps[i + 1], arrival, fromHeading(i), toHeading(i)) } catch (e: RouteNotFoundException) {
                throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points $i (%.7f %.7f) and ${i + 1} (%.7f %.7f).".format(
                    request.coordinates[i].second, request.coordinates[i].first, request.coordinates[i + 1].second, request.coordinates[i + 1].first))
            }
            // le fasi della ricerca si leggono subito: la prossima le sostituisce
            val stats = LegStats(i, source, ms(t0), r.lastAlgorithm, r.lastSearch, leg.edges.size, leg.distance, Thread.currentThread().name)
            // in cache col verso d'arrivo che conta (senza: come un tratto che non dipende dall'arrivo)
            val bit = r.arrivalKey(snaps[i], arrival)
            if (key != null) legCache.put(legKey(i, bit), leg)
            return Leg(leg, bit, stats)
        }
        /** L'arco d'arrivo del tratto [i]: l'ultimo del precedente, dove si prosegue nel verso d'arrivo. */
        fun arrival(i: Int): PathEdge? = if (i > 0 && request.passThroughAt(i)) pathAt(i - 1)?.edges?.lastOrNull() else null

        // 1. dalla cache, in ordine: un tratto che dipende dal precedente quando quello è noto
        if (key != null) for (i in 0 until m) {
            val a = if (!dependent[i]) -1 else { val prev = pathAt(i - 1) ?: continue; router.arrivalKey(snaps[i], prev.edges.lastOrNull()) }
            cached(i, a)?.let { legs.set(i, it) }
        }
        // 2. i mancanti. Un tratto dipendente col precedente mancante, se la cache ne ha una versione
        // (o se aspetta già il suo precedente), si prende dopo, in ordine: col precedente noto è di
        // solito un colpo in cache (una via aggiunta cambia 2 tratti, non tutti quelli dopo)
        val missing = (0 until m).filter { legs.get(it) == null }
        fun inCache(i: Int) = key != null && (-1..1).any { legCache.contains(legKey(i, it)) }
        val later = BooleanArray(m)
        for (i in missing) if (dependent[i] && legs.get(i - 1) == null && (later[i - 1] || inCache(i))) later[i] = true
        /** Il tratto dipendente [i] dalla cache, col verso d'arrivo del precedente già noto. */
        fun hit(r: Router, i: Int): Leg? = if (key != null && dependent[i] && pathAt(i - 1) != null) cached(i, r.arrivalKey(snaps[i], arrival(i))) else null
        val now = missing.filter { !later[it] }
        val slot = if (now.size >= 2) help.slot() else null
        if (slot == null) {
            // un thread solo: in ordine, ognuno col verso d'arrivo del precedente (come sempre)
            for (i in now) legs.set(i, hit(router, i) ?: search(router, i, arrival(i), LegStats.Source.CALC))
        } else {
            // due thread: prima i più lunghi, un tratto dipendente mai prima del precedente ancora da prendere;
            // col precedente in corso sull'altro thread si cerca senza il verso d'arrivo (da verificare)
            val order = now.sortedByDescending { i -> DistanceCalc.haversine(snaps[i].snappedLat, snaps[i].snappedLon, snaps[i + 1].snappedLat, snaps[i + 1].snappedLon) }
            inParallel(order, legs, router, help, slot, rc, after = { i -> if (dependent[i]) i - 1 else -1 }) { r, i ->
                val prev = if (i > 0) pathAt(i - 1) else null
                hit(r, i) ?: search(r, i, if (dependent[i] && prev == null) null else arrival(i), LegStats.Source.CALC)
            }
        }
        // i rimandati, in ordine (dopo un errore del precedente non servono: il passo 3 si ferma prima)
        for (i in missing) if (later[i] && legs.get(i - 1) is Leg) { rc.cancel.check(); legs.set(i, hit(router, i) ?: search(router, i, arrival(i), LegStats.Source.CALC)) }
        // 3. in ordine: l'errore del tratto d'indice minimo; i tratti dipendenti cercati con un altro verso d'arrivo, verificati
        val out = ArrayList<RoutePath>(m)
        for (i in 0 until m) {
            val l = when (val v = legs.get(i)) { is Failure -> throw v.e; else -> v as Leg }
            val a = if (dependent[i]) arrival(i) else null
            val bit = if (dependent[i]) router.arrivalKey(snaps[i], a) else -1
            // cercato senza il verso d'arrivo, o con quello di un precedente poi ricalcolato
            if (l.arrival != bit) {
                val first = l.path.edges.firstOrNull()
                // senza vincolo vale se l'arrivo non lo lega, o se parte già nel verso d'arrivo (il migliore anche col vincolo)
                val fixed = if (l.arrival == -1 && (first == null || (first.edge == snaps[i].edge && first.reverse == (bit == 1)))) {
                    if (key != null) legCache.put(legKey(i, bit), l.path)
                    Leg(l.path, bit, LegStats(i, LegStats.Source.SPEC_OK, l.stats.ms, l.stats.algorithm, l.stats.search, l.stats.edges, l.stats.meters, l.stats.thread))
                } else (if (key != null) cached(i, bit) else null) ?: search(router, i, a, LegStats.Source.RECALC)
                legs.set(i, fixed)
            }
            out += (legs.get(i) as Leg).path
        }
        for (i in 0 until m) st.legs += (legs.get(i) as Leg).stats
        st.searchWallMs += ms(tSearch)
        return out
    }

    /**
     * La feature GeoJSON di un percorso ([legs] di [request]), come in [handle]; con [variant] anche
     * `properties.navmaster.variant`. [encoder]: il nome interno del profilo.
     */
    internal fun feature(request: DirectionsRequest, legs: List<RoutePath>, encoder: String, variant: Map<String, Any?>?, stats: RequestStats = RequestStats()): Map<String, Any?> {
        val f = buildRoute(request, legs, encoder, stats).feature
        if (variant != null) {
            @Suppress("UNCHECKED_CAST")
            val props = f["properties"] as MutableMap<String, Any?>
            @Suppress("UNCHECKED_CAST")
            val nm = props["navmaster"] as? MutableMap<String, Any?> ?: linkedMapOf<String, Any?>().also { props["navmaster"] = it }
            nm["variant"] = variant
        }
        return f
    }

    /**
     * La risposta GeoJSON di `navmaster.alternatives`: le feature del migliore (se non escluso) e
     * delle varianti tenute ([NavAlternativesEngine]), ognuna con `properties.navmaster.variant` e
     * un way_point per punto; in `metadata.navmaster.alternatives` quante chieste e trovate, i ms,
     * la nota e il riepilogo dei candidati. Le feature si costruiscono su due thread se c'è l'aiutante.
     */
    private fun variantsResponse(request: DirectionsRequest, opt: NavAlternativesOptions, rc: RequestContext): Map<String, Any?> {
        val encoder = encoder(request)
        val res = NavAlternativesEngine(this, rc).run(request, opt)
        val jobs = ArrayList<Pair<List<RoutePath>, Map<String, Any?>>>()
        if (!opt.excludeBest) jobs += res.best.legs to res.bestJson()
        for (a in res.variants) jobs += a.legs to res.json(a)
        val out = arrayOfNulls<Map<String, Any?>>(jobs.size)
        fun build(parity: Int, stats: RequestStats) { for (i in jobs.indices) if (i % 2 == parity) out[i] = feature(request, jobs[i].first, encoder, jobs[i].second, stats) }
        onTwoThreads(rc, rc.cancel, if (jobs.size >= 2) helperSlot(rc, rc.cancel) else null, { build(0, rc.stats) }, if (jobs.size >= 2) { _ -> build(1, RequestStats()) } else null)
        val features = out.map { it!! }
        val all = BBox.empty()
        for (f in features) { val b = f["bbox"] as DoubleArray; all.include(b[1], b[0]); all.include(b[3], b[2]) }
        val metadata = LinkedHashMap(metadata(request))
        metadata["navmaster"] = linkedMapOf("alternatives" to linkedMapOf(
            "asked" to opt.count, "found" to 1 + res.variants.size, "variants" to res.variants.size, "ms" to res.ms, "note" to res.note, "candidates" to res.summary))
        val bboxArr = if (features.isEmpty()) doubleArrayOf() else doubleArrayOf(r6(all.minLon), r6(all.minLat), r6(all.maxLon), r6(all.maxLat))
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bboxArr, "features" to features, "metadata" to metadata)
    }

    private fun ms(t0: Long) = (System.nanoTime() - t0) / 1_000_000

    /** Un percorso nelle due forme (oggetto `routes[i]` del JSON, `features[i]` del GeoJSON) e il suo bbox. */
    private class RouteDoc(val json: Map<String, Any?>, val feature: Map<String, Any?>, val bbox: DoubleArray)

    private fun buildRoute(request: DirectionsRequest, legs: List<RoutePath>, encoder: String, stats: RequestStats): RouteDoc {
        val started = System.nanoTime()
        var instructionsNanos = 0L
        val pe = graphs.encoding.profile(encoder)
        val instructionsBuilder = InstructionsBuilder(graphs.graph, graphs.encoding, pe)
        val stepBuilder = OrsStepBuilder(StepOptions(request.language, request.units, request.maneuvers, request.htmlFormat))
        val geometry = PointList(256)
        val wayPoints = ArrayList<Int>().also { it.add(0) }
        val segments = ArrayList<Map<String, Any?>>()
        val bbox = BBox.empty()
        var totalDistance = 0.0; var totalDuration = 0.0
        val tInstructions = System.nanoTime()
        val allInstructions = legs.map { instructionsBuilder.build(it) }
        instructionsNanos += System.nanoTime() - tInstructions
        for ((li, leg) in legs.withIndex()) {
            val pts = leg.points()
            val startIndex = if (li == 0) 0 else geometry.size - 1
            for (i in (if (li == 0) 0 else 1) until pts.size) { geometry.add(pts.lat(i), pts.lon(i)); bbox.include(pts.lat(i), pts.lon(i)) }
            if (pts.size == 1 && li == 0) { geometry.add(pts.lat(0), pts.lon(0)); bbox.include(pts.lat(0), pts.lon(0)) }
            wayPoints.add(geometry.size - 1)
            val distance = roundForUnits(leg.distance * request.units.perMeter, request.units)
            val duration = OrsStepBuilder.round1(leg.millis / 1000.0)
            totalDistance += distance; totalDuration += duration
            val seg = LinkedHashMap<String, Any?>()
            seg["distance"] = distance; seg["duration"] = duration
            if (request.instructions) {
                val tSteps = System.nanoTime()
                val next = allInstructions.getOrNull(li + 1)?.firstOrNull()?.points
                val steps = stepBuilder.build(allInstructions[li], startIndex, request.coordinates.last(), next)
                seg["steps"] = steps.map { stepJson(it, request) }
                instructionsNanos += System.nanoTime() - tSteps
            }
            if ("detourfactor" in request.attributes) {
                val bee = DistanceCalc.haversine(pts.lat(0), pts.lon(0), pts.lat(pts.size - 1), pts.lon(pts.size - 1))
                seg["detourfactor"] = if (bee > 0) Math.round(leg.distance / bee * 100) / 100.0 else 0.0
            }
            if ("avgspeed" in request.attributes) seg["avgspeed"] = if (duration > 0) Math.round(distance / (if (request.units == DistanceUnit.METERS) 1000.0 else 1.0) / (duration / 3600) * 100) / 100.0 else 0.0
            segments += seg
        }
        if ("percentage" in request.attributes) for (seg in segments) (seg as MutableMap<String, Any?>)["percentage"] = if (totalDistance > 0) Math.round((seg["distance"] as Double) * 100 / totalDistance * 100) / 100.0 else 0.0
        val summary = linkedMapOf<String, Any?>("distance" to roundForUnits(totalDistance, request.units), "duration" to totalDuration)
        val bboxArr = doubleArrayOf(r6(bbox.minLon), r6(bbox.minLat), r6(bbox.maxLon), r6(bbox.maxLat))
        val extras = if (request.extraInfo.isNotEmpty()) ExtraInfoBuilder(graphs.graph, graphs.encoding, pe).build(request.extraInfo, legs, request.units.perMeter, request.units.decimals) else null
        val navmaster = if (request.navmasterRoads) {
            val rf = RoadFactsBuilder(graphs.graph, graphs.encoding, pe, graphs.guidance)
            val tRoads = System.nanoTime()
            val nm = linkedMapOf<String, Any?>("roads" to rf.build(legs))
            stats.roadsMs += ms(tRoads)
            val tPoints = System.nanoTime()
            graphs.points?.let { nm["points"] = rf.pointsAlong(legs, it) }
            stats.pointsMs += ms(tPoints)
            nm
        } else null

        val route = LinkedHashMap<String, Any?>()
        route["summary"] = summary
        route["segments"] = segments
        route["bbox"] = bboxArr
        if (request.geometry) {
            route["geometry"] = encodeGeometry(geometry)
            route["way_points"] = wayPoints
        }
        if (extras != null) route["extras"] = extras
        if (navmaster != null) route["navmaster"] = navmaster

        val coords = ArrayList<DoubleArray>(geometry.size)
        for (i in 0 until geometry.size) coords.add(doubleArrayOf(r6(geometry.lon(i)), r6(geometry.lat(i))))
        val properties = linkedMapOf<String, Any?>("segments" to segments, "way_points" to wayPoints, "summary" to summary)
        if (extras != null) properties["extras"] = extras
        if (navmaster != null) properties["navmaster"] = navmaster
        val feature = linkedMapOf<String, Any?>(
            "bbox" to bboxArr, "type" to "Feature",
            "properties" to properties,
            "geometry" to linkedMapOf("coordinates" to coords, "type" to "LineString"),
        )
        stats.instructionsMs += instructionsNanos / 1_000_000
        stats.buildMs += ms(started)
        return RouteDoc(route, feature, bboxArr)
    }

    private fun stepJson(s: RouteStep, request: DirectionsRequest): Map<String, Any?> {
        val m = LinkedHashMap<String, Any?>()
        m["distance"] = s.distance; m["duration"] = s.duration; m["type"] = s.type.code
        m["instruction"] = s.instruction; m["name"] = s.name.ifEmpty { "-" }
        if (s.exitNumber != -1) m["exit_number"] = s.exitNumber
        m["way_points"] = listOf(s.wayPointStart, s.wayPointEnd)
        s.maneuver?.let { mv -> m["maneuver"] = linkedMapOf("location" to doubleArrayOf(r6(mv.lon), r6(mv.lat)), "bearing_before" to mv.bearingBefore, "bearing_after" to mv.bearingAfter) }
        return m
    }

    private fun encodeGeometry(pl: PointList): String {
        val lats = DoubleArray(pl.size) { pl.lat(it) }; val lons = DoubleArray(pl.size) { pl.lon(it) }
        return PolylineEncoder.encode(lats, lons, pl.size)
    }

    private fun metadata(request: DirectionsRequest): Map<String, Any?> {
        val m = LinkedHashMap<String, Any?>()
        request.id?.let { m["id"] = it }
        m["attribution"] = attribution
        m["service"] = "routing"
        m["timestamp"] = System.currentTimeMillis()
        val query = LinkedHashMap<String, Any?>(request.echo)
        query["profile"] = request.profile
        query["profileName"] = request.profile
        if (!query.containsKey("preference")) query["preference"] = request.preference.name.lowercase()
        query["format"] = "json"
        m["query"] = query
        m["engine"] = engineInfo()
        return m
    }

    fun engineInfo(): Map<String, Any?> = linkedMapOf("version" to version, "build_date" to BUILD_DATE, "graph_date" to graphDate())

    private fun graphDate(): String = runCatching { Instant.ofEpochMilli(java.io.File(graphs.dir, "properties").lastModified()).toString() }.getOrDefault("0000-00-00T00:00:00Z")

    fun errorBody(e: ApiException): Map<String, Any?> =
        linkedMapOf("error" to linkedMapOf("code" to e.code, "message" to e.message), "info" to linkedMapOf("engine" to engineInfo(), "timestamp" to System.currentTimeMillis()))

    /** `/v2/status` nello stile ORS. */
    fun status(): Map<String, Any?> {
        val profiles = LinkedHashMap<String, Any?>()
        for (pe in graphs.encoding.profiles) {
            val orsName = if (pe.profile.name == "car-ors") "driving-car" else "driving-hgv"
            profiles[orsName] = linkedMapOf(
                "encoder_name" to pe.profile.name,
                "limits" to linkedMapOf("maximum_distance" to limits.maximumDistance, "maximum_waypoints" to limits.maximumWaypoints, "maximum_distance_dynamic_weights" to limits.maximumDistanceDynamicWeights),
                "preparations" to graphs.chStorages.keys.filter { it.startsWith(pe.profile.name) },
            )
        }
        return linkedMapOf(
            "engine" to engineInfo(),
            "services" to listOf("routing"),
            "languages" to InstructionTranslator.supportedLanguages,
            "profiles" to profiles,
            "graph" to linkedMapOf("nodes" to graphs.graph.nodeCount, "edges" to graphs.graph.edgeCount, "bbox" to graphs.graph.bounds.let { b -> doubleArrayOf(b.minLon, b.minLat, b.maxLon, b.maxLat) }),
        )
    }

    companion object {
        /** Quanto più lontano deve cadere l'aggancio col divieto perché il punto sia "su una strada da evitare" (metri). */
        const val AVOIDED_ROAD_MARGIN = 15.0
        /** Oltre questa distanza in linea d'aria (m) maximum_speed vale sui tempi soltanto. */
        const val FAST_MAXIMUM_SPEED_BEELINE = 60_000.0
        /** Lo stato di una voce di [inParallel]: fatta (o fuori dalle voci), da prendere, in corso. */
        private const val ITEM_DONE = 0
        private const val ITEM_FREE = 1
        private const val ITEM_RUNNING = 2
        val BUILD_DATE: String = Instant.now().toString()
        fun r6(v: Double) = Math.round(v * 1e6) / 1e6
        fun roundForUnits(v: Double, units: DistanceUnit): Double { val f = Math.pow(10.0, units.decimals.toDouble()); return Math.round(v * f) / f }
    }
}

/** Estensione NavMaster: le strade intorno a un punto, dal grafo (vedi [RoadFactsBuilder.around]). */
class RoadsService(private val graphs: RoutingGraph) {
    fun around(profile: String, lat: Double, lon: Double, radiusMeters: Double): Map<String, Any?> {
        val encoder = try { graphs.encoderName(profile) } catch (e: IllegalArgumentException) {
            throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'profile' has incorrect value of '$profile'.")
        }
        val pe = graphs.encoding.profile(encoder)
        val r = radiusMeters.coerceIn(10.0, 5000.0)
        return linkedMapOf("roads" to RoadFactsBuilder(graphs.graph, graphs.encoding, pe, graphs.guidance).around(graphs.index, lat, lon, r))
    }
}
