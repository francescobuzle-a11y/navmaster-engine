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
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingMethod
import java.time.Instant

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
) {
    companion object {
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
            if (maximumSpeed < MAXIMUM_SPEED_LOWER_BOUND) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'maximum_speed' must be at least ${MAXIMUM_SPEED_LOWER_BOUND.toInt()} km/h.")
            return DirectionsRequest(
                profile, coordinates, preference, units, body["language"]?.toString() ?: "en",
                instructions = body["instructions"] as? Boolean ?: true, htmlFormat = htmlFormat,
                geometry = body["geometry"] as? Boolean ?: true, maneuvers = body["maneuvers"] as? Boolean ?: false,
                avoidFeatures = avoid, vehicleType = vehicleType, vehicleParameters = params?.takeIf { it.hasAttributes },
                avoidPolygons = polygons, maximumSpeed = maximumSpeed,
                continueStraight = body["continue_straight"] as? Boolean ?: false,
                navmasterRoads = (body["navmaster"] as? Map<*, *>)?.get("roads") == true,
                alternatives = (body["alternative_routes"] as? Map<*, *>)?.let { a ->
                    val count = (a["target_count"] as? Number)?.toInt() ?: 2
                    if (count !in 2..3) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'alternative_routes.target_count' has incorrect value of '$count' (2 or 3).")
                    AlternativeOptions(count, (a["weight_factor"] as? Number)?.toDouble() ?: 1.4, (a["share_factor"] as? Number)?.toDouble() ?: 0.6)
                },
                radiuses = radiuses, id = body["id"]?.toString(), attributes = attributes, extraInfo = extraInfo, echo = body,
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

    fun handle(request: DirectionsRequest, format: ResponseFormat): Map<String, Any?> {
        val routeOptions = RouteOptions(
            profile = request.profile, weighting = request.preference, avoidFeatures = request.avoidFeatures,
            vehicleType = request.vehicleType, vehicleParameters = request.vehicleParameters,
            avoidPolygons = request.avoidPolygons, maximumSpeed = request.maximumSpeed,
            continueStraight = request.continueStraight,
        )
        val encoder = try { graphs.encoderName(request.profile) } catch (e: IllegalArgumentException) {
            throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Parameter 'profile' has incorrect value of '${request.profile}'.")
        }
        if (encoder !in graphs.profileNames) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, "Profile '${request.profile}' is not available on this server.")
        if (request.coordinates.size > limits.maximumWaypoints) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The specified number of waypoints must not be greater than ${limits.maximumWaypoints}.")
        var beeline = 0.0
        for (i in 1 until request.coordinates.size) {
            val (aLat, aLon) = request.coordinates[i - 1]; val (bLat, bLon) = request.coordinates[i]
            beeline += DistanceCalc.haversine(aLat, aLon, bLat, bLon)
        }
        val maxDist = if (routeOptions.isStatic) limits.maximumDistance else limits.maximumDistanceDynamicWeights
        if (beeline > maxDist) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The approximated route distance must not be greater than ${maxDist.toLong()} meters.")
        if (request.alternatives != null && beeline > limits.maximumDistanceAlternativeRoutes) throw ApiException(400, RoutingErrorCodes.REQUEST_EXCEEDS_SERVER_LIMIT, "The approximated route distance must not be greater than ${limits.maximumDistanceAlternativeRoutes.toLong()} meters for alternative routes.")

        val router = graphs.router(routeOptions)
        val snaps = request.coordinates.mapIndexed { i, (lat, lon) ->
            val radius = request.radiuses?.get(i)?.takeIf { it > 0 } ?: limits.maximumSnappingRadius
            try { router.snap(lat, lon, radius, i) } catch (e: PointNotFoundException) {
                throw ApiException(404, RoutingErrorCodes.POINT_NOT_FOUND, "Could not find routable point within a radius of ${radius.toLong()} meters of specified coordinate ${e.pointIndex}: %.7f %.7f.".format(lon, lat))
            }
        }
        val alt = request.alternatives
        val routes = ArrayList<List<RoutePath>>()
        if (alt != null) {
            // come ORS: alternative solo fra due punti, in modalità flessibile
            if (snaps.size != 2) throw ApiException(400, RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, "Alternative routes algorithm does not support more than two way points.")
            try { for (p in router.alternatives(snaps[0], snaps[1], alt.targetCount, alt.weightFactor, alt.shareFactor)) routes += listOf(p) } catch (e: RouteNotFoundException) {
                throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points 0 (%.7f %.7f) and 1 (%.7f %.7f).".format(
                    request.coordinates[0].second, request.coordinates[0].first, request.coordinates[1].second, request.coordinates[1].first))
            }
        } else {
            val legs = ArrayList<RoutePath>()
            for (i in 1 until snaps.size) {
                // continue_straight: il tratto prosegue nel verso con cui il precedente è arrivato al via-point
                val arrival = if (request.continueStraight) legs.lastOrNull()?.edges?.lastOrNull() else null
                try { legs += router.route(snaps[i - 1], snaps[i], arrival) } catch (e: RouteNotFoundException) {
                    throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${i - 1} (%.7f %.7f) and $i (%.7f %.7f).".format(
                        request.coordinates[i - 1].second, request.coordinates[i - 1].first, request.coordinates[i].second, request.coordinates[i].first))
                }
            }
            routes += legs
        }
        val docs = routes.map { buildRoute(request, it, encoder) }
        val all = BBox.empty()
        for (d in docs) { all.include(d.bbox[1], d.bbox[0]); all.include(d.bbox[3], d.bbox[2]) }
        val bboxArr = doubleArrayOf(r6(all.minLon), r6(all.minLat), r6(all.maxLon), r6(all.maxLat))
        val metadata = metadata(request)
        return when (format) {
            ResponseFormat.JSON -> linkedMapOf("bbox" to bboxArr, "routes" to docs.map { it.json }, "metadata" to metadata)
            ResponseFormat.GEOJSON -> linkedMapOf("type" to "FeatureCollection", "bbox" to bboxArr, "features" to docs.map { it.feature }, "metadata" to metadata)
        }
    }

    /** Un percorso nelle due forme (oggetto `routes[i]` del JSON, `features[i]` del GeoJSON) e il suo bbox. */
    private class RouteDoc(val json: Map<String, Any?>, val feature: Map<String, Any?>, val bbox: DoubleArray)

    private fun buildRoute(request: DirectionsRequest, legs: List<RoutePath>, encoder: String): RouteDoc {
        val pe = graphs.encoding.profile(encoder)
        val instructionsBuilder = InstructionsBuilder(graphs.graph, graphs.encoding, pe)
        val stepBuilder = OrsStepBuilder(StepOptions(request.language, request.units, request.maneuvers, request.htmlFormat))
        val geometry = PointList(256)
        val wayPoints = ArrayList<Int>().also { it.add(0) }
        val segments = ArrayList<Map<String, Any?>>()
        val bbox = BBox.empty()
        var totalDistance = 0.0; var totalDuration = 0.0
        val allInstructions = legs.map { instructionsBuilder.build(it) }
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
                val next = allInstructions.getOrNull(li + 1)?.firstOrNull()?.points
                val steps = stepBuilder.build(allInstructions[li], startIndex, request.coordinates.last(), next)
                seg["steps"] = steps.map { stepJson(it, request) }
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
            val nm = linkedMapOf<String, Any?>("roads" to rf.build(legs))
            graphs.points?.let { nm["points"] = rf.pointsAlong(legs, it) }
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
