package it.navmaster.server

import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.server.application.Application
import io.ktor.server.application.call
import io.ktor.server.application.install
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import io.ktor.server.plugins.cors.routing.CORS
import io.ktor.server.request.path
import io.ktor.server.request.receiveText
import io.ktor.server.response.respondText
import io.ktor.server.routing.get
import io.ktor.server.routing.post
import io.ktor.server.routing.route
import io.ktor.server.routing.routing
import io.ktor.http.HttpMethod
import it.navmaster.engine.Engine
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.api.ApiException
import it.navmaster.engine.api.DirectionsLimits
import it.navmaster.engine.api.DirectionsRequest
import it.navmaster.engine.api.DirectionsService
import it.navmaster.engine.api.Json
import it.navmaster.engine.api.ResponseFormat
import it.navmaster.engine.api.RoutingErrorCodes
import java.io.File

fun main(args: Array<String>) {
    fun opt(name: String) = args.firstOrNull { it.startsWith("--$name=") }?.substringAfter('=')
    val graphDir = File(opt("graph") ?: "graphs/heidelberg")
    val port = opt("port")?.toInt() ?: 8082
    val limits = DirectionsLimits(maximumDistance = opt("max-distance")?.toDouble() ?: 3_000_000.0, maximumDistanceDynamicWeights = opt("max-distance")?.toDouble() ?: 3_000_000.0)
    val apiKey = opt("api-key") ?: System.getenv("NAVMASTER_API_KEY")
    println("${Engine.NAME} ${Engine.VERSION}: carico il grafo da $graphDir")
    val t0 = System.currentTimeMillis()
    val graphs = RoutingGraph.load(graphDir)
    println("grafo pronto in ${System.currentTimeMillis() - t0} ms: ${graphs.graph.nodeCount} nodi, ${graphs.graph.edgeCount} archi, profili ${graphs.profileNames}, preparazioni ${graphs.chStorages.keys}")
    println("in ascolto sulla porta $port" + if (apiKey != null) " (chiave API richiesta nell'header Authorization)" else " (nessuna chiave API)")
    embeddedServer(Netty, port = port) { orsApi(DirectionsService(graphs, limits), apiKey) }.start(wait = true)
}

/**
 * Rotte compatibili con openrouteservice v2 sotto `/ors`. Con [apiKey] le richieste di calcolo devono
 * portare la chiave nell'header `Authorization` (come fa il client ORS di OsmAnd); `/health` resta libero.
 */
fun Application.orsApi(service: DirectionsService, apiKey: String? = null) {
    install(CORS) {
        anyHost()
        allowHeader("Authorization"); allowHeader("Content-Type"); allowHeader("Accept")
        allowMethod(HttpMethod.Get); allowMethod(HttpMethod.Post); allowMethod(HttpMethod.Options)
    }
    if (apiKey != null) {
        intercept(io.ktor.server.application.ApplicationCallPipeline.Plugins) {
            val path = call.request.path()
            if (path.startsWith("/ors/v2/") && !path.startsWith("/ors/v2/health")) {
                val provided = call.request.headers["Authorization"]?.removePrefix("Bearer ")?.trim() ?: call.request.queryParameters["api_key"]
                if (provided != apiKey) {
                    call.respondText("""{"error":{"code":403,"message":"Access to this API has been disallowed"}}""", ContentType.Application.Json, HttpStatusCode.Forbidden)
                    finish()
                }
            }
        }
    }
    routing {
        route("/ors/v2") {
            get("/health") { call.respondText("""{"status":"ready"}""", ContentType.Application.Json) }
            get("/status") { call.respondText(Json.stringify(service.status()), ContentType.Application.Json) }
            route("/directions/{profile}") {
                get { handleGet(service, ResponseFormat.GEOJSON) }
                post { handlePost(service, ResponseFormat.JSON, ContentType.Application.Json) }
                post("/json") { handlePost(service, ResponseFormat.JSON, ContentType.Application.Json) }
                post("/geojson") { handlePost(service, ResponseFormat.GEOJSON, ContentType.parse("application/geo+json")) }
                post("/{format}") {
                    respondError(service, ApiException(406, RoutingErrorCodes.UNSUPPORTED_EXPORT_FORMAT, "This response format is not supported."))
                }
            }
            post("/directions") { respondError(service, ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter 'profile' is missing.")) }
            get("/directions") { respondError(service, ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter 'profile' is missing.")) }
        }
    }
}

private suspend fun io.ktor.server.routing.RoutingContext.handlePost(service: DirectionsService, format: ResponseFormat, contentType: ContentType) {
    val profile = call.parameters["profile"]!!
    try {
        val text = call.receiveText()
        if (text.isBlank()) throw ApiException(400, RoutingErrorCodes.EMPTY_ELEMENT, "Request body is missing.")
        val body = try { Json.parse(text) as? Map<String, Any?> ?: throw ApiException(400, RoutingErrorCodes.INVALID_JSON_FORMAT, "Request body must be a JSON object.") }
            catch (e: Json.ParseException) { throw ApiException(400, RoutingErrorCodes.INVALID_JSON_FORMAT, "Invalid JSON: ${e.message}") }
        val request = DirectionsRequest.fromJson(profile, body)
        val response = service.handle(request, format)
        call.respondText(Json.stringify(response), contentType)
    } catch (e: ApiException) {
        respondError(service, e)
    } catch (e: Exception) {
        respondError(service, ApiException(500, RoutingErrorCodes.UNKNOWN, "Unable to compute route: ${e.message}"))
    }
}

private suspend fun io.ktor.server.routing.RoutingContext.handleGet(service: DirectionsService, format: ResponseFormat) {
    val profile = call.parameters["profile"]!!
    try {
        val start = call.request.queryParameters["start"] ?: throw ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter 'start' is missing.")
        val end = call.request.queryParameters["end"] ?: throw ApiException(400, RoutingErrorCodes.MISSING_PARAMETER, "Parameter 'end' is missing.")
        fun coord(s: String, name: String): List<Double> {
            val p = s.split(',').map { it.trim().toDoubleOrNull() ?: throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter '$name' has an invalid format.") }
            if (p.size != 2) throw ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_FORMAT, "Parameter '$name' has an invalid format.")
            return p
        }
        val body = linkedMapOf<String, Any?>("coordinates" to listOf(coord(start, "start"), coord(end, "end")))
        call.request.queryParameters["language"]?.let { body["language"] = it }
        val response = service.handle(DirectionsRequest.fromJson(profile, body), format)
        call.respondText(Json.stringify(response), ContentType.parse("application/geo+json"))
    } catch (e: ApiException) {
        respondError(service, e)
    } catch (e: Exception) {
        respondError(service, ApiException(500, RoutingErrorCodes.UNKNOWN, "Unable to compute route: ${e.message}"))
    }
}

private suspend fun io.ktor.server.routing.RoutingContext.respondError(service: DirectionsService, e: ApiException) {
    call.respondText(Json.stringify(service.errorBody(e)), ContentType.Application.Json, HttpStatusCode.fromValue(e.httpStatus))
}
