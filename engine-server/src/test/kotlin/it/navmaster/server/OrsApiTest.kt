package it.navmaster.server

import io.ktor.client.request.get
import io.ktor.client.request.header
import io.ktor.client.request.post
import io.ktor.client.request.setBody
import io.ktor.client.statement.bodyAsText
import io.ktor.http.ContentType
import io.ktor.http.HttpStatusCode
import io.ktor.http.contentType
import io.ktor.server.testing.testApplication
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.api.DirectionsService
import it.navmaster.engine.api.Json
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class OrsApiTest {
    @Test
    fun `ORS compatible endpoints`() {
        val dir = Files.createTempDirectory("nme-server").toFile().also { it.deleteOnExit() }
        val (rg, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        rg.use {
            val service = DirectionsService(it)
            testApplication {
                application { orsApi(service) }
                assertEquals("""{"status":"ready"}""", client.get("/ors/v2/health").bodyAsText())
                val status = Json.parse(client.get("/ors/v2/status").bodyAsText()) as Map<*, *>
                assertTrue((status["profiles"] as Map<*, *>).containsKey("driving-hgv"))

                // POST come il client di OsmAnd: /geojson con options
                val geo = client.post("/ors/v2/directions/driving-hgv/geojson") {
                    contentType(ContentType.Application.Json)
                    setBody("""{"language":"it","coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"options":{"vehicle_type":"hgv","profile_params":{"restrictions":{"height":4.0,"weight":40}},"avoid_features":["tollways"]}}""")
                }
                assertEquals(HttpStatusCode.OK, geo.status)
                assertTrue(geo.contentType()?.toString()?.contains("geo+json") == true)
                val fc = Json.parse(geo.bodyAsText()) as Map<*, *>
                assertEquals("FeatureCollection", fc["type"])
                val props = ((fc["features"] as List<*>)[0] as Map<*, *>)["properties"] as Map<*, *>
                val steps = ((props["segments"] as List<*>)[0] as Map<*, *>)["steps"] as List<*>
                assertTrue(steps.size > 3)
                assertEquals(11L, (steps[0] as Map<*, *>)["type"])

                // POST json
                val json = client.post("/ors/v2/directions/driving-car") { contentType(ContentType.Application.Json); setBody("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""") }
                assertEquals(HttpStatusCode.OK, json.status)
                val routes = (Json.parse(json.bodyAsText()) as Map<*, *>)["routes"] as List<*>
                assertTrue((routes[0] as Map<*, *>).containsKey("geometry"))

                // GET geojson
                val get = client.get("/ors/v2/directions/driving-car?start=8.6937,49.4094&end=8.7606,49.4186")
                assertEquals(HttpStatusCode.OK, get.status)

                // errori
                val notFound = client.post("/ors/v2/directions/driving-hgv") { contentType(ContentType.Application.Json); setBody("""{"coordinates":[[9.5,49.6],[8.7606,49.4186]]}""") }
                assertEquals(HttpStatusCode.NotFound, notFound.status)
                assertTrue(notFound.bodyAsText().contains("\"code\":2010"))
                val badJson = client.post("/ors/v2/directions/driving-hgv") { contentType(ContentType.Application.Json); setBody("{not json") }
                assertEquals(HttpStatusCode.BadRequest, badJson.status)
                assertTrue(badJson.bodyAsText().contains("\"code\":2000"))
                val unsupported = client.post("/ors/v2/directions/driving-hgv/gpx") { contentType(ContentType.Application.Json); setBody("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""") }
                assertEquals(HttpStatusCode.NotAcceptable, unsupported.status)
                val noProfile = client.post("/ors/v2/directions") { setBody("{}") }
                assertEquals(HttpStatusCode.BadRequest, noProfile.status)
            }
            // con chiave API
            testApplication {
                application { orsApi(service, apiKey = "segreto") }
                assertEquals(HttpStatusCode.OK, client.get("/ors/v2/health").status)
                assertEquals(HttpStatusCode.Forbidden, client.get("/ors/v2/status").status)
                val ok = client.post("/ors/v2/directions/driving-hgv") {
                    contentType(ContentType.Application.Json); header("Authorization", "segreto")
                    setBody("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""")
                }
                assertEquals(HttpStatusCode.OK, ok.status)
                assertEquals(HttpStatusCode.Forbidden, client.post("/ors/v2/directions/driving-hgv") { contentType(ContentType.Application.Json); setBody("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""") }.status)
            }
        }
    }
}
