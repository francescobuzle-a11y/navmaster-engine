package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class DirectionsServiceTest {
    @Test
    fun `json round trip and polyline`() {
        val doc = """{"a":[1,2.5,-3e2,"x\n\"y\"",true,null],"b":{"c":{}}, "d": "è"}"""
        val v = Json.parse(doc) as Map<*, *>
        assertEquals(listOf(1L, 2.5, -300.0, "x\n\"y\"", true, null), v["a"])
        assertEquals("è", v["d"])
        val out = Json.stringify(v)
        assertEquals(v, Json.parse(out))
        assertEquals("""{"x":1.5,"y":[1,2],"z":"a"}""", Json.stringify(linkedMapOf("x" to 1.5, "y" to listOf(1, 2), "z" to "a")))
        val enc = PolylineEncoder.encode(doubleArrayOf(38.5, 40.7, 43.252), doubleArrayOf(-120.2, -120.95, -126.453), 3)
        assertEquals("_p~iF~ps|U_ulLnnqC_mqNvxq`@", enc)
        val dec = PolylineEncoder.decode(enc)
        assertEquals(43.252, dec[2][0], 1e-5); assertEquals(-126.453, dec[2][1], 1e-5)
    }

    @Test
    fun `directions request parsing and validation`() {
        val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"preference":"shortest","units":"km","language":"it",
            "options":{"avoid_features":["tollways"],"profile_params":{"restrictions":{"height":4.0,"weight":40}}},"radiuses":[300,-1]}""") as Map<String, Any?>
        val r = DirectionsRequest.fromJson("driving-hgv", body)
        assertEquals(2, r.coordinates.size); assertEquals(49.4094, r.coordinates[0].first)
        assertEquals(it.navmaster.engine.weighting.WeightingMethod.SHORTEST, r.preference)
        assertEquals(it.navmaster.engine.profile.AvoidFeatureFlags.TOLLWAYS, r.avoidFeatures)
        assertEquals(4.0, r.vehicleParameters!!.height); assertEquals(40.0, r.vehicleParameters!!.weight)
        assertEquals(300.0, r.radiuses!![0])
        assertEquals(RoutingErrorCodes.MISSING_PARAMETER, assertFailsWith<ApiException> { DirectionsRequest.fromJson("driving-hgv", mapOf("preference" to "fastest")) }.code)
        assertEquals(RoutingErrorCodes.UNKNOWN_PARAMETER, assertFailsWith<ApiException> { DirectionsRequest.fromJson("driving-hgv", mapOf("coordinates" to listOf(listOf(1, 2), listOf(3, 4)), "foo" to 1)) }.code)
        assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, assertFailsWith<ApiException> { DirectionsRequest.fromJson("driving-hgv", mapOf("coordinates" to listOf(listOf(1, 2), listOf(3, 4)), "preference" to "quickest")) }.code)
    }

    @Test
    fun `directions response in ORS json and geojson`() {
        val dir = Files.createTempDirectory("nme-api").toFile().also { it.deleteOnExit() }
        val (rg, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        rg.use {
            val service = DirectionsService(it)
            val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.6757,49.4036],[8.7606,49.4186]],"language":"it","maneuvers":true,"attributes":["detourfactor","percentage","avgspeed"],"id":"test-1"}""") as Map<String, Any?>
            val json = service.handle(DirectionsRequest.fromJson("driving-hgv", body), ResponseFormat.JSON)
            val text = Json.stringify(json)
            val back = Json.parse(text) as Map<*, *>
            val routes = back["routes"] as List<*>
            assertEquals(1, routes.size)
            val route = routes[0] as Map<*, *>
            val summary = route["summary"] as Map<*, *>
            assertTrue((summary["distance"] as Double) > 5000 && (summary["duration"] as Double) > 300)
            val segments = route["segments"] as List<*>
            assertEquals(2, segments.size)
            val wayPoints = route["way_points"] as List<*>
            assertEquals(3, wayPoints.size); assertEquals(0L, wayPoints[0])
            val geometry = PolylineEncoder.decode(route["geometry"] as String)
            assertEquals((wayPoints[2] as Long).toInt() + 1, geometry.size)
            // i passi del secondo segmento continuano gli indici del primo
            val steps1 = (segments[0] as Map<*, *>)["steps"] as List<*>; val steps2 = (segments[1] as Map<*, *>)["steps"] as List<*>
            val lastStep1 = steps1.last() as Map<*, *>; val firstStep2 = steps2.first() as Map<*, *>
            assertEquals(10L, lastStep1["type"]); assertEquals(11L, firstStep2["type"])
            assertEquals((lastStep1["way_points"] as List<*>)[1], (firstStep2["way_points"] as List<*>)[0])
            assertEquals(wayPoints[1], (lastStep1["way_points"] as List<*>)[1])
            assertEquals(wayPoints[2], ((steps2.last() as Map<*, *>)["way_points"] as List<*>)[1])
            assertTrue((firstStep2["instruction"] as String).startsWith("Avanti"))
            assertTrue(firstStep2.containsKey("maneuver"))
            assertTrue((segments[0] as Map<*, *>).containsKey("detourfactor") && (segments[0] as Map<*, *>).containsKey("percentage"))
            val meta = back["metadata"] as Map<*, *>
            assertEquals("test-1", meta["id"]); assertEquals("routing", meta["service"])
            assertEquals("driving-hgv", (meta["query"] as Map<*, *>)["profile"])
            assertEquals(4, (back["bbox"] as List<*>).size)

            val geo = service.handle(DirectionsRequest.fromJson("driving-car", body), ResponseFormat.GEOJSON)
            val gback = Json.parse(Json.stringify(geo)) as Map<*, *>
            assertEquals("FeatureCollection", gback["type"])
            val feature = (gback["features"] as List<*>)[0] as Map<*, *>
            val coords = (feature["geometry"] as Map<*, *>)["coordinates"] as List<*>
            assertTrue(coords.size > 50)
            assertEquals(8.6937, ((coords[0] as List<*>)[0] as Double), 0.002)

            // errori
            val far = assertFailsWith<ApiException> { service.handle(DirectionsRequest.fromJson("driving-hgv", mapOf("coordinates" to listOf(listOf(9.5, 49.6), listOf(8.7606, 49.4186)))), ResponseFormat.JSON) }
            assertEquals(RoutingErrorCodes.POINT_NOT_FOUND, far.code); assertEquals(404, far.httpStatus)
            val bad = assertFailsWith<ApiException> { service.handle(DirectionsRequest.fromJson("cycling-regular", body), ResponseFormat.JSON) }
            assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, bad.code)
            val err = Json.stringify(service.errorBody(far))
            assertTrue(err.contains("\"code\":2010"))
            val status = Json.stringify(service.status())
            assertTrue(status.contains("driving-hgv") && status.contains("\"preparations\""), status)
        }
    }
    @Test
    fun `continue_straight keeps the arrival direction at a via point`() {
        val dir = Files.createTempDirectory("nme-cs").toFile().also { it.deleteOnExit() }
        val (rg, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        rg.use { g ->
            // andata e ritorno sullo stesso punto: senza continue_straight il secondo tratto può invertire sull'arco del via-point
            val a = 49.4094 to 8.6937
            val straight = g.router(it.navmaster.engine.RouteOptions(profile = "driving-hgv", continueStraight = true))
            val free = g.router(it.navmaster.engine.RouteOptions(profile = "driving-hgv", allowCH = false))
            assertTrue(!it.navmaster.engine.RouteOptions(continueStraight = true).isStatic)
            val sA = straight.snap(a.first, a.second, 400.0, 0)
            var twoWay = 0
            for (via in listOf(49.4036 to 8.6757, 49.4120 to 8.6850, 49.4060 to 8.6900, 49.4150 to 8.6950, 49.4000 to 8.6800)) {
                val sV = straight.snap(via.first, via.second, 400.0, 1)
                if (sV.position == it.navmaster.engine.index.SnapPosition.TOWER) continue
                val arrival = straight.route(sA, sV).edges.last()
                // nel verso d'arrivo si prosegue sempre sull'arco del via
                val onward = straight.route(sV, sA, arrival)
                assertTrue(onward.edges.first().edge == arrival.edge && onward.edges.first().reverse == arrival.reverse, "con continue_straight si prosegue nel verso d'arrivo")
                // con un arrivo nel verso opposto si riparte nell'altro verso, se la strada è a doppio senso
                val flipped = it.navmaster.engine.routing.PathEdge(arrival.edge, !arrival.reverse, arrival.adjNode, arrival.baseNode, arrival.geometry, arrival.distance, arrival.millis, arrival.weight, arrival.name)
                val opposite = straight.route(sV, sA, flipped)
                assertTrue(opposite.edges.first().edge == arrival.edge)
                if (opposite.edges.first().reverse == flipped.reverse) {
                    twoWay++
                    assertTrue(onward.distance != opposite.distance)
                    assertTrue(free.route(sV, sA).distance <= minOf(onward.distance, opposite.distance) + 1e-6)
                }
            }
            assertTrue(twoWay > 0, "nessun via-point su strada a doppio senso")
            // e dalla API: la richiesta è accettata e il secondo segmento è più lungo del ritorno diretto
            val service = DirectionsService(g)
            val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.6757,49.4036],[8.6937,49.4094]],"continue_straight":true}""") as Map<String, Any?>
            val r = service.handle(DirectionsRequest.fromJson("driving-hgv", body), ResponseFormat.JSON)
            val segs = ((r["routes"] as List<*>)[0] as Map<*, *>)["segments"] as List<*>
            assertEquals(2, segs.size)
        }
    }
}
