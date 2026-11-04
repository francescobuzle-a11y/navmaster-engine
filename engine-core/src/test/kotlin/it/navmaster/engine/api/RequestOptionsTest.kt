package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.osm.ImportOptions
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class RequestOptionsTest {
    private val pbf = File("../data/heidelberg.test.pbf")
    private fun tmp() = Files.createTempDirectory("nme-opt").toFile().also { it.deleteOnExit() }

    @Test
    fun `width option changes how many edges carry a width limit`() {
        fun count(opts: ImportOptions): Int {
            val encoding = GraphEncoding(listOf(HgvProfile()))
            BaseGraph(GraphDirectory(tmp()), encoding.registry).use { g ->
                OsmGraphImporter(g, encoding, options = opts).import(pbf)
                val it = g.createEdgeIterator(); var n = 0
                for (e in 0 until g.edgeCount) { it.setEdge(e, g.nodeA(e)); if (it.get(encoding.shared.maxWidth).isFinite()) n++ }
                return n
            }
        }
        val withWidth = count(ImportOptions(widthFromCarriageway = true))
        val strict = count(ImportOptions(widthFromCarriageway = false))
        println("archi con limite di larghezza: con tag width $withWidth, solo maxwidth $strict")
        assertTrue(strict < withWidth / 2, "il tag width dovrebbe essere la fonte principale: $strict vs $withWidth")
    }

    @Test
    fun `avoid polygons, maximum speed and extra info`() {
        val dir = tmp()
        val (rg, _) = RoutingGraph.build(pbf, dir, listOf(CarProfile(), HgvProfile()))
        rg.use {
            it.prepareCH()
            val service = DirectionsService(it)
            val base = """{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"extra_info":["tollways","waycategory","waytype","surface","roadaccessrestrictions","osmid"]}"""
            val plain = service.handle(DirectionsRequest.fromJson("driving-hgv", Json.parse(base) as Map<String, Any?>), ResponseFormat.JSON)
            val route = (plain["routes"] as List<*>)[0] as Map<*, *>
            val extras = route["extras"] as Map<*, *>
            assertEquals(setOf("tollways", "waycategory", "waytype", "surface", "roadaccessrestrictions", "osmid"), extras.keys)
            val wayType = extras["waytype"] as Map<*, *>
            val values = wayType["values"] as List<*>
            val geomSize = PolylineEncoder.decode(route["geometry"] as String).size
            // gli intervalli coprono la geometria senza buchi
            var expectedFrom = 0
            for (v in values) { val iv = v as List<*>; assertEquals(expectedFrom, iv[0]); expectedFrom = iv[1] as Int }
            assertEquals(geomSize - 1, expectedFrom)
            val summary = wayType["summary"] as List<*>
            assertTrue(summary.isNotEmpty())
            assertEquals(100.0, summary.sumOf { (it as Map<*, *>)["amount"] as Double }, 0.5)

            // maximum_speed: stesso percorso o più lento, mai più veloce
            val slow = service.handle(DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"maximum_speed":80}""") as Map<String, Any?>), ResponseFormat.JSON)
            val dPlain = ((plain["routes"] as List<*>)[0] as Map<*, *>)["summary"] as Map<*, *>
            val dSlow = ((slow["routes"] as List<*>)[0] as Map<*, *>)["summary"] as Map<*, *>
            assertTrue((dSlow["duration"] as Double) >= (dPlain["duration"] as Double) - 0.1)
            assertEquals("core_alt_edge_based", it.router(RouteOptions(profile = "driving-hgv", maximumSpeed = 80.0)).algorithm)

            // avoid_polygons: un poligono sul tracciato del percorso lo devia (o lo rende impossibile)
            val pts = PolylineEncoder.decode(route["geometry"] as String)
            val mid = pts[pts.size / 2]
            val d = 0.002
            val poly = """{"type":"Polygon","coordinates":[[[${mid[1] - d},${mid[0] - d}],[${mid[1] + d},${mid[0] - d}],[${mid[1] + d},${mid[0] + d}],[${mid[1] - d},${mid[0] + d}],[${mid[1] - d},${mid[0] - d}]]]}"""
            val req = DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"options":{"avoid_polygons":$poly}}""") as Map<String, Any?>)
            assertEquals(1, req.avoidPolygons.size)
            assertEquals("dijkstra_edge_based", it.router(RouteOptions(profile = "driving-hgv", avoidPolygons = req.avoidPolygons)).algorithm)
            val avoided = runCatching { service.handle(req, ResponseFormat.JSON) }
            avoided.getOrNull()?.let { resp ->
                val r2 = (resp["routes"] as List<*>)[0] as Map<*, *>
                val pts2 = PolylineEncoder.decode(r2["geometry"] as String)
                for (p in pts2) assertTrue(!(p[0] > mid[0] - d && p[0] < mid[0] + d && p[1] > mid[1] - d && p[1] < mid[1] + d), "punto dentro il poligono da evitare")
                assertTrue(((r2["summary"] as Map<*, *>)["distance"] as Double) >= (dPlain["distance"] as Double) - 1)
            }
            avoided.exceptionOrNull()?.let { ex -> assertTrue(ex is ApiException && ex.code == RoutingErrorCodes.ROUTE_NOT_FOUND, "errore inatteso: $ex") }
        }
    }
}
