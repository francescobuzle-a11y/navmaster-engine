package it.navmaster.engine.routing

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.api.DirectionsRequest
import it.navmaster.engine.api.DirectionsService
import it.navmaster.engine.api.Json
import it.navmaster.engine.api.ResponseFormat
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.api.ApiException
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * "Evita" come penalità (estensione NavMaster): dove esiste un percorso senza le strade da evitare è
 * quello del divieto o uno che le tocca appena (mai più del fattore); dove il divieto non trova nulla
 * (partenza su un'autostrada) la penalità trova comunque un percorso; i tempi sono quelli veri.
 */
class AvoidPenaltyTest {
    @Test
    @Suppress("UNCHECKED_CAST")
    fun `avoid as a penalty`() {
        val dir = Files.createTempDirectory("nme-avoidpen").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { it.prepareCH() }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val bg = g.graph; val sh = g.encoding.shared
            val avoid = AvoidFeatureFlags.HIGHWAYS
            val ban = g.router(RouteOptions(profile = "driving-car", avoidFeatures = avoid))
            val pen = g.router(RouteOptions(profile = "driving-car", avoidFeatures = avoid, avoidPenalty = 30.0))
            val plain = g.router(RouteOptions(profile = "driving-car"))
            val rnd = Random(7)
            var both = 0; var onlyPenalty = 0
            repeat(60) {
                val a = rnd.nextInt(bg.nodeCount); val b = rnd.nextInt(bg.nodeCount)
                // the same snaps for the three (the penalty may snap onto a motorway, the ban never does)
                val sa = try { ban.snap(bg.lat(a), bg.lon(a), 400.0, 0) } catch (e: PointNotFoundException) { return@repeat }
                val sb = try { ban.snap(bg.lat(b), bg.lon(b), 400.0, 1) } catch (e: PointNotFoundException) { return@repeat }
                fun p(r: Router) = try { r.route(sa, sb) } catch (e: RouteNotFoundException) { null }
                val pb = p(ban); val pp = p(pen); val p0 = p(plain)
                if (pp == null || p0 == null) return@repeat
                // the penalised weight is never below the plain one, the time is a real time
                assertTrue(pp.weight >= p0.weight - 0.05)
                if (pb != null) {
                    both++
                    // with a route free of motorways the penalty never does worse than the ban (its weight is a valid candidate)
                    assertTrue(pp.weight <= pb.weight + 0.05, "penalità ${pp.weight} > divieto ${pb.weight}")
                } else onlyPenalty++
            }
            println("viaggi con divieto e penalità: $both, solo con la penalità: $onlyPenalty")
            assertTrue(both > 30)
            // starting on a motorway: the ban snaps elsewhere or fails, the penalty starts on it
            val it = bg.createEdgeIterator()
            val mw = (0 until bg.edgeCount).firstOrNull { e -> it.setEdge(e, bg.nodeA(e)); it.get(sh.motorway) && bg.distance(e) > 200 }
            if (mw != null) {
                it.setEdge(mw, bg.nodeA(mw))
                val geo = it.fetchWayGeometry(FetchMode.ALL)
                val la = geo.lat(geo.size / 2); val lo = geo.lon(geo.size / 2)
                val sp = pen.snap(la, lo, 50.0, 0)
                it.setEdge(sp.edge, bg.nodeA(sp.edge))
                assertTrue(it.get(sh.motorway), "la penalità non aggancia l'autostrada sotto il mezzo")
                val sb = try { ban.snap(la, lo, 50.0, 0).edge } catch (e: PointNotFoundException) { -1 }
                if (sb >= 0) { it.setEdge(sb, bg.nodeA(sb)); assertTrue(!it.get(sh.motorway)) }
                // the whole service: the ban first, the penalty because the start is on the motorway
                val svc = DirectionsService(g)
                assertTrue(svc.onAvoidedRoad(DirectionsRequest.fromJson("driving-car", Json.parse(
                    """{"coordinates":[[$lo,$la],[8.7606,49.4186]],"options":{"avoid_features":["highways"]},"navmaster":{"avoid_penalty":true}}""") as Map<String, Any?>), 0))
                val start = { pen: Boolean ->
                    val body = Json.parse("""{"coordinates":[[$lo,$la],[8.7606,49.4186]],"radiuses":[400,-1],"options":{"avoid_features":["highways"]},"navmaster":{"avoid_penalty":$pen}}""") as Map<String, Any?>
                    val r = try { svc.handle(DirectionsRequest.fromJson("driving-car", body), ResponseFormat.GEOJSON) } catch (e: ApiException) { null }
                    r?.let { m -> ((m["features"] as List<Map<String, Any?>>)[0]["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray> }?.first()
                }
                val pStart = start(true)!!
                assertTrue(DistanceCalc.haversine(la, lo, pStart[1], pStart[0]) < 15.0, "con la penalità il percorso non parte dall'autostrada")
            }
            // the request extension
            val service = DirectionsService(g)
            val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"options":{"avoid_features":["highways"]},"navmaster":{"avoid_penalty":true}}""") as Map<String, Any?>
            val req = DirectionsRequest.fromJson("driving-car", body)
            assertTrue(req.avoidPenalty)
            assertEquals(DirectionsRequest.AVOID_PENALTY_FACTOR, service.routeOptions(req).avoidPenalty)
            service.handle(req, ResponseFormat.GEOJSON)

            // a wide radius (a stop with no road within 400 m, asked again within 5 km): "on a road to
            // avoid" is judged within the 400 m of always, so a point in the fields whose nearest road
            // within 5 km is the motorway is not "on the motorway": the ban stays, and the trip starts
            // from the nearest ordinary road, not from the motorway's carriageway
            val it2 = bg.createEdgeIterator()
            val gf = org.locationtech.jts.geom.GeometryFactory()
            // the fields next to a motorway, made with avoid polygons: a point 800 m beside a motorway
            // stretch, and a 10 m square on every ordinary road within 1.3 km of it (a road crossing a
            // polygon's border is left out, the motorway stays whole and connected): no road within
            // 400 m of the point, the motorway the nearest road within 5 km, ordinary roads at 1.3 km
            class Case(val lat: Double, val lon: Double, val polygons: List<List<DoubleArray>>)
            val cases = ArrayList<Case>()
            var e = 0
            while (e < bg.edgeCount && cases.size < 4) {
                it2.setEdge(e, bg.nodeA(e)); e++
                if (!it2.get(sh.motorway) || bg.distance(e - 1) < 200) continue
                val geo = it2.fetchWayGeometry(FetchMode.ALL)
                val la = geo.lat(geo.size / 2); val lo = geo.lon(geo.size / 2)
                val k = Math.cos(Math.toRadians(la))
                val side = Math.toRadians(DistanceCalc.bearing(geo.lat(0), geo.lon(0), geo.lat(geo.size - 1), geo.lon(geo.size - 1)) + 90.0)
                val pLat = la + 800.0 * Math.cos(side) / DistanceCalc.METERS_PER_DEGREE
                val pLon = lo + 800.0 * Math.sin(side) / (DistanceCalc.METERS_PER_DEGREE * k)
                if (DistanceCalc.haversine(pLat, pLon, 49.4186, 8.7606) < 3000) continue
                val squares = ArrayList<List<DoubleArray>>()
                val dLat = 5.0 / DistanceCalc.METERS_PER_DEGREE; val dLon = 5.0 / (DistanceCalc.METERS_PER_DEGREE * k)
                for (f in 0 until bg.edgeCount) {
                    it2.setEdge(f, bg.nodeA(f))
                    if (it2.get(sh.motorway)) continue
                    val fg = it2.fetchWayGeometry(FetchMode.ALL)
                    if (fg.size < 2) continue
                    var within = false
                    for (i in 0 until fg.size) if (DistanceCalc.haversine(pLat, pLon, fg.lat(i), fg.lon(i)) < 1300.0) { within = true; break }
                    if (!within) continue
                    // the square on the middle of the first segment (inside the line: the line crosses its border)
                    val cLat = (fg.lat(0) + fg.lat(1)) / 2; val cLon = (fg.lon(0) + fg.lon(1)) / 2
                    squares += listOf(doubleArrayOf(cLon - dLon, cLat - dLat), doubleArrayOf(cLon + dLon, cLat - dLat), doubleArrayOf(cLon + dLon, cLat + dLat), doubleArrayOf(cLon - dLon, cLat + dLat), doubleArrayOf(cLon - dLon, cLat - dLat))
                }
                cases += Case(pLat, pLon, squares)
            }
            var found = 0; var compared = 0; var routed = 0
            for (profile in listOf("driving-car", "driving-hgv")) {
                /** [polygons]: avoid polygons (GeoJSON rings, lon/lat) around the point, else nothing avoided. */
                fun check(lat: Double, lon: Double, polygons: List<List<DoubleArray>> = emptyList()) {
                    val polys = polygons.map { r -> gf.createPolygon(gf.createLinearRing(r.map { org.locationtech.jts.geom.Coordinate(it[0], it[1]) }.toTypedArray())) }
                    val free = g.router(RouteOptions(profile = profile, avoidPolygons = polys))
                    val banWide = g.router(RouteOptions(profile = profile, avoidFeatures = avoid, avoidPolygons = polys))
                    val near = try { free.snap(lat, lon, 400.0, 0); true } catch (e: PointNotFoundException) { false }
                    val wide = try { free.snap(lat, lon, 5000.0, 0) } catch (e: PointNotFoundException) { return }
                    val options = if (polygons.isEmpty()) "" else ""","avoid_polygons":{"type":"MultiPolygon","coordinates":[${polygons.joinToString(",") { r -> "[[" + r.joinToString(",") { "[%.7f,%.7f]".format(java.util.Locale.ROOT, it[0], it[1]) } + "]]" }}]}"""
                    val wideBody = Json.parse("""{"coordinates":[[$lon,$lat],[8.7606,49.4186]],"radiuses":[5000,-1],"options":{"avoid_features":["highways"]$options},"navmaster":{"avoid_penalty":true}}""") as Map<String, Any?>
                    val wideReq = DirectionsRequest.fromJson(profile, wideBody)
                    val plainReq = DirectionsRequest.fromJson(profile, Json.parse("""{"coordinates":[[$lon,$lat],[8.7606,49.4186]],"options":{"avoid_features":["highways"]$options},"navmaster":{"avoid_penalty":true}}""") as Map<String, Any?>)
                    // the same answer as with openrouteservice's radius, whatever the radius asked
                    assertEquals(service.onAvoidedRoad(plainReq, 0), service.onAvoidedRoad(wideReq, 0), "raggio largo diverso in (%.5f, %.5f)".format(lat, lon)); compared++
                    it2.setEdge(wide.edge, bg.nodeA(wide.edge))
                    if (near || !it2.get(sh.motorway)) return
                    found++
                    assertTrue(!service.onAvoidedRoad(wideReq, 0), "punto nei campi (%.5f, %.5f) preso per «sull'autostrada»".format(lat, lon))
                    val banned = try { banWide.snap(lat, lon, 5000.0, 0) } catch (e: PointNotFoundException) { return }
                    val r = try { service.handle(wideReq, ResponseFormat.GEOJSON) } catch (e: ApiException) { return }
                    val first = (((r["features"] as List<Map<String, Any?>>)[0]["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>).first()
                    val off = DistanceCalc.haversine(banned.snappedLat, banned.snappedLon, first[1], first[0])
                    assertTrue(off < 15.0, "il viaggio da (%.5f, %.5f) non parte dalla strada ordinaria (divieto) ma a %.0f m da essa".format(lat, lon, off))
                    routed++
                }
                // a grid over the extract (a road within 400 m nearly everywhere: the answers stay the same)
                var lat = 49.35
                while (lat < 49.47) {
                    var lon = 8.56
                    while (lon < 8.80) { check(lat, lon); lon += 0.01 }
                    lat += 0.01
                }
                for (c in cases) check(c.lat, c.lon, c.polygons)
            }
            println("raggio largo: $compared punti confrontati, $found nei campi con l'autostrada come strada più vicina, $routed viaggi partiti dalla strada ordinaria")
            assertTrue(compared > 100)
            assertTrue(found > 0, "nessun punto nei campi con l'autostrada come strada più vicina")
            assertTrue(routed > 0, "nessun viaggio partito dalla strada ordinaria")
        }
    }
}
