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
        }
    }
}
