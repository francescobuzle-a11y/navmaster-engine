package it.navmaster.engine.ch

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Il core largo come quello di ORS (autostrade, pedaggi, traghetti, strade veloci dentro):
 * `avoid_features` e `maximum_speed` usano il core e danno i pesi della ricerca piena.
 */
class WideCoreTest {
    @Test
    fun `wide core answers avoid_features and maximum_speed exactly`() {
        val dir = Files.createTempDirectory("nme-wide").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { it.prepareCH(coreWide = true) }
        RoutingGraph.load(dir, mmap = false).use { g ->
            assertTrue(g.coreWide)
            val narrow = g.chStorages.getValue("heavyvehicle_recommended_core")
            println("core largo del camion: ${narrow.coreNodeCount} nodi, ${narrow.shortcutCount} shortcut")
            val bg = g.graph
            val params = VehicleParameters(height = 4.0, weight = 12.0)
            val cases = listOf(
                "evita pedaggi e autostrade" to RouteOptions(profile = "driving-hgv", vehicleParameters = params, avoidFeatures = AvoidFeatureFlags.TOLLWAYS or AvoidFeatureFlags.HIGHWAYS),
                "evita traghetti" to RouteOptions(profile = "driving-hgv", vehicleParameters = params, avoidFeatures = AvoidFeatureFlags.FERRIES),
                "velocità massima 80" to RouteOptions(profile = "driving-hgv", vehicleParameters = params, maximumSpeed = 80.0),
                "auto evita autostrade" to RouteOptions(profile = "driving-car", avoidFeatures = AvoidFeatureFlags.HIGHWAYS),
                "auto velocità 80 e pedaggi" to RouteOptions(profile = "driving-car", maximumSpeed = 80.0, avoidFeatures = AvoidFeatureFlags.TOLLWAYS),
            )
            val rnd = Random(9)
            for ((name, o) in cases) {
                val core = g.router(o); val full = g.router(o.copy(allowCH = false))
                assertEquals("core_alt_edge_based", core.algorithm, name); assertEquals("dijkstra_edge_based", full.algorithm, name)
                var equal = 0; var loops = 0
                repeat(40) {
                    val a = rnd.nextInt(bg.nodeCount); val b = rnd.nextInt(bg.nodeCount)
                    var unsnapped = false
                    fun p(r: Router) = try { r.route(r.snap(bg.lat(a), bg.lon(a), 400.0, 0), r.snap(bg.lat(b), bg.lon(b), 400.0, 1)) } catch (e: RouteNotFoundException) { null } catch (e: PointNotFoundException) { unsnapped = true; null }
                    val pc = p(core); val pf = p(full)
                    if (unsnapped) return@repeat
                    val wc = pc?.weight ?: Double.POSITIVE_INFINITY; val wf = pf?.weight ?: Double.POSITIVE_INFINITY
                    if ((wc.isInfinite() && wf.isInfinite()) || abs(wc - wf) <= 0.05 + wf * 1e-5) equal++
                    else {
                        val ns = pf!!.edges.map { it.baseNode }
                        val loop = wf < wc && full.isLegal(pf) && ns.groupBy { it }.filter { it.value.size > 1 }.keys.any { r -> ns.subList(ns.indexOf(r), ns.lastIndexOf(r) + 1).any { narrow.level(it) < bg.nodeCount } }
                        assertTrue(loop, "[$name] $a→$b: core $wc, ricerca piena $wf")
                        loops++
                    }
                }
                println("$name: $equal uguali, $loops col giro noto")
                assertTrue(equal >= 30 && loops <= 3, name)
            }
        }
    }
}
