package it.navmaster.engine

import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class RoutingGraphTest {
    private val pairs = listOf(
        (49.4094 to 8.6937) to (49.4186 to 8.7606),
        (49.4036 to 8.6757) to (49.3989 to 8.6724),
        (49.4186 to 8.7606) to (49.4036 to 8.6757),
        (49.3989 to 8.6724) to (49.4094 to 8.6937),
    )

    @Test
    fun `build, prepare CH, reload and route with every algorithm`() {
        val dir = Files.createTempDirectory("nme-rg").toFile().also { it.deleteOnExit() }
        val (built, stats) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use {
            assertTrue(stats.edges > 10_000)
            it.prepareCH()
            assertEquals(setOf("car-ors_fastest", "car-ors_fastest_core", "car-ors_shortest", "car-ors_shortest_core", "heavyvehicle_recommended", "heavyvehicle_recommended_core", "heavyvehicle_shortest", "heavyvehicle_shortest_core"), it.chStorages.keys)
        }
        RoutingGraph.load(dir).use { rg ->
            assertEquals(8, rg.chStorages.size)
            for ((a, b) in pairs) {
                // Core (con restrizioni e divieti di svolta) vs Dijkstra edge-based filtrato: stesso peso
                val params = VehicleParameters(height = 4.0, weight = 12.0)
                val coreRouter = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
                assertEquals("core_alt_edge_based", coreRouter.algorithm)
                val ebRouter = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false))
                assertEquals("dijkstra_edge_based", ebRouter.algorithm)
                val c1 = coreRouter.route(coreRouter.snap(a.first, a.second, 400.0, 0), coreRouter.snap(b.first, b.second, 400.0, 1))
                val c2 = ebRouter.route(ebRouter.snap(a.first, a.second, 400.0, 0), ebRouter.snap(b.first, b.second, 400.0, 1))
                assertTrue(abs(c1.weight - c2.weight) <= 0.05 + c2.weight * 1e-4, "Core ${c1.weight} vs Dijkstra edge-based ${c2.weight}")
                for (k in 1 until c1.edges.size) assertEquals(c1.edges[k - 1].adjNode, c1.edges[k].baseNode)
                // CH (statico) vs Dijkstra senza costi di svolta: stesso peso
                val chRouter = rg.router(RouteOptions(profile = "driving-hgv", turnCosts = false))
                assertEquals("ch", chRouter.algorithm)
                val plainRouter = rg.router(RouteOptions(profile = "driving-hgv", turnCosts = false, allowCH = false))
                assertEquals("dijkstra", plainRouter.algorithm)
                val p1 = chRouter.route(chRouter.snap(a.first, a.second, 400.0, 0), chRouter.snap(b.first, b.second, 400.0, 1))
                val p2 = plainRouter.route(plainRouter.snap(a.first, a.second, 400.0, 0), plainRouter.snap(b.first, b.second, 400.0, 1))
                assertTrue(abs(p1.weight - p2.weight) <= 0.05 + p2.weight * 1e-4, "CH ${p1.weight} vs Dijkstra ${p2.weight}")
                assertEquals(p2.distance, p1.distance, p2.distance * 0.01)
                // con restrizioni del veicolo o avoid_features niente CH: il core (largo come ORS: pedaggi e autostrade dentro)
                assertTrue(rg.coreWide)
                assertEquals("core_alt_edge_based", rg.router(RouteOptions(profile = "driving-hgv", avoidFeatures = AvoidFeatureFlags.TOLLWAYS)).algorithm)
                assertEquals("core_alt", rg.router(RouteOptions(profile = "driving-hgv", avoidFeatures = AvoidFeatureFlags.TOLLWAYS, turnCosts = false)).algorithm)
                // anche "shortest" ha le sue preparazioni (CH statica senza costi di svolta, core con e senza restrizioni)
                assertEquals("ch", rg.router(RouteOptions(profile = "driving-hgv", weighting = WeightingMethod.SHORTEST, turnCosts = false)).algorithm)
                assertEquals("core_alt_edge_based", rg.router(RouteOptions(profile = "driving-hgv", weighting = WeightingMethod.SHORTEST)).algorithm)
                assertEquals("core_alt_edge_based", rg.router(RouteOptions(profile = "driving-hgv", weighting = WeightingMethod.SHORTEST, vehicleParameters = VehicleParameters(height = 4.0))).algorithm)
                assertEquals("ch", rg.router(RouteOptions(profile = "driving-car", weighting = WeightingMethod.FASTEST, turnCosts = false)).algorithm)
            }
        }
    }
}
