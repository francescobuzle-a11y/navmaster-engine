package it.navmaster.engine

import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** Il riscaldamento di un grafo mappato tocca i file di una ricerca col core e non cambia i percorsi; in RAM non fa nulla. */
class WarmTest {
    @Test
    fun `warming a mapped graph touches the core files within the budget and keeps the routes`() {
        val dir = Files.createTempDirectory("nme-warm").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile())).first.use { it.prepareCH(landmarkCount = 4) }
        val a = 49.4094 to 8.6935; val b = 49.3800 to 8.6600
        val options = RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 4.0), avoidFeatures = AvoidFeatureFlags.TOLLWAYS)
        fun weightOf(g: RoutingGraph): Double { val r = g.router(options); return r.route(r.snap(a.first, a.second, 400.0, 0), r.snap(b.first, b.second, 400.0, 1)).weight }
        RoutingGraph.load(dir, mmap = true).use { g ->
            val before = weightOf(g)
            val line = g.warm("driving-hgv")
            println(line)
            assertTrue(line.startsWith("riscaldamento heavyvehicle_recommended_core:"), line)
            for (part in listOf("livelli", "landmark ", "landmark tollways", "shortcut", "nodi", "archi")) assertTrue(part in line, "manca $part: $line")
            // un bilancio piccolo: si ferma dopo i primi file
            val short = g.warm("driving-hgv", budget = 1)
            assertTrue("archi" !in short && "livelli" in short, short)
            assertEquals(before, weightOf(g), 1e-9)
        }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val line = g.warm("driving-hgv")
            assertTrue(line.contains(" 0 MB "), line)
            assertTrue(abs(weightOf(g) - weightOf(g)) < 1e-9)
        }
    }
}
