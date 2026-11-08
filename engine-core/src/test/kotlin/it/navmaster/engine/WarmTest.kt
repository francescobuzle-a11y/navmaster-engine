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
            val seen = ArrayList<Pair<Long, Long>>()
            val line = g.warm("driving-hgv") { d, t -> seen += d to t }
            println(line)
            // l'avanzamento: crescente, fino al totale previsto
            assertTrue(seen.isNotEmpty() && seen.last().first == seen.last().second && seen.zipWithNext().all { (a, b) -> b.first >= a.first && a.second == b.second }, seen.toString())
            assertTrue(line.startsWith("riscaldamento heavyvehicle_recommended_core:"), line)
            for (part in listOf("livelli", "landmark ", "landmark tollways", "shortcut", "nodi", "svolte", "archi")) assertTrue(part in line, "manca $part: $line")
            // un bilancio che non basta a nessun file: niente; le parti scelte: solo quei file
            val none = g.warm("driving-hgv", budget = 1)
            assertTrue("livelli" !in none && "archi" !in none, none)
            val core = g.warm("driving-hgv", parts = RoutingGraph.WARM_CORE)
            assertTrue("shortcut" in core && "nodi" !in core && "archi" !in core, core)
            val rest = g.warm("driving-hgv", parts = RoutingGraph.WARM_GRAPH)
            assertTrue("svolte" in rest && "archi" in rest && "livelli" !in rest, rest)
            assertEquals(before, weightOf(g), 1e-9)
        }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val line = g.warm("driving-hgv")
            assertTrue(line.contains(" 0 MB "), line)
            assertTrue(abs(weightOf(g) - weightOf(g)) < 1e-9)
        }
    }
}
