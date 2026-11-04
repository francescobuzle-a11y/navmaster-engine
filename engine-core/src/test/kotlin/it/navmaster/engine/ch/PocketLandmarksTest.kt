package it.navmaster.engine.ch

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Landmark in zone chiuse del core (come sul grafo italiano pubblicato: raggiunti da quasi tutto il
 * core, non ne raggiungono quasi niente, o il contrario): i potenziali devono restare limiti
 * inferiori coerenti, e la ricerca core-ALT dare lo stesso peso della ricerca sul core senza landmark.
 */
class PocketLandmarksTest {
    @Test
    fun `core-ALT stays exact with landmarks in closed areas`() {
        val dir = Files.createTempDirectory("nme-pocket").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        built.use { it.prepareCH(landmarkCount = 4) }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val key = "heavyvehicle_recommended_core"
            val storage = g.chStorages.getValue(key)
            val w = WeightingFactory.create(g.encoding, g.encoding.profile("heavyvehicle"), WeightingMethod.RECOMMENDED)
            val ch = CHGraph(g.graph, storage, w)
            val n = g.graph.nodeCount
            val core = (0 until n).filter { storage.level(it) >= n }
            // le zone chiuse: nodi del core che raggiungono meno del 5 % del core e sono raggiunti da più della metà (o il contrario)
            val rnd = Random(5)
            val pockets = ArrayList<Int>(); val sinks = ArrayList<Int>()
            for (attempt in 0 until 400) {
                if (pockets.size >= 2 && sinks.size >= 2) break
                val v = core[rnd.nextInt(core.size)]
                val one = CoreLandmarks.prepareAt(ch, intArrayOf(v))
                val from = core.count { one.fromLandmark(0, it).isFinite() }; val to = core.count { one.toLandmark(0, it).isFinite() }
                if (from * 20 < core.size && to * 2 > core.size && pockets.size < 2) pockets += v
                if (to * 20 < core.size && from * 2 > core.size && sinks.size < 2) sinks += v
            }
            println("zone chiuse trovate: ${pockets.size} (in avanti), ${sinks.size} (all'indietro)")
            assertTrue(pockets.isNotEmpty() && sinks.isNotEmpty(), "zone chiuse non trovate nel grafo di prova")
            val good = g.landmarks.getValue(key)
            val params = VehicleParameters(height = 4.0, weight = 12.0)
            g.landmarks.remove(key)
            val plain = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            check(plain.algorithm == "core_edge_based") { plain.algorithm }
            val bg = g.graph
            var diffs = 0; var pairs = 0
            for ((name, nodes) in listOf("in avanti" to pockets, "all'indietro" to sinks, "misti" to (pockets + sinks + good.landmarkNodes.take(1)))) {
                g.landmarks[key] = CoreLandmarks.prepareAt(ch, nodes.toIntArray())
                val alt = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
                repeat(200) {
                    val a = rnd.nextInt(n); val b = rnd.nextInt(n)
                    fun weight(r: Router) = try { r.route(r.snap(bg.lat(a), bg.lon(a), 400.0, 0), r.snap(bg.lat(b), bg.lon(b), 400.0, 1)).weight }
                        catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY } catch (e: PointNotFoundException) { Double.NaN }
                    val w1 = weight(alt); val w2 = weight(plain)
                    if (w1.isNaN() || w2.isNaN()) return@repeat
                    pairs++
                    if (!((w1.isInfinite() && w2.isInfinite()) || abs(w1 - w2) <= 0.05 + w2 * 1e-5)) { diffs++; if (diffs <= 5) println("[$name] $a→$b: core-ALT $w1, core senza landmark $w2") }
                }
            }
            g.landmarks[key] = good
            println("landmark in zone chiuse: $diffs differenze su $pairs viaggi")
            assertTrue(diffs == 0, "core-ALT diverso dalla ricerca sul core senza landmark in $diffs viaggi su $pairs")
        }
    }
}
