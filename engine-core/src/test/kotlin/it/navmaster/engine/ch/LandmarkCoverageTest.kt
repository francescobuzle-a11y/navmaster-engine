package it.navmaster.engine.ch

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * I landmark del core devono raggiungere (ed essere raggiunti da) quasi tutto il core, qualunque sia
 * il nodo casuale da cui parte la scelta: un landmark in una zona chiusa (sensi unici, restrizioni)
 * dà distanze infinite per il resto e limiti nulli alla ricerca core-ALT.
 */
class LandmarkCoverageTest {
    @Test
    fun `landmarks reach the whole core from any start`() {
        val dir = Files.createTempDirectory("nme-lmc").toFile().also { it.deleteOnExit() }
        val (rg, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        rg.use { g ->
            g.prepareCH(landmarkCount = 4)
            val storage = g.chStorages.getValue("heavyvehicle_recommended_core")
            val w = WeightingFactory.create(g.encoding, g.encoding.profile("heavyvehicle"), WeightingMethod.RECOMMENDED)
            val ch = CHGraph(g.graph, storage, w)
            val core = (0 until g.graph.nodeCount).filter { storage.level(it) >= g.graph.nodeCount }
            var worst = 100
            for (seed in 1L..24L) {
                val lm = CoreLandmarks.prepare(ch, w, 4, seed = seed)
                for (l in 0 until lm.landmarkCount) {
                    val from = 100 * core.count { lm.fromLandmark(l, it).isFinite() } / core.size
                    val to = 100 * core.count { lm.toLandmark(l, it).isFinite() } / core.size
                    if (minOf(from, to) < worst) { worst = minOf(from, to); println("seme $seed landmark $l: raggiunge $from %, raggiunto da $to % del core") }
                }
            }
            println("copertura peggiore: $worst % del core (${core.size} nodi)")
            assertTrue(worst >= 80, "un landmark copre solo il $worst % del core")
        }
    }
}
