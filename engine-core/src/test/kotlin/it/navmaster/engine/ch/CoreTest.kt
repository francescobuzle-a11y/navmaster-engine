package it.navmaster.engine.ch

import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.BidirectionalDijkstra
import it.navmaster.engine.routing.EdgeBasedBidirectionalDijkstra
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.TurnCostEncoding
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.EdgeFilterFactory
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class CoreTest {
    @Test
    fun `core routing equals filtered Dijkstra, node-based and edge-based`() {
        val encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        val dir = Files.createTempDirectory("nme-core").toFile().also { it.deleteOnExit() }
        val gdir = GraphDirectory(dir)
        BaseGraph(gdir, encoding.registry).use { g ->
            TurnCostStorage(gdir, TurnCostEncoding(encoding.profiles.map { it.profile.name })).use { tc ->
                OsmGraphImporter(g, encoding, tc).import(File("../data/heidelberg.test.pbf"))
                val pe = encoding.profile("heavyvehicle")
                val w = WeightingFactory.create(encoding, pe, WeightingMethod.RECOMMENDED)
                val selector = CoreEdgeSelector(encoding, "heavyvehicle", tc)
                val core = selector.coreNodes(g)
                val coreCount = core.count { it }
                val storage = CHStorage(gdir, "hgv_core", g.edgeCount, g.nodeCount)
                val t0 = System.currentTimeMillis()
                // come ORS `RestrictedEdgesWeighting`: gli archi del core non partecipano alla contrazione
                // (né come shortcut né come testimoni), perché una richiesta potrebbe escluderli
                CHPreparation(g, w, storage, filter = EdgeFilter { !selector.isCoreEdge(it) }).apply { doNotContract = { core[it] } }.prepare()
                println("core: $coreCount nodi su ${g.nodeCount} (${100 * coreCount / g.nodeCount} %), ${storage.shortcutCount} shortcut, ${System.currentTimeMillis() - t0} ms")
                assertEquals(coreCount, storage.coreNodeCount)
                assertTrue(coreCount in 1 until g.nodeCount)
                val ch = CHGraph(g, storage, w)
                val lm = CoreLandmarks.prepare(ch, w, 8)
                assertEquals(8, lm.landmarkCount)
                val rnd = Random(11)
                val scenarios = listOf(
                    "nessun filtro" to EdgeFilterFactory.create(encoding, "heavyvehicle"),
                    "altezza 4 m, peso 12 t" to EdgeFilterFactory.create(encoding, "heavyvehicle", params = VehicleParameters(height = 4.0, weight = 12.0)),
                    "mezzo da 3 m e 7,5 t" to EdgeFilterFactory.create(encoding, "heavyvehicle", params = VehicleParameters(height = 3.0, weight = 7.5)),
                )
                for ((label, filter) in scenarios) {
                    var pairs = 0; var coreVisited = 0L; var altVisited = 0L; var plainVisited = 0L
                    repeat(60) {
                        val s = rnd.nextInt(g.nodeCount); val t = rnd.nextInt(g.nodeCount)
                        // node-based
                        val plain = BidirectionalDijkstra(g, w, filter).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                        val found = plain.run()
                        val cd = CoreDijkstra(ch, w, filter, null).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                        assertEquals(found, cd.run(), "[$label] raggiungibilità node-based diversa per $s→$t")
                        val alt = CoreDijkstra(ch, w, filter, null, lm).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                        assertEquals(found, alt.run(), "[$label] raggiungibilità core-ALT diversa per $s→$t")
                        if (found) {
                            assertTrue(abs(plain.bestWeight - cd.bestWeight) <= 0.01 + plain.bestWeight * 1e-5, "[$label] $s→$t node-based: dijkstra ${plain.bestWeight} vs core ${cd.bestWeight}")
                            assertTrue(abs(plain.bestWeight - alt.bestWeight) <= 0.01 + plain.bestWeight * 1e-5, "[$label] $s→$t core-ALT: dijkstra ${plain.bestWeight} vs alt ${alt.bestWeight}")
                            checkPath(g, cd.extractPath()!!, s, t); checkPath(g, alt.extractPath()!!, s, t)
                            pairs++; coreVisited += cd.visitedNodes; altVisited += alt.visitedNodes; plainVisited += plain.visitedNodes
                        }
                        // edge-based con divieti di svolta
                        val tcp = TurnCostProvider(tc, "heavyvehicle")
                        val eb = EdgeBasedBidirectionalDijkstra(g, w, tcp, filter).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                        val ebFound = eb.run()
                        val cde = CoreDijkstra(ch, w, filter, tcp, lm).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                        assertEquals(ebFound, cde.run(), "[$label] raggiungibilità edge-based diversa per $s→$t")
                        if (ebFound) {
                            assertTrue(abs(eb.bestWeight - cde.bestWeight) <= 0.01 + eb.bestWeight * 1e-5, "[$label] $s→$t edge-based core-ALT: dijkstra ${eb.bestWeight} vs core ${cde.bestWeight}")
                            checkPath(g, cde.extractPath()!!, s, t)
                        }
                    }
                    println("[$label] coppie $pairs, nodi visitati: core-ALT ${altVisited / maxOf(1, pairs)}, core ${coreVisited / maxOf(1, pairs)}, dijkstra ${plainVisited / maxOf(1, pairs)}")
                    assertTrue(coreVisited < plainVisited, "[$label] il core dovrebbe visitare meno nodi")
                    assertTrue(altVisited <= coreVisited, "[$label] i landmark non dovrebbero aumentare i nodi visitati")
                }
            }
        }
    }

    private fun checkPath(g: BaseGraph, p: it.navmaster.engine.routing.SearchPath, s: Int, t: Int) {
        assertEquals(s, p.startNode); assertEquals(t, p.endNode)
        for (i in p.edges.indices) {
            val e = p.edges[i]
            assertTrue(setOf(g.nodeA(e), g.nodeB(e)) == setOf(p.nodes[i], p.nodes[i + 1]), "arco $e non collega ${p.nodes[i]}-${p.nodes[i + 1]}")
        }
    }
}
