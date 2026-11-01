package it.navmaster.engine.ch

import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.BidirectionalDijkstra
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.BooleanEncodedValue
import it.navmaster.engine.storage.DecimalEncodedValue
import it.navmaster.engine.storage.EncodedValueRegistry
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.weighting.Weighting
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class CHTest {
    private fun tmp() = Files.createTempDirectory("nme-ch").toFile().also { it.deleteOnExit() }

    /** Peso = distanza nel verso consentito dal flag `oneway` (A→B soltanto se impostato). */
    private class DistWeighting(private val oneway: BooleanEncodedValue) : Weighting {
        override val name = "dist"
        override fun minWeight(distanceMeters: Double) = distanceMeters
        override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean): Double {
            val travellingAtoB = edge.reverse == reverse
            return if (!travellingAtoB && edge.get(oneway)) Double.POSITIVE_INFINITY else edge.distance
        }
        override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean) = (edge.distance * 100).toLong()
    }

    /** Griglia casuale con alcuni sensi unici: confronta CH e Dijkstra su tutte le coppie. */
    @Test
    fun `CH gives the same weights as plain Dijkstra on a random grid`() {
        val reg = EncodedValueRegistry.Builder().add(BooleanEncodedValue("oneway")).build()
        val oneway = reg.bool("oneway")
        val dir = tmp()
        BaseGraph(GraphDirectory(dir), reg).use { g ->
            val rnd = Random(7)
            val size = 9
            for (r in 0 until size) for (c in 0 until size) g.addNode(45.0 + r * 0.001, 9.0 + c * 0.001)
            fun id(r: Int, c: Int) = r * size + c
            for (r in 0 until size) for (c in 0 until size) {
                if (c + 1 < size && rnd.nextInt(10) > 0) addEdge(g, oneway, id(r, c), id(r, c + 1), 50 + rnd.nextInt(100), rnd.nextInt(5) == 0)
                if (r + 1 < size && rnd.nextInt(10) > 0) addEdge(g, oneway, id(r, c), id(r + 1, c), 50 + rnd.nextInt(100), rnd.nextInt(5) == 0)
            }
            val w = DistWeighting(oneway)
            val storage = CHStorage(GraphDirectory(dir), "test", g.edgeCount, g.nodeCount)
            val prep = CHPreparation(g, w, storage)
            prep.prepare()
            assertTrue(prep.shortcutsAdded > 0, "nessuno shortcut creato")
            assertEquals(0, storage.coreNodeCount)
            val ch = CHGraph(g, storage, w)
            var compared = 0; var reachable = 0
            for (s in 0 until g.nodeCount) for (t in 0 until g.nodeCount) {
                val plain = BidirectionalDijkstra(g, w).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                val found = plain.run()
                val q = DijkstraBidirectionCH(ch, w).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                val chFound = q.run()
                assertEquals(found, chFound, "raggiungibilità diversa per $s→$t")
                compared++
                if (!found) continue
                reachable++
                assertTrue(abs(plain.bestWeight - q.bestWeight) <= 0.002 + plain.bestWeight * 1e-6, "$s→$t: dijkstra ${plain.bestWeight} vs CH ${q.bestWeight}")
                val path = q.extractPath()!!
                assertEquals(s, path.startNode); assertEquals(t, path.endNode)
                // il percorso scompattato è contiguo e il suo peso coincide
                var sum = 0.0
                val it = g.createEdgeIterator()
                for (i in path.edges.indices) {
                    val e = path.edges[i]; val base = path.nodes[i]
                    it.setEdge(e, base)
                    assertEquals(path.nodes[i + 1], it.adjNode, "arco $e non contiguo in $s→$t")
                    val ew = w.edgeWeight(it, false)
                    assertTrue(ew.isFinite(), "arco $e percorso contro senso unico in $s→$t")
                    sum += ew
                }
                assertEquals(plain.bestWeight, sum, 1e-6)
            }
            assertTrue(reachable > compared / 2, "pochi raggiungibili: $reachable/$compared")
            storage.flush()
        }
    }

    private fun addEdge(g: BaseGraph, oneway: BooleanEncodedValue, a: Int, b: Int, dist: Int, ow: Boolean) {
        val e = g.addEdge(a, b, dist.toDouble())
        val f = g.encodedValues.newFlags(); oneway.set(f, ow); g.setFlags(e, f)
    }

    @Test
    fun `CH on Heidelberg matches Dijkstra and is faster`() {
        val encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        val dir = tmp()
        BaseGraph(GraphDirectory(dir), encoding.registry).use { g ->
            OsmGraphImporter(g, encoding).import(File("../data/heidelberg.test.pbf"))
            val w = WeightingFactory.create(encoding, encoding.profile("heavyvehicle"), WeightingMethod.RECOMMENDED)
            val storage = CHStorage(GraphDirectory(dir), "hgv_recommended", g.edgeCount, g.nodeCount)
            val t0 = System.currentTimeMillis()
            val prep = CHPreparation(g, w, storage)
            prep.prepare()
            val prepMs = System.currentTimeMillis() - t0
            println("CH Heidelberg: ${storage.shortcutCount} shortcut per ${g.edgeCount} archi, preparazione $prepMs ms")
            val ch = CHGraph(g, storage, w)
            val rnd = Random(42)
            var plainNs = 0L; var chNs = 0L; var plainVisited = 0L; var chVisited = 0L; var pairs = 0
            repeat(150) {
                val s = rnd.nextInt(g.nodeCount); val t = rnd.nextInt(g.nodeCount)
                val a = System.nanoTime()
                val plain = BidirectionalDijkstra(g, w).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                val found = plain.run()
                val b = System.nanoTime()
                val q = DijkstraBidirectionCH(ch, w).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                val chFound = q.run()
                val c = System.nanoTime()
                assertEquals(found, chFound, "raggiungibilità diversa per $s→$t")
                if (!found) return@repeat
                pairs++
                plainNs += b - a; chNs += c - b; plainVisited += plain.visitedNodes; chVisited += q.visitedNodes
                assertTrue(abs(plain.bestWeight - q.bestWeight) <= 0.01 + plain.bestWeight * 1e-5, "$s→$t: dijkstra ${plain.bestWeight} vs CH ${q.bestWeight}")
                val path = q.extractPath()!!
                assertEquals(s, path.startNode); assertEquals(t, path.endNode)
            }
            println("coppie $pairs: Dijkstra %.2f ms (%d nodi) vs CH %.2f ms (%d nodi) in media".format(plainNs / 1e6 / pairs, plainVisited / pairs, chNs / 1e6 / pairs, chVisited / pairs))
            assertTrue(chVisited < plainVisited / 5, "la CH dovrebbe visitare molti meno nodi: $chVisited vs $plainVisited")
            storage.flush()
            // ricarica
            val reloaded = CHStorage(GraphDirectory(dir), "hgv_recommended", g.edgeCount, g.nodeCount)
            assertTrue(reloaded.loadExisting()); assertEquals(storage.shortcutCount, reloaded.shortcutCount)
        }
    }
}
