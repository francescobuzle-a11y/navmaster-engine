package it.navmaster.engine.routing

import it.navmaster.engine.index.LocationIndex
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.BooleanEncodedValue
import it.navmaster.engine.storage.DecimalEncodedValue
import it.navmaster.engine.storage.EncodedValueRegistry
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.TurnCostEncoding
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.Weighting
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class TurnRestrictionTest {
    private fun tmp() = Files.createTempDirectory("nme-tc").toFile().also { it.deleteOnExit() }

    /** Peso = distanza, accesso in entrambi i sensi: isola l'effetto delle svolte. */
    private class DistanceWeighting : Weighting {
        override val name = "distance"
        override fun minWeight(distanceMeters: Double) = distanceMeters
        override fun edgeWeight(edge: BaseGraph.EdgeIterator, reverse: Boolean) = edge.distance
        override fun edgeMillis(edge: BaseGraph.EdgeIterator, reverse: Boolean) = (edge.distance * 100).toLong()
    }

    /**
     * Quadrato 0-1-2-3 con diagonale 0-2:
     *   0 —100— 1
     *   |  \     |
     *  100  141 100
     *   |     \  |
     *   3 —100— 2
     * Da 0 a 2 la via più corta è la diagonale (141); se è vietato svoltare da (s→0) sulla diagonale, si passa da 1 o 3 (200).
     */
    private fun square(dir: File): Pair<BaseGraph, TurnCostStorage> {
        val reg = EncodedValueRegistry.Builder().add(BooleanEncodedValue("x")).build()
        val g = BaseGraph(GraphDirectory(dir), reg)
        val s = g.addNode(0.0, -0.001)      // nodo di partenza a ovest di 0
        val n0 = g.addNode(0.0, 0.0); val n1 = g.addNode(0.0, 0.0009); val n2 = g.addNode(-0.0009, 0.0009); val n3 = g.addNode(-0.0009, 0.0)
        val eS0 = g.addEdge(s, n0, 100.0)
        g.addEdge(n0, n1, 100.0); g.addEdge(n1, n2, 100.0); g.addEdge(n2, n3, 100.0); g.addEdge(n3, n0, 100.0)
        val diag = g.addEdge(n0, n2, 141.0)
        val tc = TurnCostStorage(GraphDirectory(dir), TurnCostEncoding(listOf("p")))
        // vietato: arrivando da eS0 in n0, imboccare la diagonale
        tc.set(eS0, n0, diag, tc.encoding.forbid(0, "p"))
        return g to tc
    }

    @Test
    fun `turn cost storage set, merge and reload`() {
        val dir = tmp()
        val enc = TurnCostEncoding(listOf("car-ors", "heavyvehicle"))
        TurnCostStorage(GraphDirectory(dir), enc).use { tc ->
            tc.set(1, 5, 2, enc.forbid(0, "car-ors"))
            tc.set(1, 5, 2, enc.forbid(0, "heavyvehicle")) // stessa svolta: OR dei flag
            tc.set(7, 5, 8, enc.forbid(0, "heavyvehicle"))
            tc.set(3, 9, 4, enc.forbid(0, "car-ors"))
            assertEquals(3, tc.count)
            assertTrue(enc.isForbidden(tc.flags(1, 5, 2), "car-ors") && enc.isForbidden(tc.flags(1, 5, 2), "heavyvehicle"))
            assertFalse(enc.isForbidden(tc.flags(7, 5, 8), "car-ors")); assertTrue(enc.isForbidden(tc.flags(7, 5, 8), "heavyvehicle"))
            assertEquals(0, tc.flags(1, 5, 9)); assertEquals(0, tc.flags(1, 6, 2))
            assertTrue(tc.hasEntries(5)); assertFalse(tc.hasEntries(6)); assertFalse(tc.hasEntries(100_000))
            tc.flush()
        }
        TurnCostStorage(GraphDirectory(dir), TurnCostEncoding(listOf("car-ors", "heavyvehicle"))).use { tc ->
            assertTrue(tc.loadExisting())
            assertEquals(3, tc.count)
            assertTrue(tc.encoding.isForbidden(tc.flags(3, 9, 4), "car-ors"))
        }
    }

    @Test
    fun `edge-based search obeys a turn restriction and forbids u-turns`() {
        val (g, tc) = square(tmp())
        val w = DistanceWeighting()
        // senza divieti: diagonale, 100 + 141
        val free = BidirectionalDijkstra(g, w).also { it.addSource(0, 0.0); it.addTarget(3, 0.0) }
        assertTrue(free.run()); assertEquals(241.0, free.bestWeight, 1e-9)
        // edge-based con divieto: 100 + 200 passando per il nodo 2 o 4
        val restricted = EdgeBasedBidirectionalDijkstra(g, w, TurnCostProvider(tc, "p")).also { it.addSource(0, 0.0); it.addTarget(3, 0.0) }
        assertTrue(restricted.run()); assertEquals(300.0, restricted.bestWeight, 1e-9)
        val path = restricted.extractPath()!!
        assertEquals(3, path.edges.size)
        assertEquals(0, path.startNode); assertEquals(3, path.endNode)
        assertFalse(path.edges.contains(5), "la diagonale (arco 5) è vietata")
        for (i in 1 until path.nodes.size) {
            val e = path.edges[i - 1]
            assertTrue(setOf(g.nodeA(e), g.nodeB(e)) == setOf(path.nodes[i - 1], path.nodes[i]), "arco $e non collega ${path.nodes[i - 1]}-${path.nodes[i]}")
        }
        // edge-based senza divieti dà lo stesso peso del node-based
        val tcEmpty = TurnCostStorage(GraphDirectory(tmp()), TurnCostEncoding(listOf("p")))
        val same = EdgeBasedBidirectionalDijkstra(g, w, TurnCostProvider(tcEmpty, "p")).also { it.addSource(0, 0.0); it.addTarget(3, 0.0) }
        assertTrue(same.run()); assertEquals(241.0, same.bestWeight, 1e-9)
        // sorgente = destinazione: percorso vuoto
        val zero = EdgeBasedBidirectionalDijkstra(g, w, TurnCostProvider(tc, "p")).also { it.addSource(2, 0.0); it.addTarget(2, 0.0) }
        assertTrue(zero.run()); assertEquals(0.0, zero.bestWeight); assertTrue(zero.extractPath()!!.isEmpty)
    }

    @Test
    fun `Heidelberg restrictions are applied and routes stay consistent`() {
        val encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        val dir = tmp()
        val tcEnc = TurnCostEncoding(encoding.profiles.map { it.profile.name })
        BaseGraph(GraphDirectory(dir), encoding.registry).use { graph ->
            TurnCostStorage(GraphDirectory(dir), tcEnc).use { tc ->
                val stats = OsmGraphImporter(graph, encoding, tc).import(File("../data/heidelberg.test.pbf"))
                println("divieti: letti ${stats.turnRestrictionRelations}, applicati ${stats.turnRestrictions}, voci ${tc.count}")
                assertTrue(stats.turnRestrictions > 300, "pochi divieti applicati: ${stats.turnRestrictions}")
                assertTrue(tc.count >= stats.turnRestrictions)
                val index = LocationIndex.build(graph)
                val hgv = encoding.profile("heavyvehicle")
                val w = WeightingFactory.create(encoding, hgv, WeightingMethod.FASTEST)
                val plain = Router(graph, index, w, EdgeFilter.ALL)
                val withTurns = Router(graph, index, w, EdgeFilter.ALL, TurnCostProvider(tc, "heavyvehicle"))
                val pairs = listOf(49.4094 to 8.6937, 49.4186 to 8.7606, 49.4036 to 8.6757, 49.3989 to 8.6724)
                var differ = 0
                for (i in pairs.indices) for (j in pairs.indices) if (i != j) {
                    val a = pairs[i]; val b = pairs[j]
                    val p1 = plain.route(plain.snap(a.first, a.second, 400.0, 0), plain.snap(b.first, b.second, 400.0, 1))
                    val p2 = withTurns.route(withTurns.snap(a.first, a.second, 400.0, 0), withTurns.snap(b.first, b.second, 400.0, 1))
                    assertTrue(p2.weight >= p1.weight - 1e-6, "con i divieti il costo non può diminuire: ${p2.weight} < ${p1.weight}")
                    if (p2.weight > p1.weight + 1e-6) differ++
                    for (k in 1 until p2.edges.size) assertEquals(p2.edges[k - 1].adjNode, p2.edges[k].baseNode)
                }
                println("percorsi cambiati dai divieti: $differ su ${pairs.size * (pairs.size - 1)}")
            }
        }
    }
}
