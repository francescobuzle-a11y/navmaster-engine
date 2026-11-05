package it.navmaster.engine.ch

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.routing.BidirectionalDijkstra
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.EdgeFilterFactory
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * Gli insiemi di landmark con delle strade escluse (ORS `lmsets`): preparati sul core senza
 * autostrade (o senza pedaggi), danno limiti validi e più stretti alle richieste che le evitano.
 * La ricerca deve restare esatta (stesso peso del Dijkstra filtrato) e visitare meno nodi che con
 * l'insieme completo; ogni richiesta deve prendere l'insieme giusto, anche dopo il ricaricamento
 * da disco (l'indice del core condiviso con l'insieme completo).
 */
class AvoidLandmarksTest {
    @Test
    fun `landmark sets prepared without motorways or tollways are exact, tighter and chosen per request`() {
        val dir = Files.createTempDirectory("nme-lmsets").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { g ->
            g.prepareCH(landmarkCount = 8, landmarkSets = listOf(AvoidFeatureFlags.TOLLWAYS, AvoidFeatureFlags.HIGHWAYS))
            assertEquals(listOf("tollways", "highways"), g.landmarkSetNames)
            val sh = g.encoding.shared
            val it = g.graph.createEdgeIterator()
            var motorways = 0; var tolls = 0
            for (e in 0 until g.graph.edgeCount) { it.setEdge(e, g.graph.nodeA(e)); if (it.get(sh.motorway)) motorways++; if (it.get(sh.toll) == Toll.ALL || it.get(sh.toll) == Toll.HGV) tolls++ }
            println("Heidelberg: $motorways archi di autostrada, $tolls a pedaggio")
            assertTrue(motorways > 0)
            for (key in g.chStorages.keys.filter { it.endsWith("_core") }) {
                val sets = g.avoidLandmarks.getValue(key)
                assertEquals(listOf(AvoidFeatureFlags.TOLLWAYS, AvoidFeatureFlags.HIGHWAYS), sets.map { it.avoidFeatures })
                val all = g.landmarks.getValue(key)
                assertEquals("allow_all", all.setName)
                // la scelta: l'insieme ammissibile (strade escluse ⊆ evitate) con più strade escluse
                assertSame(all, g.landmarksFor(key, 0))
                assertSame(all, g.landmarksFor(key, AvoidFeatureFlags.FERRIES))
                assertSame(sets[0], g.landmarksFor(key, AvoidFeatureFlags.TOLLWAYS))
                assertSame(sets[0], g.landmarksFor(key, AvoidFeatureFlags.TOLLWAYS or AvoidFeatureFlags.FERRIES))
                assertSame(sets[1], g.landmarksFor(key, AvoidFeatureFlags.HIGHWAYS or AvoidFeatureFlags.FORDS))
                // a parità (entrambi ammissibili, una strada esclusa ciascuno) il primo preparato
                assertSame(sets[0], g.landmarksFor(key, AvoidFeatureFlags.HIGHWAYS or AvoidFeatureFlags.TOLLWAYS))
            }
            checkSearches(g)
        }
        // ricaricati da disco, in RAM e mappati: stessi insiemi, stesse scelte, stesse ricerche
        for (mmap in listOf(false, true)) RoutingGraph.load(dir, mmap).use { g ->
            assertEquals(listOf("tollways", "highways"), g.landmarkSetNames)
            for (key in g.chStorages.keys.filter { it.endsWith("_core") }) {
                assertEquals(listOf(AvoidFeatureFlags.TOLLWAYS, AvoidFeatureFlags.HIGHWAYS), g.avoidLandmarks.getValue(key).map { it.avoidFeatures })
                assertEquals(0, g.landmarks.getValue(key).avoidFeatures)
            }
            checkSearches(g)
            // dal router: la richiesta che evita le autostrade usa l'insieme `highways`, quella che evita i traghetti quello completo
            val a = 49.4094 to 8.6935; val b = 49.3800 to 8.6600
            for ((avoid, expected) in listOf(AvoidFeatureFlags.HIGHWAYS to " ALT highways", AvoidFeatureFlags.TOLLWAYS to " ALT tollways", AvoidFeatureFlags.FERRIES to " ALT", 0 to " ALT")) {
                val r = g.router(RouteOptions(profile = "driving-hgv", avoidFeatures = avoid, vehicleParameters = it.navmaster.engine.weighting.VehicleParameters(height = 4.0)))
                assertEquals("core_alt_edge_based", r.algorithm)
                val p = r.route(r.snap(a.first, a.second, 400.0, 0), r.snap(b.first, b.second, 400.0, 1))
                assertTrue(p.weight.isFinite())
                assertTrue(r.lastStats.endsWith(expected), "evitando $avoid: ${r.lastStats}")
            }
        }
    }

    /** Per ogni core e insieme: peso uguale al Dijkstra filtrato (node-based) e al core con l'insieme completo (edge-based), meno nodi visitati. */
    private fun checkSearches(g: RoutingGraph) {
        val rnd = Random(7)
        for ((key, sets) in g.avoidLandmarks) {
            val name = key.substringBefore('_')
            val method = WeightingMethod.valueOf(key.substringAfter('_').substringBefore('_').uppercase())
            val w = WeightingFactory.create(g.encoding, g.encoding.profile(name), method)
            val ch = CHGraph(g.graph, g.chStorages.getValue(key), w)
            val all = g.landmarks.getValue(key)
            for (lm in sets) {
                val filter = EdgeFilterFactory.create(g.encoding, name, avoidFeatures = lm.avoidFeatures)
                val tcp = TurnCostProvider(g.turnCosts, name)
                var pairs = 0; var withSet = 0L; var withAll = 0L; var unreachableByLandmarks = 0
                repeat(40) {
                    val s = rnd.nextInt(g.graph.nodeCount); val t = rnd.nextInt(g.graph.nodeCount)
                    val plain = BidirectionalDijkstra(g.graph, w, filter).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                    val found = plain.run()
                    val alt = CoreDijkstra(ch, w, filter, null, lm).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                    assertEquals(found, alt.run(), "[$key ${lm.setName}] raggiungibilità diversa per $s→$t")
                    val base = CoreDijkstra(ch, w, filter, null, all).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                    base.run()
                    if (found) {
                        assertTrue(abs(plain.bestWeight - alt.bestWeight) <= 0.01 + plain.bestWeight * 1e-5, "[$key ${lm.setName}] $s→$t: dijkstra ${plain.bestWeight} vs core-ALT ${alt.bestWeight}")
                        pairs++; withSet += alt.visitedNodes; withAll += base.visitedNodes
                    }
                    // edge-based: i due insiemi devono dare lo stesso peso (stessa ricerca, limiti diversi)
                    val e1 = CoreDijkstra(ch, w, filter, tcp, lm).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                    val e2 = CoreDijkstra(ch, w, filter, tcp, all).also { it.addSource(s, 0.0); it.addTarget(t, 0.0) }
                    assertEquals(e2.run(), e1.run(), "[$key ${lm.setName}] raggiungibilità edge-based diversa per $s→$t")
                    if (e1.bestWeight.isFinite()) assertTrue(abs(e1.bestWeight - e2.bestWeight) <= 0.01 + e2.bestWeight * 1e-5, "[$key ${lm.setName}] $s→$t edge-based: ${lm.setName} ${e1.bestWeight} vs allow_all ${e2.bestWeight}")
                }
                // i limiti dell'insieme: finiti dove il core senza quelle strade è connesso
                val core = (0 until g.graph.nodeCount).filter { g.chStorages.getValue(key).level(it) >= g.graph.nodeCount }
                for (l in 0 until lm.landmarkCount) if (core.none { lm.fromLandmark(l, it).isFinite() }) unreachableByLandmarks++
                assertEquals(0, unreachableByLandmarks, "[$key ${lm.setName}] landmark senza distanze")
                println("[$key ${lm.setName}] coppie $pairs, nodi visitati: con l'insieme ${withSet / maxOf(1, pairs)}, con allow_all ${withAll / maxOf(1, pairs)}")
                assertTrue(pairs >= 10, "[$key ${lm.setName}] troppe poche coppie raggiungibili: $pairs")
                // su Heidelberg (poche strade a pedaggio) il guadagno è piccolo e i landmark sono nodi diversi: non più
                // del 10 % in più di nodi; sui grafi veri la differenza è grande (Italia, evita pedaggi: 337 000 nodi → vedi CI)
                assertTrue(withSet <= withAll * 1.1 + 50, "[$key ${lm.setName}] l'insieme dedicato visita molti più nodi ($withSet) di quello completo ($withAll)")
            }
        }
    }

    @Test
    fun `set names round-trip and a narrow core prepares no sets`() {
        assertEquals("allow_all", CoreLandmarks.setName(0))
        assertEquals("highways_tollways", CoreLandmarks.setName(AvoidFeatureFlags.HIGHWAYS or AvoidFeatureFlags.TOLLWAYS))
        assertEquals(AvoidFeatureFlags.HIGHWAYS or AvoidFeatureFlags.TOLLWAYS, CoreLandmarks.parseSetName("highways_tollways"))
        assertEquals(AvoidFeatureFlags.HIGHWAYS or AvoidFeatureFlags.TOLLWAYS, CoreLandmarks.parseSetName("tollways+highways"))
        assertEquals(0, CoreLandmarks.parseSetName("allow_all"))
        assertEquals("heavyvehicle_recommended_core_tollways", CoreLandmarks.fileName("heavyvehicle_recommended_core", AvoidFeatureFlags.TOLLWAYS))
        assertEquals("heavyvehicle_recommended_core", CoreLandmarks.fileName("heavyvehicle_recommended_core", 0))
        val dir = Files.createTempDirectory("nme-lmsets-narrow").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        built.use { g ->
            // col core ridotto le strade da evitare non sono nel core (e possono stare negli shortcut): nessun insieme
            g.prepareCH(landmarkCount = 4, coreWide = false)
            assertTrue(g.avoidLandmarks.isEmpty())
            assertEquals(emptyList(), g.landmarkSetNames)
            assertNotNull(g.landmarksFor("heavyvehicle_recommended_core", AvoidFeatureFlags.TOLLWAYS))
            assertNull(g.avoidLandmarks["heavyvehicle_recommended_core"])
        }
        RoutingGraph.load(dir, false).use { g -> assertTrue(g.avoidLandmarks.isEmpty()) }
    }
}
