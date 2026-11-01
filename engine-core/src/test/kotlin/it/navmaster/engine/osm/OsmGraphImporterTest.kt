package it.navmaster.engine.osm

import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class OsmGraphImporterTest {
    private val pbf = File("../data/heidelberg.test.pbf")

    @Test
    fun `import Heidelberg for car and truck`() {
        assertTrue(pbf.exists(), "manca ${pbf.absolutePath}")
        val encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        val dir = Files.createTempDirectory("nme-import").toFile().also { it.deleteOnExit() }
        BaseGraph(GraphDirectory(dir), encoding.registry).use { graph ->
            val stats = OsmGraphImporter(graph, encoding).import(pbf)
            println(stats)
            assertTrue(stats.osmNodes > 100_000 && stats.osmWays > 10_000, "estratto inatteso: $stats")
            assertTrue(stats.acceptedWays in 5_000..stats.osmWays)
            assertTrue(graph.nodeCount > 5_000, "pochi nodi: ${graph.nodeCount}")
            assertTrue(graph.edgeCount > graph.nodeCount, "pochi archi: ${graph.edgeCount} per ${graph.nodeCount} nodi")
            assertTrue(stats.turnRestrictionRelations > 10)
            assertEquals(0, stats.turnRestrictions) // nessun TurnCostStorage passato
            // Heidelberg e dintorni (l'estratto contiene way lunghe che sconfinano fino a lon 7.7–9.1)
            assertTrue(graph.bounds.minLat > 49.0 && graph.bounds.maxLat < 50.0 && graph.bounds.minLon > 7.0 && graph.bounds.maxLon < 10.0, "${graph.bounds}")

            val car = encoding.profile("car-ors"); val hgv = encoding.profile("heavyvehicle")
            val it = graph.createEdgeIterator()
            var motorways = 0; var namedEdges = 0; var pillarEdges = 0; var totalDist = 0.0
            var truckSlowerSomewhere = false; var hgvBlockedSomewhere = false
            for (e in 0 until graph.edgeCount) {
                it.setEdge(e, graph.nodeA(e))
                totalDist += it.distance
                if (it.name.isNotEmpty()) namedEdges++
                if (it.fetchWayGeometry(FetchMode.PILLAR_ONLY).size > 0) pillarEdges++
                if (it.get(encoding.shared.roadClass) == RoadClass.MOTORWAY) {
                    motorways++
                    if (it.get(car.access)) {
                        assertTrue(it.get(car.speed) >= 60.0, "autostrada lenta per l'auto: ${it.get(car.speed)}")
                    }
                }
                if (it.get(car.access) && it.get(hgv.access) && it.get(hgv.speed) < it.get(car.speed)) truckSlowerSomewhere = true
                if (it.get(car.access) && !it.get(hgv.access)) hgvBlockedSomewhere = true
            }
            assertTrue(motorways > 0, "nessuna autostrada trovata")
            assertTrue(namedEdges > graph.edgeCount / 4, "pochi archi con nome: $namedEdges")
            assertTrue(pillarEdges > graph.edgeCount / 4, "pochi archi con geometria: $pillarEdges")
            assertTrue(truckSlowerSomewhere, "il camion dovrebbe essere più lento dell'auto da qualche parte")
            assertTrue(hgvBlockedSomewhere, "ci si aspetta qualche strada vietata ai camion ma non alle auto")
            assertTrue(totalDist > 500_000, "rete troppo corta: $totalDist m")

            graph.flush()
        }
        // ricarica da disco
        BaseGraph(GraphDirectory(dir), GraphEncoding(listOf(CarProfile(), HgvProfile())).registry).use { g ->
            assertTrue(g.loadExisting())
            assertTrue(g.nodeCount > 5_000)
            assertEquals(g.edgeCount, g.edgeCount)
        }
    }
}
