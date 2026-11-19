package it.navmaster.engine

import it.navmaster.engine.osm.OsmSink
import it.navmaster.engine.osm.OsmSource
import it.navmaster.engine.osm.OsmWay
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfileOptions
import it.navmaster.engine.routing.Router
import it.navmaster.engine.weighting.VehicleParameters
import java.nio.file.Files
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * I divieti di transito ([it.navmaster.engine.profile.HgvTransit]) in un grafo intero: CH, core e
 * landmark sono preparati col ×10 del camion, e il percorso del camion gira dal tratto vietato con ogni
 * algoritmo; l'auto (anche con le misure di un camper) no.
 */
class HgvTransitGraphTest {
    /**
     * A–M–B: secondaria corta col divieto del Fernpass (`maxweightrating:hgv=7.5` + deroga per destinazione);
     * A–D1–D2–B: statale più lunga. B–C1–C2: limiti di classe del solo camion senza deroga.
     */
    private object Area : OsmSource {
        val nodes = listOf(
            Triple(1L, 47.0, 10.0), Triple(3L, 47.0, 10.05), Triple(2L, 47.0, 10.1),
            Triple(4L, 47.05, 10.0), Triple(5L, 47.05, 10.1),
            Triple(6L, 46.99, 10.1), Triple(7L, 46.98, 10.1),
        )
        val fernpass = mapOf("highway" to "secondary", "ref" to "B 179", "maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ destination; none @ delivery")
        val ways = listOf(
            OsmWay(10, fernpass, longArrayOf(1, 3)),
            OsmWay(11, fernpass, longArrayOf(3, 2)),
            OsmWay(12, mapOf("highway" to "primary"), longArrayOf(1, 4, 5, 2)),
            OsmWay(13, mapOf("highway" to "secondary", "maxweight:hgv" to "7.5"), longArrayOf(2, 6)),
            OsmWay(14, mapOf("highway" to "secondary", "maxweightrating:hgv" to "7.5"), longArrayOf(6, 7)),
        )
        override val nodeCount get() = nodes.size.toLong()
        override val wayCount get() = ways.size.toLong()
        override fun read(sink: OsmSink, nodes: Boolean, ways: Boolean, relations: Boolean) {
            if (nodes) for ((id, lat, lon) in this.nodes) sink.node(id, lat, lon, null)
            if (ways) for (w in this.ways) sink.way(w)
        }
    }

    private val a = 47.0 to 10.0
    private val m = 47.0 to 10.05
    private val b = 47.0 to 10.1
    private val c2 = 46.98 to 10.1

    private fun Router.path(from: Pair<Double, Double>, to: Pair<Double, Double>) = route(snap(from.first, from.second, 200.0, 0), snap(to.first, to.second, 200.0, 1))

    @Test
    fun `CH and core route the truck around the banned stretch`() {
        val dir = Files.createTempDirectory("nme-trg").toFile().also { it.deleteOnExit() }
        val profiles = listOf(
            CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "AT")),
            HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "AT")),
        )
        val (built, stats) = RoutingGraph.build(Area, dir, profiles)
        built.use { it.prepareCH(withCore = true, landmarkSets = emptyList()) }
        assertEquals(2, stats.hgvTransitEdges)
        RoutingGraph.load(dir, mmap = false).use { rg ->
            assertTrue(rg.encoding.shared.withTransit)
            val direct = 7_600.0  // A–M–B, circa
            val detour = 18_700.0 // A–D1–D2–B, circa
            val truck = VehicleParameters(weight = 40.0, height = 4.0)
            val core = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = truck))
            val ch = rg.router(RouteOptions(profile = "driving-hgv", turnCosts = false))
            assertEquals("ch", ch.algorithm)
            val plain = rg.router(RouteOptions(profile = "driving-hgv", turnCosts = false, allowCH = false))
            val eb = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = truck, allowCH = false))
            for ((name, r) in listOf("core" to core, "ch" to ch, "dijkstra" to plain, "dijkstra edge-based" to eb)) {
                val p = r.path(a, b)
                println("transito, camion A→B ($name, ${r.algorithm}): %.0f m, peso %.1f".format(p.distance, p.weight))
                assertEquals(detour, p.distance, 300.0, "$name: il camion gira dalla statale")
                // a metà del tratto vietato ci arriva dal capo vicino (solo destinazione, non vietato)
                assertEquals(direct / 2, r.path(a, m).distance, 300.0, "$name: A → M")
            }
            assertEquals(eb.path(a, b).weight, core.path(a, b).weight, 0.05 + 1e-4 * core.path(a, b).weight)
            assertEquals(plain.path(a, b).weight, ch.path(a, b).weight, 0.05 + 1e-4 * ch.path(a, b).weight)

            // l'auto, anche col camper: dalla secondaria, e i limiti di classe del camion non la fermano
            val camper = VehicleParameters(weight = 12.0, height = 3.4, width = 2.4, length = 9.0)
            for (r in listOf(rg.router(RouteOptions(profile = "driving-car", turnCosts = false)), rg.router(RouteOptions(profile = "driving-car", vehicleParameters = camper)))) {
                assertEquals(direct, r.path(a, b).distance, 300.0, "auto (${r.algorithm})")
                val toC = r.path(b, c2)
                assertTrue(abs(toC.distance - 2_220.0) < 100.0, "auto per maxweight:hgv e maxweightrating:hgv (${r.algorithm}): ${toC.distance}")
            }
            // il camion lì non passa
            assertTrue(runCatching { core.path(b, c2) }.isFailure, "camion (core) per maxweight:hgv e maxweightrating:hgv senza deroga")
            assertTrue(runCatching { ch.path(b, c2) }.isFailure, "camion (ch) per maxweight:hgv e maxweightrating:hgv senza deroga")
        }
    }
}
