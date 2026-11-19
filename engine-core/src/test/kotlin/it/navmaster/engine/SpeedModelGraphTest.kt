package it.navmaster.engine

import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.PriorityModel
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.storage.GraphProperties
import it.navmaster.engine.profile.VehicleProfileOptions
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

/** Un grafo NAVMASTER intero (CH, core, landmark): i ritardi stanno nei pesi, le preparazioni li vedono. */
class SpeedModelGraphTest {
    private val pairs = listOf(
        (49.4094 to 8.6937) to (49.4186 to 8.7606),
        (49.4036 to 8.6757) to (49.3989 to 8.6724),
        (49.4186 to 8.7606) to (49.4036 to 8.6757),
        (49.3989 to 8.6724) to (49.4094 to 8.6937),
    )

    @Test
    fun `navmaster graph - properties, CH and core equal to Dijkstra`() {
        val dir = Files.createTempDirectory("nme-smg").toFile().also { it.deleteOnExit() }
        val profiles = listOf(
            CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")),
            HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")),
        )
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, profiles).first.use { it.prepareCH(landmarkSets = emptyList()) }
        val orsDir = Files.createTempDirectory("nme-smg-ors").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), orsDir, listOf(CarProfile(), HgvProfile())).first.use { it.prepareCH(landmarkSets = emptyList()) }
        RoutingGraph.load(orsDir).use { ors ->
            assertEquals("ors", ors.speedModel); assertNull(ors.country)
            RoutingGraph.load(dir).use { rg ->
                assertEquals("navmaster", rg.speedModel); assertEquals("DE", rg.country)
                // CH, core e landmark preparati col peso consigliato di NAVMASTER, e il grafo lo ricorda
                assertEquals(PriorityModel.NAVMASTER, rg.encoding.priorityModel); assertEquals(PriorityModel.ORS, ors.encoding.priorityModel)
                // i divieti di transito dei mezzi pesanti: solo nel grafo NAVMASTER, che i motori di prima rifiutano
                assertTrue(rg.encoding.shared.withTransit); assertTrue(!ors.encoding.shared.withTransit)
                assertEquals("$GRAPH_FORMAT_VERSION.${RoutingGraph.TRANSIT_VARIANT}", GraphProperties(File(dir, "properties")).apply { load() }.get("graph.format_version"))
                assertEquals(GRAPH_FORMAT_VERSION.toString(), GraphProperties(File(orsDir, "properties")).apply { load() }.get("graph.format_version"))
                for ((a, b) in pairs) {
                    val params = VehicleParameters(height = 4.0, weight = 12.0)
                    val core = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
                    val eb = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false))
                    val c1 = core.route(core.snap(a.first, a.second, 400.0, 0), core.snap(b.first, b.second, 400.0, 1))
                    val c2 = eb.route(eb.snap(a.first, a.second, 400.0, 0), eb.snap(b.first, b.second, 400.0, 1))
                    assertTrue(abs(c1.weight - c2.weight) <= 0.05 + c2.weight * 1e-4, "Core ${c1.weight} vs Dijkstra edge-based ${c2.weight}")
                    for (profile in listOf("driving-hgv", "driving-car")) {
                        val ch = rg.router(RouteOptions(profile = profile, turnCosts = false))
                        assertEquals("ch", ch.algorithm)
                        val plain = rg.router(RouteOptions(profile = profile, turnCosts = false, allowCH = false))
                        val p1 = ch.route(ch.snap(a.first, a.second, 400.0, 0), ch.snap(b.first, b.second, 400.0, 1))
                        val p2 = plain.route(plain.snap(a.first, a.second, 400.0, 0), plain.snap(b.first, b.second, 400.0, 1))
                        assertTrue(abs(p1.weight - p2.weight) <= 0.05 + p2.weight * 1e-4, "$profile CH ${p1.weight} vs Dijkstra ${p2.weight}")
                        assertEquals(p2.millis.toDouble(), p1.millis.toDouble(), 1000.0 + p2.millis * 1e-3)
                        val o = ors.router(RouteOptions(profile = profile, turnCosts = false))
                        val p0 = o.route(o.snap(a.first, a.second, 400.0, 0), o.snap(b.first, b.second, 400.0, 1))
                        println("$profile $a → $b: ORS %.1f km %.1f min, NAVMASTER %.1f km %.1f min".format(p0.distance / 1000, p0.millis / 60000.0, p1.distance / 1000, p1.millis / 60000.0))
                    }
                }
            }
        }
    }

    @Test
    fun `a navmaster graph prepared before the priority model opens with the ORS priority`() {
        val dir = Files.createTempDirectory("nme-smg-old").toFile().also { it.deleteOnExit() }
        val profiles = listOf(HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")))
        // un grafo NAVMASTER di prima dei divieti di transito
        val encoding = it.navmaster.engine.storage.GraphEncoding(profiles, withTransit = false)
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, profiles, encoding = encoding).first.use { it.prepareCH(withCore = false, landmarkSets = emptyList()) }
        assertEquals("navmaster", GraphProperties(File(dir, "profiles")).apply { load() }.get("priority_model"))
        // il peso consigliato di tutti gli archi: cambia col modello letto (autostrade e strade migliori)
        fun dijkstraWeight(rg: RoutingGraph): Double {
            val w = it.navmaster.engine.weighting.WeightingFactory.create(rg.encoding, rg.encoding.profile("heavyvehicle"), it.navmaster.engine.weighting.WeightingMethod.RECOMMENDED)
            val iter = rg.graph.createEdgeIterator()
            var sum = 0.0
            for (e in 0 until rg.graph.edgeCount) { iter.setEdge(e, rg.graph.nodeA(e)); val x = w.edgeWeight(iter, false); if (x.isFinite()) sum += x }
            return sum
        }
        val nav = RoutingGraph.load(dir).use { assertEquals(PriorityModel.NAVMASTER, it.encoding.priorityModel); dijkstraWeight(it) }
        // un motore di prima vuole la versione esatta del formato: questo grafo lo rifiuterebbe
        assertEquals("$GRAPH_FORMAT_VERSION.navmaster", GraphProperties(File(dir, "properties")).apply { load() }.get("graph.format_version"))
        // i grafi NAVMASTER di prima non hanno la proprietà: la loro CH è col peso di ORS, e così si aprono
        val text = File(dir, "profiles").readLines().filterNot { it.startsWith("priority_model") }
        File(dir, "profiles").writeText(text.joinToString("\n", postfix = "\n"))
        GraphProperties(File(dir, "properties")).apply { load(); set("graph.format_version", GRAPH_FORMAT_VERSION.toString()); save() }
        val old = RoutingGraph.load(dir).use { assertEquals(PriorityModel.ORS, it.encoding.priorityModel); dijkstraWeight(it) }
        // premi di ORS più forti sulle strade sopra le statali: la rete pesa meno, e il modello letto arriva davvero al peso
        assertTrue(old < nav - 100.0, "peso ORS $old, NAVMASTER $nav")
    }

    @Test
    fun `a graph with transit bans says so in its format, and an inconsistent one is refused`() {
        val dir = Files.createTempDirectory("nme-smg-tr").toFile().also { it.deleteOnExit() }
        val profiles = listOf(HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")))
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, profiles).first.close()
        val props = File(dir, "properties")
        assertEquals("$GRAPH_FORMAT_VERSION.${RoutingGraph.TRANSIT_VARIANT}", GraphProperties(props).apply { load() }.get("graph.format_version"))
        RoutingGraph.load(dir).use { assertTrue(it.encoding.shared.withTransit); assertEquals(PriorityModel.NAVMASTER, it.encoding.priorityModel) }
        // un motore di prima (variante `navmaster` esatta e schema senza transito) non lo apre; qui: variante tolta → rifiutato
        GraphProperties(props).apply { load(); set("graph.format_version", "$GRAPH_FORMAT_VERSION.${RoutingGraph.PRIORITY_VARIANT}"); save() }
        val e = kotlin.test.assertFailsWith<IllegalStateException> { RoutingGraph.load(dir).close() }
        assertTrue(e.message!!.contains("divieti di transito"), e.message)
    }
}
