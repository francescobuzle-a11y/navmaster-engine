package it.navmaster.engine.instructions

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.osm.OsmSink
import it.navmaster.engine.osm.OsmSource
import it.navmaster.engine.osm.OsmWay
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfileOptions
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.PointList
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue
import kotlin.math.abs

class InstructionsTest {
    @Test
    fun `sign thresholds and cardinal directions`() {
        // partenza a (0,0), orientamento verso est (0 rad); punto a nord-est → leggera sinistra
        assertEquals(Sign.CONTINUE_ON_STREET, InstructionsBuilder.calculateSign(0.0, 0.0, 0.0, 0.001, 0.0))
        assertEquals(Sign.TURN_SLIGHT_LEFT, InstructionsBuilder.calculateSign(0.0, 0.0, 0.0005, 0.001, 0.0))
        assertEquals(Sign.TURN_LEFT, InstructionsBuilder.calculateSign(0.0, 0.0, 0.001, 0.0001, 0.0))
        assertEquals(Sign.TURN_RIGHT, InstructionsBuilder.calculateSign(0.0, 0.0, -0.001, 0.0001, 0.0))
        assertEquals(Sign.TURN_SHARP_RIGHT, InstructionsBuilder.calculateSign(0.0, 0.0, -0.0005, -0.001, 0.0))
        assertEquals(CardinalDirection.N, OrsStepBuilder.calcDirection(45.0, 9.0, 45.01, 9.0))
        assertEquals(CardinalDirection.E, OrsStepBuilder.calcDirection(45.0, 9.0, 45.0, 9.01))
        assertEquals(CardinalDirection.SW, OrsStepBuilder.calcDirection(45.0, 9.0, 44.99, 8.986))
        assertEquals(90.0, AngleCalc.calcAzimuth(45.0, 9.0, 45.0, 9.01), 0.01)
        assertEquals(0.0, AngleCalc.calcAzimuth(45.0, 9.0, 45.01, 9.0), 0.01)
    }

    @Test
    fun `arrival direction from the last segment`() {
        val pl = PointList(); pl.add(45.0, 9.0); pl.add(45.0, 9.001) // verso est
        assertEquals(ArrivalDirection.LEFT, OrsStepBuilder.arrivalDirection(pl, 45.0005 to 9.001))
        assertEquals(ArrivalDirection.RIGHT, OrsStepBuilder.arrivalDirection(pl, 44.9995 to 9.001))
        assertEquals(ArrivalDirection.STRAIGHT_AHEAD, OrsStepBuilder.arrivalDirection(pl, 45.0 to 9.001000001))
    }

    @Test
    fun `translations`() {
        val it = InstructionTranslator.forLanguage("it-IT")
        assertEquals("Entra nella rotatoria e prendi la seconda uscita su Via Roma", it.roundabout(2, "Via Roma"))
        assertEquals("Gira a sinistra su Via Roma", it.turn(InstructionType.TURN_LEFT, "Via Roma"))
        assertEquals("Continua dritto", it.continueOn(InstructionType.CONTINUE, ""))
        assertEquals("Avanti Nord-Est su B 3", it.depart(CardinalDirection.NE, "B 3"))
        assertEquals("Arrivo a destinazione, sulla destra", it.arrive(ArrivalDirection.RIGHT, ""))
        val en = InstructionTranslator.forLanguage("en")
        assertEquals("Enter the roundabout and take the 1st exit onto Main St", en.roundabout(1, "Main St"))
        assertEquals("Keep right onto A5", en.keep(InstructionType.KEEP_RIGHT, "A5"))
        assertEquals("Head north", en.depart(CardinalDirection.N, ""))
    }

    @Test
    fun `steps for a Heidelberg truck route`() {
        val dir = Files.createTempDirectory("nme-instr").toFile().also { it.deleteOnExit() }
        val (rg, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        rg.use {
            val router = it.router(RouteOptions(profile = "driving-hgv"))
            val from = 49.4094 to 8.6937; val to = 49.4186 to 8.7606
            val path = router.route(router.snap(from.first, from.second, 400.0, 0), router.snap(to.first, to.second, 400.0, 1))
            val builder = InstructionsBuilder(it.graph, it.encoding, it.encoding.profile("heavyvehicle"))
            val instructions = builder.build(path)
            assertTrue(instructions.size >= 3, "poche istruzioni: ${instructions.size}")
            assertEquals(Sign.FINISH, instructions.last().sign)
            // la somma delle distanze delle istruzioni è la distanza del percorso
            assertEquals(path.distance, instructions.sumOf { i -> i.distance }, 0.5)
            // i punti delle istruzioni coprono tutta la geometria (ultimo punto nel FINISH)
            assertEquals(path.points().size, instructions.sumOf { i -> i.points.size })
            for (ins in instructions) if (ins.isRoundabout) assertTrue(ins.exitNumber in 1..10, "uscita rotatoria ${ins.exitNumber}")

            val steps = OrsStepBuilder(StepOptions(language = "it", maneuvers = true)).build(instructions, 0, to)
            assertEquals(InstructionType.DEPART, steps.first().type)
            assertEquals(InstructionType.FINISH, steps.last().type)
            assertTrue(steps.first().instruction.startsWith("Avanti "), steps.first().instruction)
            assertTrue(steps.last().instruction.startsWith("Arrivo a"), steps.last().instruction)
            // way_points contigui e che terminano sull'ultimo punto della geometria
            var prevEnd = 0
            for (s in steps) { assertEquals(prevEnd, s.wayPointStart); assertTrue(s.wayPointEnd >= s.wayPointStart); prevEnd = s.wayPointEnd }
            assertEquals(path.points().size - 1, steps.last().wayPointEnd)
            assertEquals(steps.last().wayPointStart, steps.last().wayPointEnd)
            for (s in steps) {
                val m = s.maneuver!!
                assertTrue(m.bearingBefore in 0..360 && m.bearingAfter in 0..360)
                assertTrue(s.duration >= 0 && s.distance >= 0)
            }
            println(steps.joinToString("\n") { s -> "${s.type.code}\t${"%.0f".format(s.distance)} m\t${s.instruction}" })
        }
    }

    /**
     * Grafo NAVMASTER (le velocità del camion cambiano lungo la strada per salite e curve): una biforcazione
     * su una provinciale diritta verso est (A → B → C, «SP1»), con a B una laterale della stessa classe che
     * se ne stacca di ~17° a sinistra («Via Laterale», 30 km/h). Il camion va da A a C; le velocità dei due
     * archi della SP1 si scelgono per caso; il segno dato a B (IGNORE se nessuna istruzione).
     */
    private fun forkSign(navmasterSpeeds: Boolean, ab: Pair<Double, Double>, bc: Pair<Double, Double>, profile: String = "heavyvehicle"): List<Int> {
        val nodeList = listOf(1L to (45.0 to 9.0), 2L to (45.0 to 9.002), 3L to (45.0 to 9.004), 4L to (45.0006 to 9.004))
        val main = mapOf("highway" to "secondary", "name" to "SP1", "maxspeed" to "70")
        val side = mapOf("highway" to "secondary", "name" to "Via Laterale", "maxspeed" to "30")
        val wayList = listOf(OsmWay(10, main, longArrayOf(1, 2)), OsmWay(11, main, longArrayOf(2, 3)), OsmWay(12, side, longArrayOf(2, 4)))
        val osm = object : OsmSource {
            override val nodeCount = nodeList.size.toLong()
            override val wayCount = wayList.size.toLong()
            override fun read(sink: OsmSink, nodes: Boolean, ways: Boolean, relations: Boolean) {
                if (nodes) for ((id, p) in nodeList) sink.node(id, p.first, p.second, null)
                if (ways) for (w in wayList) sink.way(w)
            }
        }
        val profiles = listOf(
            CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "IT")),
            HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "IT")),
        )
        val encoding = GraphEncoding(profiles)
        val dir = Files.createTempDirectory("nme-fork").toFile().also { it.deleteOnExit() }
        return BaseGraph(GraphDirectory(dir), encoding.registry).use { graph ->
            OsmGraphImporter(graph, encoding).import(osm)
            val pe = encoding.profile(profile)
            fun node(lon: Double) = (0 until graph.nodeCount).first { abs(graph.lon(it) - lon) < 1e-7 && abs(graph.lat(it) - 45.0) < 1e-7 }
            val a = node(9.0); val b = node(9.002); val c = node(9.004)
            fun edge(x: Int, y: Int) = (0 until graph.edgeCount).first { e -> graph.nodeA(e) == x && graph.nodeB(e) == y || graph.nodeA(e) == y && graph.nodeB(e) == x }
            // le velocità del caso: (da A verso C, da C verso A)
            fun setSpeeds(e: Int, from: Int, v: Pair<Double, Double>) {
                val f = graph.getFlags(e); val rev = graph.nodeA(e) != from
                pe.speed.set(f, rev, v.first); pe.speed.set(f, !rev, v.second); graph.setFlags(e, f)
            }
            val e1 = edge(a, b); val e2 = edge(b, c)
            setSpeeds(e1, a, ab); setSpeeds(e2, b, bc)
            fun pathEdge(e: Int, from: Int, to: Int): PathEdge {
                val pl = PointList(); pl.add(graph.lat(from), graph.lon(from)); pl.add(graph.lat(to), graph.lon(to))
                return PathEdge(e, graph.nodeA(e) != from, from, to, pl, graph.distance(e), 1000L, 1.0, graph.name(e))
            }
            val path = RoutePath(listOf(pathEdge(e1, a, b), pathEdge(e2, b, c)), graph.distance(e1) + graph.distance(e2), 2000L, 2.0)
            InstructionsBuilder(graph, encoding, pe, navmasterSpeeds).build(path).map { it.sign }
        }
    }

    @Test
    fun `NAVMASTER graphs - terrain speed noise at a fork gives the same instructions, a real drop still gives its own`() {
        val clean = forkSign(true, 65.0 to 65.0, 65.0 to 65.0)
        println("biforcazione, velocità uguali: $clean")
        assertEquals(listOf(Sign.CONTINUE_ON_STREET, Sign.FINISH), clean, "la SP1 continua diritta: nessuna istruzione a B")
        // il rumore del terreno: in salita 50 da B a C (la strada resta a 65 all'indietro), o una curva lontana (60 nei due sensi)
        assertEquals(clean, forkSign(true, 65.0 to 65.0, 50.0 to 65.0), "salita dopo la biforcazione")
        assertEquals(clean, forkSign(true, 65.0 to 65.0, 60.0 to 60.0), "5 km/h in meno per una curva")
        assertEquals(clean, forkSign(true, 60.0 to 65.0, 65.0 to 60.0), "salita prima, discesa dopo")
        // un calo vero (65 → 40 nei due sensi): l'istruzione a B resta, come in GraphHopper
        val drop = forkSign(true, 65.0 to 65.0, 40.0 to 40.0)
        println("biforcazione, calo vero: $drop")
        assertEquals(3, drop.size, "calo da 65 a 40: un'istruzione a B ($drop)")
        assertEquals(drop, forkSign(false, 65.0 to 65.0, 40.0 to 40.0), "come senza tolleranza")
        // senza tolleranza (grafi ORS) il rumore dà un'istruzione: per questo serve la tolleranza
        assertEquals(3, forkSign(false, 65.0 to 65.0, 60.0 to 60.0).size)
        // l'auto non ha terreno: uguaglianza esatta anche sui grafi NAVMASTER
        assertEquals(3, forkSign(true, 65.0 to 65.0, 60.0 to 60.0, profile = "car-ors").size, "auto: nessuna tolleranza")
        assertEquals(listOf(Sign.CONTINUE_ON_STREET, Sign.FINISH), forkSign(true, 65.0 to 65.0, 65.0 to 65.0, profile = "car-ors"))
    }
}
