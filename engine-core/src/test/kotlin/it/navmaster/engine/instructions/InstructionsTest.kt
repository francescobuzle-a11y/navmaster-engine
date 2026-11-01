package it.navmaster.engine.instructions

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.PointList
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

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
}
