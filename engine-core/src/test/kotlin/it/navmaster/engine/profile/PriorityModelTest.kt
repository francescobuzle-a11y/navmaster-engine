package it.navmaster.engine.profile

import it.navmaster.engine.storage.GraphEncoding
import kotlin.math.pow
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

/** I fattori del peso consigliato: ORS come prima bit per bit, NAVMASTER con premi leggeri e le stesse penalità. */
class PriorityModelTest {
    /** La formula di prima (`PriorityWeighting` fino a novembre 2026). */
    private fun orsBefore(priority: Double): Double = 2.0.pow((priority * 7.0 - 4.0) / (4.0 - 7.0))

    @Test
    fun `ORS factors are exactly the ones of before`() {
        for (code in PriorityCode.entries) {
            val p = code.factor
            assertEquals(orsBefore(p).toRawBits(), PriorityModel.ORS.factor(p).toRawBits(), "codice $code")
        }
        assertEquals(0.5, PriorityModel.ORS.factor(PriorityCode.BEST.factor), 1e-12)
        assertEquals(1.0, PriorityModel.ORS.factor(PriorityCode.UNCHANGED.factor), 1e-12)
        assertEquals(2.52, PriorityModel.ORS.factor(PriorityCode.WORST.factor), 0.005)
    }

    @Test
    fun `NAVMASTER - light bonuses, ORS penalties, still ordered`() {
        val nav = PriorityModel.NAVMASTER; val ors = PriorityModel.ORS
        for (code in PriorityCode.entries) {
            val p = code.factor
            when {
                code.value > 4 -> assertTrue(nav.factor(p) > ors.factor(p) && nav.factor(p) < 1.0, "premio di $code: ${nav.factor(p)}")
                else -> assertEquals(ors.factor(p), nav.factor(p), 1e-12, "penalità di $code")
            }
        }
        val f = PriorityCode.entries.map { nav.factor(it.factor) }
        for (i in 1 until f.size) assertTrue(f[i] < f[i - 1], "fattori non decrescenti: $f")
        // statale contro autostrada: +6 % (ORS +26 %), il tempo decide
        val primaryVsMotorway = nav.factor(PriorityCode.PREFER.factor) / nav.factor(PriorityCode.VERY_NICE.factor)
        assertEquals(1.06, primaryVsMotorway, 0.01)
        assertEquals(1.26, ors.factor(PriorityCode.PREFER.factor) / ors.factor(PriorityCode.VERY_NICE.factor), 0.01)
    }

    @Test
    fun `the model is read from the graph and chosen from the speed model`() {
        assertEquals(PriorityModel.ORS, PriorityModel.parse(null))
        assertEquals(PriorityModel.ORS, PriorityModel.parse(""))
        assertEquals(PriorityModel.ORS, PriorityModel.parse("ors"))
        assertEquals(PriorityModel.NAVMASTER, PriorityModel.parse(" NAVMASTER "))
        assertFailsWith<IllegalStateException> { PriorityModel.parse("altro") }
        for (m in PriorityModel.entries) assertEquals(m, PriorityModel.parse(m.id))
        val nav = VehicleProfileOptions(speedModel = SpeedModel.NAVMASTER)
        assertEquals(PriorityModel.ORS, GraphEncoding(listOf(CarProfile(), HgvProfile())).priorityModel)
        assertEquals(PriorityModel.NAVMASTER, GraphEncoding(listOf(CarProfile(nav), HgvProfile(nav))).priorityModel)
    }
}
