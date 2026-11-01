package it.navmaster.engine.osm

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class OsmValuesTest {
    @Test
    fun `parseKmh handles units and zone values`() {
        assertEquals(50.0, OsmValues.parseKmh("50"))
        assertEquals(50.0, OsmValues.parseKmh("50 km/h"))
        assertEquals(30 * 1.609344, OsmValues.parseKmh("30 mph"), 1e-9)
        assertEquals(10 * 1.852, OsmValues.parseKmh("10 knots"), 1e-9)
        assertEquals(150.0, OsmValues.parseKmh("none"))
        assertEquals(50.0, OsmValues.parseKmh("IT:urban"))
        assertEquals(80.0, OsmValues.parseKmh("DE:rural"))
        assertEquals(6.0, OsmValues.parseKmh("walk"))
        assertTrue(OsmValues.parseKmh("").isNaN())
        assertTrue(OsmValues.parseKmh(null).isNaN())
        assertTrue(OsmValues.parseKmh("signals").isNaN())
        assertTrue(OsmValues.parseKmh("0").isNaN())
    }

    @Test
    fun `parseMeter handles metric, imperial and approximate values`() {
        assertEquals(3.5, OsmValues.parseMeter("3.5"))
        assertEquals(3.5, OsmValues.parseMeter("3.5 m"))
        assertEquals(350 * 0.01, OsmValues.parseMeter("350 cm"), 1e-9)
        assertEquals(12 * 0.3048, OsmValues.parseMeter("12 ft"), 1e-9)
        assertEquals(12 * 0.3048 + 6 * 0.0254, OsmValues.parseMeter("12'6\""), 1e-9)
        assertEquals(4.0 * 0.8, OsmValues.parseMeter("~4"), 1e-9)
        assertTrue(OsmValues.parseMeter("default").isNaN())
        assertTrue(OsmValues.parseMeter("none").isNaN())
        assertTrue(OsmValues.parseMeter("3,5").isNaN())
        assertTrue(OsmValues.parseMeter("2;3").isNaN())
    }

    @Test
    fun `parseTons handles t, kg, lbs and short tons`() {
        assertEquals(7.5, OsmValues.parseTons("7.5"))
        assertEquals(7.5, OsmValues.parseTons("7.5 t"))
        assertEquals(7.5, OsmValues.parseTons("7.5 tons"))
        assertEquals(3.5, OsmValues.parseTons("3500 kg"), 1e-9)
        assertEquals(1000 * 0.00045359237, OsmValues.parseTons("1000 lbs"), 1e-9)
        assertEquals(2 * 0.907194048807, OsmValues.parseTons("2 st"), 1e-9)
        assertTrue(OsmValues.parseTons("unsigned").isNaN())
    }

    @Test
    fun `parseDurationSeconds accepts hh-mm, minutes and ISO`() {
        assertEquals(90 * 60L, OsmValues.parseDurationSeconds("01:30"))
        assertEquals(45 * 60L, OsmValues.parseDurationSeconds("45"))
        assertEquals(3600 + 30 * 60L, OsmValues.parseDurationSeconds("PT1H30M"))
        assertEquals(-1L, OsmValues.parseDurationSeconds("abc"))
    }
}
