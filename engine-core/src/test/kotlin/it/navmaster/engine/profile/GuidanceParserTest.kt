package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class GuidanceParserTest {
    private fun way(vararg tags: Pair<String, String>) = OsmWay(1, mapOf(*tags), longArrayOf(1, 2))

    @Test
    fun `a width is a ban only when it is signed`() {
        // la larghezza della carreggiata non è un divieto
        assertFalse(GuidanceParser.parse(way("highway" to "primary", "width" to "7")).widthSigned)
        assertTrue(GuidanceParser.parse(way("highway" to "residential", "maxwidth" to "2.2")).widthSigned)
        assertTrue(GuidanceParser.parse(way("highway" to "residential", "maxwidth:physical" to "2.5", "width" to "4")).widthSigned)
    }

    @Test
    fun `speeds, lanes, ADR category and conditional restrictions`() {
        val g = GuidanceParser.parse(way("highway" to "trunk", "oneway" to "yes", "maxspeed" to "90", "maxspeed:hgv" to "70",
            "lanes" to "2", "turn:lanes" to "through|right", "hazmat:C" to "no", "maxweight:conditional" to "7.5 @ (Mo-Fr 07:00-19:00)"))
        assertEquals(90, g.maxSpeedFwd); assertEquals(70, g.maxSpeedHgvFwd)
        assertEquals(1, g.oneway); assertEquals("through|right", g.turnLanesFwd); assertEquals("", g.turnLanesBwd)
        assertEquals(3, g.adrTunnel)
        assertEquals("maxweight:conditional=7.5 @ (Mo-Fr 07:00-19:00)", g.conditional)
    }
}
