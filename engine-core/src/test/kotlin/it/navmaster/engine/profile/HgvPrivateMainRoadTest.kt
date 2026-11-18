package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** Le corsie `private` per il camion sulle strade principali (valico di Mikulov, E461): solo col modello NAVMASTER. */
class HgvPrivateMainRoadTest {
    private val nav = HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "CZ"))
    private val ors = HgvProfile()
    private fun way(vararg tags: Pair<String, String>) = OsmWay(1, mapOf(*tags), longArrayOf(1, 2))

    @Test
    fun `the Mikulov customs lanes are open to trucks, only if needed`() {
        // way 1557368265 e 754693464: il raccordo col confine e le corsie verso l'Austria
        for (w in listOf(way("highway" to "primary", "hgv" to "private", "oneway" to "no"),
                         way("highway" to "primary", "hgv" to "private", "oneway" to "yes", "covered" to "yes"))) {
            assertEquals(Access.WAY, nav.access(w))
            assertTrue(nav.privateMainRoad(w))
            assertEquals(PriorityCode.REACH_DEST, nav.priority(w))
            // ORS: divieto, come prima
            assertEquals(Access.CAN_SKIP, ors.access(w))
            assertFalse(ors.privateMainRoad(w))
        }
        for (h in listOf("motorway", "motorway_link", "trunk", "trunk_link", "primary_link")) assertEquals(Access.WAY, nav.access(way("highway" to h, "access" to "private")), h)
    }

    @Test
    fun `real bans and private minor roads stay closed`() {
        for (w in listOf(
            way("highway" to "primary", "hgv" to "no"),
            way("highway" to "primary", "access" to "no"),
            way("highway" to "primary", "hgv" to "military"),
            way("highway" to "secondary", "hgv" to "private"),
            way("highway" to "service", "access" to "private"),
            way("highway" to "residential", "hgv" to "private"),
            // la prima chiave presente decide (hgv prima di access): hgv=no su un access=private
            way("highway" to "primary", "hgv" to "no", "access" to "private"),
        )) {
            assertEquals(Access.CAN_SKIP, nav.access(w), w.tags.toString())
            assertFalse(nav.privateMainRoad(w), w.tags.toString())
        }
        // hgv=yes prevale su access=private: aperta come sempre, priorità normale
        val open = way("highway" to "primary", "hgv" to "yes", "access" to "private")
        assertEquals(Access.WAY, nav.access(open)); assertFalse(nav.privateMainRoad(open))
        assertEquals(PriorityCode.PREFER, nav.priority(way("highway" to "primary")))
    }
}
