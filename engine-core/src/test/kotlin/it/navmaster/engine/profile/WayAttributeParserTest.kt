package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

fun way(vararg tags: Pair<String, String>, id: Long = 1, nodes: Int = 2): OsmWay =
    OsmWay(id, mapOf(*tags), LongArray(nodes) { it.toLong() + 1 })

class WayAttributeParserTest {
    private val hgv = HeavyVehicleAttributes.HGV
    private val delivery = HeavyVehicleAttributes.DELIVERY
    private val any = HeavyVehicleAttributes.ANY

    @Test
    fun `no restriction blocks nothing`() {
        assertEquals(0, WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "primary")))
    }

    @Test
    fun `hgv=no blocks only hgv`() {
        assertEquals(hgv, WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "residential", "hgv" to "no")))
    }

    @Test
    fun `hgv=delivery allows delivery and blocks other hgv`() {
        val blocked = WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "residential", "hgv" to "delivery"))
        assertTrue(blocked and hgv != 0)
        assertTrue(blocked and delivery == 0)
    }

    @Test
    fun `access=no blocks all heavy vehicles unless listed`() {
        assertEquals(any, WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "service", "access" to "no")))
        val blocked = WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "service", "access" to "no", "hgv" to "yes"))
        assertEquals(any and hgv.inv(), blocked)
    }

    @Test
    fun `motor_vehicle=hgv allows only hgv`() {
        val blocked = WayAttributeParser.blockedHeavyVehicleTypes(way("highway" to "service", "motor_vehicle" to "hgv"))
        assertEquals(any and hgv.inv(), blocked)
    }

    @Test
    fun `hazmat restrictions`() {
        assertTrue(WayAttributeParser.hazmatForbidden(way("highway" to "primary", "hazmat" to "no")))
        assertTrue(WayAttributeParser.hazmatForbidden(way("highway" to "primary", "hazmat:B" to "no")))
        assertFalse(WayAttributeParser.hazmatForbidden(way("highway" to "primary", "hazmat" to "yes")))
        assertFalse(WayAttributeParser.hazmatForbidden(way("highway" to "primary", "hazmat:water" to "no")))
    }

    @Test
    fun `dimensions default to infinity and parse units`() {
        val a = WayAttributeParser.parse(way("highway" to "primary"), isFerry = false)
        assertEquals(Double.POSITIVE_INFINITY, a.maxHeight)
        assertEquals(Double.POSITIVE_INFINITY, a.maxWeight)
        val b = WayAttributeParser.parse(
            way("highway" to "primary", "maxheight" to "3.5", "maxweight" to "7.5 t", "maxwidth:physical" to "2.3", "maxaxleload" to "10", "maxlength" to "12"),
            isFerry = false,
        )
        assertEquals(3.5, b.maxHeight)
        assertEquals(7.5, b.maxWeight)
        assertEquals(2.3, b.maxWidth)
        assertEquals(10.0, b.maxAxleLoad)
        assertEquals(12.0, b.maxLength)
    }

    @Test
    fun `toll, road access, road class, way type and surface`() {
        val a = WayAttributeParser.parse(
            way("highway" to "motorway_link", "toll" to "yes", "motor_vehicle" to "destination", "surface" to "asphalt"),
            isFerry = false,
        )
        assertEquals(Toll.ALL, a.toll)
        assertEquals(RoadAccess.DESTINATION, a.roadAccess)
        assertEquals(RoadClass.MOTORWAY, a.roadClass)
        assertEquals(WayType.STATE_ROAD, a.wayType)
        assertEquals(WaySurface.ASPHALT, a.waySurface)
        assertTrue(a.isMotorway)

        val b = WayAttributeParser.parse(way("highway" to "track", "toll:hgv" to "yes", "surface" to "gravel;ground"), isFerry = false)
        assertEquals(Toll.HGV, b.toll)
        assertEquals(RoadClass.TRACK, b.roadClass)
        assertEquals(WayType.TRACK, b.wayType)
        assertEquals(WaySurface.GRAVEL, b.waySurface)

        val f = WayAttributeParser.parse(way("route" to "ferry"), isFerry = true)
        assertEquals(WayType.FERRY, f.wayType)
        assertEquals(RoadClass.OTHER, f.roadClass)
    }

    @Test
    fun `road access picks the most restrictive value and HGV keys override`() {
        val w = way("highway" to "residential", "access" to "yes", "hgv" to "no", "motor_vehicle" to "destination")
        assertEquals(RoadAccess.DESTINATION, WayAttributeParser.roadAccess(w, WayAttributeParser.carRestrictions))
        assertEquals(RoadAccess.NO, WayAttributeParser.roadAccess(w, WayAttributeParser.hgvRestrictions))
    }
}
