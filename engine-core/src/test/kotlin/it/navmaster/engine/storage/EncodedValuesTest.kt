package it.navmaster.engine.storage

import it.navmaster.engine.profile.RoadAccess
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class EncodedValuesTest {
    @Test
    fun `allocator never straddles an int boundary`() {
        val a = BitAllocator()
        a.next(30); assertEquals(0, a.dataIndex); assertEquals(0, a.shift)
        a.next(5); assertEquals(1, a.dataIndex); assertEquals(0, a.shift) // non sta nei 2 bit rimasti
        a.next(27); assertEquals(1, a.dataIndex); assertEquals(5, a.shift)
        a.next(1); assertEquals(2, a.dataIndex); assertEquals(0, a.shift)
        assertEquals(3, a.requiredInts)
    }

    @Test
    fun `values pack and unpack independently`() {
        val access = BooleanEncodedValue("access", twoDirections = true)
        val speed = DecimalEncodedValue("speed", 5, 5.0, twoDirections = true)
        val height = DecimalEncodedValue("max_height", 11, 0.01, Double.POSITIVE_INFINITY)
        val weight = DecimalEncodedValue("max_weight", 11, 0.1, Double.POSITIVE_INFINITY)
        val osmId = IntEncodedValue("osm_way_id", 31)
        val roadAccess = EnumEncodedValue("road_access", RoadAccess.entries.toTypedArray())
        val reg = EncodedValueRegistry.Builder().add(access).add(speed).add(height).add(weight).add(osmId).add(roadAccess).build()
        // int0: access 2 + speed 10 + height 11 = 23; weight 11 non entra → int1; osm 31 → int2; road_access 4 → int3
        assertEquals(4, reg.intsForFlags)
        val f = reg.newFlags()

        access.set(f, false, true); access.set(f, true, false)
        speed.set(f, false, 85.0); speed.set(f, true, 50.0)
        height.set(f, 3.5); weight.set(f, 7.5)
        osmId.set(f, 123456789); roadAccess.set(f, RoadAccess.DESTINATION)

        assertTrue(access.get(f, false)); assertTrue(!access.get(f, true))
        assertEquals(85.0, speed.get(f, false)); assertEquals(50.0, speed.get(f, true))
        assertEquals(3.5, height.get(f), 1e-9); assertEquals(7.5, weight.get(f), 1e-9)
        assertEquals(123456789, osmId.get(f))
        assertEquals(RoadAccess.DESTINATION, roadAccess.get(f))

        // il default (+∞) si memorizza come 0 e si rilegge come +∞
        val g = reg.newFlags()
        assertEquals(Double.POSITIVE_INFINITY, height.get(g))
        height.set(g, Double.POSITIVE_INFINITY)
        assertEquals(0, height.rawGet(g, false))
        // oltre il massimo: saturazione
        height.set(g, 99.0)
        assertEquals(height.maxDecimal, height.get(g), 1e-9)
    }

    @Test
    fun `decimal with maximum as infinity`() {
        val tc = DecimalEncodedValue("turn_cost", 1, 1.0, useMaximumAsInfinity = true)
        val reg = EncodedValueRegistry.Builder().add(tc).build()
        val f = reg.newFlags()
        tc.set(f, Double.POSITIVE_INFINITY)
        assertEquals(Double.POSITIVE_INFINITY, tc.get(f))
        assertEquals(0.0, tc.maxDecimal)
    }

    @Test
    fun `speed rounds to the factor`() {
        val speed = DecimalEncodedValue("speed", 5, 5.0)
        EncodedValueRegistry.Builder().add(speed).build()
        val f = IntArray(1)
        speed.set(f, 27.0); assertEquals(25.0, speed.get(f))
        speed.set(f, 28.0); assertEquals(30.0, speed.get(f))
        speed.set(f, 200.0); assertEquals(155.0, speed.get(f)) // 31 * 5
    }

    @Test
    fun `enum bits`() {
        assertEquals(1, EnumEncodedValue.bitsFor(2))
        assertEquals(2, EnumEncodedValue.bitsFor(4))
        assertEquals(3, EnumEncodedValue.bitsFor(5))
        assertEquals(4, EnumEncodedValue.bitsFor(9))
    }

    @Test
    fun `reverse set on one-directional value fails`() {
        val ev = IntEncodedValue("x", 3)
        EncodedValueRegistry.Builder().add(ev).build()
        assertFailsWith<IllegalArgumentException> { ev.set(IntArray(1), true, 1) }
        assertEquals(0, ev.get(IntArray(1), true)) // la lettura inversa ricade sulla diretta
    }
}
