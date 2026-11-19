package it.navmaster.engine.storage

import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.profile.VehicleProfileOptions
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/** I divieti di transito (`hgv_destination`, `hgv_weight_exempt`) nello schema dei flag: solo NAVMASTER, ultimi, nel sesto int. */
class GraphEncodingTransitTest {
    private fun nav(): List<VehicleProfile> = listOf(
        CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "AT")),
        HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "AT")),
    )
    private fun ors(): List<VehicleProfile> = listOf(CarProfile(), HgvProfile())

    /** I bit occupati: la fine dell'ultimo valore registrato. */
    private fun usedBits(e: GraphEncoding): Int {
        val alloc = BitAllocator()
        for (ev in e.registry.all) { alloc.next(ev.bits); if (ev.storeTwoDirections) alloc.next(ev.bits) }
        return alloc.requiredBits
    }

    @Test
    fun `only NAVMASTER graphs get the transit values, last, in the sixth int`() {
        val now = GraphEncoding(nav())
        assertTrue(now.shared.withTransit)
        assertNotNull(now.shared.hgvDestination); assertNotNull(now.shared.hgvWeightExempt)
        // car-ors + heavyvehicle (grafo-nm): 189 bit di prima + 3 = 192, sei int pieni (nessun bit libero)
        assertEquals(6, now.registry.intsForFlags)
        assertEquals(192, usedBits(now))
        val before = GraphEncoding(nav(), withTransit = false)
        assertEquals(189, usedBits(before))
        assertEquals(6, before.registry.intsForFlags)
        // i bit di prima restano dove erano
        assertTrue(now.registry.describe().startsWith(before.registry.describe() + ";"))
        // ORS: lo schema di sempre
        val o = GraphEncoding(ors())
        assertFalse(o.shared.withTransit); assertNull(o.shared.hgvDestination)
        assertEquals(before.registry.describe(), o.registry.describe())
        // senza i ritardi (schemi di prima) mai
        assertFalse(GraphEncoding(nav(), withDelay = false).shared.withTransit)
        assertFalse(GraphEncoding(nav(), withAdr = false, withDelay = false).shared.withTransit)
    }

    @Test
    fun `every older graph kind still opens`() {
        // load usa i profili di default (ORS): lo schema coi divieti va provato esplicitamente
        val transit = GraphEncoding(nav()).registry.describe()
        val e0 = GraphEncoding.forExisting(ors(), transit)
        assertEquals(transit, e0.registry.describe()); assertTrue(e0.shared.withTransit)
        val navNoTransit = GraphEncoding(nav(), withTransit = false).registry.describe()
        val e1 = GraphEncoding.forExisting(ors(), navNoTransit)
        assertEquals(navNoTransit, e1.registry.describe()); assertFalse(e1.shared.withTransit)
        assertNotNull(e1.profile("heavyvehicle").delay)
        val noDelay = GraphEncoding(nav(), withDelay = false).registry.describe()
        val e2 = GraphEncoding.forExisting(ors(), noDelay)
        assertEquals(noDelay, e2.registry.describe()); assertNull(e2.profile("heavyvehicle").delay); assertFalse(e2.shared.withTransit)
        val old = GraphEncoding(nav(), withAdr = false, withDelay = false).registry.describe()
        val e3 = GraphEncoding.forExisting(ors(), old)
        assertEquals(old, e3.registry.describe()); assertNull(e3.shared.adrTunnel)
        // anche coi profili NAVMASTER (le prove che costruiscono a mano)
        assertEquals(navNoTransit, GraphEncoding.forExisting(nav(), navNoTransit).registry.describe())
        assertEquals(transit, GraphEncoding.forExisting(nav(), transit).registry.describe())
        // un solo profilo heavyvehicle: 160 → 163 bit, cinque int diventano sei
        val hgvOnly = listOf(nav()[1])
        assertEquals(6, GraphEncoding(hgvOnly).registry.intsForFlags)
        assertEquals(5, GraphEncoding(hgvOnly, withTransit = false).registry.intsForFlags)
        assertEquals(GraphEncoding(hgvOnly).registry.describe(), GraphEncoding.forExisting(listOf(HgvProfile()), GraphEncoding(hgvOnly).registry.describe()).registry.describe())
    }

    @Test
    fun `old flags read no transit ban, new flags keep both directions apart`() {
        val old = GraphEncoding(nav(), withTransit = false)
        val now = GraphEncoding(nav())
        val f = old.registry.newFlags()
        old.profile("heavyvehicle").delay!!.set(f, true, 63.0)
        old.shared.hazmatWater!!.set(f, true)
        assertFalse(now.shared.hgvDestination!!.get(f, false)); assertFalse(now.shared.hgvDestination!!.get(f, true))
        assertFalse(now.shared.hgvWeightExempt!!.get(f))
        assertEquals(63.0, now.profile("heavyvehicle").delay!!.get(f, true))
        val g = now.registry.newFlags()
        now.shared.hgvDestination!!.set(g, true, true)
        assertFalse(now.shared.hgvDestination!!.get(g, false)); assertTrue(now.shared.hgvDestination!!.get(g, true))
        assertEquals(0.0, now.profile("heavyvehicle").delay!!.get(g, true))
    }
}
