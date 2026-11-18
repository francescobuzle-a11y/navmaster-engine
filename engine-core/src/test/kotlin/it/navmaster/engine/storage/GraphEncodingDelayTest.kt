package it.navmaster.engine.storage

import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/** I ritardi ai nodi (`<profilo>_delay`) nello schema dei flag: nessun int in più, grafi vecchi ancora apribili. */
class GraphEncodingDelayTest {
    private fun profiles() = listOf(CarProfile(), HgvProfile())

    @Test
    fun `the delays fit in the sixth int`() {
        val now = GraphEncoding(profiles())
        assertEquals(6, now.registry.intsForFlags)
        assertNotNull(now.profile("heavyvehicle").delay)
        val before = GraphEncoding(profiles(), withDelay = false)
        assertEquals(6, before.registry.intsForFlags)
        // i bit di prima restano dove erano: i valori comuni hanno la stessa descrizione e lo stesso ordine
        assertTrue(now.registry.describe().startsWith(before.registry.describe() + ";"))
    }

    @Test
    fun `graphs built before open with their schema`() {
        val noDelay = GraphEncoding(profiles(), withAdr = true, withDelay = false).registry.describe()
        val e1 = GraphEncoding.forExisting(profiles(), noDelay)
        assertEquals(noDelay, e1.registry.describe())
        assertNull(e1.profile("car-ors").delay)
        val noAdr = GraphEncoding(profiles(), withAdr = false, withDelay = false).registry.describe()
        val e2 = GraphEncoding.forExisting(profiles(), noAdr)
        assertEquals(noAdr, e2.registry.describe())
        assertNull(e2.shared.adrTunnel)
        val current = GraphEncoding.forExisting(profiles(), null)
        assertNotNull(current.profile("heavyvehicle").delay)
    }

    @Test
    fun `old flags read zero delay and same values`() {
        val old = GraphEncoding(profiles(), withDelay = false)
        val now = GraphEncoding(profiles())
        val f = old.registry.newFlags()
        old.profile("heavyvehicle").speed.set(f, false, 70.0)
        old.shared.hazmatWater!!.set(f, true)
        assertEquals(70.0, now.profile("heavyvehicle").speed.get(f, false))
        assertTrue(now.shared.hazmatWater!!.get(f))
        assertEquals(0.0, now.profile("heavyvehicle").delay!!.get(f, false))
        assertSame(null, old.profile("heavyvehicle").delay)
    }
}
