package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

/**
 * La memoria dei pesi rapidi ([QuickWeightCache]): un peso finito si riusa sotto ogni limite (+∞ se
 * non sta sotto quello chiesto); «∞ sotto l» solo per i limiti ≤ l; una voce peggiore non sostituisce;
 * LRU oltre maxEntries; clear(); chiavi con opzioni diverse non si confondono.
 */
class QuickWeightCacheTest {
    private val hgv = RouteOptions(profile = "driving-hgv")
    private fun key(o: RouteOptions = hgv, a: Double = 45.0, b: Double = 46.0) = QuickKey(o, a.toBits(), 9.0.toBits(), 3.0.toBits(), b.toBits(), 10.0.toBits(), 3.0.toBits())

    @Test
    fun `a finite weight is reused under any limit`() {
        val c = QuickWeightCache()
        c.put(key(), 1000.0, 1500.0)
        assertEquals(1000.0, c.get(key(), Double.POSITIVE_INFINITY))
        assertEquals(1000.0, c.get(key(), 1200.0))
        assertEquals(1000.0, c.get(key(), 1000.5))
        // non sta sotto il limite chiesto: +∞ (nessun percorso sotto quel limite), senza ricerca
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 900.0))
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 1000.0))
    }

    @Test
    fun `no path below a limit is reused only for lower limits`() {
        val c = QuickWeightCache()
        c.put(key(), Double.POSITIVE_INFINITY, 1500.0)
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 1500.0))
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 1000.0))
        assertNull(c.get(key(), 1600.0))
        assertNull(c.get(key(), Double.POSITIVE_INFINITY))
        assertNull(c.get(key(a = 45.5), 1000.0), "un'altra chiave")
    }

    @Test
    fun `a worse entry never replaces a better one`() {
        val c = QuickWeightCache()
        c.put(key(), Double.POSITIVE_INFINITY, 1500.0)
        // un limite più basso: resta quello alto
        c.put(key(), Double.POSITIVE_INFINITY, 1000.0)
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 1500.0))
        // un limite più alto sostituisce
        c.put(key(), Double.POSITIVE_INFINITY, 2000.0)
        assertEquals(Double.POSITIVE_INFINITY, c.get(key(), 2000.0))
        // un peso finito sostituisce e poi resta
        c.put(key(), 2500.0, 3000.0)
        assertEquals(2500.0, c.get(key(), Double.POSITIVE_INFINITY))
        c.put(key(), Double.POSITIVE_INFINITY, 2000.0)
        assertEquals(2500.0, c.get(key(), Double.POSITIVE_INFINITY))
        assertEquals(1, c.size)
    }

    @Test
    fun `least recently used entries go beyond the maximum, clear empties`() {
        val c = QuickWeightCache(maxEntries = 3)
        c.put(key(a = 1.0), 1.0, 10.0); c.put(key(a = 2.0), 2.0, 10.0); c.put(key(a = 3.0), 3.0, 10.0)
        // la prima è usata adesso: esce la seconda
        assertEquals(1.0, c.get(key(a = 1.0), 10.0))
        c.put(key(a = 4.0), 4.0, 10.0)
        assertEquals(3, c.size)
        assertNull(c.get(key(a = 2.0), 10.0))
        assertEquals(1.0, c.get(key(a = 1.0), 10.0))
        assertEquals(4.0, c.get(key(a = 4.0), 10.0))
        c.clear()
        assertEquals(0, c.size)
        assertNull(c.get(key(a = 1.0), 10.0))
    }

    @Test
    fun `different router options are different keys`() {
        val c = QuickWeightCache()
        c.put(key(), 1000.0, 1500.0)
        assertNull(c.get(key(hgv.copy(avoidFeatures = 1)), 1500.0))
        assertNull(c.get(key(hgv.copy(maximumSpeed = 80.0)), 1500.0))
        assertNull(c.get(key(hgv.copy(maximumSpeedTimeOnly = true)), 1500.0))
        assertNull(c.get(key(hgv.copy(avoidPenalty = 30.0)), 1500.0))
        assertEquals(1000.0, c.get(key(hgv.copy()), 1500.0))
        assertEquals(1, c.size)
    }
}
