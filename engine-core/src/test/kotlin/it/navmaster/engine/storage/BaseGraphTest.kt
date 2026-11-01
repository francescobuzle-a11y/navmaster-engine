package it.navmaster.engine.storage

import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class BaseGraphTest {
    private fun tmpDir(): File = Files.createTempDirectory("nme-graph").toFile().also { it.deleteOnExit() }

    // ogni registro ha le proprie istanze: un EncodedValue si inizializza una volta sola
    private fun registry() = EncodedValueRegistry.Builder()
        .add(BooleanEncodedValue("access", twoDirections = true))
        .add(DecimalEncodedValue("speed", 5, 5.0, twoDirections = true))
        .build()

    /** Triangolo 0-1-2 più un arco pendente 2-3 con due pillar. */
    private fun build(g: BaseGraph) {
        val access = g.encodedValues.bool("access")
        val speed = g.encodedValues.decimal("speed")
        val n0 = g.addNode(45.0, 9.0)
        val n1 = g.addNode(45.001, 9.0)
        val n2 = g.addNode(45.001, 9.001)
        val n3 = g.addNode(45.002, 9.002)
        val e01 = g.addEdge(n0, n1, 111.2)
        val e12 = g.addEdge(n1, n2, 78.6)
        val e20 = g.addEdge(n2, n0, 136.3)
        val e23 = g.addEdge(n2, n3, 200.0)
        val f = g.encodedValues.newFlags()
        access.set(f, false, true); access.set(f, true, false) // e01 solo in avanti (0→1)
        speed.set(f, false, 50.0); speed.set(f, true, 30.0)
        g.setFlags(e01, f)
        val both = g.encodedValues.newFlags()
        access.set(both, false, true); access.set(both, true, true)
        speed.set(both, false, 80.0); speed.set(both, true, 80.0)
        g.setFlags(e12, both); g.setFlags(e20, both); g.setFlags(e23, both)
        val pillars = PointList()
        pillars.add(45.0013, 9.0013); pillars.add(45.0017, 9.0017)
        g.setWayGeometry(e23, pillars)
        g.setName(e01, "Via Roma"); g.setName(e23, "Corso Italia"); g.setName(e12, "Via Roma")
    }

    private fun verify(g: BaseGraph) {
        val access = g.encodedValues.bool("access")
        val speed = g.encodedValues.decimal("speed")
        assertEquals(4, g.nodeCount); assertEquals(4, g.edgeCount)
        assertEquals(45.001, g.lat(1), 1e-7); assertEquals(9.001, g.lon(2), 1e-7)
        assertEquals(111.2, g.distance(0), 1e-3)

        // adiacenza del nodo 2: archi 3 (2→3), 2 (2→0), 1 (2→1, inverso) in ordine di inserimento inverso
        val it = g.createEdgeIterator()
        val seen = mutableListOf<Triple<Int, Int, Boolean>>()
        it.setBaseNode(2)
        while (it.next()) seen += Triple(it.edge, it.adjNode, it.reverse)
        assertEquals(listOf(Triple(3, 3, false), Triple(2, 0, false), Triple(1, 1, true)), seen)

        // direzioni dell'arco 0: da 0 accesso avanti sì; da 1 (inverso) no
        it.setBaseNode(0)
        var found = false
        while (it.next()) if (it.edge == 0) {
            found = true
            assertFalse(it.reverse); assertTrue(it.get(access)); assertFalse(it.getReverse(access))
            assertEquals(50.0, it.get(speed)); assertEquals(30.0, it.getReverse(speed))
            assertEquals(1, it.adjNode); assertEquals(0, it.edgeKey)
        }
        assertTrue(found)
        it.setBaseNode(1)
        while (it.next()) if (it.edge == 0) {
            assertTrue(it.reverse); assertFalse(it.get(access)); assertTrue(it.getReverse(access))
            assertEquals(30.0, it.get(speed)); assertEquals(1, it.edgeKey)
        }

        // geometria: ALL da 2→3 e da 3→2
        val fwd = g.fetchWayGeometry(3, reverse = false, FetchMode.ALL)
        assertEquals(4, fwd.size)
        assertEquals(45.001, fwd.lat(0), 1e-7); assertEquals(45.0013, fwd.lat(1), 1e-7); assertEquals(45.002, fwd.lat(3), 1e-7)
        val bwd = g.fetchWayGeometry(3, reverse = true, FetchMode.ALL)
        assertEquals(45.002, bwd.lat(0), 1e-7); assertEquals(45.0017, bwd.lat(1), 1e-7); assertEquals(45.001, bwd.lat(3), 1e-7)
        assertEquals(2, g.fetchWayGeometry(3, false, FetchMode.PILLAR_ONLY).size)
        assertEquals(3, g.fetchWayGeometry(3, false, FetchMode.BASE_AND_PILLAR).size)
        assertEquals(2, g.fetchWayGeometry(3, false, FetchMode.TOWER_ONLY).size)
        assertEquals(2, g.fetchWayGeometry(0, false, FetchMode.ALL).size) // senza pillar
        assertEquals(0, g.fetchWayGeometry(0, false, FetchMode.PILLAR_ONLY).size)

        assertEquals("Via Roma", g.name(0)); assertEquals("Corso Italia", g.name(3)); assertEquals("", g.name(2))
        assertEquals(g.nameRef(0), g.nameRef(1)) // nomi uguali, stesso puntatore
        assertTrue(g.bounds.contains(45.0015, 9.0015)); assertFalse(g.bounds.contains(46.0, 9.0))
    }

    @Test
    fun `build, iterate and fetch geometry in RAM`() {
        BaseGraph(GraphDirectory(tmpDir()), registry()).use { g -> build(g); verify(g) }
    }

    @Test
    fun `flush and reload with RAM and mmap`() {
        val dir = tmpDir()
        BaseGraph(GraphDirectory(dir), registry()).use { g -> build(g); g.freeze(); g.flush() }
        BaseGraph(GraphDirectory(dir), registry()).use { g -> assertTrue(g.loadExisting()); assertTrue(g.frozen); verify(g) }
        BaseGraph(GraphDirectory(dir, mmap = true, readOnly = true), registry()).use { g -> assertTrue(g.loadExisting()); verify(g) }
    }

    @Test
    fun `loading with different encoded values fails`() {
        val dir = tmpDir()
        BaseGraph(GraphDirectory(dir), registry()).use { g -> build(g); g.flush() }
        val other = EncodedValueRegistry.Builder().add(BooleanEncodedValue("x")).build()
        val ex = runCatching { BaseGraph(GraphDirectory(dir), other).use { it.loadExisting() } }.exceptionOrNull()
        assertTrue(ex is IllegalStateException, "atteso errore di compatibilità, ottenuto $ex")
    }

    @Test
    fun `empty directory has no graph`() {
        assertFalse(BaseGraph(GraphDirectory(tmpDir()), registry()).use { it.loadExisting() })
    }

    @Test
    fun `self loop is linked once`() {
        BaseGraph(GraphDirectory(tmpDir()), registry()).use { g ->
            val n = g.addNode(1.0, 1.0)
            val e = g.addEdge(n, n, 10.0)
            val it = g.createEdgeIterator().setBaseNode(n)
            var count = 0
            while (it.next()) { count++; assertEquals(e, it.edge); assertEquals(n, it.adjNode) }
            assertEquals(1, count)
        }
    }
}
