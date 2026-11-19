package it.navmaster.engine.storage

import it.navmaster.engine.profile.GuidanceAttributes
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/** I casi limite dei file compatti del formato 5 (turn_index, guidance rada, geometria a scarti). */
class FormatFiveStorageTest {
    private fun tmpDir(): File = Files.createTempDirectory("nme-f5").toFile().also { it.deleteOnExit() }

    @Test
    fun `turn index - same answers after flush and reload, flush of a loaded store changes nothing`() {
        val dir = tmpDir()
        val enc = TurnCostEncoding(listOf("car-ors", "heavyvehicle"))
        TurnCostStorage(GraphDirectory(dir), enc).use { tc ->
            tc.set(1, 5, 2, enc.forbid(0, "car-ors"))
            tc.set(1, 5, 2, enc.forbid(0, "heavyvehicle"))
            tc.set(7, 5, 8, enc.forbid(0, "heavyvehicle"))
            tc.set(3, 130, 4, enc.forbid(0, "car-ors")) // oltre la prima parola e il primo blocco di ranghi
            tc.set(9, 64 * 8 * 3 + 1, 10, enc.forbid(0, "car-ors"))
            tc.flush()
            // dopo flush risponde dalle teste compatte
            assertTrue(tc.hasEntries(5)); assertTrue(tc.hasEntries(130)); assertFalse(tc.hasEntries(6)); assertFalse(tc.hasEntries(100_000))
        }
        assertFalse(File(dir, "turn_heads").exists())
        assertTrue(File(dir, "turn_index").exists())
        TurnCostStorage(GraphDirectory(dir), enc).use { tc ->
            assertTrue(tc.loadExisting())
            assertEquals(4, tc.count)
            assertTrue(tc.hasEntries(5)); assertTrue(tc.hasEntries(130)); assertTrue(tc.hasEntries(64 * 8 * 3 + 1))
            assertFalse(tc.hasEntries(0)); assertFalse(tc.hasEntries(131)); assertFalse(tc.hasEntries(-1))
            assertTrue(enc.isForbidden(tc.flags(1, 5, 2), "car-ors") && enc.isForbidden(tc.flags(1, 5, 2), "heavyvehicle"))
            assertTrue(enc.isForbidden(tc.flags(7, 5, 8), "heavyvehicle") && !enc.isForbidden(tc.flags(7, 5, 8), "car-ors"))
            assertEquals(0, tc.flags(2, 5, 1))
            tc.flush() // un deposito caricato: nessuna conversione, nessun file cancellato
        }
        assertTrue(File(dir, "turn_index").exists())
    }

    @Test
    fun `turn index left from another graph is ignored for a format 4 graph`() {
        val dir = tmpDir()
        val enc = TurnCostEncoding(listOf("p"))
        TurnCostStorage(GraphDirectory(dir), enc).use { tc -> tc.set(1, 5, 2, enc.forbid(0, "p")); tc.flush() }
        // un grafo di formato 4 scritto sopra nella stessa cartella: turn_costs e turn_heads suoi, turn_index rimasto
        val gd = GraphDirectory(dir)
        gd.create("turn_costs", 1 shl 16).use { c ->
            c.ensureCapacity(32); c.setInt(0, 10); c.setInt(4, 11); c.setInt(8, enc.forbid(0, "p")); c.setInt(12, -1)
            c.setHeader(0, 4); c.setHeader(1, 1); c.flush()
        }
        gd.create("turn_heads", 1 shl 16).use { h ->
            h.ensureCapacity(16 * 4); for (n in 0 until 16) h.setInt(n * 4L, if (n == 3) 0 else -1)
            h.setHeader(0, 4); h.setHeader(1, 16); h.flush()
        }
        TurnCostStorage(GraphDirectory(dir), enc).use { tc ->
            assertTrue(tc.loadExisting())
            assertTrue(tc.hasEntries(3)); assertFalse(tc.hasEntries(5))
            assertTrue(enc.isForbidden(tc.flags(10, 3, 11), "p"))
        }
    }

    @Test
    fun `sparse guidance - same values as the dense records, before and after reload`() {
        val dir = tmpDir()
        val gd = GraphDirectory(dir)
        val names = StringStore(gd.create("names"))
        val edges = 130 // non multiplo di 64; gli ultimi archi senza dati
        val store = GuidanceStore.forImport(gd, names)
        val attrs = mapOf(
            0 to GuidanceAttributes(maxSpeedFwd = 50, lanes = 2, oneway = 1),
            2 to GuidanceAttributes(maxSpeedFwd = 90, maxSpeedHgvBwd = 70, ref = "A4", turnLanesFwd = "left|through", destinationBwd = "Milano"),
            63 to GuidanceAttributes(tunnel = true),
            64 to GuidanceAttributes(adrTunnel = 3),
            65 to GuidanceAttributes(widthSigned = true),
            70 to GuidanceAttributes(conditional = "hgv=no @ (Mo-Fr 07:00-19:00)"),
            100 to GuidanceAttributes(lanesFwd = 3, lanesBwd = 1, bridge = true, laneSignsFwd = "Bolzano|A22|blue"),
        )
        for ((e, a) in attrs) store.set(e, a, if (e == 2) "12|Verona" else null, null, if (e == 70) "toll_booth@1" else null, if (e == 100) "Trento" else null, null)
        fun dump(g: GuidanceStore): String = (0 until edges + 3).joinToString("\n") { e ->
            listOf(false, true).joinToString("|") { r ->
                "${g.maxSpeed(e, r)},${g.maxSpeedHgv(e, r)},${g.lanes(e, r)},${g.lanesExplicit(e, r)},${g.turnLanes(e, r)},${g.destination(e, r)}," +
                    "${g.destinationRef(e, r)},${g.junctionAtStart(e, r)},${g.laneSigns(e, r)},${g.relationSign(e, r)}"
            } + "|${g.lanesTotal(e)},${g.tunnel(e)},${g.bridge(e)},${g.oneway(e)},${g.ref(e)},${g.string(g.refPointer(e))},${g.specialNodes(e)},${g.conditional(e)},${g.adrTunnel(e)},${g.widthSigned(e)}"
        }
        val dense = dump(store)
        store.flush(edges)
        names.flush()
        assertEquals(dense, dump(store), "valori diversi dopo la conversione al formato rado")
        assertFalse(File(dir, "guidance.import").exists())
        store.close()
        val names2 = StringStore(gd.create("names")).also { assertTrue(it.loadExisting(gd)) }
        val loaded = GuidanceStore.load(gd, names2)!!
        assertEquals(dense, dump(loaded), "valori diversi dopo la riapertura")
        loaded.close()
    }

    @Test
    fun `varint geometry - long ways, extreme coordinates and records across segments`() {
        val dir = tmpDir()
        val reg = EncodedValueRegistry.Builder().add(BooleanEncodedValue("access", twoDirections = true)).build()
        // i double che il formato 4 restituiva: intToDeg dell'intero salvato
        fun q(d: Double) = BaseGraph.intToDeg(BaseGraph.degToInt(d))
        val expected = HashMap<Int, List<Pair<Double, Double>>>()
        BaseGraph(GraphDirectory(dir), reg).use { g ->
            val a = g.addNode(-89.9999999, -179.9999999)
            val b = g.addNode(89.9999999, 179.9999999)
            // un arco con migliaia di punti e scarti enormi (da un capo all'altro del mondo)
            val big = PointList()
            for (i in 0 until 3000) big.add(if (i % 2 == 0) 89.9 else -89.9, if (i % 2 == 0) 179.9 else -179.9)
            val e0 = g.addEdge(a, b, 1.0); g.setWayGeometry(e0, big)
            expected[e0] = (0 until big.size).map { q(big.lat(it)) to q(big.lon(it)) }
            // tanti archi corti fino a passare il confine del segmento da 1 MiB più volte
            var e = 0
            while (e < 60_000) {
                val n1 = g.addNode(45.0 + e * 1e-6, 9.0 - e * 1e-6)
                val n2 = g.addNode(45.1, 9.1)
                val pl = PointList()
                for (k in 0 until 1 + e % 9) pl.add(45.05 + k * 1e-5 - e * 1e-7, 9.05 - k * 3e-5)
                val id = g.addEdge(n1, n2, 1.0); g.setWayGeometry(id, pl)
                if (e % 97 == 0) expected[id] = (0 until pl.size).map { q(pl.lat(it)) to q(pl.lon(it)) }
                e++
            }
            g.flush()
        }
        BaseGraph(GraphDirectory(dir, mmap = true, readOnly = true), reg).use { g ->
            assertTrue(g.loadExisting())
            for ((edge, pts) in expected) {
                val pl = g.fetchWayGeometry(edge, false, FetchMode.PILLAR_ONLY)
                assertEquals(pts, (0 until pl.size).map { pl.lat(it) to pl.lon(it) }, "arco $edge")
                val all = g.fetchWayGeometry(edge, true, FetchMode.ALL)
                assertEquals(pts.size + 2, all.size)
                assertEquals(g.lat(g.nodeB(edge)), all.lat(0)); assertEquals(g.lon(g.nodeA(edge)), all.lon(all.size - 1))
            }
        }
    }
}
