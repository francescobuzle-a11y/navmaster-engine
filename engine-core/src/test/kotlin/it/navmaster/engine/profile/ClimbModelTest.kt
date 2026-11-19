package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import it.navmaster.engine.terrain.TerrainSpeeds
import it.navmaster.engine.terrain.WayTerrain
import it.navmaster.engine.util.DistanceCalc
import kotlin.math.cos
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** Il mezzo di riferimento delle salite e delle curve (modello NAVMASTER) e il profilo di velocità di una way. */
class ClimbModelTest {
    private val m = ClimbModel.HGV

    @Test
    fun `uphill speeds of a 40 t truck with 280 kW match the table`() {
        // la tabella della decisione (40 t, 280 kW alle ruote, Crr 0,006, CdA 6 m², ρ 1,2)
        val table = listOf(0.03 to 65.0, 0.04 to 53.0, 0.05 to 45.0, 0.06 to 38.0, 0.07 to 34.0, 0.08 to 30.0, 0.10 to 24.0, 0.12 to 20.0, 0.14 to 18.0)
        for ((g, v) in table) assertEquals(v, m.uphillKmh(g), 1.0, "salita ${g * 100} %")
        assertEquals(Double.POSITIVE_INFINITY, m.uphillKmh(0.0))
        assertEquals(Double.POSITIVE_INFINITY, m.uphillKmh(-0.05))
        assertTrue(m.uphillKmh(0.01) > 90, "all'1 % il camion non rallenta: ${m.uphillKmh(0.01)}")
        assertTrue(m.uphillKmh(0.02) > 80, "al 2 % nemmeno in autostrada: ${m.uphillKmh(0.02)}")
        assertTrue(m.uphillKmh(0.16) in 14.0..17.0, "16 %: ${m.uphillKmh(0.16)}")
        assertTrue(m.uphillKmh(0.20) in 10.0..14.0, "20 % (oltre la tabella): ${m.uphillKmh(0.20)}")
        // crescente con la pendenza
        var last = Double.POSITIVE_INFINITY
        for (k in 1..40) { val v = m.uphillKmh(k * 0.004); assertTrue(v <= last); last = v }
    }

    @Test
    fun `downhill caps and curve speeds`() {
        val table = listOf(0.04 to 70.0, 0.05 to 60.0, 0.06 to 50.0, 0.07 to 45.0, 0.08 to 40.0, 0.10 to 35.0, 0.12 to 30.0, 0.14 to 25.0)
        for ((g, v) in table) assertEquals(v, m.downhillKmh(g), 1e-9, "discesa ${g * 100} %")
        assertEquals(37.5, m.downhillKmh(0.09), 1e-9)
        assertEquals(25.0, m.downhillKmh(0.20))
        assertEquals(Double.POSITIVE_INFINITY, m.downhillKmh(0.035))
        assertEquals(50.0, m.slopeKmh(-0.06), 1e-9)
        assertEquals(m.uphillKmh(0.06), m.slopeKmh(0.06))
        // curve: 3,6·√(1,5·R); sulle rampe 2,0; almeno 10 km/h
        assertEquals(15.3, m.curveKmh(12.0), 0.1)
        assertEquals(24.1, m.curveKmh(30.0), 0.1)
        assertEquals(44.1, m.curveKmh(100.0), 0.1)
        assertEquals(62.4, m.curveKmh(200.0), 0.1)
        assertEquals(3.6 * Math.sqrt(2.0 * 50), m.curveKmh(50.0, link = true), 1e-9)
        assertEquals(10.0, m.curveKmh(1.0))
        // in salita la potenza limita l'accelerazione
        assertEquals(0.5, m.acceleration(5.0, 0.0), 1e-9)
        assertTrue(m.acceleration(10.0, 0.06) < 0.1)
        assertEquals(0.0, m.acceleration(15.0, 0.08), 1e-9)
    }

    // ---------------------------------------------------------------- profilo di velocità

    private val lat0 = 46.0
    private val kx = cos(Math.toRadians(lat0)) * DistanceCalc.METERS_PER_DEGREE

    /** Una way diritta verso nord di [len] m con un punto ogni [step] m. */
    private fun straight(len: Double, step: Double, tags: Map<String, String> = mapOf("highway" to "primary")): Triple<OsmWay, DoubleArray, DoubleArray> {
        val n = (len / step).toInt() + 1
        val lats = DoubleArray(n) { lat0 + it * step / DistanceCalc.METERS_PER_DEGREE }
        val lons = DoubleArray(n) { 11.0 }
        return Triple(OsmWay(1, tags, LongArray(n) { it + 1L }), lats, lons)
    }

    @Test
    fun `a flat straight road keeps the base speed exactly`() {
        val (way, lats, lons) = straight(2000.0, 100.0)
        val t = WayTerrain.analyse(way, lats, lons, lats.size, { _, _ -> 312.0 })
        assertTrue(t.neutral, "strada piana e diritta: niente da fare")
        assertEquals(1.0, t.demCoverage)
        val sp = TerrainSpeeds(t, m, 63.0)
        assertEquals(63.0, sp.speedKmh(0.0, t.length, true))
        assertEquals(63.0, sp.speedKmh(0.0, t.length, false))
        assertEquals(63.0, sp.speedKmh(300.0, 900.0, true))
        // sotto il 2 % la pendenza non c'è
        val gentle = WayTerrain.analyse(way, lats, lons, lats.size, { la, _ -> (la - lat0) * DistanceCalc.METERS_PER_DEGREE * 0.015 })
        assertTrue(gentle.neutral)
    }

    @Test
    fun `a 6 percent climb - the truck slows to its power limit going up and to the downhill cap going down`() {
        val (way, lats, lons) = straight(2000.0, 100.0)
        val t = WayTerrain.analyse(way, lats, lons, lats.size, { la, _ -> 400 + (la - lat0) * DistanceCalc.METERS_PER_DEGREE * 0.06 })
        assertEquals(0.06, t.grade!![t.cells / 2], 1e-6)
        val sp = TerrainSpeeds(t, m, 63.0)
        assertEquals(m.uphillKmh(0.06), sp.speedKmh(0.0, t.length, true), 0.01)
        assertEquals(50.0, sp.speedKmh(0.0, t.length, false), 0.01)
        // il tetto di classe: in autostrada al massimo il 7 %
        val (mw, la2, lo2) = straight(2000.0, 100.0, mapOf("highway" to "motorway"))
        val steep = WayTerrain.analyse(mw, la2, lo2, la2.size, { la, _ -> (la - lat0) * DistanceCalc.METERS_PER_DEGREE * 0.12 })
        assertEquals(0.07, steep.grade!![10], 1e-9)
    }

    @Test
    fun `a hairpin costs about 20 s from 60 kmh`() {
        // 300 m diritti, un tornante di raggio 12 m (un punto ogni 3 m), 300 m indietro
        val pts = ArrayList<Pair<Double, Double>>() // (x, y) in metri
        for (k in 0..15) pts += 0.0 to k * 20.0
        for (k in 1 until 12) { val a = Math.PI - k * Math.PI / 12; pts += (12 + 12 * Math.cos(a)) to (300 + 12 * Math.sin(a)) }
        for (k in 0..15) pts += 24.0 to 300 - k * 20.0
        val lats = DoubleArray(pts.size) { lat0 + pts[it].second / DistanceCalc.METERS_PER_DEGREE }
        val lons = DoubleArray(pts.size) { 11.0 + pts[it].first / kx }
        val way = OsmWay(1, mapOf("highway" to "secondary"), LongArray(pts.size) { it + 1L })
        val t = WayTerrain.analyse(way, lats, lons, pts.size, null)
        assertEquals(1, t.curves, "un tornante = una curva")
        assertTrue(t.curveRadius[0] in 9.0..16.0, "raggio ${t.curveRadius[0]}")
        val sp = TerrainSpeeds(t, m, 60.0)
        val loss = sp.seconds(0.0, t.length, true) - t.length / (60 / 3.6)
        println("tornante R=${t.curveRadius[0]}: ${"%.1f".format(loss)} s persi da 60 km/h")
        assertTrue(loss in 14.0..28.0, "perdita del tornante $loss s")
        assertEquals(sp.seconds(0.0, t.length, true), sp.seconds(0.0, t.length, false), 1.0)
    }
}
