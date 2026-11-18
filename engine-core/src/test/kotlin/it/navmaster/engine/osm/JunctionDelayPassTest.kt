package it.navmaster.engine.osm

import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.NodeDelays
import it.navmaster.engine.profile.NodeDelays.Control
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import java.nio.file.Files
import java.util.BitSet
import kotlin.test.Test
import kotlin.test.assertEquals

/** Le perdite agli incroci su grafi fatti a mano (velocità 50 km/h per tutti, salvo dove detto). */
class JunctionDelayPassTest {
    private val encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
    private val hgv = encoding.profile("heavyvehicle")
    private val truckGiveWay = Math.round(NodeDelays.seconds(Control.GIVE_WAY, 50.0, 0.5, 1.0)).toDouble()
    private val truckSignals = Math.round(NodeDelays.seconds(Control.SIGNALS, 50.0, 0.5, 1.0)).toDouble()

    private fun graph(): BaseGraph =
        BaseGraph(GraphDirectory(Files.createTempDirectory("nme-jdp").toFile().also { it.deleteOnExit() }), encoding.registry)

    private var lat = 45.0
    private fun node(g: BaseGraph): Int { lat += 0.001; return g.addNode(lat, 9.0) }

    private fun road(g: BaseGraph, a: Int, b: Int, rc: RoadClass, link: Boolean = false, roundabout: Boolean = false, oneway: Boolean = false, speed: Double = 50.0, length: Double = 100.0): Int {
        val e = g.addEdge(a, b, length)
        val f = encoding.registry.newFlags()
        encoding.shared.roadClass.set(f, rc); encoding.shared.roadClassLink.set(f, link); encoding.shared.roundabout.set(f, roundabout)
        for (pe in encoding.profiles) {
            pe.access.set(f, false, true); pe.access.set(f, true, !oneway)
            pe.speed.set(f, false, speed); pe.speed.set(f, true, speed)
        }
        g.setFlags(e, f)
        return e
    }

    private fun run(g: BaseGraph, controls: IntIntHashMap = IntIntHashMap(), nearEnd: BitSet = BitSet()) =
        JunctionDelayPass(g, encoding, controls, nearEnd).also { it.run() }

    private fun delay(g: BaseGraph, e: Int, reverse: Boolean) = hgv.delay!!.get(g.getFlags(e), reverse)

    @Test
    fun `T junction - the minor road gives way, the main road passes`() = graph().use { g ->
        val c = node(g); val p1 = node(g); val p2 = node(g); val t = node(g)
        val a = road(g, p1, c, RoadClass.PRIMARY); val b = road(g, c, p2, RoadClass.PRIMARY); val m = road(g, t, c, RoadClass.TERTIARY)
        val pass = run(g)
        assertEquals(truckGiveWay, delay(g, m, false)) // la tertiary che arriva in c
        assertEquals(0.0, delay(g, m, true))            // verso t (capo morto)
        assertEquals(0.0, delay(g, a, false)); assertEquals(0.0, delay(g, b, true)) // la primary che prosegue
        assertEquals(1, pass.events[Control.IMPLIED_GIVE_WAY.ordinal])
    }

    @Test
    fun `a degree-2 node between two primaries costs nothing`() = graph().use { g ->
        val a = node(g); val b = node(g); val c = node(g)
        val e1 = road(g, a, b, RoadClass.PRIMARY); val e2 = road(g, b, c, RoadClass.PRIMARY, speed = 70.0)
        run(g)
        for (e in listOf(e1, e2)) for (r in listOf(false, true)) assertEquals(0.0, delay(g, e, r))
    }

    @Test
    fun `slip roads merge onto a motorway for free but give way on a primary`() = graph().use { g ->
        val x = node(g); val m1 = node(g); val m2 = node(g); val l = node(g)
        road(g, m1, x, RoadClass.MOTORWAY, oneway = true, speed = 80.0); road(g, x, m2, RoadClass.MOTORWAY, oneway = true, speed = 80.0)
        val onRamp = road(g, l, x, RoadClass.MOTORWAY, link = true, oneway = true)
        val y = node(g); val p1 = node(g); val p2 = node(g); val l2 = node(g)
        road(g, p1, y, RoadClass.PRIMARY); road(g, y, p2, RoadClass.PRIMARY)
        val offRamp = road(g, l2, y, RoadClass.MOTORWAY, link = true, oneway = true)
        run(g)
        assertEquals(0.0, delay(g, onRamp, false))
        assertEquals(truckGiveWay, delay(g, offRamp, false))
    }

    @Test
    fun `an approach signal on a degree-2 node stops only the traffic heading to the junction`() = graph().use { g ->
        // p0 —100 m— s (semaforo, way spezzata) —20 m— j (incrocio a T con la primary che prosegue e una tertiary)
        val p0 = node(g); val s = node(g); val j = node(g); val p2 = node(g); val t = node(g)
        val before = road(g, p0, s, RoadClass.PRIMARY); val after = road(g, s, j, RoadClass.PRIMARY, length = 20.0)
        road(g, j, p2, RoadClass.PRIMARY); road(g, t, j, RoadClass.TERTIARY)
        val pass = run(g, IntIntHashMap().apply { put(s, Control.SIGNALS.ordinal) })
        assertEquals(truckSignals, delay(g, before, false)) // verso l'incrocio
        assertEquals(0.0, delay(g, after, true))            // in uscita dall'incrocio: non è il suo semaforo
        assertEquals(1, pass.events[Control.SIGNALS.ordinal])
    }

    @Test
    fun `signals on the tower slow every approach`() = graph().use { g ->
        val c = node(g); val p1 = node(g); val p2 = node(g); val t = node(g)
        val a = road(g, p1, c, RoadClass.PRIMARY); val b = road(g, c, p2, RoadClass.PRIMARY); val m = road(g, t, c, RoadClass.TERTIARY)
        val controls = IntIntHashMap().apply { put(c, Control.SIGNALS.ordinal) }
        run(g, controls)
        assertEquals(truckSignals, delay(g, a, false))
        assertEquals(truckSignals, delay(g, b, true))
        assertEquals(truckSignals, delay(g, m, false))
        assertEquals(0.0, delay(g, a, true)) // verso p1: nessun semaforo
    }

    @Test
    fun `entering a roundabout gives way, leaving it is free`() = graph().use { g ->
        val n = node(g); val r1 = node(g); val r2 = node(g); val ap = node(g)
        val in1 = road(g, r1, n, RoadClass.PRIMARY, roundabout = true, oneway = true, speed = 25.0)
        val out1 = road(g, n, r2, RoadClass.PRIMARY, roundabout = true, oneway = true, speed = 25.0)
        val approach = road(g, ap, n, RoadClass.SECONDARY)
        val pass = run(g)
        assertEquals(truckGiveWay, delay(g, approach, false))
        assertEquals(0.0, delay(g, approach, true))
        assertEquals(0.0, delay(g, in1, false)); assertEquals(0.0, delay(g, out1, false))
        assertEquals(1, pass.events[Control.ROUNDABOUT_ENTRY.ordinal])
    }

    @Test
    fun `an explicit stop just before the junction replaces the implied give way`() = graph().use { g ->
        val c = node(g); val p1 = node(g); val p2 = node(g); val t = node(g)
        road(g, p1, c, RoadClass.PRIMARY); road(g, c, p2, RoadClass.PRIMARY); val m = road(g, t, c, RoadClass.TERTIARY)
        val near = BitSet().apply { set(m * 2) }
        run(g, nearEnd = near)
        assertEquals(0.0, delay(g, m, false))
    }

    @Test
    fun `a stop on the tower applies only to the minor approach`() = graph().use { g ->
        val c = node(g); val p1 = node(g); val p2 = node(g); val t = node(g)
        val a = road(g, p1, c, RoadClass.PRIMARY); road(g, c, p2, RoadClass.PRIMARY); val m = road(g, t, c, RoadClass.TERTIARY)
        run(g, IntIntHashMap().apply { put(c, Control.STOP.ordinal) })
        assertEquals(Math.round(NodeDelays.seconds(Control.STOP, 50.0, 0.5, 1.0)).toDouble(), delay(g, m, false))
        assertEquals(0.0, delay(g, a, false))
    }
}
