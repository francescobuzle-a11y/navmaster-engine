package it.navmaster.engine.profile

import it.navmaster.engine.profile.NodeDelays.Control
import it.navmaster.engine.storage.GraphEncoding
import kotlin.test.Test
import kotlin.test.assertEquals

class NodeDelaysTest {
    private val truck = HgvProfile()
    private val car = CarProfile()

    private fun s(c: Control, v: Double, p: VehicleProfile) = NodeDelays.seconds(c, v, p.acceleration, p.deceleration)

    @Test
    fun `seconds lost at 50 kmh`() {
        assertEquals(18.4, s(Control.SIGNALS, 50.0, truck), 0.06)
        assertEquals(22.8, s(Control.STOP, 50.0, truck), 0.06)
        assertEquals(12.5, s(Control.GIVE_WAY, 50.0, truck), 0.06)
        assertEquals(20.4, s(Control.TOLL, 50.0, truck), 0.06)
        assertEquals(12.5, s(Control.RAIL, 50.0, truck), 0.06)
        assertEquals(7.2, s(Control.CROSSING_SIGNALS, 50.0, truck), 0.06)
        assertEquals(12.1, s(Control.SIGNALS, 50.0, car), 0.06)
        assertEquals(10.1, s(Control.STOP, 50.0, car), 0.06)
        assertEquals(5.5, s(Control.GIVE_WAY, 50.0, car), 0.06)
        // la precedenza implicita, la rotatoria e la mini-rotatoria come una precedenza scritta
        assertEquals(s(Control.GIVE_WAY, 50.0, truck), s(Control.IMPLIED_GIVE_WAY, 50.0, truck))
        assertEquals(s(Control.GIVE_WAY, 50.0, truck), s(Control.ROUNDABOUT_ENTRY, 50.0, truck))
    }

    @Test
    fun `other speeds and the 70 kmh cap`() {
        assertEquals(14.2, s(Control.SIGNALS, 30.0, truck), 0.06)
        assertEquals(5.7, s(Control.GIVE_WAY, 30.0, truck), 0.06)
        assertEquals(31.2, s(Control.STOP, 70.0, truck), 0.06)
        assertEquals(s(Control.STOP, 70.0, truck), s(Control.STOP, 90.0, truck))
        assertEquals(13.3, s(Control.STOP, 70.0, car), 0.06)
        assertEquals(0.0, s(Control.NONE, 50.0, car))
    }

    @Test
    fun `classify node tags`() {
        assertEquals(Control.SIGNALS, NodeDelays.control(NodeDelays.classify(mapOf("highway" to "traffic_signals"))))
        val dir = NodeDelays.classify(mapOf("highway" to "traffic_signals", "traffic_signals:direction" to "backward"))
        assertEquals(NodeDelays.DIR_BACKWARD, NodeDelays.direction(dir))
        val stop = NodeDelays.classify(mapOf("highway" to "stop", "stop" to "all", "direction" to "forward"))
        assertEquals(Control.STOP, NodeDelays.control(stop)); assertEquals(NodeDelays.DIR_FORWARD, NodeDelays.direction(stop))
        assertEquals(true, NodeDelays.allWay(stop))
        assertEquals(Control.TOLL, NodeDelays.control(NodeDelays.classify(mapOf("barrier" to "toll_booth"))))
        assertEquals(Control.RAIL, NodeDelays.control(NodeDelays.classify(mapOf("railway" to "level_crossing"))))
        assertEquals(0, NodeDelays.classify(mapOf("railway" to "crossing")))
        assertEquals(0, NodeDelays.classify(mapOf("highway" to "crossing", "crossing" to "uncontrolled")))
        assertEquals(Control.CROSSING_SIGNALS, NodeDelays.control(NodeDelays.classify(mapOf("highway" to "crossing", "crossing" to "traffic_signals"))))
        assertEquals(Control.MINI_ROUNDABOUT, NodeDelays.control(NodeDelays.classify(mapOf("highway" to "mini_roundabout"))))
        assertEquals(0, NodeDelays.classify(null))
    }

    @Test
    fun `stored delay is rounded and capped at 63 s`() {
        val enc = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        val pe = enc.profile("heavyvehicle")
        val f = enc.registry.newFlags()
        pe.writeDelay(f, 18.4, 200.0)
        assertEquals(18.0, pe.delay!!.get(f, false))
        assertEquals(63.0, pe.delay!!.get(f, true))
        pe.writeDelay(f, -1.0, Double.NaN)
        assertEquals(0.0, pe.delay!!.get(f, false)); assertEquals(0.0, pe.delay!!.get(f, true))
    }
}
