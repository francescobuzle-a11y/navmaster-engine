package it.navmaster.engine.profile

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

class HgvProfileTest {
    private val hgv = HgvProfile()
    private val car = CarProfile()

    private fun speed(profile: VehicleProfile, vararg tags: Pair<String, String>): Double {
        val w = way(*tags)
        val r = profile.analyse(w, profile.access(w))
        assertNotNull(r, "la way dovrebbe essere accessibile: ${tags.toList()}")
        return r.speedKmh
    }

    @Test
    fun `truck speeds are lower than car speeds on main roads`() {
        // 85 e non 80: in ORS la tabella heavyvehicle.json sovrascrive il valore del codice Java
        assertEquals(85.0, speed(hgv, "highway" to "motorway"))
        assertEquals(100.0, speed(car, "highway" to "motorway"))
        assertEquals(60.0, speed(hgv, "highway" to "primary"))
        assertEquals(65.0, speed(car, "highway" to "primary"))
        assertEquals(50.0, speed(hgv, "highway" to "motorway_link"))
    }

    @Test
    fun `explicit maxspeed gives 90 percent of the limit`() {
        assertEquals(45.0, speed(car, "highway" to "residential", "maxspeed" to "50"))
        // per il camion un maxspeed sopra la velocità di default viene ignorato
        assertEquals(85.0, speed(hgv, "highway" to "motorway", "maxspeed" to "130"))
        // maxspeed:hgv ha la precedenza
        assertEquals(60 * 0.9, speed(hgv, "highway" to "motorway", "maxspeed:hgv" to "60"), 1e-9)
        // un maxspeed sotto la default conta anche per il camion
        assertEquals(30 * 0.9, speed(hgv, "highway" to "primary", "maxspeed" to "30"), 1e-9)
    }

    @Test
    fun `maxspeed none is capped at the profile maximum`() {
        assertEquals(90.0, speed(hgv, "highway" to "motorway", "maxspeed:hgv" to "none"))
        assertEquals(135.0, speed(car, "highway" to "motorway", "maxspeed" to "none"))
    }

    @Test
    fun `zone maxspeed applies when no explicit maxspeed`() {
        assertEquals(50.0, speed(car, "highway" to "primary", "zone:maxspeed" to "IT:urban"))
    }

    @Test
    fun `surface and tracktype reduce speed`() {
        assertEquals(30.0, speed(car, "highway" to "primary", "surface" to "gravel"))
        assertEquals(20.0, speed(car, "highway" to "track", "tracktype" to "grade3"))
        // l'auto non percorre track oltre grade3 (maximum_grade_level = 3)
        assertEquals(Access.CAN_SKIP, car.access(way("highway" to "track", "tracktype" to "grade5")))
    }

    @Test
    fun `roundabout caps speed by lanes`() {
        assertEquals(40.0, speed(car, "highway" to "primary", "junction" to "roundabout", "lanes" to "2"))
        assertEquals(35.0, speed(car, "highway" to "primary", "junction" to "roundabout", "lanes" to "1"))
    }

    @Test
    fun `access rules for trucks`() {
        assertEquals(Access.WAY, hgv.access(way("highway" to "primary")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "footway")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "residential", "access" to "no")))
        // come in ORS: `hgv` è la prima chiave di accesso del camion, quindi hgv=yes riapre una strada con motor_vehicle=no
        assertEquals(Access.WAY, hgv.access(way("highway" to "residential", "motor_vehicle" to "no", "hgv" to "yes")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "residential", "motor_vehicle" to "yes", "hgv" to "no")))
        // ...e `access` precede `motorcar`: access=no + motorcar=yes resta chiusa al camion ma non all'auto
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "residential", "access" to "no", "motorcar" to "yes")))
        assertEquals(Access.WAY, car.access(way("highway" to "residential", "access" to "no", "motorcar" to "yes")))
        assertEquals(Access.CAN_SKIP, car.access(way("highway" to "residential", "motor_vehicle" to "no", "hgv" to "yes")))
        // il camion esclude i track non pavimentati (maximum_grade_level = 1)
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "track", "tracktype" to "grade2")))
        assertEquals(Access.WAY, car.access(way("highway" to "track", "tracktype" to "grade2")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "residential", "maxwidth" to "1.8")))
        assertEquals(Access.FERRY, hgv.access(way("route" to "ferry")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("route" to "ferry", "foot" to "yes")))
        assertEquals(Access.CAN_SKIP, hgv.access(way("highway" to "primary", "smoothness" to "impassable")))
        // delivery è vietato all'auto ma ammesso al camion
        assertEquals(Access.CAN_SKIP, car.access(way("highway" to "service", "access" to "delivery")))
        assertEquals(Access.WAY, hgv.access(way("highway" to "service", "access" to "delivery")))
    }

    @Test
    fun `oneway and roundabout directions`() {
        val w = way("highway" to "primary", "oneway" to "yes")
        val r = hgv.analyse(w, hgv.access(w))!!
        assertTrue(r.forward); assertFalse(r.backward)
        val rev = way("highway" to "primary", "oneway" to "-1")
        val rr = hgv.analyse(rev, hgv.access(rev))!!
        assertFalse(rr.forward); assertTrue(rr.backward)
        val rb = way("highway" to "primary", "junction" to "roundabout")
        val rrb = hgv.analyse(rb, hgv.access(rb))!!
        assertTrue(rrb.forward); assertFalse(rrb.backward)
    }

    @Test
    fun `truck priorities`() {
        assertEquals(PriorityCode.VERY_NICE, hgv.priority(way("highway" to "motorway")))
        assertEquals(PriorityCode.PREFER, hgv.priority(way("highway" to "primary")))
        assertEquals(PriorityCode.UNCHANGED, hgv.priority(way("highway" to "tertiary")))
        assertEquals(PriorityCode.AVOID, hgv.priority(way("highway" to "residential")))
        assertEquals(PriorityCode.REACH_DESTINATION, hgv.priority(way("highway" to "residential", "maxspeed" to "30")))
        assertEquals(PriorityCode.VERY_BAD, hgv.priority(way("highway" to "living_street")))
        assertEquals(PriorityCode.BEST, hgv.priority(way("highway" to "residential", "hgv" to "designated")))
        // un limite basso su una primaria indica un centro abitato
        assertEquals(PriorityCode.AVOID, hgv.priority(way("highway" to "primary", "maxspeed" to "40")))
        assertEquals(PriorityCode.UNCHANGED, hgv.priority(way("highway" to "primary", "maxspeed" to "50")))
    }

    @Test
    fun `barriers on nodes`() {
        assertTrue(hgv.isBarrier(mapOf("barrier" to "bollard")))
        assertFalse(hgv.isBarrier(mapOf("barrier" to "bollard", "motor_vehicle" to "yes")))
        assertTrue(hgv.isBarrier(mapOf("barrier" to "gate"))) // blockBarriers=true
        assertFalse(HgvProfile(VehicleProfileOptions(blockBarriers = false)).isBarrier(mapOf("barrier" to "gate")))
        assertTrue(hgv.isBarrier(mapOf("barrier" to "gate", "access" to "no")))
        assertTrue(hgv.isBarrier(mapOf("ford" to "yes")))
        assertFalse(car.isBarrier(mapOf("barrier" to "bollard", "motorcar" to "yes")))
        assertTrue(car.isBarrier(mapOf("barrier" to "bus_trap")))
    }

    @Test
    fun `ferry speed from duration and distance`() {
        val w = way("route" to "ferry")
        // 10 km in 30 minuti: 20 km/h / 1.4 ≈ 14
        val r = hgv.analyse(w, Access.FERRY, estimatedDistance = 10_000.0, ferryDurationSeconds = 1800)
        assertEquals(14.0, r!!.speedKmh)
        // sconosciuta: penalizzata a 5
        assertEquals(5.0, hgv.analyse(w, Access.FERRY)!!.speedKmh)
    }

    @Test
    fun `acceleration model lowers the average speed on short roads`() {
        val p = CarProfile(VehicleProfileOptions(useAcceleration = true))
        val short = p.adjustSpeedForAcceleration(100.0, 50.0)
        val long = p.adjustSpeedForAcceleration(5000.0, 50.0)
        assertTrue(short < long && long <= 50.0, "short=$short long=$long")
        assertEquals(90.0, p.adjustSpeedForAcceleration(100.0, 90.0))
    }

    @Test
    fun `way info on motorway links`() {
        assertEquals("destinations: Milano, Bologna", hgv.wayInfo(way("highway" to "motorway_link", "destination" to "Milano;Bologna")))
        assertEquals("", hgv.wayInfo(way("highway" to "primary", "destination" to "Milano")))
    }

    @Test
    fun `track grade levels`() {
        assertEquals(3, VehicleProfile.trackGradeLevel("grade3"))
        assertEquals(3, VehicleProfile.trackGradeLevel("grade2;grade3"))
        assertEquals(0, VehicleProfile.trackGradeLevel("unknown"))
        assertEquals(0, VehicleProfile.trackGradeLevel("paved"))
    }
}
