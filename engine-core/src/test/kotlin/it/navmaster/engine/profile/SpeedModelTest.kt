package it.navmaster.engine.profile

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/** Le velocità di marcia del modello NAVMASTER (prima dell'arrotondamento a 5 km/h) contro quelle di ORS. */
class SpeedModelTest {
    private fun hgv(country: String? = null) = HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = country))
    private fun car(country: String? = null) = CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = country))

    /** Come la sonda: una way di 11 m (il ponte della 52 ceca). */
    private fun speed(profile: VehicleProfile, vararg tags: Pair<String, String>, distance: Double = 11.0): Double {
        val w = way(*tags)
        val r = profile.analyse(w, profile.access(w), distance)
        assertNotNull(r, "la way dovrebbe essere accessibile: ${tags.toList()}")
        return r.speedKmh
    }

    @Test
    fun `the Czech 52 bridge of 11 m no longer drops the truck to 10 kmh`() {
        assertEquals(72.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "90"), 1e-9)
        assertEquals(81.0, speed(car("CZ"), "highway" to "primary", "maxspeed" to "90"), 1e-9)
        // ORS: il maxspeed scartato (90 > 60), poi l'accelerazione su 11 m
        val orsHgv = speed(HgvProfile(), "highway" to "primary", "maxspeed" to "90")
        assertTrue(orsHgv < 12.0, "ORS camion: $orsHgv")
        assertEquals(81.0, speed(CarProfile(), "highway" to "primary", "maxspeed" to "90"), 1e-9)
    }

    @Test
    fun `sign and law for the truck by country`() {
        assertEquals(63.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "70"), 1e-9)
        assertEquals(63.0, speed(car("CZ"), "highway" to "primary", "maxspeed" to "70"), 1e-9)
        assertEquals(63.0, speed(hgv("AT"), "highway" to "primary", "maxspeed" to "100", "maxspeed:hgv" to "70"), 1e-9)
        assertEquals(80.0, speed(hgv("IT"), "highway" to "motorway", "maxspeed" to "130"), 1e-9)
        assertEquals(117.0, speed(car("IT"), "highway" to "motorway", "maxspeed" to "130"), 1e-9)
        // AT: 80 solo in autostrada, 70 sulle altre strade fuori dai centri (come SpeedRules.legal dell'app)
        assertEquals(70.0, speed(hgv("AT"), "highway" to "trunk", "maxspeed" to "100"), 1e-9)
        assertEquals(80.0, speed(hgv("AT"), "highway" to "motorway", "maxspeed" to "130"), 1e-9)
        assertEquals(63.0, speed(hgv("IT"), "highway" to "primary", "maxspeed" to "90"), 1e-9)
        assertEquals(54.0, speed(hgv("FR"), "highway" to "secondary", "maxspeed" to "80"), 1e-9)
        assertEquals(54.0, speed(hgv("DE"), "highway" to "primary", "maxspeed" to "100"), 1e-9)
    }

    @Test
    fun `zone tags give country and context`() {
        assertEquals(72.0, speed(hgv(), "highway" to "primary", "zone:maxspeed" to "CZ:rural"), 1e-9)
        // senza zona e senza Paese: il default del tipo, sotto la riga di riserva (80 × 0,9)
        assertEquals(60.0, speed(hgv(), "highway" to "primary"), 1e-9)
        // il Paese del tag vince su quello del grafo
        assertEquals(63.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "90", "source:maxspeed" to "IT:rural"), 1e-9)
        assertEquals(45.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "CZ:urban"), 1e-9)
        // maxspeed=none (Germania) limitato dalla legge
        assertEquals(80.0, speed(hgv("DE"), "highway" to "motorway", "maxspeed" to "none"), 1e-9)
    }

    @Test
    fun `conditional truck limits for a 40 t articulated`() {
        assertEquals(54.0, speed(hgv("DE"), "highway" to "primary", "maxspeed" to "100", "maxspeed:hgv:conditional" to "60 @ (weight>7.5)"), 1e-9)
        // gli orari si ignorano
        assertEquals(72.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "90", "maxspeed:hgv:conditional" to "40 @ (22:00-06:00)"), 1e-9)
        assertEquals(60.0, HgvProfile.conditionalHgvSpeed("60 @ (trailer); 70 @ (weight>45)"))
        assertTrue(HgvProfile.conditionalHgvSpeed("70 @ (weight>45)").isNaN())
    }

    @Test
    fun `type caps for residential roads and roundabouts`() {
        assertEquals(30.0, speed(hgv("IT"), "highway" to "residential", "maxspeed" to "50"), 1e-9)
        assertEquals(40.0, speed(car("IT"), "highway" to "residential", "maxspeed" to "50"), 1e-9)
        assertEquals(25.0, speed(hgv("IT"), "highway" to "primary", "junction" to "roundabout", "maxspeed" to "50"), 1e-9)
        assertEquals(30.0, speed(car("IT"), "highway" to "primary", "junction" to "roundabout", "maxspeed" to "50"), 1e-9)
        assertEquals(30.0, speed(hgv("IT"), "highway" to "primary", "junction" to "roundabout", "lanes" to "2"), 1e-9)
    }

    @Test
    fun `no acceleration per way and no residential penalty`() {
        // la stessa velocità su 11 m e su 5 km
        for (d in listOf(11.0, 120.0, 5000.0)) {
            assertEquals(72.0, speed(hgv("CZ"), "highway" to "primary", "maxspeed" to "90", distance = d), 1e-9)
            assertEquals(45.0, speed(hgv("CZ"), "highway" to "secondary", "maxspeed" to "50", distance = d), 1e-9)
        }
        val w = way("highway" to "residential", "maxspeed" to "30", nodes = 12)
        assertEquals(27.0, hgv("IT").analyse(w, Access.WAY, 100.0)!!.speedKmh, 1e-9)
    }

    @Test
    fun `priority does not regress`() {
        val res = way("highway" to "residential", "maxspeed" to "50")
        assertEquals(PriorityCode.AVOID_IF_POSSIBLE, HgvProfile().priority(res))
        assertEquals(PriorityCode.AVOID_IF_POSSIBLE, hgv("IT").priority(res))
    }

    @Test
    fun `the ORS model is the default and unchanged`() {
        assertEquals(SpeedModel.ORS, HgvProfile().options.speedModel)
        assertEquals(SpeedModel.ORS, CarProfile().options.speedModel)
        assertEquals(85.0, speed(HgvProfile(), "highway" to "motorway", "maxspeed" to "130", distance = Double.NaN))
    }

    @Test
    fun `stored speeds are rounded to 5 kmh`() {
        val enc = it.navmaster.engine.storage.GraphEncoding(listOf(hgv("CZ")))
        val pe = enc.profiles[0]
        val f = enc.registry.newFlags()
        for ((v, stored) in listOf(72.0 to 70.0, 63.0 to 65.0, 81.0 to 80.0, 54.0 to 55.0, 45.0 to 45.0)) {
            pe.speed.set(f, false, v)
            assertEquals(stored, pe.speed.get(f, false))
        }
    }
}
