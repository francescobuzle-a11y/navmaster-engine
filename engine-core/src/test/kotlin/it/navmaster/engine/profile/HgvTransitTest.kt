package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmWay
import it.navmaster.engine.profile.HgvTransit.BOTH
import it.navmaster.engine.profile.HgvTransit.BWD
import it.navmaster.engine.profile.HgvTransit.EXEMPT
import it.navmaster.engine.profile.HgvTransit.FWD
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * I divieti di transito dei mezzi pesanti ([HgvTransit]) sui tag veri (taginfo Geofabrik e Overpass,
 * novembre 2026): AT B179 Fernpass, B171, B178, DE, CH, FR, IT.
 */
class HgvTransitTest {
    private fun w(vararg tags: Pair<String, String>) = OsmWay(1, mapOf("highway" to "secondary", *tags), longArrayOf(1, 2))
    private val hgvBit = HeavyVehicleAttributes.HGV

    private fun nav(way: OsmWay) = WayAttributeParser.parse(way, isFerry = false, privateMainRoads = true, transitBans = true)
    private fun ors(way: OsmWay) = WayAttributeParser.parse(way, isFerry = false)

    /** (descrizione, way, bit attesi) */
    private val soft = listOf(
        Triple("B179 Fernpass", w("ref" to "B 179", "maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ destination; none @ delivery"), BOTH),
        Triple("AT ordine inverso", w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ delivery; none @ destination"), BOTH),
        Triple("elenco fra parentesi", w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ (destination;delivery)"), BOTH),
        Triple("B171 elenco con spazio", w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ (destination; delivery)"), BOTH),
        Triple("AT no @ delivery", w("maxweightrating:hgv" to "3.5", "maxweightrating:hgv:conditional" to "no @ delivery"), BOTH),
        Triple("DE no @ (delivery)", w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "no @ (delivery)"), BOTH),
        Triple("B171 solo all'indietro", w("maxweightrating:hgv:backward" to "7.5", "maxweightrating:hgv:backward:conditional" to "none @ destination; none @ delivery"), BWD),
        Triple("B171 solo in avanti", w("maxweightrating:hgv:forward" to "7.5", "maxweightrating:hgv:forward:conditional" to "none @ destination"), FWD),
        Triple("senso con la deroga generale", w("maxweightrating:hgv:forward" to "7.5", "maxweightrating:hgv:conditional" to "none @ destination"), FWD),
        Triple("B178 vehicle:conditional", w("ref" to "B 178", "vehicle:conditional" to "destination @ (weight > 7.5)"), BOTH),
        Triple("AT virgola decimale", w("motor_vehicle:conditional" to "delivery @ (weight>7,5)"), BOTH),
        Triple("hgv=destination", w("hgv" to "destination"), BOTH),
        Triple("hgv=delivery", w("hgv" to "delivery"), BOTH),
        Triple("hgv=local", w("hgv" to "local"), BOTH),
        Triple("AT hgv=destination;delivery", w("hgv" to "destination;delivery"), BOTH),
        Triple("goods senza hgv", w("goods" to "destination"), BOTH),
        Triple("hgv=yes con la condizione di peso", w("hgv" to "yes", "hgv:conditional" to "destination @ (weight>7.5)"), BOTH),
        Triple("DE private @ peso", w("hgv:conditional" to "private @ (weight > 7.5)"), BOTH),
        Triple("access:conditional", w("access:conditional" to "destination @ (weightrating>12)"), BOTH),
        Triple("maxweightrating:hgv=30 vale per 40 t", w("maxweightrating:hgv" to "30", "maxweightrating:hgv:conditional" to "none @ destination"), BOTH),
        Triple("maxweight:hgv con deroga", w("maxweight:hgv" to "7.5", "maxweight:hgv:conditional" to "none @ destination"), BOTH),
        Triple("FR maxweightrating con deroga", w("maxweightrating" to "19", "maxweightrating:conditional" to "none @ delivery"), BOTH),
        Triple("DE maxweight con deroga", w("maxweight" to "7.5", "maxweight:conditional" to "none @ destination"), BOTH or EXEMPT),
        Triple("DE maxweight (destination)", w("maxweight" to "7.5", "maxweight:conditional" to "none @ (destination)"), BOTH or EXEMPT),
        Triple("AT bus e destinazione", w("maxweight" to "3.5", "maxweight:conditional" to "none @ bus; none @ destination"), BOTH or EXEMPT),
        Triple("CH no @ destination", w("maxweight" to "18", "maxweight:conditional" to "no @ destination"), BOTH or EXEMPT),
        Triple("hgv=no con yes @ destination", w("hgv" to "no", "hgv:conditional" to "yes @ destination"), BOTH),
        Triple("DE hgv=no con yes @ delivery", w("hgv" to "no", "hgv:conditional" to "yes @ (delivery)"), BOTH),
        Triple("l'ultima parte valida vince", w("hgv:conditional" to "yes @ (weight>3.5); destination @ (weight>7.5)"), BOTH),
    )

    private val none = listOf(
        "nessun tag" to w(),
        "hgv=yes" to w("hgv" to "yes"),
        "hgv=designated" to w("hgv" to "designated"),
        "AT a orario" to w("hgv:conditional" to "destination @ (22:00-06:00)"),
        "orario e peso" to w("hgv:conditional" to "no @ (21:00-08:00 AND weight>3.5)"),
        "a orario, divieto" to w("hgv:conditional" to "no @ (Mo-Fr 22:00-05:00)"),
        "giorni e peso" to w("vehicle:conditional" to "destination @ (Sa,Su AND weight>7.5)"),
        "CH sotto il peso" to w("hgv:conditional" to "destination @ (weight<18)"),
        "IT neve" to w("maxweightrating:hgv:conditional" to "7.5 @ snow"),
        "maxweightrating:hgv=40" to w("maxweightrating:hgv" to "40", "maxweightrating:hgv:conditional" to "none @ destination"),
        "maxweightrating=44" to w("maxweightrating" to "44"),
        "peso oltre 40" to w("hgv:conditional" to "destination @ (weight>44)"),
        "Mikulov hgv=private sulla statale" to OsmWay(1, mapOf("highway" to "primary", "hgv" to "private", "oneway" to "no"), longArrayOf(1, 2)),
        "l'ultima parte valida apre" to w("hgv:conditional" to "destination @ (weight>7.5); yes @ (weight>3.5)"),
        "la chiave più specifica decide" to w("hgv:conditional" to "yes @ (weight>3.5)", "vehicle:conditional" to "destination @ (weight>7.5)"),
        "maxweight senza deroga" to w("maxweight" to "7.5"),
        "maxweight, deroga agricola" to w("maxweight" to "7.5", "maxweight:conditional" to "none @ agricultural"),
        "hgv=no senza deroga" to w("hgv" to "no"),
    )

    @Test
    fun `transit bans from the real tags`() {
        for ((name, way, bits) in soft) {
            assertEquals(bits, HgvTransit.flags(way), name)
            assertEquals(bits, nav(way).hgvTransit, name)
        }
        for ((name, way) in none) assertEquals(0, HgvTransit.flags(way), name)
    }

    @Test
    fun `the ORS model sees nothing new`() {
        for (way in soft.map { it.second } + none.map { it.second } + hardWays()) {
            val o = ors(way)
            assertEquals(0, o.hgvTransit, way.tags.toString())
            assertEquals(WayAttributeParser.parse(way, isFerry = false, transitBans = false), o)
            assertEquals(WayAttributeParser.blockedHeavyVehicleTypes(way), o.blockedHeavyVehicleTypes)
        }
        // come prima: hgv=delivery blocca gli altri mezzi pesanti, hgv=no blocca anche con la deroga, i limiti di classe non si leggono
        assertEquals(hgvBit, ors(w("hgv" to "delivery")).blockedHeavyVehicleTypes and hgvBit)
        assertEquals(hgvBit, ors(w("hgv" to "no", "hgv:conditional" to "yes @ destination")).blockedHeavyVehicleTypes and hgvBit)
        assertEquals(Double.POSITIVE_INFINITY, ors(w("maxweightrating:hgv" to "7.5")).maxWeight)
        assertEquals(0, ors(w("hgv:conditional" to "no @ (weight>26)")).blockedHeavyVehicleTypes)
    }

    private fun hardWays() = listOf(
        w("hgv:conditional" to "no @ (weight>26)"),
        w("maxweightrating" to "19"),
        w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ (destination AND 06:00-22:00)"),
    )

    @Test
    fun `real bans block the truck`() {
        // FR: no @ (weight>26) è un divieto, come hgv=no
        val fr = nav(w("hgv:conditional" to "no @ (weight>26)"))
        assertEquals(hgvBit, fr.blockedHeavyVehicleTypes and hgvBit)
        assertEquals(0, fr.hgvTransit)
        assertEquals(hgvBit, nav(w("motor_vehicle:conditional" to "no @ (weight > 7,5)")).blockedHeavyVehicleTypes and hgvBit)
        // a orario: niente
        assertEquals(0, nav(w("hgv:conditional" to "no @ (Mo-Fr 22:00-05:00)")).blockedHeavyVehicleTypes)
        assertEquals(0, nav(w("hgv:conditional" to "no @ (weight>40)")).blockedHeavyVehicleTypes)
        // FR: i ponti con maxweightrating senza deroga sono limiti di peso veri
        val bridge = nav(w("maxweightrating" to "19"))
        assertEquals(19.0, bridge.maxWeight); assertEquals(0, bridge.hgvTransit)
        assertEquals(30.0, nav(w("maxweightrating:hgv" to "30")).maxWeight)
        assertEquals(Double.POSITIVE_INFINITY, nav(w("maxweightrating:hgv" to "40")).maxWeight)
        assertEquals(7.5, nav(w("maxweight:hgv" to "7.5")).maxWeight)
        assertEquals(12.0, nav(w("maxweightrating:hgv:forward" to "12")).maxWeight, "un limite per senso limita l'arco")
        // il minimo con maxweight
        assertEquals(3.5, nav(w("maxweight" to "3.5", "maxweightrating:hgv" to "7.5")).maxWeight)
        assertEquals(7.5, nav(w("maxweight" to "12", "maxweightrating:hgv" to "7.5")).maxWeight)
        // DE: la deroga a orario non è una deroga
        val timed = nav(w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ (destination AND 06:00-22:00)"))
        assertEquals(7.5, timed.maxWeight); assertEquals(0, timed.hgvTransit)
        // con la deroga non è un limite: solo destinazione
        val fernpass = nav(w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ destination; none @ delivery"))
        assertEquals(Double.POSITIVE_INFINITY, fernpass.maxWeight); assertEquals(BOTH, fernpass.hgvTransit)
        // maxweight con deroga: il limite resta (per l'auto coi pesi e per i fatti), il filtro del camion lo salta
        val exempt = nav(w("maxweight" to "7.5", "maxweight:conditional" to "none @ destination"))
        assertEquals(7.5, exempt.maxWeight); assertEquals(BOTH or EXEMPT, exempt.hgvTransit)
        // un limite di classe senza deroga sulla stessa way: niente deroga sul peso
        val mixed = nav(w("maxweight" to "7.5", "maxweight:conditional" to "none @ destination", "maxweightrating" to "19"))
        assertEquals(7.5, mixed.maxWeight); assertEquals(BOTH, mixed.hgvTransit)
    }

    @Test
    fun `destination-only streets are no longer cut off for the truck`() {
        // hgv=delivery: prima il camion (vehicle_type=hgv) non poteva neanche arrivarci
        assertEquals(0, nav(w("hgv" to "delivery")).blockedHeavyVehicleTypes and hgvBit)
        assertEquals(0, nav(w("goods" to "delivery")).blockedHeavyVehicleTypes and HeavyVehicleAttributes.GOODS)
        // hgv=no con la deroga per destinazione: né bloccata né scartata
        val exempted = w("hgv" to "no", "hgv:conditional" to "yes @ destination")
        assertEquals(0, nav(exempted).blockedHeavyVehicleTypes and hgvBit)
        val navProfile = HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "AT"))
        assertEquals(Access.WAY, navProfile.access(exempted))
        assertTrue(navProfile.transitExemption(exempted))
        // la priorità non cambia: una sola penalità, il ×10 del peso
        assertEquals(navProfile.priority(w()), navProfile.priority(exempted))
        assertEquals(PriorityCode.PREFER, navProfile.priority(w("maxweightrating:hgv" to "7.5", "maxweightrating:hgv:conditional" to "none @ destination")))
        // ORS: scartata come prima
        assertEquals(Access.CAN_SKIP, HgvProfile().access(exempted))
        assertFalse(HgvProfile().transitExemption(exempted))
        // senza deroga resta un divieto anche in NAVMASTER
        assertEquals(Access.CAN_SKIP, navProfile.access(w("hgv" to "no")))
        assertEquals(Access.CAN_SKIP, navProfile.access(w("hgv" to "no", "hgv:conditional" to "yes @ (Mo-Fr 06:00-10:00)")))
        assertEquals(hgvBit, nav(w("hgv" to "no")).blockedHeavyVehicleTypes and hgvBit)
    }

    @Test
    fun `conditional parsing details`() {
        assertEquals(listOf("none @ (destination;delivery)", "none @ bus"), HgvTransit.parts("none @ (destination;delivery); none @ bus"))
        assertTrue(HgvTransit.exemption("none @ (destination; delivery)", setOf("none")))
        assertFalse(HgvTransit.exemption("none @ (destination AND 06:00-22:00)", setOf("none")))
        assertFalse(HgvTransit.exemption("none @ agricultural", setOf("none")))
        assertEquals(HgvTransit.Rule.DESTINATION, HgvTransit.weightRule("destination @ (weight > 7.5 t)"))
        assertEquals(HgvTransit.Rule.DESTINATION, HgvTransit.weightRule("destination @ (maxweightrating>=40)"))
        assertEquals(HgvTransit.Rule.NONE, HgvTransit.weightRule("destination @ (weight>=40.5)"))
        assertEquals(HgvTransit.Rule.NO, HgvTransit.weightRule("no @ (weight>3.5)"))
        assertEquals(HgvTransit.Rule.OPEN, HgvTransit.weightRule("no @ (weight>3.5); yes @ (weight>7.5)"))
        assertEquals(HgvTransit.Rule.NONE, HgvTransit.weightRule("no @ wet"))
        assertEquals(7.5, HgvTransit.limitTons("7,5"))
        assertEquals(7.5, HgvTransit.limitTons("7.5 t"))
        assertEquals(null, HgvTransit.limitTons("none"))
        // i limiti di velocità condizionali non cambiano: la virgola e maxweightrating restano fuori
        assertTrue(HgvProfile.conditionalHgvSpeed("60 @ (weight>7,5)").isNaN())
        assertTrue(HgvProfile.conditionalHgvSpeed("60 @ (maxweightrating>7.5)").isNaN())
        assertEquals(60.0, HgvProfile.conditionalHgvSpeed("60 @ (weight>7.5)"))
    }

    @Test
    fun `the exemptions reach the guidance data only on graphs with transit bans`() {
        val b171 = w("maxweightrating:hgv:backward" to "7.5", "maxweightrating:hgv:backward:conditional" to "none @ destination")
        assertEquals("maxweightrating:hgv:backward:conditional=none @ destination", GuidanceParser.conditional(b171, transitBans = true))
        assertEquals("", GuidanceParser.conditional(b171))
    }
}
