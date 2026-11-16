package it.navmaster.engine.weighting

import it.navmaster.engine.api.DirectionsRequest
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.WayAttributeParser
import it.navmaster.engine.profile.way
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

/**
 * Merci pericolose complete: gallerie per categoria ADR contro il codice galleria del carico,
 * `hazmat=no` per tutti, `hazmat:water=no` per le merci inquinanti per le acque; senza codice come
 * ORS (ogni galleria ADR vietata); i grafi di prima dei valori ADR si aprono e si comportano come prima.
 */
class HazmatFilterTest {
    private val roads = listOf(
        "plain" to arrayOf("highway" to "primary"),
        "tunnelC" to arrayOf("highway" to "primary", "tunnel" to "yes", "hazmat:C" to "no"),
        "tunnelCatE" to arrayOf("highway" to "motorway", "tunnel" to "yes", "hazmat:adr_tunnel_category" to "E"),
        "tunnelBandD" to arrayOf("highway" to "primary", "tunnel" to "yes", "hazmat:B" to "no", "hazmat:D" to "no"),
        "noHazmat" to arrayOf("highway" to "secondary", "hazmat" to "no"),
        "water" to arrayOf("highway" to "secondary", "hazmat:water" to "no"),
    )

    private fun graph(encoding: GraphEncoding): Pair<BaseGraph, Map<String, Int>> {
        val g = BaseGraph(GraphDirectory(Files.createTempDirectory("nme-hazmat").toFile().also { it.deleteOnExit() }), encoding.registry)
        val ids = LinkedHashMap<String, Int>()
        for ((i, r) in roads.withIndex()) {
            val a = g.addNode(45.0 + i * 0.01, 9.0); val b = g.addNode(45.0 + i * 0.01, 9.01)
            val e = g.addEdge(a, b, 780.0)
            val f = encoding.registry.newFlags()
            encoding.shared.write(f, WayAttributeParser.parse(way(*r.second), isFerry = false))
            g.setFlags(e, f)
            ids[r.first] = e
        }
        return g to ids
    }

    /** Le strade che il camion con [p] può percorrere. */
    private fun open(encoding: GraphEncoding, p: VehicleParameters?): Set<String> {
        val (g, ids) = graph(encoding)
        g.use {
            val f = HeavyVehicleEdgeFilter(encoding.shared, HeavyVehicleAttributes.HGV, p)
            val it = g.createEdgeIterator()
            return ids.filter { (_, e) -> f.accept(it.setEdge(e, g.nodeA(e))) }.keys
        }
    }

    private val enc = GraphEncoding(listOf(HgvProfile()))
    private val all = roads.map { it.first }.toSet()

    @Test
    fun `tunnel category of a way is the most restrictive tag`() {
        assertEquals(3, WayAttributeParser.adrTunnel(way("hazmat:C" to "no")))
        assertEquals(5, WayAttributeParser.adrTunnel(way("hazmat:adr_tunnel_category" to "E")))
        assertEquals(4, WayAttributeParser.adrTunnel(way("hazmat:B" to "no", "hazmat:D" to "no")))
        assertEquals(0, WayAttributeParser.adrTunnel(way("hazmat:adr_tunnel_category" to "A")))
        assertEquals(0, WayAttributeParser.adrTunnel(way("highway" to "primary")))
    }

    @Test
    fun `no dangerous goods, no hazmat rule`() {
        assertEquals(all, open(enc, null))
        assertEquals(all, open(enc, VehicleParameters(height = 4.0)))
    }

    @Test
    fun `without a tunnel code every ADR tunnel is closed, as in ORS`() {
        assertEquals(setOf("plain", "water"), open(enc, VehicleParameters(hazmat = true)))
    }

    @Test
    fun `tunnel code against tunnel category`() {
        // B is the strictest code: no ADR tunnel at all
        assertEquals(setOf("plain", "water"), open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 2)))
        // C: tunnels C, D, E closed
        assertEquals(setOf("plain", "water"), open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 3)))
        // D: the C tunnel opens, D (B+D) and E stay closed
        assertEquals(setOf("plain", "water", "tunnelC"), open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 4)))
        // E: only the E tunnel is closed
        assertEquals(setOf("plain", "water", "tunnelC", "tunnelBandD"), open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 5)))
    }

    @Test
    fun `hazmat=no closes the road to every code, hazmat water=no only to water polluting goods`() {
        assertFalse("noHazmat" in open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 5)))
        assertFalse("water" in open(enc, VehicleParameters(hazmat = true, adrTunnelCode = 5, hazmatWater = true)))
        // only water polluting goods, no tunnel code: tunnels open, water protection areas and hazmat=no closed
        assertEquals(setOf("plain", "tunnelC", "tunnelCatE", "tunnelBandD"), open(enc, VehicleParameters(hazmatWater = true)))
    }

    @Test
    fun `graphs built before the ADR values open and keep the old rule`() {
        val old = GraphEncoding(listOf(HgvProfile()), withAdr = false)
        assertFalse(old.shared.withAdr)
        assertTrue(old.registry.describe() != enc.registry.describe())
        assertFalse(GraphEncoding.forExisting(listOf(HgvProfile()), old.registry.describe()).shared.withAdr)
        assertTrue(GraphEncoding.forExisting(listOf(HgvProfile()), enc.registry.describe()).shared.withAdr)
        // the old bits stay where they were: same description up to the ADR values
        assertTrue(enc.registry.describe().startsWith(old.registry.describe()))
        // old graph: hazmat_forbidden only (hazmat=no, hazmat:X=no); the category-only tunnel and water are unknown
        assertEquals(setOf("plain", "water", "tunnelCatE"), open(old, VehicleParameters(hazmat = true)))
        assertEquals(setOf("plain", "water", "tunnelCatE"), open(old, VehicleParameters(hazmat = true, adrTunnelCode = 5)))
        assertEquals(all, open(old, VehicleParameters(hazmatWater = true)))
    }

    @Test
    fun `request tunnel codes`() {
        assertEquals(0, DirectionsRequest.adrTunnelCode(null))
        assertEquals(0, DirectionsRequest.adrTunnelCode("none"))
        assertEquals(0, DirectionsRequest.adrTunnelCode("A"))
        assertEquals(2, DirectionsRequest.adrTunnelCode("B"))
        assertEquals(3, DirectionsRequest.adrTunnelCode("(C/D)"))
        assertEquals(2, DirectionsRequest.adrTunnelCode("B1000C"))
        assertEquals(5, DirectionsRequest.adrTunnelCode("e"))
    }
}
