package it.navmaster.engine.routing

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * La ricerca piena edge-based (`dijkstra_edge_based`: avoid_features, avoid_polygons,
 * maximum_speed sui viaggi brevi) confrontata con il core (`core_alt_edge_based`) su Heidelberg,
 * con le restrizioni di un camion: dove il core trova un percorso, anche la ricerca piena.
 */
class EdgeBasedVsCoreTest {
    @Test
    fun `dijkstra edge-based finds a path wherever the core does, mid-edge points included`() {
        val dir = Files.createTempDirectory("nme-ebcore").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        built.use { it.prepareCH() }
        RoutingGraph.load(dir, mmap = false).use { rg ->
            val g = rg.graph
            val params = VehicleParameters(height = 4.0, weight = 12.0)
            val core = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            assertEquals("core_alt_edge_based", core.algorithm)
            val eb = rg.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false))
            assertEquals("dijkstra_edge_based", eb.algorithm)
            val tc = TurnCostProvider(rg.turnCosts, "heavyvehicle")

            // il caso trovato: partenza a metà arco; per andare verso la destinazione il camion deve
            // tornare indietro, ma la svolta è vietata e l'inversione pure: gira intorno a un isolato
            // e ripercorre la stessa strada nel verso opposto. Prima la ricerca piena lo scartava
            // come un anello e dava «nessun percorso» (il core ne trova uno, più lungo).
            val from = eb.snap(49.4066038, 8.6587631, 400.0, 0)
            val to = eb.snap(49.4049966, 8.6696132, 400.0, 1)
            assertNotEquals(SnapPosition.TOWER, from.position)
            val p = eb.route(from, to)
            checkPath(p, tc, eb)
            assertTrue(p.edges.any { e -> p.edges.any { it.edge == e.edge && it.reverse != e.reverse } }, "il percorso dovrebbe ripercorrere una strada nel verso opposto")
            val c = core.route(from, to)
            checkPath(c, tc, core)
            assertTrue(p.weight <= c.weight + 0.05 + c.weight * 1e-4, "ricerca piena ${p.weight} vs core ${c.weight}")

            // coppie a caso, con punti su torri e a metà arco
            val rnd = Random(23)
            var pairs = 0; var midEdge = 0; var found = 0; var onlyFull = 0; var lighter = 0; var repeated = 0
            repeat(1000) {
                val a = randomSnap(g, eb, rnd, 0) ?: return@repeat
                val b = randomSnap(g, eb, rnd, 1) ?: return@repeat
                pairs++
                if (a.position != SnapPosition.TOWER) midEdge++
                if (b.position != SnapPosition.TOWER) midEdge++
                val label = "da (%.7f, %.7f) a (%.7f, %.7f)".format(a.queryLat, a.queryLon, b.queryLat, b.queryLon)
                val cp = try { core.route(a, b) } catch (e: RouteNotFoundException) { null }
                val ep = try { eb.route(a, b) } catch (e: RouteNotFoundException) { null }
                if (cp != null) assertNotNull(ep, "$label: nessun percorso con la ricerca piena, il core ne trova uno di peso ${cp.weight}")
                cp?.let { checkPath(it, tc, core, label) }
                if (ep == null) return@repeat
                found++
                checkPath(ep, tc, eb, label)
                val rep = repeatsEdge(ep)
                if (rep) repeated++
                if (cp == null) { onlyFull++; return@repeat }
                val tol = 0.05 + cp.weight * 1e-4
                // la ricerca piena è esatta: mai più pesante del core
                assertTrue(ep.weight <= cp.weight + tol, "$label: ricerca piena ${ep.weight} vs core ${cp.weight}")
                // più leggera solo quando gira intorno a un isolato e ripassa su un arco: un giro che
                // torna allo stesso nodo il core non lo rappresenta (nessuno shortcut da un nodo a se stesso)
                if (ep.weight < cp.weight - tol) {
                    lighter++
                    assertTrue(rep, "$label: ricerca piena ${ep.weight} più leggera del core ${cp.weight} senza ripassare su un arco")
                }
            }
            println("ricerca piena vs core: $pairs coppie ($midEdge punti a metà arco), $found percorsi, $repeated ripassano su un arco, $lighter più leggeri del core, $onlyFull trovati solo dalla ricerca piena")
            assertTrue(midEdge > pairs, "la maggior parte dei punti dovrebbe stare a metà arco")
            assertTrue(repeated > 0, "fra le coppie dovrebbero esserci percorsi che ripassano su un arco")
        }
    }

    /**
     * Il percorso è continuo e ogni svolta è consentita (inversioni a U comprese), anche fra il
     * tratto agganciato e la sua torre: la ricerca riceve l'arco agganciato, come il tratto
     * virtuale di GraphHopper. Lo stesso controllo di [Router.isLegal] (usato dalle alternative).
     */
    private fun checkPath(p: RoutePath, tc: TurnCostProvider, router: Router, label: String = "") {
        for (k in 1 until p.edges.size) assertEquals(p.edges[k - 1].adjNode, p.edges[k].baseNode, "$label: percorso spezzato all'arco $k")
        for (k in 1 until p.edges.size) {
            val prev = p.edges[k - 1]; val cur = p.edges[k]
            assertTrue(tc.turnWeight(prev.edge, prev.adjNode, cur.edge).isFinite(), "$label (${router.algorithm}): svolta vietata da ${prev.edge} a ${cur.edge} nel nodo ${prev.adjNode}, arco $k di ${p.edges.size}")
        }
        assertTrue(router.isLegal(p), "$label (${router.algorithm}): percorso con una svolta vietata")
    }

    private fun repeatsEdge(p: RoutePath): Boolean {
        val seen = HashSet<Int>()
        return p.edges.any { !seen.add(it.edge) }
    }

    /** Un punto a caso, agganciato come in una richiesta: una torre o un punto a metà di un arco. */
    private fun randomSnap(g: BaseGraph, router: Router, rnd: Random, index: Int): Snap? {
        val edge = rnd.nextInt(g.edgeCount)
        val geo = g.fetchWayGeometry(edge, false, FetchMode.ALL)
        val (lat, lon) = if (rnd.nextInt(4) == 0 || geo.size < 2) {
            val n = if (rnd.nextBoolean()) g.nodeA(edge) else g.nodeB(edge)
            g.lat(n) to g.lon(n)
        } else {
            val i = rnd.nextInt(geo.size - 1); val t = 0.05 + 0.9 * rnd.nextDouble()
            (geo.lat(i) + (geo.lat(i + 1) - geo.lat(i)) * t) to (geo.lon(i) + (geo.lon(i + 1) - geo.lon(i)) * t)
        }
        return try { router.snap(lat, lon, 400.0, index) } catch (e: PointNotFoundException) { null }
    }
}
