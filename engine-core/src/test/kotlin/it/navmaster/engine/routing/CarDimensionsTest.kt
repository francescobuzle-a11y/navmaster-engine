package it.navmaster.engine.routing

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.ch.CoreEdgeSelector
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.GraphProperties
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * L'auto con dimensioni (camper, furgoni): i limiti fisici delle strade valgono, i divieti per i
 * mezzi pesanti no; il core dell'auto (edizione 2) dà gli stessi pesi della ricerca piena; un grafo
 * con il core di prima manda queste richieste alla ricerca piena.
 */
class CarDimensionsTest {
    @Test
    fun `car with dimensions avoids limited roads, exactly, without hgv bans`() {
        val dir = Files.createTempDirectory("nme-cardim").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { it.prepareCH() }
        RoutingGraph.load(dir, mmap = false).use { g ->
            assertEquals(CoreEdgeSelector.CORE_VERSION, g.coreVersion)
            val bg = g.graph; val sh = g.encoding.shared
            val it = bg.createEdgeIterator()
            // gli archi con un limite fisico, e quelli vietati ai camion ma senza limiti (dove l'auto con dimensioni deve passare)
            val limited = ArrayList<Int>(); val hgvOnly = ArrayList<Int>()
            for (e in 0 until bg.edgeCount) {
                it.setEdge(e, bg.nodeA(e))
                val lim = it.get(sh.maxHeight).isFinite() || it.get(sh.maxWeight).isFinite() || it.get(sh.maxWidth).isFinite()
                if (lim) limited += e
                if (!lim && it.get(sh.hgvBlocked) != 0) hgvOnly += e
            }
            println("archi con limiti ${limited.size}, vietati ai camion senza limiti ${hgvOnly.size}")
            assertTrue(limited.size > 20 && hgvOnly.size > 20)
            val camper = VehicleParameters(height = 3.2, width = 2.3, weight = 3.5)
            val big = VehicleParameters(height = 4.0, width = 2.55, weight = 12.0)
            val plain = g.router(RouteOptions(profile = "driving-car", turnCosts = false))
            val coreCar = g.router(RouteOptions(profile = "driving-car", vehicleParameters = big))
            val fullCar = g.router(RouteOptions(profile = "driving-car", vehicleParameters = big, allowCH = false))
            assertEquals("core_alt_edge_based", coreCar.algorithm); assertEquals("dijkstra_edge_based", fullCar.algorithm)
            val st = g.chStorages.getValue("car-ors_fastest_core")
            println("core dell'auto: ${st.coreNodeCount} nodi, ${st.shortcutCount} shortcut")
            val rnd = Random(3)
            var equal = 0; var differentFromPlain = 0; var loops = 0
            repeat(80) {
                val a = rnd.nextInt(bg.nodeCount); val b = rnd.nextInt(bg.nodeCount)
                var unsnapped = false
                fun p(r: Router) = try { r.route(r.snap(bg.lat(a), bg.lon(a), 400.0, 0), r.snap(bg.lat(b), bg.lon(b), 400.0, 1)) } catch (e: RouteNotFoundException) { null } catch (e: PointNotFoundException) { unsnapped = true; null }
                val pc = p(coreCar); val pf = p(fullCar); val pp = p(plain)
                if (unsnapped) return@repeat
                val wc = pc?.weight ?: Double.POSITIVE_INFINITY; val wf = pf?.weight ?: Double.POSITIVE_INFINITY; val wp = pp?.weight ?: Double.POSITIVE_INFINITY
                if ((wc.isInfinite() && wf.isInfinite()) || abs(wc - wf) <= 0.05 + wf * 1e-5) equal++
                else {
                    // il giro attorno a un isolato fuori dal core (inversioni vietate): limite noto del core, come in ORS
                    val ns = pf!!.edges.map { it.baseNode }
                    val loop = wf < wc && fullCar.isLegal(pf) && ns.groupBy { it }.filter { it.value.size > 1 }.keys.any { r -> ns.subList(ns.indexOf(r), ns.lastIndexOf(r) + 1).any { st.level(it) < bg.nodeCount } }
                    assertTrue(loop, "$a→$b: core $wc, ricerca piena $wf")
                    loops++
                }
                if (wp.isFinite() && wc > wp + 1) differentFromPlain++
            }
            println("auto 4 m / 12 t: $equal viaggi uguali fra core e ricerca piena, $loops col giro della ricerca piena, $differentFromPlain più lunghi che senza misure")
            assertTrue(equal > 60 && loops <= 3)
            // il camper passa su una strada vietata ai camion (hgv=no) ma senza limiti: sullo stesso arco, come l'auto
            var passes = 0
            for (e in hgvOnly.take(40)) {
                val r = g.router(RouteOptions(profile = "driving-car", vehicleParameters = camper))
                val p = try { r.route(r.snap(bg.lat(bg.nodeA(e)), bg.lon(bg.nodeA(e)), 50.0, 0), r.snap(bg.lat(bg.nodeB(e)), bg.lon(bg.nodeB(e)), 50.0, 1)) }
                    catch (ex: RouteNotFoundException) { continue } catch (ex: PointNotFoundException) { continue }
                if (p.edges.any { it.edge == e }) passes++
            }
            assertTrue(passes > 0, "il camper non passa mai su strade vietate ai soli camion")
            // una strada con limite d'altezza sotto il camper: il camper non ci passa, l'auto senza misure sì
            var avoided = 0; var tried = 0
            for (e in limited) {
                it.setEdge(e, bg.nodeA(e))
                val h = it.get(sh.maxHeight); if (!(h.isFinite() && h < 3.2 && h > 0)) continue
                val r = g.router(RouteOptions(profile = "driving-car", vehicleParameters = camper))
                val p = try { r.route(r.snap(bg.lat(bg.nodeA(e)), bg.lon(bg.nodeA(e)), 50.0, 0), r.snap(bg.lat(bg.nodeB(e)), bg.lon(bg.nodeB(e)), 50.0, 1)) }
                    catch (ex: RouteNotFoundException) { null } catch (ex: PointNotFoundException) { continue }
                tried++
                if (p == null || p.edges.none { it.edge == e }) avoided++
            }
            println("sottopassi sotto 3,2 m: $tried, evitati dal camper $avoided")
            assertTrue(tried > 0 && avoided == tried)
        }
        // un grafo preparato con la regola di prima: l'auto con misure usa la ricerca piena
        GraphProperties(File(dir, "profiles")).apply { load(); set("core_version", "1"); save() }
        RoutingGraph.load(dir, mmap = false).use { g ->
            assertEquals(1, g.coreVersion)
            assertEquals("dijkstra_edge_based", g.router(RouteOptions(profile = "driving-car", vehicleParameters = VehicleParameters(height = 3.2))).algorithm)
            assertEquals("core_alt_edge_based", g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 3.2))).algorithm)
            // senza misure, con i costi di svolta: il core (la CH node-based ignora i divieti di svolta)
            assertEquals("core_alt_edge_based", g.router(RouteOptions(profile = "driving-car")).algorithm)
            assertEquals("ch", g.router(RouteOptions(profile = "driving-car", turnCosts = false)).algorithm)
        }
    }
}
