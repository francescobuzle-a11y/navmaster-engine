package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * Le misure di una richiesta ([RequestStats]): la riga `stats` ha i tratti con algoritmo e ms, e non
 * comincia con nessuno dei prefissi che RouteProgress (app) interpreta. Le misure non cambiano la risposta.
 */
class RequestStatsTest {
    /** I prefissi delle righe del motore che RouteProgress interpreta (RouteProgress.kt). */
    private val progressPrefixes = listOf("alternativa ", "alternative dei viaggi lunghi:", "sequenze ", "valico scelto", "viaggio tra Paesi:")

    @Test
    fun `stats line has legs, algorithm and times`() {
        val dir = Files.createTempDirectory("nme-stats").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile())).first.use { it.prepareCH(landmarkCount = 4) }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val service = DirectionsService(g)
            val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.6757,49.4036],[8.7606,49.4186],[8.6900,49.4000]],"navmaster":{"roads":true}}""") as Map<String, Any?>
            val request = DirectionsRequest.fromJson("driving-hgv", body)
            val plain = service.handle(request, ResponseFormat.GEOJSON)
            // senza la cache dei tratti: la stessa richiesta li riprenderebbe tutti da lì
            val rc = RequestContext(useCache = false)
            val measured = service.handle(request, ResponseFormat.GEOJSON, rc)
            assertEquals(Json.stringify(plain["features"]), Json.stringify(measured["features"]))

            val s = rc.stats
            assertEquals(3, s.legs.size)
            assertEquals(listOf(0, 1, 2), s.legs.map { it.index })
            assertTrue(s.legs.all { it.source == LegStats.Source.CALC && it.algorithm == "core_alt_edge_based" && it.edges > 0 && it.meters > 0 })
            assertTrue(s.legs.all { it.search.contains("fase1") }, "le fasi della ricerca core: ${s.legs.map { it.search }}")
            assertEquals(4, s.snapsNew); assertEquals(0, s.snapsCached)
            assertEquals(4, s.points); assertEquals("driving-hgv", s.profile)
            assertTrue(s.totalMs >= s.buildMs && s.buildMs >= s.instructionsMs)
            val line = s.line()
            println(line)
            assertTrue(line.startsWith("stats hgv 4 punti: aggancio "), line)
            assertTrue(line.contains(" | tratti ") && line.contains("core_alt_edge_based") && line.contains(" ms") && line.contains(" | risposta "), line)
            for (p in progressPrefixes) assertTrue(!line.startsWith(p), "la riga non deve sembrare una riga di RouteProgress: $p")

            // le parti (un pezzo di un viaggio tra Paesi) nella riga del genitore
            val part = rc.child("de")
            service.handle(DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""") as Map<String, Any?>), ResponseFormat.JSON, part)
            assertEquals(1, part.stats.legs.size)
            assertTrue(s.line().contains("[de: hgv 2 punti: "), s.line())
            assertEquals(3, s.legs.size)

            // le alternative ORS: un tratto con le alternative trovate
            val altRc = RequestContext()
            service.handle(DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"alternative_routes":{"target_count":2}}""") as Map<String, Any?>), ResponseFormat.GEOJSON, altRc)
            assertEquals(1, altRc.stats.legs.size)
            assertTrue(altRc.stats.legs[0].algorithm.contains("alternative"))
        }
    }

    @Test
    fun `quick search - the leg names the search that made it`() {
        // col core ridotto «evita autostrade» prova prima la ricerca rapida sul core senza il filtro
        val dir = Files.createTempDirectory("nme-stats-quick").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile())).first.use { it.prepareCH(landmarkCount = 4, coreWide = false) }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val r = g.router(RouteOptions(profile = "driving-hgv", avoidFeatures = AvoidFeatureFlags.HIGHWAYS))
            assertEquals("dijkstra_edge_based", r.algorithm)
            val bg = g.graph
            val rnd = Random(5)
            var taken = 0; var discarded: Pair<Pair<Double, Double>, Pair<Double, Double>>? = null
            for (k in 0 until 400) {
                if (taken > 0 && discarded != null) break
                val a = rnd.nextInt(bg.nodeCount); val b = rnd.nextInt(bg.nodeCount)
                val (sa, sb) = try { r.snap(bg.lat(a), bg.lon(a), 400.0, 0) to r.snap(bg.lat(b), bg.lon(b), 400.0, 1) } catch (e: PointNotFoundException) { continue }
                try { r.route(sa, sb) } catch (e: RouteNotFoundException) { continue }
                if (r.lastAlgorithm.startsWith("rapida")) {
                    assertEquals("rapida core_alt_edge_based", r.lastAlgorithm)
                    assertTrue(r.lastSearch.contains("fase1") && !r.lastSearch.contains("scartata"), r.lastSearch)
                    taken++
                } else {
                    assertEquals("dijkstra_edge_based", r.lastAlgorithm)
                    assertTrue(Regex("""\d+ nodi; rapida core_alt_edge_based scartata: .*fase1.*""").matches(r.lastSearch), r.lastSearch)
                    if (discarded == null) discarded = (sa.queryLat to sa.queryLon) to (sb.queryLat to sb.queryLon)
                }
            }
            assertTrue(taken > 0, "nessun percorso dalla ricerca rapida")
            val (p, q) = assertNotNull(discarded, "nessuna ricerca rapida scartata")
            // nella riga: il tratto con la ricerca piena e i suoi nodi, la rapida come scartata
            val rc = RequestContext()
            DirectionsService(g).handle(DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[${p.second},${p.first}],[${q.second},${q.first}]],"options":{"avoid_features":["highways"]}}""") as Map<String, Any?>), ResponseFormat.JSON, rc)
            val leg = rc.stats.legs.single()
            assertEquals("dijkstra_edge_based", leg.algorithm)
            assertTrue(leg.search.contains(" nodi; rapida core_alt_edge_based scartata: "), leg.search)
            println(rc.stats.line())
        }
    }

    @Test
    fun `an attempt set aside stays apart in the line`() {
        val s = RequestStats()
        s.profile = "driving-hgv"; s.points = 3; s.snapsNew = 3; s.snapMs = 5; s.searchWallMs = 40; s.totalMs = 50
        s.legs += LegStats(0, LegStats.Source.CALC, 20, "core_alt_edge_based", "", 10, 1000.0, "")
        s.legs += LegStats(1, LegStats.Source.CALC, 20, "core_alt_edge_based", "", 10, 1000.0, "")
        s.setAside("divieto scartato")
        assertTrue(s.legs.isEmpty()); assertEquals(0, s.snapsNew); assertEquals(0L, s.totalMs)
        s.snapsNew = 3; s.totalMs = 90
        s.legs += LegStats(0, LegStats.Source.CALC, 30, "core_alt_edge_based", "", 10, 1000.0, "")
        s.addCrossings("a → b: 4 ricerche in 10 ms"); s.addCrossings("x")
        val line = s.line()
        println(line)
        assertTrue(line.startsWith("stats hgv 3 punti: aggancio 0 ms (3 nuovi, 0 in cache) | tratti 0 ms: 0 calc core_alt_edge_based 30 ms"), line)
        assertTrue(line.contains(" | valichi a → b: 4 ricerche in 10 ms; x | totale 90 ms"), line)
        assertTrue(line.contains("[divieto scartato: hgv 3 punti: aggancio 5 ms (3 nuovi, 0 in cache) | tratti 40 ms: 0 calc"), line)
        assertTrue(line.contains("1 calc core_alt_edge_based 20 ms") && line.contains("totale 50 ms]"), line)
    }
}
