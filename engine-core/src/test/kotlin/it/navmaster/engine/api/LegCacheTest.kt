package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.PointList
import java.io.File
import java.nio.file.Files
import java.util.Random
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertSame
import kotlin.test.assertTrue

/** Heidelberg (camion, core largo, 4 landmark) caricato una volta per le prove di cache, pass_through e tratti in parallelo. */
internal object Heidelberg {
    val graph: RoutingGraph by lazy {
        val dir = Files.createTempDirectory("nme-legs").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile())).first.use { it.prepareCH(landmarkCount = 4) }
        RoutingGraph.load(dir, mmap = false)
    }

    /** [n] punti raggiungibili fra loro (lat, lon) a caso con il seme [seed], lontani almeno 500 m fra loro. */
    fun points(n: Int, seed: Long): List<Pair<Double, Double>> {
        val g = graph
        val r = g.router(RouteOptions(profile = "driving-hgv"))
        val rnd = Random(seed)
        val out = ArrayList<Pair<Double, Double>>()
        val bg = g.graph
        while (out.size < n) {
            val k = rnd.nextInt(bg.nodeCount)
            val p = bg.lat(k) + (rnd.nextDouble() - 0.5) * 0.002 to bg.lon(k) + (rnd.nextDouble() - 0.5) * 0.002
            try { r.snap(p.first, p.second, 400.0, 0) } catch (e: PointNotFoundException) { continue }
            if (out.any { o -> it.navmaster.engine.util.DistanceCalc.haversine(o.first, o.second, p.first, p.second) < 500 }) continue
            // raggiungibili fra loro: da e verso il primo
            if (out.isNotEmpty()) {
                val s0 = r.snap(out[0].first, out[0].second, 400.0, 0); val s = r.snap(p.first, p.second, 400.0, 1)
                try { r.route(s0, s); r.route(s, s0) } catch (e: it.navmaster.engine.routing.RouteNotFoundException) { continue }
            }
            out += p
        }
        return out
    }

    fun body(points: List<Pair<Double, Double>>, extra: String = ""): Map<String, Any?> {
        @Suppress("UNCHECKED_CAST")
        return Json.parse("""{"coordinates":[${points.joinToString(",") { "[${it.second},${it.first}]" }}]$extra}""") as Map<String, Any?>
    }

    fun features(doc: Map<String, Any?>): String = Json.stringify(doc["features"])
}

/**
 * La cache dei tratti e degli agganci ([LegCache], [SnapCache]): con la cache le risposte sono
 * quelle senza; una tappa aggiunta ricalcola solo i tratti nuovi; i poligoni rianalizzati danno lo
 * stesso colpo; l'esclusione per archi; i percorsi in cache non cambiano con la risposta.
 */
class LegCacheTest {
    private val g get() = Heidelberg.graph

    @Test
    fun `answers with the cache are those without`() {
        val service = DirectionsService(g)
        val pool = Heidelberg.points(8, 42)
        val rnd = Random(42)
        var hits = 0
        for (k in 0 until 40) {
            val n = 3 + rnd.nextInt(3)
            val pts = List(n) { pool[rnd.nextInt(pool.size)] }.let { l -> l.filterIndexed { i, p -> i == 0 || p != l[i - 1] } }
            if (pts.size < 2) continue
            val extra = StringBuilder()
            when (rnd.nextInt(3)) {
                1 -> extra.append(""","continue_straight":true""")
                2 -> extra.append(""","navmaster":{"pass_through":[${pts.indices.joinToString(",") { i -> (i > 0 && i < pts.size - 1 && rnd.nextBoolean()).toString() }}]}""")
            }
            if (rnd.nextBoolean()) extra.append(""","bearings":[${pts.indices.joinToString(",") { if (rnd.nextInt(3) == 0) "[${rnd.nextInt(360)},90]" else "[]" }}]""")
            val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, extra.toString()))
            val plain = try { Heidelberg.features(service.handle(request, ResponseFormat.GEOJSON, RequestContext(useCache = false))) } catch (e: ApiException) { "errore ${e.code} ${e.message}" }
            val rc = RequestContext()
            val cached = try { Heidelberg.features(service.handle(request, ResponseFormat.GEOJSON, rc)) } catch (e: ApiException) { "errore ${e.code} ${e.message}" }
            assertEquals(plain, cached, "richiesta $k: $extra")
            hits += rc.stats.legs.count { it.source == LegStats.Source.CACHE }
        }
        println("cache dei tratti: ${service.legCacheInfo}, $hits tratti dalla cache")
        assertTrue(hits > 0, "nessun tratto dalla cache")
    }

    @Test
    fun `a stop added before a stop - only the new legs are searched`() {
        val service = DirectionsService(g)
        val (a, b, c) = Heidelberg.points(3, 7)
        val v = Heidelberg.points(1, 8)[0]
        service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(a, b, c))), ResponseFormat.GEOJSON)
        // V «passa per di qui», B resta una sosta: il tratto B → C non dipende dall'arrivo in B
        val rc = RequestContext()
        val withVia = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(a, v, b, c), ""","navmaster":{"pass_through":[false,true,false,false]}""")), ResponseFormat.GEOJSON, rc)
        assertEquals(listOf(LegStats.Source.CALC, LegStats.Source.CALC, LegStats.Source.CACHE), rc.stats.legs.map { it.source }, rc.stats.line())
        assertEquals(3, rc.stats.snapsCached, rc.stats.line())
        val fresh = DirectionsService(g).handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(a, v, b, c), ""","navmaster":{"pass_through":[false,true,false,false]}""")), ResponseFormat.GEOJSON)
        assertEquals(Heidelberg.features(fresh), Heidelberg.features(withVia))
    }

    @Test
    fun `a via added before via points - the dependent legs after the new ones come from the cache`() {
        // vie «passa per di qui» dentro archi a doppio senso lungo il migliore A → C (il tratto dopo dipende dal verso d'arrivo)
        val router = g.router(RouteOptions(profile = "driving-hgv"))
        var a = 0.0 to 0.0; var c = 0.0 to 0.0
        var two: List<Pair<Double, Double>> = emptyList()
        for (seed in 11L..40L) {
            val (p, q) = Heidelberg.points(2, seed)
            @Suppress("UNCHECKED_CAST")
            val geo = ((DirectionsService(g).handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(p, q))), ResponseFormat.GEOJSON)["features"] as List<Map<String, Any?>>)[0]
                ["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            // a metà fra due punti della geometria: dentro un arco (un nodo non lega il verso d'arrivo)
            two = (1 until geo.size - 2).map { k -> (geo[k][1] + geo[k + 1][1]) / 2 to (geo[k][0] + geo[k + 1][0]) / 2 }
                .filter { m -> try { !router.directionFree(router.snap(m.first, m.second, 400.0, 0)) } catch (e: PointNotFoundException) { false } }
            if (two.size >= 8) { a = p; c = q; break }
        }
        assertTrue(two.size >= 8, "nessun viaggio con vie su archi a doppio senso")
        fun at(f: Double) = two[(f * (two.size - 1)).toInt()]
        // tre vie, poi una quarta fra A e la prima
        val vias = listOf(at(0.3), at(0.5), at(0.75))
        val pts = listOf(a) + vias + listOf(c)
        val pts2 = listOf(a, at(0.1)) + vias + listOf(c)
        fun pass(n: Int) = ""","navmaster":{"pass_through":[${(0 until n).joinToString(",") { (it in 1 until n - 1).toString() }}]}"""
        val fresh = Heidelberg.features(DirectionsService(g).handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts2, pass(pts2.size))), ResponseFormat.GEOJSON))
        // un thread solo e due thread (l'aiutante)
        for (parallel in listOf(false, true)) {
            val service = DirectionsService(g)
            val pool = Executors.newFixedThreadPool(1)
            try {
                fun rc() = if (parallel) RequestContext(executor = pool, slots = SearchSlots(2)) else RequestContext()
                service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, pass(pts.size))), ResponseFormat.GEOJSON, rc())
                val r = rc()
                val again = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts2, pass(pts2.size))), ResponseFormat.GEOJSON, r)
                val calc = r.stats.legs.count { it.source != LegStats.Source.CACHE }
                println("via in più, due thread $parallel: ${r.stats.line()}")
                assertTrue(calc <= 2, "due thread $parallel: $calc tratti cercati: ${r.stats.line()}")
                assertEquals(fresh, Heidelberg.features(again), "due thread $parallel")
            } finally { pool.shutdown(); pool.awaitTermination(10, TimeUnit.SECONDS) }
        }
    }

    @Test
    fun `polygons parsed again hit the same entries`() {
        val service = DirectionsService(g)
        val (a, b) = Heidelberg.points(2, 9)
        // un poligono lontano dal percorso: la ricerca piena, la stessa chiave dalle coordinate
        val poly = ""","options":{"avoid_polygons":{"type":"Polygon","coordinates":[[[8.60,49.36],[8.61,49.36],[8.61,49.37],[8.60,49.37],[8.60,49.36]]]}}"""
        val first = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(a, b), poly)), ResponseFormat.GEOJSON)
        val rc = RequestContext()
        val again = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(a, b), poly)), ResponseFormat.GEOJSON, rc)
        assertEquals(listOf(LegStats.Source.CACHE), rc.stats.legs.map { it.source }, rc.stats.line())
        assertEquals(2, rc.stats.snapsCached)
        assertEquals(Heidelberg.features(first), Heidelberg.features(again))
    }

    private fun path(edges: Int): RoutePath = RoutePath(List(edges) { PathEdge(it, false, 0, 1, PointList(), 1.0, 1L, 1.0, "") }, edges.toDouble(), edges.toLong(), edges.toDouble())

    private fun key(i: Int) = LegKey(RouteOptions(), SnapKey(i, 0, false, 0, 0L, 0L), SnapKey(i + 1, 0, false, 0, 0L, 0L), -1, 0L, 0L)

    @Test
    fun `eviction by edges and entries, least recently used first`() {
        val cache = LegCache(maxEdges = 500, maxEntries = 256)
        for (i in 0 until 5) cache.put(key(i), path(100))
        assertEquals(5, cache.size)
        // il tratto 0 usato di recente: esce l'1
        cache.get(key(0))
        cache.put(key(5), path(100))
        assertEquals(5, cache.size)
        assertTrue(cache.get(key(0)) != null && cache.get(key(1)) == null && cache.get(key(5)) != null)
        // uno grande fa uscire i più vecchi finché ci sta
        cache.put(key(6), path(450))
        assertEquals(1, cache.size)
        // più grande del tetto: non entra
        cache.put(key(7), path(501))
        assertTrue(cache.get(key(7)) == null && cache.get(key(6)) != null)
        val few = LegCache(maxEdges = 500, maxEntries = 3)
        for (i in 0 until 5) few.put(key(i), path(1))
        assertEquals(3, few.size)
        assertTrue(few.get(key(1)) == null && few.get(key(2)) != null)
        println(few.info)
    }

    @Test
    fun `the caches of a process share one edge budget`() {
        // due servizi (due Paesi): oltre il tetto comune esce il tratto meno usato di recente, di qualunque cache
        val budget = LegBudget(maxEdges = 500)
        val a = LegCache(budget = budget); val b = LegCache(budget = budget)
        for (i in 0 until 3) a.put(key(i), path(100))
        b.put(key(0), path(100))
        a.get(key(0))
        b.put(key(1), path(100))
        assertEquals(5, a.size + b.size)
        // il sesto fa uscire il tratto 1 di a (il 0 di a usato di recente, poi il 0 di b)
        b.put(key(2), path(100))
        assertTrue(a.get(key(1)) == null && a.get(key(0)) != null && a.get(key(2)) != null && b.get(key(0)) != null, "${a.info} / ${b.info}")
        // uno grande fa uscire i più vecchi di tutte, mai sé stesso
        b.put(key(3), path(400))
        assertTrue(b.get(key(3)) != null)
        assertEquals(2, a.size + b.size, "${a.info} / ${b.info}")
        // più grande del tetto comune: non entra
        a.put(key(9), path(501))
        assertTrue(a.get(key(9)) == null)
    }

    @Test
    fun `cached geometry stays intact after building and merging answers`() {
        val service = DirectionsService(g)
        // lo stesso grafo nei panni di due Paesi, valichi finti (come CancelTokenTest): i pezzi si uniscono con merge
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(11)
        val crossings = ArrayList<Transfer>()
        for (attempt in 0 until 4000) {
            if (crossings.size >= 6) break
            val n0 = rnd.nextInt(bg.nodeCount)
            val n = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
            if (crossings.any { t -> it.navmaster.engine.util.DistanceCalc.haversine(t.lat, t.lon, bg.lat(n), bg.lon(n)) < 3_000.0 }) continue
            crossings += Transfer(bg.lat(n), bg.lon(n), 1 + crossings.size % 3)
        }
        val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> crossings })
        val request = DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.6757,49.4036],[8.7606,49.4186]],"navmaster":{"roads":true}}""") as Map<String, Any?>)
        val countries = listOf("a", "a", "b")
        val want = Heidelberg.features(cb.handle(request, countries))
        // i tratti dei pezzi, dalla cache (gli stessi oggetti della risposta), e la loro geometria
        val legs = cb.lastPieces.flatMap { (cg, r) -> cg.service.solve(r).legs }
        fun copy(p: RoutePath) = p.edges.map { e -> List(e.geometry.size) { i -> e.geometry.lat(i) to e.geometry.lon(i) } }
        val before = legs.map { copy(it) }
        val rc = RequestContext()
        assertEquals(want, Heidelberg.features(cb.handle(request, countries, rc)))
        assertTrue(rc.stats.line().contains(" cache "), rc.stats.line())
        assertEquals(before, legs.map { copy(it) })
        // e ancora gli stessi oggetti in cache
        val again = cb.lastPieces.flatMap { (cg, r) -> cg.service.solve(r).legs }
        for ((x, y) in legs.zip(again)) assertSame(x, y)
        // svuotata la cache, la stessa risposta
        service.clearCaches()
        assertEquals(0, service.legCacheSize)
        assertEquals(want, Heidelberg.features(cb.handle(request, countries)))
    }
}
