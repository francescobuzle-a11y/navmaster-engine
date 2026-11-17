package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.routing.SearchCancelledException.Reason
import it.navmaster.engine.ch.CHGraph
import it.navmaster.engine.ch.DijkstraBidirectionCH
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import java.util.Random
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertFalse
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * L'annullamento delle ricerche ([CancelToken]) su Heidelberg: ogni algoritmo si ferma con
 * [SearchCancelledException] e il motivo giusto, `handle` non lo trasforma in [ApiException], e dopo
 * un annullamento un router nuovo dà lo stesso percorso. Più i posti di ricerca ([SearchSlots]).
 */
class CancelTokenTest {
    private lateinit var dir: File
    private lateinit var g: RoutingGraph

    @BeforeTest
    fun setUp() {
        dir = Files.createTempDirectory("nme-cancel").toFile().also { it.deleteOnExit() }
        RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile())).first.use { it.prepareCH(landmarkCount = 4) }
        g = RoutingGraph.load(dir, mmap = false)
    }

    @AfterTest
    fun tearDown() { g.close() }

    private fun cancelled(why: Reason = Reason.CANCELLED) = CancelToken().also { it.cancel(why) }

    /** Due punti lontani raggiungibili con [r] (un percorso con molti nodi visitati). */
    private fun pair(r: Router): Pair<Snap, Snap> {
        val rnd = Random(42)
        val bg = g.graph
        while (true) {
            val a = rnd.nextInt(bg.nodeCount); val b = rnd.nextInt(bg.nodeCount)
            try {
                val sa = r.snap(bg.lat(a), bg.lon(a), 400.0, 0); val sb = r.snap(bg.lat(b), bg.lon(b), 400.0, 1)
                if (r.route(sa, sb).distance > 5_000) return sa to sb
            } catch (e: PointNotFoundException) { } catch (e: RouteNotFoundException) { }
        }
    }

    @Test
    fun `every search stops on a cancelled token`() {
        val cases = listOf(
            RouteOptions(profile = "driving-hgv") to "core_alt_edge_based",
            RouteOptions(profile = "driving-hgv", turnCosts = false) to "ch",
            RouteOptions(profile = "driving-hgv", allowCH = false) to "dijkstra_edge_based",
            RouteOptions(profile = "driving-hgv", allowCH = false, turnCosts = false) to "dijkstra",
        )
        for ((o, algo) in cases) {
            val r = g.router(o)
            assertEquals(algo, r.algorithm)
            val (a, b) = pair(r)
            val want = r.route(a, b)
            r.cancel = cancelled()
            assertEquals(Reason.CANCELLED, assertFailsWith<SearchCancelledException>("$algo") { r.route(a, b) }.reason)
            // scaduto: DEADLINE (anche per un figlio di un token vivo)
            r.cancel = CancelToken().child(System.nanoTime() - 1)
            assertEquals(Reason.DEADLINE, assertFailsWith<SearchCancelledException>("$algo") { r.route(a, b) }.reason)
            // dopo l'annullamento, un router nuovo: lo stesso percorso
            val again = g.router(o).route(a, b)
            assertEquals(want.weight, again.weight, 1e-9)
            assertEquals(want.edges.map { it.edge }, again.edges.map { it.edge })
        }
        // le alternative (ricerca piena coi plateau)
        val alt = g.router(RouteOptions(profile = "driving-hgv", allowCH = false))
        val (a, b) = pair(alt)
        alt.cancel = cancelled()
        assertFailsWith<SearchCancelledException> { alt.alternatives(a, b, 3, 1.4, 0.6) }
    }

    /** Conta le verifiche e si annulla alla numero [at] (0: mai): un annullamento che arriva a metà ricerca. */
    private class Counting(val at: Int = 0) : CancelToken() {
        var checks = 0
        override fun check() { if (++checks == at) cancel(); super.check() }
    }

    @Test
    fun `every search stops inside its loops`() {
        val cases = listOf(
            RouteOptions(profile = "driving-hgv") to "core_alt_edge_based",
            RouteOptions(profile = "driving-hgv", allowCH = false) to "dijkstra_edge_based",
            RouteOptions(profile = "driving-hgv", allowCH = false, turnCosts = false) to "dijkstra",
        )
        val bg = g.graph
        for ((o, algo) in cases) {
            val r = g.router(o)
            assertEquals(algo, r.algorithm)
            // una ricerca lunga abbastanza da passare dalle verifiche nel ciclo (ogni 256 passi), oltre a quella d'ingresso
            val rnd = Random(3)
            var best: Triple<Snap, Snap, Int>? = null
            for (k in 0 until 300) {
                val x = rnd.nextInt(bg.nodeCount); val y = rnd.nextInt(bg.nodeCount)
                val count = Counting()
                r.cancel = count
                try { r.route(r.snap(bg.lat(x), bg.lon(x), 400.0, 0), r.snap(bg.lat(y), bg.lon(y), 400.0, 1)) } catch (e: PointNotFoundException) { continue } catch (e: RouteNotFoundException) { continue }
                if (best == null || count.checks > best.third) best = Triple(r.snap(bg.lat(x), bg.lon(x), 400.0, 0), r.snap(bg.lat(y), bg.lon(y), 400.0, 1), count.checks)
                if (count.checks >= 4) break
            }
            val (a, b, checks) = assertNotNull(best)
            val count = Counting(); r.cancel = count; r.route(a, b)
            assertEquals(checks, count.checks)
            println("$algo: ${count.checks} verifiche (${r.lastSearch})")
            assertTrue(count.checks >= 2, "$algo: ${count.checks} verifiche")
            for (at in setOf(2, maxOf(2, count.checks / 2), count.checks)) {
                val r2 = g.router(o)
                r2.cancel = Counting(at)
                assertEquals(Reason.CANCELLED, assertFailsWith<SearchCancelledException>("$algo alla verifica $at di ${count.checks}") { r2.route(a, b) }.reason)
            }
        }
        // la CH: su Heidelberg una ricerca da punto a punto visita meno di 256 nodi; da molte partenze e senza arrivo
        // sale da tutte, fino a esaurire le ricerche
        val w = WeightingFactory.create(g.encoding, g.encoding.profile("heavyvehicle"), WeightingMethod.RECOMMENDED)
        val ch = CHGraph(bg, assertNotNull(g.chStorages["heavyvehicle_${w.name}"], "CH ${w.name}: ${g.chStorages.keys}"), w)
        val sources = Random(4).let { rnd -> List(1000) { rnd.nextInt(bg.nodeCount) } }
        val chCount = Counting()
        DijkstraBidirectionCH(ch, w, cancel = chCount).also { q -> sources.forEach { q.addSource(it, 0.0) } }.run()
        println("ch: ${chCount.checks} verifiche")
        assertTrue(chCount.checks >= 2, "ch: ${chCount.checks} verifiche")
        for (at in setOf(2, chCount.checks)) assertFailsWith<SearchCancelledException>("ch alla verifica $at di ${chCount.checks}") {
            DijkstraBidirectionCH(ch, w, cancel = Counting(at)).also { q -> sources.forEach { q.addSource(it, 0.0) } }.run()
        }

        // molti a uno: preparazione e avanzamento
        val r = g.router(RouteOptions(profile = "driving-hgv"))
        val (a, b) = pair(r)
        val (c, d) = pair(g.router(RouteOptions(profile = "driving-hgv", turnCosts = false)))
        val count = Counting()
        r.cancel = count
        val m = assertNotNull(r.manyToOne(a, listOf(b, c, d), rootIsTarget = false))
        val prepared = count.checks
        m.advance { (0..2).all { m.isFinal(it) } }
        println("molti a uno: $prepared verifiche nella preparazione, ${count.checks} in tutto")
        assertTrue(count.checks >= prepared + 2, "molti a uno: ${count.checks} verifiche, $prepared nella preparazione")
        for (at in setOf(2, maxOf(2, prepared), prepared + 2, count.checks)) {
            val r2 = g.router(RouteOptions(profile = "driving-hgv"))
            r2.cancel = Counting(at)
            assertFailsWith<SearchCancelledException>("molti a uno alla verifica $at di ${count.checks}") {
                val m2 = r2.manyToOne(a, listOf(b, c, d), rootIsTarget = false)!!
                m2.advance { (0..2).all { m2.isFinal(it) } }
            }
        }
    }

    @Test
    fun `many to one stops in prepare and in advance`() {
        val r = g.router(RouteOptions(profile = "driving-hgv"))
        val (a, b) = pair(r)
        val (c, d) = pair(g.router(RouteOptions(profile = "driving-hgv", turnCosts = false)))
        r.cancel = cancelled()
        assertEquals(Reason.CANCELLED, assertFailsWith<SearchCancelledException> { r.manyToOne(a, listOf(b, c, d), rootIsTarget = false) }.reason)
        val token = CancelToken()
        r.cancel = token
        val m = assertNotNull(r.manyToOne(a, listOf(b, c, d), rootIsTarget = true))
        token.cancel(Reason.PREEMPTED)
        assertEquals(Reason.PREEMPTED, assertFailsWith<SearchCancelledException> { m.advance { false } }.reason)
    }

    @Test
    fun `a deadline stops a running request`() {
        val r = g.router(RouteOptions(profile = "driving-hgv", allowCH = false))
        val (a, b) = pair(r)
        r.cancel = CancelToken(System.nanoTime() + 30_000_000)
        val started = System.nanoTime()
        val e = assertFailsWith<SearchCancelledException> { while (true) r.route(a, b) }
        assertEquals(Reason.DEADLINE, e.reason)
        assertTrue(System.nanoTime() - started < 10_000_000_000L)
    }

    @Test
    fun `handle throws SearchCancelledException, not ApiException, and nothing changes after`() {
        val service = DirectionsService(g)
        val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.6757,49.4036],[8.7606,49.4186]],"navmaster":{"roads":true}}""") as Map<String, Any?>
        val request = DirectionsRequest.fromJson("driving-hgv", body)
        val before = features(service.handle(request, ResponseFormat.GEOJSON))
        val e = assertFailsWith<SearchCancelledException> { service.handle(request, ResponseFormat.GEOJSON, RequestContext(cancelled())) }
        assertEquals(Reason.CANCELLED, e.reason)
        assertFailsWith<SearchCancelledException> { service.routes(request, RequestContext(cancelled(Reason.DEADLINE))) }
        // con "evita" come penalità (la ricorsione col divieto passa lo stesso contesto)
        val avoid = DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"options":{"avoid_features":["tollways"]},"navmaster":{"avoid_penalty":true}}""") as Map<String, Any?>)
        assertFailsWith<SearchCancelledException> { service.handle(avoid, ResponseFormat.GEOJSON, RequestContext(cancelled())) }
        assertEquals(before, features(service.handle(request, ResponseFormat.GEOJSON)))
        assertEquals(before, features(service.handle(request, ResponseFormat.GEOJSON, RequestContext(CancelToken(), slots = SearchSlots()))))
    }

    @Test
    fun `cross border handle throws SearchCancelledException`() {
        val service = DirectionsService(g)
        val cb = CrossBorderDirections(listOf(CountryGraph("de", service)), { _, _ -> emptyList() })
        val request = DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]}""") as Map<String, Any?>)
        val want = features(cb.handle(request, listOf(null, null)))
        assertFailsWith<SearchCancelledException> { cb.handle(request, listOf(null, null), RequestContext(cancelled())) }
        assertEquals(want, features(cb.handle(request, listOf(null, null), RequestContext(slots = SearchSlots()))))
    }

    @Test
    fun `cross border trip between two countries stops on cancel`() {
        // lo stesso grafo nei panni di due Paesi, valichi finti su nodi lontani fra loro (come CrossBorderChoiceTest)
        val service = DirectionsService(g)
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
        assertTrue(crossings.size >= 3, "valichi finti: ${crossings.size}")
        val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> crossings })
        val request = DirectionsRequest.fromJson("driving-hgv", Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"instructions":false}""") as Map<String, Any?>)
        val countries = listOf("a", "b")
        val count = Counting()
        // senza le cache dei tratti e degli agganci: ogni prova rifà le stesse ricerche (e verifiche)
        val rc = RequestContext(count, useCache = false)
        val want = features(cb.handle(request, countries, rc))
        val line = rc.stats.line()
        println("tra Paesi: ${count.checks} verifiche; $line")
        assertTrue(line.contains(" | valichi a → b: ") && line.contains("[a: ") && line.contains("[b: "), line)
        assertTrue(count.checks >= 4, "tra Paesi: ${count.checks} verifiche")
        assertFailsWith<SearchCancelledException> { cb.handle(request, countries, RequestContext(cancelled())) }
        assertEquals(Reason.DEADLINE, assertFailsWith<SearchCancelledException> { cb.handle(request, countries, RequestContext(CancelToken().child(System.nanoTime() - 1))) }.reason)
        // a metà: nella scelta dei valichi e nei pezzi finali
        for (at in setOf(2, count.checks / 4, count.checks / 2, count.checks * 3 / 4).filter { it >= 2 }) {
            assertFailsWith<SearchCancelledException>("tra Paesi alla verifica $at di ${count.checks}") { cb.handle(request, countries, RequestContext(Counting(at), useCache = false)) }
        }
        // dopo gli annullamenti, lo stesso viaggio
        assertEquals(want, features(cb.handle(request, countries, RequestContext(slots = SearchSlots()))))
    }

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>): String = Json.stringify(doc["features"])

    @Test
    fun `search slots - at most two, background yields to main`() {
        val slots = SearchSlots(2)
        // rientrante: una richiesta annidata sullo stesso thread non prende un altro posto
        slots.primary(RequestContext.MAIN, CancelToken.NEVER) {
            slots.primary(RequestContext.MAIN, CancelToken.NEVER) { assertEquals(1, slots.inUse) }
        }
        assertEquals(0, slots.inUse)

        // un lavoro in sottofondo tiene un posto primario, il suo aiutante il secondo
        val bgHolding = CountDownLatch(1); val bgRelease = CountDownLatch(1)
        val bg = Thread { slots.primary(RequestContext.BACKGROUND, CancelToken.NEVER) { bgHolding.countDown(); bgRelease.await() } }.also { it.start() }
        assertTrue(bgHolding.await(5, TimeUnit.SECONDS))
        val helper = assertNotNull(slots.tryHelper(RequestContext.BACKGROUND, CancelToken.NEVER))
        assertEquals(2, slots.inUse)
        assertNull(slots.tryHelper(RequestContext.MAIN, CancelToken.NEVER), "nessun posto libero: niente aiutante")
        // un secondo lavoro in sottofondo aspetta, e si ferma col suo token
        val bg2 = CancelToken()
        val bg2Error = AtomicReference<Throwable?>()
        val waiting = Thread { try { slots.primary(RequestContext.BACKGROUND, bg2) { } } catch (e: Throwable) { bg2Error.set(e) } }.also { it.start() }
        Thread.sleep(120)
        assertTrue(waiting.isAlive)
        bg2.cancel()
        waiting.join(5_000)
        assertEquals(Reason.CANCELLED, (bg2Error.get() as SearchCancelledException).reason)

        // una richiesta MAIN: l'aiutante in sottofondo cede il posto (PREEMPTED)
        val mainRan = CountDownLatch(1)
        val main = Thread { slots.primary(RequestContext.MAIN, CancelToken.NEVER) { assertTrue(slots.inUse <= 2); mainRan.countDown() } }.also { it.start() }
        val deadline = System.currentTimeMillis() + 5_000
        while (!helper.token.isCancelled && System.currentTimeMillis() < deadline) Thread.sleep(5)
        assertEquals(Reason.PREEMPTED, assertFailsWith<SearchCancelledException> { helper.token.check() }.reason)
        assertFalse(mainRan.await(100, TimeUnit.MILLISECONDS), "la richiesta MAIN aspetta che l'aiutante lasci il posto")
        // l'aiutante, fermato, lo lascia
        helper.close(); helper.close()
        assertTrue(mainRan.await(5, TimeUnit.SECONDS))
        main.join(5_000)
        bgRelease.countDown(); bg.join(5_000)
        assertEquals(0, slots.inUse)

        // un aiutante conta come titolare: le richieste annidate sul suo thread non aspettano
        slots.primary(RequestContext.MAIN, CancelToken.NEVER) {
            val h = assertNotNull(slots.tryHelper(RequestContext.MAIN, CancelToken.NEVER))
            assertEquals(2, slots.inUse)
            val nested = CountDownLatch(1)
            val t = Thread { h.use { it.run { slots.primary(RequestContext.MAIN, CancelToken.NEVER) { if (slots.inUse == 2) nested.countDown() } } } }.also { it.start() }
            assertTrue(nested.await(5, TimeUnit.SECONDS))
            t.join(5_000)
            assertEquals(1, slots.inUse)
        }
        assertEquals(0, slots.inUse)
    }
}
