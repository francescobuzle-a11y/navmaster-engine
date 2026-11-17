package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.util.DistanceCalc
import java.util.Random
import java.util.concurrent.AbstractExecutorService
import java.util.concurrent.CountDownLatch
import java.util.concurrent.ExecutorService
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicReference
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/** Un punto a metà di un arco a caso (il primo segmento della sua geometria), con il seme [rnd]. */
private fun midEdge(rnd: Random): Pair<Double, Double> {
    val bg = Heidelberg.graph.graph
    val e = rnd.nextInt(bg.edgeCount)
    val geo = bg.fetchWayGeometry(e, false, FetchMode.ALL)
    return (geo.lat(0) + geo.lat(1)) / 2 to (geo.lon(0) + geo.lon(1)) / 2
}

/** [Router.directionFree][it.navmaster.engine.routing.Router.directionFree] e [Router.arrivalKey][it.navmaster.engine.routing.Router.arrivalKey]. */
class DirectionFreeTest {
    @Test
    fun `towers and one-way edges are direction free, two-way mid-edge snaps are not`() {
        val g = Heidelberg.graph
        val r = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(3)
        var tower = 0; var oneWay = 0; var twoWay = 0
        for (k in 0 until 3000) {
            if (tower > 3 && oneWay > 3 && twoWay > 3) break
            val (lat, lon) = midEdge(rnd)
            val s = try { r.snap(lat, lon, 50.0, 0) } catch (e: PointNotFoundException) { continue }
            val it = bg.createEdgeIterator().setEdge(s.edge, bg.nodeA(s.edge))
            val free = r.directionFree(s)
            if (s.position == SnapPosition.TOWER) { assertTrue(free); tower++; continue }
            // un arrivo sullo stesso arco: conta solo dove l'arco si percorre nei due versi
            val arrival = PathEdge(s.edge, true, bg.nodeB(s.edge), bg.nodeA(s.edge), it.fetchWayGeometry(FetchMode.ALL), 1.0, 1, 1.0, "")
            if (free) { oneWay++; assertEquals(-1, r.arrivalKey(s, arrival)) } else { twoWay++; assertEquals(1, r.arrivalKey(s, arrival)) }
            assertEquals(-1, r.arrivalKey(s, null))
            val other = PathEdge(s.edge + 1, true, 0, 0, it.fetchWayGeometry(FetchMode.ALL), 1.0, 1, 1.0, "")
            assertEquals(-1, r.arrivalKey(s, other))
        }
        // una torre: ci si aggancia esattamente su un nodo
        val n = (0 until bg.nodeCount).first { n -> runCatching { r.snap(bg.lat(n), bg.lon(n), 5.0, 0).position == SnapPosition.TOWER }.getOrDefault(false) }
        assertTrue(r.directionFree(r.snap(bg.lat(n), bg.lon(n), 5.0, 0)))
        println("agganci: $tower torri, $oneWay a senso unico, $twoWay a doppio senso")
        assertTrue(oneWay > 0 && twoWay > 0, "agganci: $oneWay a senso unico, $twoWay a doppio senso")
    }
}

/** `navmaster.pass_through`: per punto come `continue_straight`; la lunghezza va controllata; anche tra Paesi. */
class PassThroughTest {
    private val g get() = Heidelberg.graph

    @Test
    fun `pass_through everywhere is continue_straight`() {
        val pool = Heidelberg.points(6, 21)
        val service = DirectionsService(g)
        for (n in 3..5) {
            val pts = pool.take(n)
            val cs = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","continue_straight":true""")), ResponseFormat.GEOJSON, RequestContext(useCache = false))
            val all = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[${pts.joinToString(",") { "true" }}]}""")), ResponseFormat.GEOJSON, RequestContext(useCache = false))
            assertEquals(Heidelberg.features(cs), Heidelberg.features(all))
            // solo le tappe interne contano: le estremità no
            val inner = service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[${pts.indices.joinToString(",") { (it > 0 && it < n - 1).toString() }}]}""")), ResponseFormat.GEOJSON)
            assertEquals(Heidelberg.features(cs), Heidelberg.features(inner))
        }
    }

    @Test
    fun `pass_through with the wrong length is 2003`() {
        val pts = Heidelberg.points(3, 22)
        for (bad in listOf("[true,false]", "[false,true,false,true]", "true", "[0,1,0]", "[false,\"true\",false]", "[false,null,false]")) {
            val e = assertFailsWith<ApiException> { DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":$bad}""")) }
            assertEquals(400, e.httpStatus); assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, e.code)
        }
        val ok = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[false,true,false]}"""))
        assertTrue(!ok.passThroughAt(0) && ok.passThroughAt(1) && !ok.passThroughAt(2) && !ok.continueStraight)
        assertTrue(DirectionsService(g).routeOptions(ok).continueStraight)
        // tutti falsi: come senza
        val none = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[false,false,false]}"""))
        assertEquals(null, none.passThrough)
    }

    @Test
    fun `across borders pass_through at a stop is continue_straight there`() {
        val service = DirectionsService(g)
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(11)
        val crossings = ArrayList<Transfer>()
        for (attempt in 0 until 4000) {
            if (crossings.size >= 6) break
            val n0 = rnd.nextInt(bg.nodeCount)
            val n = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
            if (crossings.any { t -> DistanceCalc.haversine(t.lat, t.lon, bg.lat(n), bg.lon(n)) < 3_000.0 }) continue
            crossings += Transfer(bg.lat(n), bg.lon(n), 1 + crossings.size % 3)
        }
        val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> crossings })
        val pts = listOf(49.4094 to 8.6937, 49.4036 to 8.6757, 49.4120 to 8.6850, 49.4186 to 8.7606)
        for (countries in listOf(listOf("a", "a", "b", "b"), listOf("a", "b", "b", "b"), listOf("a", "a", "a", "b"))) {
            val cs = cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","continue_straight":true,"instructions":false""")), countries, RequestContext(useCache = false))
            val pt = cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[false,true,true,false]},"instructions":false""")), countries)
            assertEquals(Heidelberg.features(cs), Heidelberg.features(pt), "Paesi $countries")
        }
    }
}

/**
 * I tratti in parallelo ([DirectionsService.solve] con un aiutante): gli stessi pesi dei tratti in
 * serie, la correzione dei tratti cercati senza il verso d'arrivo (anche ricalcolati), l'errore del
 * tratto d'indice minimo.
 */
class ParallelLegsTest {
    private val g get() = Heidelberg.graph

    private fun <T> withPool(block: (RequestContext) -> T): T {
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try { return block(RequestContext(executor = pool, slots = SearchSlots(2), useCache = false)) } finally { pool.shutdown(); pool.awaitTermination(10, TimeUnit.SECONDS) }
    }

    /**
     * Ripete [attempt] (al più 40 volte) finché il secondo tratto viene da [expected]; ogni giro
     * non speculativo dev'essere un calcolo normale (CALC). Vero se la speculazione è capitata.
     */
    private fun untilSpeculative(expected: LegStats.Source, attempt: () -> RequestStats): Boolean {
        repeat(40) {
            val st = attempt()
            val src = st.legs[1].source
            if (src == expected) { println(st.line()); return true }
            assertEquals(LegStats.Source.CALC, src, st.line())
        }
        return false
    }

    @Test
    fun `parallel legs equal sequential legs`() {
        // viaggi a zig-zag per Heidelberg: oltre i 100 km in linea d'aria
        val service = DirectionsService(g, DirectionsLimits(maximumDistance = 1e7, maximumDistanceDynamicWeights = 1e7))
        val pool = Heidelberg.points(10, 31)
        val rnd = Random(31)
        val sources = HashMap<LegStats.Source, Int>()
        val threads = HashSet<String>()
        for (k in 0 until 12) {
            val n = 3 + rnd.nextInt(5)
            val pts = List(n) { pool[(k + it * 3) % pool.size] }
            val flags = if (k % 3 == 0) ""","continue_straight":true""" else ""","navmaster":{"pass_through":[${pts.indices.joinToString(",") { (it > 0 && it < n - 1 && rnd.nextBoolean()).toString() }}]}"""
            val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, flags))
            val seq = service.solve(request, RequestContext(useCache = false))
            withPool { rc ->
                val par = service.solve(request, rc)
                assertEquals(seq.legs.size, par.legs.size)
                for ((a, b) in seq.legs.zip(par.legs)) assertEquals(a.weight, b.weight, 1e-9 * maxOf(1.0, a.weight), "richiesta $k")
                for (l in rc.stats.legs) { sources.merge(l.source, 1, Int::plus); threads += l.thread }
                assertEquals(0, rc.slots!!.inUse)
            }
        }
        println("tratti in parallelo: $sources su $threads")
        assertTrue("nm-leg" in threads, "l'aiutante non ha mai lavorato: $threads")
    }

    /** A, V (a metà di un arco a doppio senso, «passa per di qui»), C: senza il verso d'arrivo il tratto V → C riparte all'indietro. */
    private fun recalcCase(): List<Pair<Double, Double>> {
        val r = g.router(RouteOptions(profile = "driving-hgv", continueStraight = true))
        val rnd = Random(5)
        val ends = Heidelberg.points(12, 41)
        for (k in 0 until 4000) {
            val v = midEdge(rnd)
            val sv = try { r.snap(v.first, v.second, 30.0, 1) } catch (e: PointNotFoundException) { continue }
            if (sv.position == SnapPosition.TOWER || r.directionFree(sv)) continue
            val a = ends[rnd.nextInt(ends.size)]; val c = ends[rnd.nextInt(ends.size)]
            if (a == c) continue
            val sa = r.snap(a.first, a.second, 400.0, 0); val sc = r.snap(c.first, c.second, 400.0, 2)
            val toV = try { r.route(sa, sv) } catch (e: RouteNotFoundException) { continue }
            val arrival = toV.edges.lastOrNull() ?: continue
            if (r.arrivalKey(sv, arrival) == -1) continue
            val free = try { r.route(sv, sc) } catch (e: RouteNotFoundException) { continue }
            val first = free.edges.firstOrNull() ?: continue
            if (first.reverse == arrival.reverse) continue
            // V → C il più lungo: cercato per primo, senza il verso d'arrivo
            if (DistanceCalc.haversine(sv.snappedLat, sv.snappedLon, sc.snappedLat, sc.snappedLon) <= DistanceCalc.haversine(sa.snappedLat, sa.snappedLon, sv.snappedLat, sv.snappedLon) * 1.5) continue
            return listOf(a, sv.snappedLat to sv.snappedLon, c)
        }
        error("nessun caso da ricalcolare")
    }

    @Test
    fun `a speculative leg against the arrival is searched again`() {
        val service = DirectionsService(g)
        val pts = recalcCase()
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[false,true,false]}"""))
        val seq = service.solve(request, RequestContext(useCache = false))
        // se l'aiutante parte tardi (macchina carica) i tratti si fanno in ordine e nessuno è
        // speculativo: si riprova finché la speculazione capita, ogni giro dev'essere giusto comunque
        assertTrue(untilSpeculative(LegStats.Source.RECALC) {
            withPool { rc ->
                val par = service.solve(request, rc)
                for ((a, b) in seq.legs.zip(par.legs)) assertEquals(a.weight, b.weight, 1e-9 * maxOf(1.0, a.weight))
                // il tratto ricalcolato (o cercato dopo il precedente) riparte nel verso d'arrivo
                assertEquals(par.legs[0].edges.last().reverse, par.legs[1].edges.first().reverse)
                rc.stats
            }
        }, "nessun giro con il tratto speculativo ricalcolato")
        // con la cache: il tratto col verso d'arrivo giusto si ritrova (nessun ricalcolo)
        val cache = DirectionsService(g)
        cache.solve(request)
        val pool = Executors.newFixedThreadPool(1)
        try {
            val rc = RequestContext(executor = pool, slots = SearchSlots(2))
            cache.solve(request, rc)
            assertTrue(rc.stats.legs.all { it.source == LegStats.Source.CACHE }, rc.stats.line())
        } finally { pool.shutdown() }
    }

    @Test
    fun `without a running helper no leg is speculative`() {
        // l'executor rifiuta il lavoro: il posto da aiutante c'è, l'aiutante no; un thread solo, in ordine di dipendenza
        val service = DirectionsService(g)
        val pts = recalcCase()
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"pass_through":[false,true,false]}"""))
        val seq = service.solve(request, RequestContext(useCache = false))
        val pool = Executors.newFixedThreadPool(1).also { it.shutdown() }
        val slots = SearchSlots(2)
        val rc = RequestContext(executor = pool, slots = slots, useCache = false)
        val par = service.solve(request, rc)
        assertEquals(listOf(LegStats.Source.CALC, LegStats.Source.CALC), rc.stats.legs.map { it.source }, rc.stats.line())
        for ((a, b) in seq.legs.zip(par.legs)) assertEquals(a.weight, b.weight, 1e-9 * maxOf(1.0, a.weight))
        assertEquals(0, slots.inUse)
    }

    @Test
    fun `a speculative leg along the arrival is kept`() {
        // V a un terzo del percorso A → C, a metà di un arco a doppio senso: V → C prosegue comunque nel verso d'arrivo
        val r = g.router(RouteOptions(profile = "driving-hgv", continueStraight = true))
        val ends = Heidelberg.points(12, 61)
        var pts: List<Pair<Double, Double>>? = null
        loop@ for (a in ends) for (c in ends) {
            if (a == c) continue
            val path = try { r.route(r.snap(a.first, a.second, 400.0, 0), r.snap(c.first, c.second, 400.0, 1)) } catch (e: RouteNotFoundException) { continue }
            if (path.distance < 3_000) continue
            var along = 0.0
            for (e in path.edges) {
                along += e.distance
                if (along < path.distance / 4 || e.geometry.size < 2) continue
                if (along > path.distance / 2.5) break
                val v = (e.geometry.lat(0) + e.geometry.lat(1)) / 2 to (e.geometry.lon(0) + e.geometry.lon(1)) / 2
                val sv = try { r.snap(v.first, v.second, 5.0, 1) } catch (x: PointNotFoundException) { continue }
                if (sv.edge != e.edge || sv.position == SnapPosition.TOWER || r.directionFree(sv)) continue
                pts = listOf(a, sv.snappedLat to sv.snappedLon, c); break@loop
            }
        }
        val service = DirectionsService(g)
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(assertNotNull(pts, "nessun via sul percorso"), ""","navmaster":{"pass_through":[false,true,false]}"""))
        val seq = service.solve(request, RequestContext(useCache = false))
        assertTrue(untilSpeculative(LegStats.Source.SPEC_OK) {
            withPool { rc ->
                val par = service.solve(request, rc)
                for ((a, b) in seq.legs.zip(par.legs)) assertEquals(a.weight, b.weight, 1e-9 * maxOf(1.0, a.weight))
                rc.stats
            }
        }, "nessun giro con il tratto speculativo tenuto")
    }

    /**
     * A, V1, V2, C (V1 e V2 a metà di archi a doppio senso, «passa per di qui»): senza il verso
     * d'arrivo V1 → V2 riparte all'indietro, e quello che riparte nel verso d'arrivo arriva a V2
     * dall'altra parte: il tratto V2 → C cercato dopo il primo va cercato di nuovo.
     */
    private fun chainCase(): List<Pair<Double, Double>> {
        val r = g.router(RouteOptions(profile = "driving-hgv", continueStraight = true))
        val rnd = Random(7)
        val ends = Heidelberg.points(12, 71)
        fun via(): Snap? {
            val v = midEdge(rnd)
            val s = try { r.snap(v.first, v.second, 30.0, 1) } catch (e: PointNotFoundException) { return null }
            return s.takeIf { it.position != SnapPosition.TOWER && !r.directionFree(it) }
        }
        for (k in 0 until 20_000) {
            val s1 = via() ?: continue
            val s2 = via() ?: continue
            if (s1.edge == s2.edge) continue
            val a = ends[rnd.nextInt(ends.size)]; val c = ends[rnd.nextInt(ends.size)]
            if (a == c) continue
            val sa = r.snap(a.first, a.second, 400.0, 0); val sc = r.snap(c.first, c.second, 400.0, 3)
            val arrival = (try { r.route(sa, s1) } catch (e: RouteNotFoundException) { continue }).edges.lastOrNull() ?: continue
            val bit = r.arrivalKey(s1, arrival)
            if (bit == -1) continue
            val free = try { r.route(s1, s2) } catch (e: RouteNotFoundException) { continue }
            val first = free.edges.firstOrNull() ?: continue
            if (first.edge != s1.edge || first.reverse == (bit == 1)) continue
            val bound = try { r.route(s1, s2, arrival) } catch (e: RouteNotFoundException) { continue }
            val stale = r.arrivalKey(s2, free.edges.lastOrNull()); val fin = r.arrivalKey(s2, bound.edges.lastOrNull())
            if (stale == -1 || fin == -1 || stale == fin) continue
            if (try { r.route(s2, sc); false } catch (e: RouteNotFoundException) { true }) continue
            return listOf(a, s1.snappedLat to s1.snappedLon, s2.snappedLat to s2.snappedLon, c)
        }
        error("nessuna catena da ricalcolare")
    }

    /** Ferma il thread [main] alla sua prima verifica con l'aiutante al lavoro ([SearchSlots.inUse] 2), finché l'aiutante non lascia il posto. */
    private class HoldMain(val slots: SearchSlots, val main: Thread) : CancelToken() {
        val held = CountDownLatch(1)
        override fun check() {
            if (Thread.currentThread() === main && held.count > 0 && slots.inUse == 2) {
                held.countDown()
                val end = System.nanoTime() + 20_000_000_000L
                while (slots.inUse == 2 && System.nanoTime() < end) Thread.sleep(2)
            }
            super.check()
        }
    }

    /** Un executor che fa partire ogni lavoro solo dopo [gate] (al più 20 s). */
    private class Gated(val pool: ExecutorService, val gate: CountDownLatch) : AbstractExecutorService() {
        override fun execute(command: Runnable) = pool.execute { gate.await(20, TimeUnit.SECONDS); command.run() }
        override fun shutdown() = pool.shutdown()
        override fun shutdownNow(): MutableList<Runnable> = pool.shutdownNow()
        override fun isShutdown() = pool.isShutdown
        override fun isTerminated() = pool.isTerminated
        override fun awaitTermination(timeout: Long, unit: TimeUnit) = pool.awaitTermination(timeout, unit)
    }

    @Test
    fun `a leg searched after a speculative one is checked again`() {
        val pts = chainCase()
        val flags = ""","navmaster":{"pass_through":[false,true,true,false]}"""
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, flags))
        val seq = DirectionsService(g).solve(request, RequestContext(useCache = false))
        val service = DirectionsService(g)
        // gli agganci in cache (da un viaggio con altri tratti): l'aiutante lavora solo sui tratti
        service.solve(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts.reversed(), flags)))
        val slots = SearchSlots(2)
        val hold = HoldMain(slots, Thread.currentThread())
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try {
            // questo thread prende il tratto 0 e si ferma; l'aiutante cerca l'1 senza il verso d'arrivo, poi il 2 col suo
            val rc = RequestContext(hold, executor = Gated(pool, hold.held), slots = slots)
            val par = service.solve(request, rc)
            val line = rc.stats.line()
            println(line)
            assertEquals(0L, hold.held.count, line)
            assertEquals(listOf(LegStats.Source.CALC, LegStats.Source.RECALC, LegStats.Source.RECALC), rc.stats.legs.map { it.source }, line)
            for ((a, b) in seq.legs.zip(par.legs)) assertEquals(a.weight, b.weight, 1e-9 * maxOf(1.0, a.weight), line)
            // ai due via si riparte nel verso d'arrivo
            for (i in 1..2) assertEquals(par.legs[i - 1].edges.last().reverse, par.legs[i].edges.first().reverse, "via $i: $line")
            assertEquals(0, slots.inUse)
        } finally { pool.shutdown() }
    }

    @Test
    fun `a background solve gives the helper slot back as soon as the helper is done`() {
        val service = DirectionsService(g)
        // due punti nuovi: gli agganci su due thread, poi un tratto solo
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(Heidelberg.points(2, 81)))
        val slots = SearchSlots(2)
        val bgThread = AtomicReference<Thread>()
        val inLeg = CountDownLatch(1); val release = CountDownLatch(1)
        // il lavoro in sottofondo si ferma dentro la ricerca del tratto
        val token = object : CancelToken() {
            override fun check() {
                if (Thread.currentThread() === bgThread.get() && inLeg.count > 0 && Thread.currentThread().stackTrace.any { it.methodName.startsWith("legsOf") }) {
                    inLeg.countDown(); release.await(20, TimeUnit.SECONDS)
                }
                super.check()
            }
        }
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try {
            val result = AtomicReference<Any?>()
            val bg = Thread { result.set(try { service.solve(request, RequestContext(token, RequestContext.BACKGROUND, pool, slots, useCache = false)) } catch (e: Throwable) { e }) }
            bgThread.set(bg); bg.start()
            assertTrue(inLeg.await(20, TimeUnit.SECONDS), "la ricerca del tratto non è partita")
            assertEquals(1, slots.inUse, "l'aiutante degli agganci ha lasciato il posto")
            // una richiesta MAIN trova subito il posto, mentre il tratto in sottofondo è ancora in ricerca
            val mainRan = CountDownLatch(1)
            val main = Thread { slots.primary(RequestContext.MAIN, CancelToken.NEVER) { mainRan.countDown() } }.also { it.start() }
            val ran = mainRan.await(2, TimeUnit.SECONDS)
            release.countDown()
            main.join(5_000); bg.join(20_000)
            assertTrue(ran, "la richiesta MAIN ha aspettato il lavoro in sottofondo")
            assertTrue(result.get() is DirectionsService.Solved, "${result.get()}")
            assertEquals(0, slots.inUse)
        } finally { pool.shutdown() }
    }

    @Test
    fun `the error names the leg with the smallest index`() {
        val r = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(13)
        val (a, b, c) = Heidelberg.points(3, 51)
        val sa = r.snap(a.first, a.second, 400.0, 0)
        // un punto su una piccola rete isolata, agganciato senza il controllo (come un passaggio di confine)
        var island: Pair<Double, Double>? = null
        for (k in 0 until 20_000) {
            val e = rnd.nextInt(bg.edgeCount)
            val geo = bg.fetchWayGeometry(e, false, FetchMode.ALL)
            val p = (geo.lat(0) + geo.lat(1)) / 2 to (geo.lon(0) + geo.lon(1)) / 2
            val s: Snap = try { r.snap(p.first, p.second, 20.0, 0, checkNetwork = false) } catch (e: PointNotFoundException) { continue }
            val reached = try { r.route(sa, s); true } catch (e: RouteNotFoundException) { false }
            val back = try { r.route(s, sa); true } catch (e: RouteNotFoundException) { false }
            if (!reached && !back) { island = p; break }
        }
        val x = assertNotNull(island, "nessuna rete isolata a Heidelberg")
        val service = DirectionsService(g)
        // i tratti 0 (A → B) e 2 (C → X) e 3 (X → A): l'errore è del 2, qualunque thread finisca prima
        val pts = listOf(a, b, c, x, a)
        val exact = BooleanArray(pts.size) { it == 3 }
        val request = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts)).withPoints(pts, null, exact)
        val seq = assertFailsWith<ApiException> { service.solve(request, RequestContext(useCache = false)) }
        assertEquals(RoutingErrorCodes.ROUTE_NOT_FOUND, seq.code)
        assertTrue(seq.message!!.contains("between points 2 "), seq.message)
        withPool { rc ->
            val par = assertFailsWith<ApiException> { service.solve(request, rc) }
            assertEquals(seq.message, par.message)
            assertEquals(0, rc.slots!!.inUse)
        }
    }
}
