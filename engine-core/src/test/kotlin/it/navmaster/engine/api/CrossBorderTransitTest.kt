package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import java.util.Collections
import java.util.Random
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * Le correzioni della scelta dei valichi dopo le prove sui grafi veri (gruppi A, C, D, E, F): il tetto
 * del transito riletto mentre l'altra sequenza trova il suo migliore, la memoria della scelta dei due
 * valichi e dei percorsi mirati fra la richiesta del migliore e quella delle varianti, il tetto dei
 * percorsi mirati, il limite del Paese di mezzo per uscita, i minimi garantiti alle varianti dentro il
 * loro tempo, i giri per un Paese vicino filtrati prima di tagliarli. Su Heidelberg con Paesi finti (lo
 * stesso grafo, valichi finti); il migliore sempre identico.
 */
class CrossBorderTransitTest {
    private val g get() = Heidelberg.graph

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = doc["features"] as List<Map<String, Any?>>
    private fun meta(doc: Map<String, Any?>) = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>

    @Suppress("UNCHECKED_CAST")
    private fun withoutVariant(f: Map<String, Any?>): String {
        val props = LinkedHashMap(f["properties"] as Map<String, Any?>)
        val nm = (props["navmaster"] as? Map<String, Any?>)?.let { LinkedHashMap(it) }
        nm?.remove("variant")
        if (nm == null || nm.isEmpty()) props.remove("navmaster") else props["navmaster"] = nm
        return Json.stringify(LinkedHashMap(f).also { it["properties"] = props })
    }

    /** [n] valichi finti su nodi della rete, a più di [apart] metri l'uno dall'altro. */
    private fun crossings(seed: Long, n: Int, apart: Double = 4_000.0): List<Transfer> {
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(seed)
        val out = ArrayList<Transfer>()
        for (attempt in 0 until 8000) {
            if (out.size >= n) break
            val k = rnd.nextInt(bg.nodeCount)
            val node = try { core.snap(bg.lat(k), bg.lon(k), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
            if (out.any { t -> DistanceCalc.haversine(t.lat, t.lon, bg.lat(node), bg.lon(node)) < apart }) continue
            out += Transfer(bg.lat(node), bg.lon(node), 1 + out.size % 3)
        }
        return out
    }

    /** «Q ricerche (H dalla memoria[, X per le varianti])» → (Q, H, X). */
    private fun queries(line: String): Triple<Int, Int, Int> {
        val m = Regex("""(\d+) ricerche \((\d+) dalla memoria(?:, (\d+) per le varianti)?\)""").find(line) ?: error("riga senza ricerche: $line")
        return Triple(m.groupValues[1].toInt(), m.groupValues[2].toInt(), m.groupValues[3].ifEmpty { "0" }.toInt())
    }

    /** Le uscite esaminate dall'ultimo transito [path] («a→b→d») del registro; null se non c'è. */
    private fun examined(logs: List<String>, path: String): Int? = synchronized(logs) { logs.toList() }.lastOrNull { "transito ($path)" in it }
        ?.let { Regex("""(\d+) uscite esaminate""").find(it)?.groupValues?.get(1)?.toInt() }

    /** I percorsi mirati di ogni ricerca di limite del registro («+ N mirati»). */
    private fun routes(logs: List<String>): List<Int> = synchronized(logs) { logs.toList() }.flatMap { l -> Regex("""\+ (\d+) mirati""").findAll(l).map { it.groupValues[1].toInt() }.toList() }

    private fun close(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || Math.abs(x - y) <= 1e-6 * Math.abs(y) + 1e-9

    private class Run(val cb: CrossBorderDirections, val logs: MutableList<String>, val doc: Map<String, Any?>?, val rc: RequestContext)

    /** Il viaggio [pts] da «a» a [to] coi valichi [pairs] su un servizio nuovo; [setup] prepara il motore prima della richiesta. */
    private fun run(ids: List<String>, pairs: Map<Set<String>, List<Transfer>>, pts: List<Pair<Double, Double>>, to: String, rc: RequestContext,
                    nav: NavAlternativesOptions? = null, service: DirectionsService = DirectionsService(g), onLog: (String) -> Unit = {},
                    logs: MutableList<String> = Collections.synchronizedList(ArrayList<String>()), setup: (CrossBorderDirections) -> Unit = {}): Run {
        val cb = CrossBorderDirections(ids.map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it; onLog(it) })
        setup(cb)
        val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")).let { if (nav == null) it else it.withNavAlternatives(nav) }
        val doc = try { cb.handle(req, listOf("a", to), rc) } catch (e: ApiException) { null }
        return Run(cb, logs, doc, rc)
    }

    // ------------------------------------------------------------------ il tetto del transito riletto

    /**
     * Due sequenze, a-d (diretta) e a-b-d (transito, due valichi insieme), su due thread. Il transito
     * comincia mentre a-d non ha ancora il suo migliore (la prova lo ferma all'inizio della scelta finché
     * a-d non ha finito, e ferma a-d finché il transito non è partito): col tetto letto una volta sola
     * (prima: ∞) esaminava tutte le uscite che esamina senza a-d; col tetto riletto ne esamina quante
     * una dopo l'altra, dove il transito vede subito il migliore di a-d. Il migliore è lo stesso.
     */
    @Test
    fun `the transit rereads the ceiling - a direct sequence found meanwhile stops it`() {
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "aiutante").apply { isDaemon = true } }
        var trips = 0; var fewer = 0; var helper = 0
        try {
            for (seed in 1L..24L) {
                val pairs = mapOf(setOf("a", "d") to crossings(300 + seed, 3), setOf("a", "b") to crossings(400 + seed, 4), setOf("b", "d") to crossings(500 + seed, 8, 2_500.0))
                val pts = Heidelberg.points(2, seed + 2100)
                val ids = listOf("a", "b", "d")
                // una dopo l'altra: il transito (sequenza 1) col tetto di a-d dall'inizio
                val seq = run(ids, pairs, pts, "d", RequestContext(useCache = false)) { it.parallelSequences = false }
                if (seq.doc == null) continue
                val eSeq = examined(seq.logs, "a→b→d") ?: continue
                // senza la sequenza diretta: il transito col tetto infinito (quello che il tetto letto una volta dava)
                val inf = run(ids, pairs - setOf(setOf("a", "d")), pts, "d", RequestContext(useCache = false))
                val eInf = examined(inf.logs, "a→b→d")
                // su due thread, il transito partito prima che a-d finisca
                val started = CountDownLatch(1)
                val parLogs = Collections.synchronizedList(ArrayList<String>())
                val par = run(ids, pairs, pts, "d", RequestContext(executor = pool, slots = SearchSlots(2), useCache = false),
                    onLog = { if (it.startsWith("  a-d confine")) started.await(2, TimeUnit.SECONDS) }, logs = parLogs) { cb ->
                    cb.beforeTwo = {
                        started.countDown()
                        val end = System.nanoTime() + 2_000_000_000L
                        while (System.nanoTime() < end && synchronized(parLogs) { parLogs.none { it.startsWith("  sequenza a-d") } }) Thread.sleep(1)
                    }
                }
                trips++
                assertEquals(withoutVariant(features(seq.doc)[0]), withoutVariant(features(par.doc!!)[0]), "seme $seed: migliore diverso su due thread")
                assertTrue(close(par.cb.lastWeight, seq.cb.lastWeight), "seme $seed: peso ${par.cb.lastWeight} invece di ${seq.cb.lastWeight}")
                assertEquals(seq.cb.lastCrossings.map { it.lat to it.lon }, par.cb.lastCrossings.map { it.lat to it.lon })
                val ePar = examined(par.logs, "a→b→d")
                if (synchronized(par.logs) { par.logs.any { it.startsWith("  sequenza ") && it.endsWith("@aiutante") } }) helper++
                println("seme $seed: uscite esaminate una dopo l'altra $eSeq, su due thread $ePar, senza a-d $eInf; migliore ${seq.cb.lastChains.firstOrNull()?.countries}")
                if (ePar != null) assertTrue(ePar <= eSeq, "seme $seed: il transito ha esaminato $ePar uscite, una dopo l'altra $eSeq (tetto non riletto?)")
                if (eInf != null && eInf > eSeq) fewer++
            }
        } finally { pool.shutdown(); pool.awaitTermination(10, TimeUnit.SECONDS) }
        println("tetto riletto: $trips viaggi, $fewer dove il tetto infinito esaminava più uscite, $helper con l'aiutante")
        assertTrue(trips >= 8, "viaggi: $trips")
        assertTrue(fewer >= 1, "nessun viaggio dove il tetto conta")
        assertTrue(helper >= 1, "nessuna sequenza sull'aiutante")
    }

    // ------------------------------------------------------------------ la memoria del transito

    /**
     * Un transito a-b-c (una sequenza sola, i due valichi scelti insieme): la richiesta delle varianti,
     * subito dopo quella del migliore, prende la scelta dalla memoria del servizio (anche con un motore
     * nuovo, come fa l'app a ogni richiesta): nessuna ricerca della scelta rifatta, lo stesso migliore.
     * Senza cache le ricerche si fanno; clearCaches svuota la memoria.
     */
    @Test
    fun `the variants request takes the transit choice from the service memory`() {
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..12L) {
            val pairs = mapOf(setOf("a", "b") to crossings(700 + seed, 4), setOf("b", "c") to crossings(800 + seed, 6, 2_500.0))
            val pts = Heidelberg.points(2, seed + 2300)
            val ids = listOf("a", "b", "c")
            val service = DirectionsService(g)
            val first = run(ids, pairs, pts, "c", RequestContext(), service = service)
            if (first.doc == null || examined(first.logs, "a→b→c") == null) continue
            trips++
            val (q0, _, _) = queries(first.rc.stats.crossings)
            assertTrue(q0 >= 3, "prima richiesta: ${first.rc.stats.crossings}")
            assertTrue(service.transitCache.size >= 1, "nessuna scelta in memoria")
            val second = run(ids, pairs, pts, "c", RequestContext(), nav = opt, service = service)
            val line = second.rc.stats.crossings
            val (q, h, x) = queries(line)
            println("seme $seed: prima ${first.rc.stats.crossings} | varianti $line")
            assertEquals(withoutVariant(features(first.doc)[0]), withoutVariant(features(second.doc!!)[0]), "seme $seed: migliore diverso")
            assertTrue(close(second.cb.lastWeight, first.cb.lastWeight))
            assertEquals(0, q - x, "seme $seed: ricerche della scelta rifatte: $line")
            assertTrue(h >= 1, "seme $seed: niente dalla memoria: $line")
            assertTrue(synchronized(second.logs) { second.logs.any { "transito" !in it && "a→b→c): dalla memoria" in it } }, "seme $seed: la scelta non viene dalla memoria")
            // senza cache: la scelta si rifà
            val noCache = run(ids, pairs, pts, "c", RequestContext(useCache = false), service = service)
            assertTrue(queries(noCache.rc.stats.crossings).first >= 3, "senza cache: ${noCache.rc.stats.crossings}")
            assertEquals(withoutVariant(features(first.doc)[0]), withoutVariant(features(noCache.doc!!)[0]))
            service.clearCaches()
            assertEquals(0, service.transitCache.size)
        }
        assertTrue(trips >= 5, "viaggi: $trips")
    }

    /**
     * Fra due Paesi il valico migliore si ricorda nel servizio: la stessa richiesta lo riprende senza
     * cercare nulla, quella delle varianti parte da lui (le altre ricerche sono per le varianti); lo
     * stesso migliore, anche con un motore nuovo per richiesta come nell'app.
     */
    @Test
    fun `between two countries the best crossing comes from the service memory`() {
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..12L) {
            val pairs = mapOf(setOf("a", "b") to crossings(1400 + seed, 8, 2_000.0))
            val pts = Heidelberg.points(2, seed + 3300)
            val service = DirectionsService(g)
            val first = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), service = service)
            if (first.doc == null) continue
            trips++
            val again = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), service = service)
            assertEquals(withoutVariant(features(first.doc)[0]), withoutVariant(features(again.doc!!)[0]))
            assertTrue(synchronized(again.logs) { again.logs.any { it.contains("valico scelto: dalla memoria") } }, "seme $seed: la scelta non viene dalla memoria")
            assertTrue(synchronized(again.logs) { again.logs.none { it.contains("lato vicino") } }, "seme $seed: la scelta rifatta")
            val v = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), nav = opt, service = service)
            val (q, h, x) = queries(v.rc.stats.crossings)
            println("seme $seed: prima ${first.rc.stats.crossings} | varianti ${v.rc.stats.crossings}")
            assertEquals(withoutVariant(features(first.doc)[0]), withoutVariant(features(v.doc!!)[0]), "seme $seed: migliore diverso")
            assertTrue(close(v.cb.lastWeight, first.cb.lastWeight))
            assertEquals(first.cb.lastCrossings.map { it.lat to it.lon }, v.cb.lastCrossings.map { it.lat to it.lon })
            assertEquals(0, q - x, "seme $seed: ricerche del migliore rifatte: ${v.rc.stats.crossings}")
            assertTrue(h >= 1)
        }
        assertTrue(trips >= 6, "viaggi: $trips")
    }

    /** La memoria dei transiti: una scelta esatta vale sempre, «nessuna coppia sotto l» solo per tetti ≤ l; una voce peggiore non sostituisce. */
    @Test
    fun `transit memory - an exact choice always, none below a ceiling only for lower ceilings`() {
        val c = TransitCache(maxEntries = 2)
        val x = Transfer(49.4, 8.6, 1); val y = Transfer(49.5, 8.7, 1)
        c.put("k", null, null, Double.POSITIVE_INFINITY, 1000.0)
        assertEquals(1000.0, c.get("k")!!.limit)
        // un tetto più basso non sostituisce, uno più alto sì
        c.put("k", null, null, Double.POSITIVE_INFINITY, 800.0)
        assertEquals(1000.0, c.get("k")!!.limit)
        c.put("k", null, null, Double.POSITIVE_INFINITY, 1500.0)
        assertEquals(1500.0, c.get("k")!!.limit)
        // una scelta esatta sostituisce e poi resta
        c.put("k", x, y, 1200.0, 2000.0)
        assertEquals(1200.0, c.get("k")!!.w); assertEquals(x, c.get("k")!!.x)
        c.put("k", null, null, Double.POSITIVE_INFINITY, 5000.0)
        assertEquals(1200.0, c.get("k")!!.w)
        // la meno usata di recente esce oltre il massimo
        c.put("k2", x, y, 10.0, 20.0); c.get("k"); c.put("k3", x, y, 10.0, 20.0)
        assertNull(c.get("k2")); assertEquals(1200.0, c.get("k")!!.w)
        c.clear(); assertEquals(0, c.size)
    }

    // ------------------------------------------------------------------ i percorsi mirati

    /**
     * Fra due Paesi coi percorsi mirati sempre ([CrossBorderDirections.focusBeyond] negativo): al più
     * [CrossBorderDirections.focusMax] per ricerca di limite, poi la ricerca verso tutti va avanti; lo
     * stesso valico e lo stesso peso che senza tetto (dove i mirati erano di più).
     */
    @Test
    fun `targeted routes are capped, then the search toward all crossings goes on - same crossing`() {
        var trips = 0; var over = 0
        for (seed in 1L..16L) {
            val pairs = mapOf(setOf("a", "b") to crossings(900 + seed, 14, 1_500.0))
            val pts = Heidelberg.points(2, seed + 2500)
            val free = run(listOf("a", "b"), pairs, pts, "b", RequestContext(useCache = false)) { it.focusBeyond = -1.0; it.focusMax = Int.MAX_VALUE }
            if (free.doc == null) continue
            val capped = run(listOf("a", "b"), pairs, pts, "b", RequestContext(useCache = false)) { it.focusBeyond = -1.0; it.focusMax = 2 }
            trips++
            assertEquals(withoutVariant(features(free.doc)[0]), withoutVariant(features(capped.doc!!)[0]), "seme $seed: migliore diverso")
            assertTrue(close(capped.cb.lastWeight, free.cb.lastWeight))
            assertEquals(free.cb.lastCrossings.map { it.lat to it.lon }, capped.cb.lastCrossings.map { it.lat to it.lon })
            val rf = routes(free.logs); val rc = routes(capped.logs)
            println("seme $seed: mirati senza tetto $rf, col tetto 2 $rc")
            assertTrue(rc.all { it <= 2 }, "seme $seed: oltre il tetto: $rc")
            if (rf.any { it > 2 }) over++
        }
        assertTrue(trips >= 8, "viaggi: $trips")
        assertTrue(over >= 1, "nessun viaggio con più di 2 mirati senza tetto")
    }

    /**
     * I percorsi mirati vanno nella memoria dei pesi rapidi: la stessa richiesta di nuovo (o quella delle
     * varianti) non li rifà, nessuna ricerca della scelta; senza memoria si rifanno.
     */
    @Test
    fun `targeted routes come from the memory in the second request`() {
        var trips = 0
        for (seed in 1L..12L) {
            val pairs = mapOf(setOf("a", "b") to crossings(1000 + seed, 10, 2_000.0))
            val pts = Heidelberg.points(2, seed + 2700)
            val service = DirectionsService(g)
            val first = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), service = service) { it.focusBeyond = -1.0 }
            if (first.doc == null || routes(first.logs).sum() == 0) continue
            trips++
            val second = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), service = service) { it.focusBeyond = -1.0 }
            val (q, h, _) = queries(second.rc.stats.crossings)
            println("seme $seed: prima ${first.rc.stats.crossings} (mirati ${routes(first.logs)}) | seconda ${second.rc.stats.crossings}")
            assertEquals(withoutVariant(features(first.doc)[0]), withoutVariant(features(second.doc!!)[0]))
            assertEquals(0, q, "seme $seed: ricerche rifatte: ${second.rc.stats.crossings}")
            assertTrue(h > 0, "seme $seed: niente dalla memoria")
            assertTrue(routes(second.logs).sum() == 0, "seme $seed: mirati rifatti: ${routes(second.logs)}")
        }
        assertTrue(trips >= 5, "viaggi: $trips")
    }

    // ------------------------------------------------------------------ il Paese di mezzo per uscita

    /**
     * Nel transito la ricerca del Paese di mezzo va avanti per ogni uscita prima di esaminarla: con la
     * finestra più stretta (limiti della frontiera, deboli) le uscite esaminate non sono più di quelle
     * con la ricerca di mezzo portata a termine (limiti esatti); prima, con la finestra stretta, ogni
     * uscita col limite della frontiera sotto il migliore costava un pezzo finale e una ricerca. Stesso migliore.
     */
    @Test
    fun `the middle country's search goes on for each exit - no more exits than with exact bounds`() {
        var trips = 0
        for (seed in 1L..16L) {
            val pairs = mapOf(setOf("a", "b") to crossings(1100 + seed, 4), setOf("b", "d") to crossings(1200 + seed, 12, 1_500.0))
            val pts = Heidelberg.points(2, seed + 2900)
            val ids = listOf("a", "b", "d")
            val exact = run(ids, pairs, pts, "d", RequestContext(useCache = false)) { it.windowFactor = 1e9; it.windowSlack = 1e9 }
            if (exact.doc == null) continue
            val weak = run(ids, pairs, pts, "d", RequestContext(useCache = false)) { it.windowFactor = 1.0; it.windowSlack = 0.0 }
            val eExact = examined(exact.logs, "a→b→d") ?: continue
            val eWeak = examined(weak.logs, "a→b→d") ?: continue
            trips++
            assertEquals(withoutVariant(features(exact.doc)[0]), withoutVariant(features(weak.doc!!)[0]), "seme $seed: migliore diverso")
            assertTrue(close(weak.cb.lastWeight, exact.cb.lastWeight))
            println("seme $seed: uscite esaminate coi limiti esatti $eExact, con la finestra stretta $eWeak")
            assertTrue(eWeak <= eExact, "seme $seed: con la finestra stretta $eWeak uscite, coi limiti esatti $eExact")
        }
        assertTrue(trips >= 8, "viaggi: $trips")
    }

    // ------------------------------------------------------------------ i minimi garantiti dentro il tempo

    /**
     * I minimi garantiti delle varianti tra Paesi (fonte V per altri Paesi, fonti M e P del pezzo più lungo)
     * stanno nella coda di `budget_ms` che il passo 2 lascia libera, mai dopo (prima: una grazia di 1,5 s per
     * passata e 3 s al pezzo più lungo partivano a tempo finito, fino a +6 s: varianti automatiche oltre 25 s).
     */
    @Test
    fun `the guaranteed minimums stay inside the variants time`() {
        val ms = 1_000_000L
        // la coda: 1,5 s alla fonte V e 3 s alle fonti M e P, al più il 40 % del tempo
        assertEquals(4_500L, CrossBorderDirections.variantsTailMs(25_000))
        assertEquals(4_000L, CrossBorderDirections.variantsTailMs(10_000))
        assertEquals(0L, CrossBorderDirections.variantsTailMs(0))
        // il passo 2 finisce sempre prima della coda, anche dopo un passo 1 lento
        val start = 1_000 * ms
        for (t in 0L..25_000L step 250) {
            val d = CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + t * ms)
            assertTrue(d <= start + 20_500 * ms, "passo 1 di $t ms: passo 2 fino a ${(d - start) / ms} ms")
        }
        // il pezzo più lungo: il minimo prima della riserva delle catene V, mai oltre la fine del tempo
        assertEquals(7_000L, CrossBorderDirections.pieceBudgetMs(0, 10_000, 3_000))
        assertEquals(3_000L, CrossBorderDirections.pieceBudgetMs(0, 4_000, 3_000))
        assertEquals(2_000L, CrossBorderDirections.pieceBudgetMs(0, 2_000, 3_000))
        assertEquals(0L, CrossBorderDirections.pieceBudgetMs(0, -500, 0))
        assertEquals(1_000L, CrossBorderDirections.pieceBudgetMs(1, 4_000, 3_000))
        assertEquals(0L, CrossBorderDirections.pieceBudgetMs(1, 2_000, 3_000))
        // le fonti di un Paese: il riempimento dentro la fine rigida, le fonti 0,5 s prima
        val (s0, f0) = NavAlternativesEngine.stops(start + 3_000 * ms, start + 2_000 * ms)
        assertEquals(start + 1_500 * ms, s0); assertEquals(start + 2_000 * ms, f0)
        val (s1, f1) = NavAlternativesEngine.stops(start + 3_000 * ms, start + 10_000 * ms)
        assertEquals(start + 3_000 * ms, s1); assertEquals(start + 5_000 * ms, f1)
        val (s2, f2) = NavAlternativesEngine.stops(start + 3_000 * ms, Long.MAX_VALUE)
        assertEquals(start + 3_000 * ms, s2); assertEquals(start + 5_000 * ms, f2)

        // a tempo finito (1 ms) nessuna fase dopo la fine: niente minimo da 3 s né grazia, il migliore uguale
        val service = DirectionsService(g)
        var trips = 0
        for ((opt, seeds) in listOf(
            NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 1, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02) to 1L..6L,
            NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 1, sources = setOf(VariantSource.CROSSING, VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02) to 1L..6L)) {
            for (seed in seeds) {
                val pairs = mapOf(setOf("a", "b") to crossings(1300 + seed, 6))
                val pts = Heidelberg.points(2, seed + 3100)
                val plain = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), service = service)
                if (plain.doc == null) continue
                trips++
                val v = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), nav = opt, service = service)
                assertEquals(withoutVariant(features(plain.doc)[0]), withoutVariant(features(v.doc!!)[0]), "seme $seed: migliore diverso")
                val took = (meta(v.doc)["ms"] as Number).toLong()
                val logs = synchronized(v.logs) { v.logs.toList() }
                println("1 ms, ${opt.sources}, seme $seed: ${meta(v.doc)["variants"]} varianti in $took ms (${meta(v.doc)["note"]})")
                assertTrue(logs.none { "grazia" in it || "il minimo di" in it }, "seme $seed: una fase dopo la fine del tempo")
                // il minimo da 3 s o la grazia da 1,5 s lo porterebbero oltre
                assertTrue(took < 1_500, "seme $seed: varianti in $took ms con 1 ms di tempo")
            }
        }
        assertTrue(trips >= 4, "viaggi: $trips")

        // con il tempo normale le varianti arrivano come prima
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 25_000, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02)
        var with = 0; var n = 0
        for (seed in 1L..6L) {
            val pairs = mapOf(setOf("a", "b") to crossings(1300 + seed, 6))
            val pts = Heidelberg.points(2, seed + 3100)
            val v = run(listOf("a", "b"), pairs, pts, "b", RequestContext(), nav = opt, service = service)
            if (v.doc == null) continue
            n++
            if ((meta(v.doc)["variants"] as Int) > 0) with++
        }
        assertTrue(n >= 3 && with >= 1, "col tempo normale $with viaggi su $n con varianti")
    }

    // ------------------------------------------------------------------ i giri per un Paese vicino

    /**
     * Un viaggio in «a»: due vicini col centro vicino al viaggio ma senza valichi utili, uno col centro
     * lontano (un rettangolo grande, come la Germania da Innsbruck a Salisburgo) con due valichi lungo
     * la strada. Prima i primi due per distanza erano tagliati prima del filtro dei valichi e il giro
     * buono non si guardava mai; ora il filtro viene prima, come nell'app.
     */
    @Test
    fun `a neighbour with its centre far away but two useful crossings is looked at`() {
        val boxes = mapOf("a" to BBox(10.0, 12.0, 45.0, 47.0), "n1" to BBox(11.9, 12.5, 45.0, 46.7), "n2" to BBox(9.5, 10.1, 45.0, 46.7),
            "de" to BBox(10.5, 12.5, 46.6, 52.0))
        val p = 46.6 to 11.0; val q = 46.6 to 11.9
        val list = mapOf("n1" to listOf(Transfer(45.1, 11.95, 1), Transfer(45.3, 11.95, 1)), "n2" to listOf(Transfer(45.1, 10.05, 1), Transfer(45.3, 10.05, 1)),
            "de" to listOf(Transfer(46.65, 11.2, 1), Transfer(46.65, 11.7, 1)))
        val seqs = CrossBorderDirections.countrySequences(boxes.keys.toList(), { boxes[it] }, { _, _ -> true }, p.first, p.second, q.first, q.second, "a", "a")
        // tutti i vicini, i centri più vicini al viaggio prima
        assertEquals(listOf("n1", "n2", "de"), seqs.map { it[1] })
        val around = CrossBorderDirections.aroundSequences(seqs, { list.getValue(it) }, p.first, p.second, q.first, q.second)
        assertEquals(listOf(listOf("a", "de", "a")), around)
        // al più AROUND_MAX, nell'ordine
        val many = CrossBorderDirections.aroundSequences(listOf("x", "y", "z").map { listOf("a", it, "a") }, { list.getValue("de") }, p.first, p.second, q.first, q.second)
        assertEquals(listOf("x", "y").take(CrossBorderDirections.AROUND_MAX), many.map { it[1] })
    }
}
