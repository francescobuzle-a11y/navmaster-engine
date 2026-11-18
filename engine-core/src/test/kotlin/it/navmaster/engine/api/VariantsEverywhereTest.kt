package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Le varianti per tutti i viaggi, non solo fra Paesi in autostrada: le sequenze di Paesi lunghe (5-6
 * grafi), le sequenze in più del passo 2, i giri per un Paese vicino in un viaggio in un Paese solo
 * (A-B-A), la scelta per confine con la tappa nel Paese di mezzo, la fonte V prima delle varianti dei
 * pezzi e il tempo del passo 2 dopo un passo 1 lento. Su Heidelberg con Paesi finti (lo stesso grafo).
 */
class VariantsEverywhereTest {
    private val g get() = Heidelberg.graph
    private companion object {
        const val AVOID_PENALTY = ""","options":{"avoid_features":["tollways"]},"navmaster":{"avoid_penalty":true}"""
    }

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = doc["features"] as List<Map<String, Any?>>
    private fun variant(f: Map<*, *>) = ((f["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"] as Map<*, *>
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
    private fun crossings(seed: Long, n: Int, apart: Double = 5_500.0): List<Transfer> {
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(seed)
        val out = ArrayList<Transfer>()
        for (attempt in 0 until 8000) {
            if (out.size >= n) break
            val n0 = rnd.nextInt(bg.nodeCount)
            val node = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
            if (out.any { t -> DistanceCalc.haversine(t.lat, t.lon, bg.lat(node), bg.lon(node)) < apart }) continue
            out += Transfer(bg.lat(node), bg.lon(node), 1 + out.size % 3)
        }
        return out
    }

    // ------------------------------------------------------------------ sequenze di Paesi

    /** Rettangoli in fila (c1 … c7), ognuno tocca solo i vicini: la linea p → q li attraversa tutti. */
    private val row = (1..7).associate { i -> "c$i" to BBox(10.0 + i - 0.05, 11.0 + i + 0.05, 45.0, 46.0) }
    private fun rowLinked(a: String, b: String) = Math.abs(a.drop(1).toInt() - b.drop(1).toInt()) == 1

    @Test
    fun `country sequences - 5 and 6 graphs only when no sequence of 4 exists`() {
        val ids = row.keys.toList()
        fun seqs(from: String, to: String, extra: Int = 0) = CrossBorderDirections.countrySequences(ids, { row[it] }, ::rowLinked,
            45.5, 10.0 + from.drop(1).toInt() + 0.5, 45.5, 10.0 + to.drop(1).toInt() + 0.5, from, to, extra)
        // 4 grafi: come sempre
        assertEquals(listOf(listOf("c1", "c2", "c3", "c4")), seqs("c1", "c4"))
        // 5 e 6 grafi: prima nessuna sequenza (nessun percorso, anche per il migliore), ora quella
        assertEquals(listOf(listOf("c1", "c2", "c3", "c4", "c5")), seqs("c1", "c5"))
        assertEquals(listOf(listOf("c1", "c2", "c3", "c4", "c5", "c6")), seqs("c1", "c6"))
        // oltre 6 grafi no
        assertEquals(emptyList(), seqs("c1", "c7"))
    }

    @Test
    fun `country sequences - the first 6 never change, extra ones only for step 2`() {
        // una griglia 3×3 di rettangoli che si toccano tutti coi vicini (anche in diagonale): molte sequenze da g00 a g22
        val boxes = HashMap<String, BBox>()
        for (r in 0..2) for (c in 0..2) boxes["g$r$c"] = BBox(10.0 + c - 0.05, 11.0 + c + 0.05, 45.0 + r - 0.05, 46.0 + r + 0.05)
        val ids = boxes.keys.sorted()
        fun linked(a: String, b: String) = boxes.getValue(a).intersects(boxes.getValue(b))
        val base = CrossBorderDirections.countrySequences(ids, { boxes[it] }, ::linked, 45.5, 10.5, 47.5, 12.5, "g00", "g22")
        val more = CrossBorderDirections.countrySequences(ids, { boxes[it] }, ::linked, 45.5, 10.5, 47.5, 12.5, "g00", "g22", extra = CrossBorderDirections.EXTRA_SEQUENCES)
        assertEquals(6, base.size)
        assertEquals(base, more.take(6), "le prime 6 (il passo 1, il migliore) sono le stesse")
        assertTrue(more.size in 7..6 + CrossBorderDirections.EXTRA_SEQUENCES, "sequenze in più: ${more.size - 6}")
        for (s in more.drop(6)) { assertTrue(s !in base); assertEquals("g00", s.first()); assertEquals("g22", s.last()); assertTrue(s.size <= 4) }
        // in ordine di lunghezza
        assertEquals(more.drop(6).map { it.size }, more.drop(6).map { it.size }.sorted())
    }

    @Test
    fun `country sequences - a trip in one country looks at the neighbours as A-B-A`() {
        val boxes = mapOf("a" to BBox(10.0, 12.0, 45.0, 47.0), "b" to BBox(11.9, 14.0, 46.0, 48.0), "far" to BBox(30.0, 31.0, 60.0, 61.0), "c" to BBox(8.0, 10.1, 45.0, 47.0))
        fun linked(x: String, y: String) = setOf(x, y) != setOf("a", "c")
        val s = CrossBorderDirections.countrySequences(boxes.keys.toList(), { boxes[it] }, ::linked, 46.5, 11.0, 46.8, 11.8, "a", "a")
        // b vicino e collegato; c senza valichi; far lontano
        assertEquals(listOf(listOf("a", "b", "a")), s)
        // il filtro dei valichi: due valichi in luoghi diversi entro 1,2 volte la linea
        val p = 46.5 to 11.0; val q = 46.5 to 11.6
        val near = listOf(Transfer(46.55, 11.2, 1), Transfer(46.55, 11.4, 1))
        val far = listOf(Transfer(47.5, 11.2, 1), Transfer(47.5, 11.4, 1))
        val samePlace = listOf(Transfer(46.55, 11.2, 1), Transfer(46.551, 11.201, 1))
        assertTrue(Transfers.aroundOk(near, p.first, p.second, q.first, q.second))
        assertTrue(!Transfers.aroundOk(far, p.first, p.second, q.first, q.second))
        assertTrue(!Transfers.aroundOk(samePlace, p.first, p.second, q.first, q.second), "rientro allo stesso valico")
    }

    // ------------------------------------------------------------------ un Paese solo: il giro per un Paese vicino

    /**
     * Un viaggio con tutti e due i punti nel Paese «a», «b» vicino (lo stesso grafo, valichi finti):
     * senza varianti la risposta è quella del Paese (il ramo di sempre); con le varianti il migliore
     * resta identico e fra le catene della fonte V ci sono i giri a-b-a (rientro a un altro valico),
     * entro il tetto di peso, come varianti «per b» coi valichi e i Paesi.
     */
    @Test
    fun `a trip in one country gets variants through a neighbouring country, the best unchanged`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0; var withAround = 0; var keptAround = 0
        for (seed in 1L..16L) {
            val transfers = crossings(40 + seed, 14, apart = 3_000.0)
            val graphs = listOf(CountryGraph("a", service), CountryGraph("b", service))
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(graphs, { _, _ -> transfers }, { logs += it })
            val pts = Heidelberg.points(2, seed + 1200)
            val body = Heidelberg.body(pts)
            val countries = listOf("a", "a")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries) } catch (e: ApiException) { continue }
            // senza varianti: il ramo di un Paese solo, la risposta del servizio
            assertEquals(Json.stringify(features(service.handle(DirectionsRequest.fromJson("driving-hgv", body), ResponseFormat.GEOJSON))[0]),
                Json.stringify(features(plainDoc)[0]))
            val plain = features(plainDoc)[0]
            trips++
            logs.clear()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries)
            val fs = features(doc)
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]), "seme $seed: il migliore cambia")
            val around = cb.lastChains.drop(1)
            for (c in around) {
                assertEquals(listOf("a", "b", "a"), c.countries)
                assertEquals(2, c.via.size)
                assertTrue(DistanceCalc.haversine(c.via[0].lat, c.via[0].lon, c.via[1].lat, c.via[1].lon) >= Transfers.PLACE, "rientro allo stesso valico")
                assertTrue(c.exact < cb.lastChains[0].exact * (1 + PenaltyCandidates.CAP_SLACK * (opt.maxFactor - 1)), "oltre il tetto")
            }
            if (around.isNotEmpty()) withAround++
            for ((i, f) in fs.withIndex()) {
                assertEquals(2, ((f["properties"] as Map<*, *>)["way_points"] as List<*>).size)
                val v = variant(f)
                assertEquals(i, v["rank"])
                if (v["source"] == "crossing") {
                    keptAround++
                    assertEquals(listOf("a", "b", "a"), v["countries"])
                    assertTrue((v["label"] as String).contains(", per "), "titolo senza il Paese: ${v["label"]}")
                    assertTrue((v["factor"] as Double) <= opt.maxFactor + 1e-3)
                }
            }
            println("un Paese, seme $seed: giri ${around.size}, ${fs.size - 1} varianti (${fs.drop(1).joinToString { "${variant(it)["source"]} ${variant(it)["label"]}" }}; ${meta(doc)["candidates"]})")
        }
        println("un Paese: $trips viaggi, $withAround con giri per b, $keptAround varianti per b")
        assertTrue(trips >= 8, "viaggi: $trips")
        assertTrue(withAround > 0, "nessun giro per il Paese vicino")
        assertTrue(keptAround > 0, "nessuna variante per il Paese vicino fra le tenute")
    }

    // ------------------------------------------------------------------ grafi per le sole varianti

    /**
     * I grafi aperti solo per le varianti ([CrossBorderDirections.bestGraphs]) non cambiano il
     * migliore neanche fuori dal passo 1: il grafo di un punto senza Paese indicato e i Paesi dove
     * «evita» diventa penalità si scelgono fra i grafi del migliore. Qui il grafo «c», solo per le
     * varianti, è il primo dell'elenco: prima un punto senza Paese finiva in «c» (stesso aggancio, a
     * parità vince il primo) e la richiesta delle varianti aveva un altro migliore.
     */
    @Test
    fun `graphs for the variants only change neither the points' graphs nor where avoid is a penalty`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..8L) {
            val ab = crossings(300 + seed, 3); val ac = crossings(320 + seed, 3); val bc = crossings(340 + seed, 3)
            val pairs = mapOf(setOf("a", "b") to ab, setOf("a", "c") to ac, setOf("b", "c") to bc)
            val logBest = ArrayList<String>(); val logVar = java.util.Collections.synchronizedList(ArrayList<String>())
            val best = CrossBorderDirections(listOf("a", "b").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logBest += it })
            val withExtra = CrossBorderDirections(listOf("c", "a", "b").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logVar += it })
            withExtra.bestGraphs = setOf("a", "b")
            val body = Heidelberg.body(Heidelberg.points(2, seed + 2100), AVOID_PENALTY)
            // il primo punto senza Paese indicato: lo sceglie l'aggancio
            val countries = listOf(null, "b")
            val plainDoc = try { best.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext(useCache = false)) } catch (e: ApiException) { continue }
            trips++
            val doc = withExtra.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, RequestContext(useCache = false))
            assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            assertEquals((plainDoc["metadata"] as Map<*, *>)["countries"], (doc["metadata"] as Map<*, *>)["countries"], "seme $seed")
            fun penalty(l: List<String>) = l.filter { it.startsWith("evita come penalità in") }.map { it.substringBefore(" (") }
            assertEquals(penalty(logBest), penalty(logVar), "seme $seed: «evita» come penalità in altri Paesi")
            assertTrue(logVar.none { it.startsWith("punti nei grafi:") && "c" in it.substringAfter(":").split(",").map { x -> x.trim() } }, "seme $seed: un punto nel grafo delle sole varianti")
        }
        assertTrue(trips >= 5, "viaggi: $trips")
    }

    // ------------------------------------------------------------------ tappa nel Paese di mezzo

    /**
     * Tre Paesi coi soli confini diretti e una tappa in quello di mezzo (Milano → Innsbruck → München):
     * senza varianti la scelta dei due valichi insieme ([CrossBorderDirections] chooseTwo); con le
     * varianti la scelta per confine, che registra le altre catene: la fonte V non tace più, il
     * migliore è lo stesso.
     */
    @Test
    fun `with a stop in the middle country the variants request weighs the other crossings`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0; var withV = 0
        for (seed in 1L..10L) {
            val ab = crossings(60 + seed, 4); val bc = crossings(80 + seed, 4)
            val pairs = mapOf(setOf("a", "b") to ab, setOf("b", "c") to bc)
            val graphs = listOf("a", "b", "c").map { CountryGraph(it, service) }
            val cb = CrossBorderDirections(graphs, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() })
            val pts = Heidelberg.points(3, seed + 1400)
            val body = Heidelberg.body(pts)
            val countries = listOf("a", "b", "c")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries) } catch (e: ApiException) { continue }
            val plain = features(plainDoc)[0]
            val twoBefore = cb.twoChoices
            val bestX = cb.lastCrossings.map { it.lat to it.lon }
            trips++
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries)
            val fs = features(doc)
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]), "seme $seed: il migliore cambia")
            val cand = meta(doc)["candidates"] as String
            assertTrue("V no (nessun'altra catena)" !in cand || cb.lastChains.size <= 1, "seme $seed: $cand")
            if (cand.contains(Regex("V [1-9]"))) withV++
            println("tappa nel Paese di mezzo, seme $seed: ${fs.size - 1} varianti; $cand; valichi del migliore $bestX; scelte insieme prima ${twoBefore}")
        }
        assertTrue(trips >= 5, "viaggi: $trips")
        assertTrue(withV > 0, "la fonte V non trova mai un altro valico con la tappa nel Paese di mezzo")
    }

    // ------------------------------------------------------------------ l'ordine delle fonti e il tempo

    /** Le catene V per altri Paesi si pesano prima delle varianti M/P dei pezzi (che prima le lasciavano senza tempo). */
    @Test
    fun `the crossings through other countries are looked at before the pieces' variants`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
        var checked = 0
        for (seed in 1L..16L) {
            val transfers = crossings(40 + seed, 14, apart = 3_000.0)
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> transfers }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(2, seed + 1200))
            try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), listOf("a", "a"), RequestContext(log = { logs += it })) } catch (e: ApiException) { continue }
            val v = logs.indexOfFirst { it.startsWith("varianti: V Paesi") }
            val m = logs.indexOfFirst { it.startsWith("varianti: M ") || it.startsWith("varianti: M rete") || it.startsWith("varianti: P ") }
            if (v < 0 || m < 0) continue
            checked++
            assertTrue(v < m, "seme $seed: la fonte V dopo le varianti dei pezzi")
        }
        assertTrue(checked > 0, "nessun viaggio con le due fonti")
    }

    /**
     * Il posto per un altro Paese anche fra livelli diversi: quando nessuna tenuta passa per altri Paesi,
     * nessuna candidata V per altri Paesi resta fuori per «posti finiti» (ha preso l'ultimo posto, o è
     * scartata per una ragione della regola).
     */
    @Test
    fun `a variant through other countries is never left out only for lack of places`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0; var withOther = 0; var swapped = 0
        for (seed in 1L..10L) {
            val ab = crossings(21 + seed * 10, 3); val bc = crossings(22 + seed * 10, 3); val cd = crossings(23 + seed * 10, 3)
            val pairs = mapOf(setOf("a", "b") to ab, setOf("b", "c") to bc, setOf("c", "d") to cd, setOf("b", "e") to crossings(24 + seed * 10, 3), setOf("e", "d") to crossings(25 + seed * 10, 3))
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(listOf("a", "b", "c", "d", "e").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(2, seed + 900))
            val doc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), listOf("a", "d")) } catch (e: ApiException) { continue }
            trips++
            val bestSeq = cb.lastChains.first().countries
            val kept = features(doc).drop(1).map { variant(it) }
            val otherKept = kept.any { it["source"] == "crossing" && (it["countries"] as List<*>) != bestSeq }
            val otherLines = logs.filter { it.startsWith("varianti: V Paesi") && "→" !in it && !it.contains(bestSeq.joinToString("-") + " via") }
            if (otherLines.isNotEmpty()) withOther++
            if (logs.any { it.startsWith("varianti: posto per altri Paesi") }) swapped++
            if (!otherKept && kept.size == opt.count - 1) for (l in otherLines) assertTrue("posti finiti" !in l, "seme $seed: $l")
            println("posto per altri Paesi, seme $seed: ${kept.joinToString { "${it["source"]} ${it["label"]} livello ${it["tier"]}" }}; ${otherLines.joinToString(" / ")}")
        }
        println("posto per altri Paesi: $trips viaggi, $withOther con candidate per altri Paesi, $swapped scambi")
        assertTrue(trips >= 5)
    }

    @Test
    fun `the time of step 2 after a slow step 1`() {
        val ms = 1_000_000L
        val start = 1_000 * ms
        // tempo 25 s, passo 2 fino a 15 s dall'inizio: un passo 1 di 3 s non cambia nulla
        assertEquals(start + 15_000 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 3_000 * ms))
        // un passo 1 di 10 s: ancora nulla, alle fonti M e P restano tutti i loro 10 s
        assertEquals(start + 15_000 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 10_000 * ms))
        // un passo 1 di 12 s: 0,2 × 25 s dalla sua fine, cioè 17 s (alle fonti M e P 8 s, più di metà di quel che resta)
        assertEquals(start + 17_000 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 12_000 * ms))
        // un passo 1 di 14 s: 19 s, alle fonti M e P 6 s (prima del rilievo il passo 2 arrivava a 24 s)
        assertEquals(start + 19_000 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 14_000 * ms))
        // un passo 1 di 20 s: metà di quel che resta, 22,5 s; l'altra metà alle varianti dei pezzi
        assertEquals(start + 22_500 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 20_000 * ms))
        // un passo 1 oltre il tempo meno 1 s: mai oltre 24 s
        assertEquals(start + 24_000 * ms, CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, start + 24_500 * ms))
        // le fonti M e P hanno sempre almeno metà del tempo che resta dopo il passo 1
        for (t in 0L..25_000L step 250) {
            val now = start + t * ms
            val d = CrossBorderDirections.topDeadlineAfter(start, 25_000, start + 15_000 * ms, now)
            if (d > start + 15_000 * ms) assertTrue(start + 25_000 * ms - d >= (start + 25_000 * ms - now) / 2, "passo 1 di $t ms")
        }
    }

    /**
     * Dopo un passo 1 più lungo del tempo del passo 2 (memoria fredda, tempo corto) il passo 2 ha
     * ancora tempo ([CrossBorderDirections] extendTopBudget), e il migliore è lo stesso.
     */
    @Test
    fun `a slow step 1 still leaves time to step 2`() {
        val service = DirectionsService(g)
        var extended = 0; var trips = 0
        for (seed in 1L..8L) {
            val ab = crossings(21 + seed * 10, 3); val bc = crossings(22 + seed * 10, 3); val cd = crossings(23 + seed * 10, 3)
            val pairs = mapOf(setOf("a", "b") to ab, setOf("b", "c") to bc, setOf("c", "d") to cd, setOf("b", "e") to crossings(24 + seed * 10, 3), setOf("e", "d") to crossings(25 + seed * 10, 3))
            val graphs = listOf("a", "b", "c", "d", "e").map { CountryGraph(it, service) }
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(graphs, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(2, seed + 900))
            val countries = listOf("a", "d")
            val plain = try { features(cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext(useCache = false)))[0] } catch (e: ApiException) { continue }
            trips++
            // tempo minimo (1 s): il passo 1 a freddo supera i suoi 0,6 s
            val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 1_000, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
            logs.clear()
            val t0 = System.nanoTime()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, RequestContext(useCache = false))
            val ms = (System.nanoTime() - t0) / 1_000_000
            assertEquals(withoutVariant(plain), withoutVariant(features(doc)[0]))
            if (logs.any { "tempo delle altre catene allungato" in it }) extended++
            println("passo 1 lento, seme $seed: $ms ms; ${logs.filter { it.startsWith("varianti: tempo") || it.startsWith("varianti: catene") }.joinToString(" / ")}")
        }
        println("passo 2 allungato in $extended viaggi su $trips")
        assertTrue(trips >= 4)
    }
}
