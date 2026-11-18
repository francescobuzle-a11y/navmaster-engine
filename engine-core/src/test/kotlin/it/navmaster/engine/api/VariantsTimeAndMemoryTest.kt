package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.util.DistanceCalc
import java.util.Random
import java.util.concurrent.CountDownLatch
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Il tempo e la memoria della richiesta delle varianti (su Heidelberg con Paesi finti, lo stesso grafo):
 * il passo 1 comincia dal migliore ricordato (un transito non si rifà mentre la sequenza diretta lavora
 * sull'altro thread: le «ricerche rifatte» di (i)); col migliore ricordato gli altri valichi fra due Paesi
 * stanno nel tempo delle varianti (Lione → Francoforte senza autostrade: 14 s di ricerche per la sola fonte
 * V, le varianti dei pezzi senza tempo); con una tappa le varianti che cambiano un tratto solo.
 */
class VariantsTimeAndMemoryTest {
    private val g get() = Heidelberg.graph

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = doc["features"] as List<Map<String, Any?>>

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

    /** «N ricerche (M dalla memoria, K per le varianti)» della riga dei valichi: le ricerche rifatte, N − K (-1 se manca). */
    private fun redone(line: String): Int {
        val m = Regex("""(\d+) ricerche \((\d+) dalla memoria(?:, (\d+) per le varianti)?\)""").find(line) ?: return -1
        return m.groupValues[1].toInt() - m.groupValues[3].ifEmpty { "0" }.toInt()
    }

    /**
     * Il passo 1 della richiesta delle varianti comincia dal migliore della richiesta di prima (la memoria
     * del passo 1): il transito per un terzo Paese, che la memoria dà per «nessuna coppia sotto il migliore»,
     * si riprende senza ricerche anche quando la sequenza diretta (il migliore) lavora ancora sull'altro
     * thread. Prima il suo tetto era ancora infinito, l'attesa finiva e il transito si rifaceva (CI: Milano →
     * Innsbruck → Monaco 3 ricerche rifatte, Lione → Francoforte senza autostrade 18 s): qui la sequenza
     * diretta aspetta apposta la fine del transito. Il migliore non cambia.
     */
    @Test
    fun `the variants request starts step 1 at the remembered best - the transit is not redone`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try {
            var trips = 0; var noneBelow = 0
            for (seed in 1L..24L) {
                val pairs = mapOf(setOf("a", "c") to crossings(500 + seed, 4), setOf("a", "b") to crossings(520 + seed, 3), setOf("b", "c") to crossings(540 + seed, 3))
                val logs = java.util.Collections.synchronizedList(ArrayList<String>())
                val cb = CrossBorderDirections(listOf("a", "b", "c").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
                val body = Heidelberg.body(Heidelberg.points(2, seed + 3100))
                val countries = listOf("a", "c")
                // la richiesta del migliore, una sequenza dopo l'altra: il transito sotto il peso del diretto
                cb.parallelSequences = false
                val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext()) } catch (e: ApiException) { continue }
                if (cb.lastChains.firstOrNull()?.countries != listOf("a", "c")) continue
                if (logs.any { it.contains("due valichi insieme, transito") && it.contains("nessuno (tetto ") }) noneBelow++
                // le varianti: due thread, la sequenza diretta (0) aspetta la fine del transito (1)
                cb.parallelSequences = true
                val transitDone = CountDownLatch(1)
                val threads = java.util.Collections.synchronizedSet(HashSet<String>())
                cb.onSequence = { i, start ->
                    threads += Thread.currentThread().name
                    if (i == 0 && start) transitDone.await(5, TimeUnit.SECONDS)
                    if (i == 1 && !start) transitDone.countDown()
                }
                logs.clear()
                val rc = RequestContext(CancelToken(), RequestContext.MAIN, pool, SearchSlots(2), null)
                val doc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, rc) } finally { cb.onSequence = null }
                assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
                assertEquals(listOf("a", "c"), cb.lastChains.first().countries, "seme $seed")
                if (threads.size < 2) continue
                trips++
                val line = rc.stats.crossings
                assertEquals(0, redone(line), "seme $seed: ricerche rifatte nel passo 1: $line\n" + logs.joinToString("\n"))
                assertTrue(logs.any { it.startsWith("  passo 1 dalla memoria") }, "seme $seed: il passo 1 non comincia dalla memoria")
                // il transito dalla memoria, mai ricalcolato
                assertTrue(logs.none { it.contains("due valichi insieme, transito (") }, "seme $seed: transito ricalcolato\n" + logs.joinToString("\n"))
                println("passo 1 dalla memoria, seme $seed: $line")
            }
            println("passo 1 dalla memoria: $trips viaggi con due thread, $noneBelow con «nessuna coppia» del transito nella memoria")
            assertTrue(trips >= 3, "viaggi: $trips")
            assertTrue(noneBelow >= 1, "nessun transito ricordato come «nessuna coppia sotto il migliore»")
        } finally { pool.shutdownNow() }
    }

    /** Il viaggio a → c (diretto o in transito per b) di [seed], i suoi valichi e il servizio comune; null se non c'è percorso. */
    private class Transit(val cb: CrossBorderDirections, val logs: MutableList<String>, val body: Map<String, Any?>, val plainDoc: Map<String, Any?>)

    private fun transit(service: DirectionsService, seed: Long, ac: Int = 4, best: List<String> = listOf("a", "c"), extra: String = ""): Transit? {
        val pairs = mapOf(setOf("a", "c") to crossings(500 + seed, ac), setOf("a", "b") to crossings(520 + seed, 3), setOf("b", "c") to crossings(540 + seed, 3))
        val logs = java.util.Collections.synchronizedList(ArrayList<String>())
        val cb = CrossBorderDirections(listOf("a", "b", "c").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
        cb.parallelSequences = false
        val body = Heidelberg.body(Heidelberg.points(2, seed + 3100), extra)
        val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf("a", "c"), RequestContext()) } catch (e: ApiException) { return null }
        if (cb.lastChains.firstOrNull()?.countries != best) return null
        return Transit(cb, logs, body, plainDoc)
    }

    /**
     * La memoria del passo 1 ha solo il peso e la sequenza del migliore: i suoi valichi si ritrovano con le ricerche
     * della sua sequenza. Con le altre memorie svuotate (scelte dei valichi, tratti, pesi rapidi: sul tablet limiti
     * propri, grafo riaperto) quelle ricerche sono vere e restano «rifatte» in (i), non per le varianti: prima tutte le ricerche del
     * passo 1 passavano per le varianti e (i) non poteva più accorgersene.
     */
    @Test
    fun `with the best remembered but the leg memory emptied the searches of the best sequence are not for the variants`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0; var exposed = 0
        for (seed in 1L..24L) for ((ac, best) in listOf(4 to listOf("a", "c"), 1 to listOf("a", "b", "c"))) for (avoid in listOf("", "tollways")) {
            val extra = if (avoid.isEmpty()) "" else ""","options":{"avoid_features":["$avoid"]}"""
            val t = transit(service, seed, ac, best, extra) ?: continue
            // restano solo le memorie del passo 1 (peso e sequenza): le scelte dei valichi, i tratti e i pesi rapidi via
            val cache = service.transitCache
            for (k in cache.keys) if ((k as? List<*>)?.firstOrNull() != "passo 1") cache.remove(k)
            service.clearLegAndQuickCaches()
            t.logs.clear()
            val rc = RequestContext()
            val doc = t.cb.handle(DirectionsRequest.fromJson("driving-hgv", t.body).withNavAlternatives(opt), listOf("a", "c"), rc)
            assertEquals(withoutVariant(features(t.plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            if (t.logs.none { it.startsWith("  passo 1 dalla memoria") }) continue
            trips++
            val m = t.logs.firstNotNullOfOrNull { Regex("""passo 1 col migliore dalla memoria: (\d+) ricerche delle altre sequenze per le catene della fonte V, (\d+) della sequenza del migliore""").find(it) }
            assertTrue(m != null, "seme $seed: manca la riga del passo 1\n" + t.logs.joinToString("\n"))
            val again = m.groupValues[2].toInt()
            if (again > 0) exposed++
            assertTrue(redone(rc.stats.crossings) >= again, "seme $seed ${best.joinToString("-")} $avoid: le ricerche per ritrovare il migliore contate per le varianti: ${rc.stats.crossings}")
            println("memoria dei tratti svuotata, seme $seed ${best.joinToString("-")} $avoid: ${rc.stats.crossings}, $again per ritrovare il migliore")
        }
        println("memoria dei tratti svuotata: $trips viaggi, $exposed con ricerche per ritrovare il migliore")
        assertTrue(trips >= 3, "viaggi: $trips")
        assertTrue(exposed >= 2, "nessuna ricerca vera per ritrovare il migliore contata come tale: $exposed su $trips")
    }

    /**
     * Una memoria del passo 1 che non torna (peso diverso da quello ritrovato) si toglie: la scelta da capo la rimette
     * giusta e la richiesta dopo parte di nuovo dalla memoria. Prima [TransitCache.put] non sostituiva il peso finito
     * sbagliato e ogni richiesta delle varianti faceva il passo 1 due volte.
     */
    @Test
    fun `a step 1 memory that does not match is replaced`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..24L) {
            val t = transit(service, seed) ?: continue
            val cache = service.transitCache
            val key = cache.keys.lastOrNull { k -> (k as? List<*>)?.firstOrNull() == "passo 1" && cache.get(k)?.let { it.w.isFinite() && it.seq >= 0 } == true } ?: continue
            val e = cache.get(key)!!
            // la voce sbagliata: un peso un po' più basso
            cache.remove(key); cache.put(key, null, null, e.w * 0.98, e.w * 0.98, e.seq)
            t.logs.clear()
            val doc = t.cb.handle(DirectionsRequest.fromJson("driving-hgv", t.body).withNavAlternatives(opt), listOf("a", "c"), RequestContext())
            assertEquals(withoutVariant(features(t.plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            if (t.logs.none { it.contains("da capo senza memoria") }) continue
            trips++
            assertEquals(e.w, cache.get(key)?.w, "seme $seed: la memoria del passo 1 sbagliata è rimasta")
            assertEquals(e.seq, cache.get(key)?.seq, "seme $seed")
            // la richiesta dopo parte dalla memoria giusta, una volta sola
            t.logs.clear()
            val doc2 = t.cb.handle(DirectionsRequest.fromJson("driving-hgv", t.body).withNavAlternatives(opt), listOf("a", "c"), RequestContext())
            assertEquals(withoutVariant(features(t.plainDoc)[0]), withoutVariant(features(doc2)[0]), "seme $seed: il migliore cambia")
            assertTrue(t.logs.none { it.contains("da capo senza memoria") }, "seme $seed: di nuovo da capo\n" + t.logs.joinToString("\n"))
        }
        assertTrue(trips >= 3, "viaggi: $trips")
    }

    /**
     * Con la tappa nel Paese di mezzo la richiesta del migliore sceglie i due valichi insieme, quella delle varianti
     * per confine (per avere le altre catene di ogni confine): se la memoria ha i due valichi insieme il migliore è
     * noto e le ricerche per confine sono per la fonte V, non «rifatte» (CI: Milano → Innsbruck → Monaco, 3-19).
     */
    @Test
    fun `with a stop in the middle country the per-border searches of the variants request are for the variants`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..10L) {
            val pairs = mapOf(setOf("a", "b") to crossings(60 + seed, 4), setOf("b", "c") to crossings(80 + seed, 4))
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(listOf("a", "b", "c").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(3, seed + 1400))
            val countries = listOf("a", "b", "c")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext()) } catch (e: ApiException) { continue }
            if (logs.none { it.contains("due valichi insieme (a→b→c)") }) continue
            logs.clear()
            val rc = RequestContext()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, rc)
            assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            trips++
            assertTrue(logs.any { it.startsWith("varianti: i due valichi del migliore dalla memoria") }, "seme $seed\n" + logs.joinToString("\n"))
            assertTrue(logs.any { it.startsWith("varianti: valichi per confine uguali a quelli della memoria") }, "seme $seed\n" + logs.joinToString("\n"))
            assertEquals(0, redone(rc.stats.crossings), "seme $seed: ${rc.stats.crossings}")
            println("tappa nel Paese di mezzo, seme $seed: ${rc.stats.crossings}")
        }
        assertTrue(trips >= 4, "viaggi: $trips")
    }

    /**
     * Le ricerche per confine si contano per le varianti solo se i valichi trovati sono quelli scelti insieme nella
     * memoria: con una voce che dice altri valichi (qui spostata apposta) il migliore della risposta l'hanno deciso le
     * scelte per confine, le loro ricerche restano del migliore e il registro lo dice. Prima bastava la voce.
     */
    @Test
    fun `with a stop in the middle country the per-border searches are for the variants only when the crossings match the memory`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..10L) {
            val pairs = mapOf(setOf("a", "b") to crossings(60 + seed, 4), setOf("b", "c") to crossings(80 + seed, 4))
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(listOf("a", "b", "c").map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(3, seed + 1400))
            val countries = listOf("a", "b", "c")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext()) } catch (e: ApiException) { continue }
            if (logs.none { it.contains("due valichi insieme (a→b→c)") }) continue
            // la scelta dei due valichi insieme nella memoria, col primo valico spostato
            val cache = service.transitCache
            var moved = 0
            for (k in cache.keys) {
                if ((k as? List<*>)?.firstOrNull() == "passo 1") continue
                val e = cache.get(k) ?: continue
                if (e.x == null || e.y == null || !e.w.isFinite()) continue
                cache.remove(k); cache.put(k, Transfer(e.x.lat + 0.01, e.x.lon, e.x.rank), e.y, e.w, e.limit); moved++
            }
            if (moved == 0) continue
            logs.clear()
            val rc = RequestContext()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, rc)
            assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            val m = logs.firstNotNullOfOrNull { Regex("""^varianti: valichi per confine diversi da quelli della memoria .*: le (\d+) ricerche del passo 1 restano del migliore""").find(it) }
            assertTrue(m != null, "seme $seed\n" + logs.joinToString("\n"))
            val kept = m.groupValues[1].toInt()
            assertTrue(redone(rc.stats.crossings) >= kept, "seme $seed: le scelte per confine contate per le varianti: ${rc.stats.crossings}")
            if (kept > 0) trips++
            println("valichi della memoria diversi, seme $seed: ${rc.stats.crossings}")
        }
        assertTrue(trips >= 3, "viaggi: $trips")
    }

    /**
     * Col migliore fra due Paesi dalla memoria tutto il resto della scelta (le ricerche verso i passaggi, i
     * percorsi mirati, i pezzi) serve solo alla fonte V e sta nel tempo delle varianti: a tempo già finito
     * nessuna ricerca, il migliore resta e la risposta arriva. Prima le ricerche verso i passaggi e i
     * percorsi mirati andavano avanti senza tempo (Lione → Francoforte senza autostrade: 14 s su 10).
     */
    @Test
    fun `with the best remembered the other crossings of two countries stay within the variants time`() {
        val service = DirectionsService(g)
        var trips = 0; var looked = 0
        for (seed in 1L..16L) {
            val transfers = crossings(600 + seed, 14, apart = 3_000.0)
            val logs = java.util.Collections.synchronizedList(ArrayList<String>())
            val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> transfers }, { logs += it })
            val body = Heidelberg.body(Heidelberg.points(2, seed + 3300))
            val countries = listOf("a", "b")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries, RequestContext()) } catch (e: ApiException) { continue }
            trips++
            // con abbastanza tempo la fonte V guarda gli altri valichi (le stesse ricerche di prima)
            val roomy = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 30_000, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
            logs.clear()
            val doc0 = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(roomy), countries, RequestContext())
            assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc0)[0]), "seme $seed: il migliore cambia")
            if (logs.any { it.contains("valico scelto: dalla memoria") } && logs.any { Regex("valico scelto: [0-9.,]+ peso \\d+ fra \\d+ passaggi").containsMatchIn(it) }) looked++
            // a tempo finito (budget di 1 ms): il migliore dalla memoria, nessuna ricerca per gli altri valichi
            val none = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 1, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
            logs.clear()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(none), countries, RequestContext())
            assertEquals(withoutVariant(features(plainDoc)[0]), withoutVariant(features(doc)[0]), "seme $seed: il migliore cambia")
            assertTrue(logs.any { it.contains("valico scelto: dalla memoria") }, "seme $seed: il migliore non viene dalla memoria\n" + logs.joinToString("\n"))
            assertTrue(logs.none { Regex("valico scelto: [0-9.,]+ peso \\d+ fra \\d+ passaggi").containsMatchIn(it) },
                "seme $seed: ricerche per gli altri valichi a tempo finito\n" + logs.joinToString("\n"))
            assertTrue(logs.any { it.startsWith("varianti: valichi nessun altro") && "tempo finito" in it }, "seme $seed\n" + logs.joinToString("\n"))
            // dove va il tempo: la riga dei tempi tra Paesi
            assertTrue(logs.any { it.startsWith("varianti: tempi tra Paesi:") }, "seme $seed: manca la riga dei tempi")
        }
        println("altri valichi nel tempo delle varianti: $trips viaggi, $looked guardati col tempo")
        assertTrue(trips >= 8, "viaggi: $trips")
        assertTrue(looked >= 4, "la fonte V non guarda più gli altri valichi col tempo: $looked")
    }

    /**
     * Con una tappa (Canneto → Cremona → Piacenza evitando pedaggi o autostrade) le varianti trovate cambiavano
     * tutte lo stesso tratto (o due tratti insieme, oltre il tempo): finché mancano varianti, le penalità su un
     * tratto solo (tutti gli archi dei percorsi trovati in quel tratto, gli altri tratti restano del migliore)
     * e le combinazioni fra i tratti dei percorsi già trovati (in RAM, nessuna ricerca). Varianti nuove, il
     * migliore uguale, ogni tratto lecito.
     */
    @Test
    fun `with a stop the variants change one leg alone - penalties on one leg and legs mixed`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.2, maxFactor = 1.5, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02)
        var trips = 0; var keptLeg = 0; var keptMix = 0; var oneLeg = 0
        for (seed in 1L..20L) for (avoid in listOf("", "tollways", "highways")) for (pt in listOf(false, true)) {
            val pts = Heidelberg.points(3, seed + 700)
            val extra = (if (avoid.isEmpty()) "" else ""","options":{"avoid_features":["$avoid"]}""") + (if (pt) ""","navmaster":{"pass_through":[false,true,false]}""" else "")
            val body = Heidelberg.body(pts, extra)
            val plain = try { service.solve(DirectionsRequest.fromJson("driving-hgv", body)) } catch (e: ApiException) { continue }
            val lines = java.util.Collections.synchronizedList(ArrayList<String>())
            val r = NavAlternativesEngine(service, RequestContext(log = { lines += it })).run(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), opt)
            trips++
            assertEquals(plain.legs.map { l -> l.edges.map { it.edge } }, r.best.legs.map { l -> l.edges.map { it.edge } }, "seme $seed $avoid: migliore diverso")
            val router = service.routingGraph.router(r.best.options)
            // ogni svolta lecita, salvo quelle del migliore stesso (con pass_through il ripiego all'indietro sulla tappa)
            val tc = router.access(service.routingGraph.encoding.shared).turnCosts
            fun turns(l: it.navmaster.engine.routing.RoutePath) = (1 until l.edges.size).map { i -> Triple(l.edges[i - 1].edge, l.edges[i - 1].adjNode, l.edges[i].edge) }
            val bestTurns = r.best.legs.flatMap { turns(it) }.toSet()
            for (a in r.variants) {
                val bad = a.legs.flatMap { turns(it) }.filter { t -> tc != null && t !in bestTurns && tc.turnWeight(t.first, t.second, t.third).isInfinite() }
                assertTrue(bad.isEmpty(), "seme $seed $avoid pass_through $pt: svolte vietate $bad in ${a.draft.tag.ifEmpty { a.draft.source.json }}")
                assertEquals(r.best.legs.size, a.legs.size)
                // un tratto solo cambiato: gli altri sono quelli del migliore
                val changed = a.legs.indices.count { j -> a.legs[j].edges.map { it.edge } != r.best.legs[j].edges.map { it.edge } }
                if (changed == 1) oneLeg++
            }
            keptLeg += lines.count { it.startsWith("varianti: P SP") && ", tratto " in it && "→ tenuta" in it }
            keptMix += lines.count { it.startsWith("varianti: tratti misti:") && "→ tenuta" in it }
        }
        println("tratti uno alla volta su Heidelberg (3 punti): $trips viaggi, $keptLeg varianti dalle penalità su un tratto, $keptMix dai tratti misti, $oneLeg varianti con un tratto solo cambiato")
        assertTrue(trips >= 60, "viaggi: $trips")
        assertTrue(keptLeg >= 1, "nessuna variante dalle penalità su un tratto solo")
        assertTrue(keptMix >= 1, "nessuna variante dai tratti misti")
    }
}
