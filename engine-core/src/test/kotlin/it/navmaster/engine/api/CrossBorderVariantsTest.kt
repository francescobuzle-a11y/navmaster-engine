package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.util.DistanceCalc
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Le varianti di un viaggio tra Paesi (passo 7) su Heidelberg: lo stesso grafo nei panni di due
 * Paesi, valichi finti. La prima feature è la risposta senza varianti; ogni feature ha un way_point
 * per punto e `properties.navmaster.variant`; le durate entro max_factor del viaggio; dalle due
 * fonti (il pezzo più lungo a valichi fissi e gli altri valichi) almeno una variante; i pezzi
 * finali dalla cache dei tratti.
 */
class CrossBorderVariantsTest {
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

    private fun crossings(seed: Long, n: Int): List<Transfer> {
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(seed)
        val out = ArrayList<Transfer>()
        for (attempt in 0 until 4000) {
            if (out.size >= n) break
            val n0 = rnd.nextInt(bg.nodeCount)
            val node = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
            if (out.any { t -> DistanceCalc.haversine(t.lat, t.lon, bg.lat(node), bg.lon(node)) < 5_500.0 }) continue
            out += Transfer(bg.lat(node), bg.lon(node), 1 + out.size % 3)
        }
        return out
    }

    @Test
    fun `variants across borders - best unchanged, pieces and crossings`() {
        val service = DirectionsService(g)
        val transfers = crossings(11, 6)
        assertTrue(transfers.size >= 4, "valichi finti: ${transfers.size}")
        val opt = NavAlternativesOptions(count = 3, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
        val bySource = HashMap<String, Int>()
        var trips = 0
        // le varianti da un altro valico di una richiesta con la sola fonte V (quando le altre fonti le battono)
        var crossingOnly = 0
        // titoli tutti diversi (anche dal migliore), mai «altro valico, altro valico»
        fun checkTitles(fs: List<Map<*, *>>) {
            val labels = fs.map { (((it["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"] as Map<*, *>)["label"] as String }
            assertEquals(labels.size, labels.toSet().size, "titoli ripetuti: $labels")
            assertTrue(labels.none { it.split("altro valico").size > 2 }, "titolo doppio: $labels")
        }
        fun checkCrossing(v: Map<*, *>) {
            val tier = v["tier"] as Int
            assertTrue(tier in 1..3, "livello $tier")
            assertTrue(v["longer"] != true || (tier >= 2 && (v["factor"] as Double) >= opt.tier1Factor), "più lunga: $v")
        }
        for (seed in 1L..8L) {
            val n = if (seed % 3 == 0L) 3 else 2
            val pts = Heidelberg.points(n, seed + 700)
            val countries = List(n) { if (it < n - 1) "a" else "b" }
            val cb = CrossBorderDirections(listOf(CountryGraph("a", service), CountryGraph("b", service)), { _, _ -> transfers })
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")), countries) } catch (e: ApiException) { continue }
            val plain = features(plainDoc)[0]
            trips++
            val d0 = ((plain["properties"] as Map<*, *>)["summary"] as Map<*, *>)["duration"] as Double
            val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")).withNavAlternatives(opt)
            val rc = RequestContext()
            val doc = cb.handle(req, countries, rc)
            val fs = features(doc)
            assertTrue(fs.isNotEmpty())
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]), "la prima feature è il migliore")
            // i pezzi finali del migliore dalla cache (li ha appena calcolati la richiesta senza varianti)
            val pieceLegs = rc.stats.children.filter { it.first == "a" || it.first == "b" }.flatMap { it.second.legs }
            assertTrue(pieceLegs.isNotEmpty() && pieceLegs.all { it.source == LegStats.Source.CACHE }, rc.stats.line())
            for ((i, f) in fs.withIndex()) {
                val props = f["properties"] as Map<*, *>
                assertEquals(n, (props["way_points"] as List<*>).size)
                val v = (props["navmaster"] as Map<*, *>)["variant"] as Map<*, *>
                assertEquals(i, v["rank"])
                if (i > 0) bySource.merge(v["source"] as String, 1, Int::plus)
                // un altro valico: il livello dalla regola sul viaggio intero, «più lunga» coerente
                if (i > 0 && v["source"] == "crossing") checkCrossing(v)
                assertTrue(((props["summary"] as Map<*, *>)["duration"] as Double) <= 1.5 * d0 + 1, "durata ${(props["summary"] as Map<*, *>)["duration"]} oltre 1,5 × $d0")
                assertTrue((v["factor"] as Double) <= 1.5 + 1e-3, "fattore ${v["factor"]}")
            }
            checkTitles(fs)
            val meta = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            assertEquals(3, meta["asked"]); assertEquals(fs.size, meta["found"])
            println("tra Paesi $n punti, seme $seed: ${fs.size - 1} varianti (${fs.drop(1).joinToString { ((it["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"].let { v -> "${(v as Map<*, *>)["source"]} ${v["label"]}" } }}; ${meta["candidates"]}; ${meta["note"]})")
            // exclude_best: le stesse varianti senza il migliore
            val ex = features(cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")).withNavAlternatives(
                NavAlternativesOptions(count = 3, excludeBest = true, sources = opt.sources, minDetourScale = 0.02)), countries))
            assertEquals(fs.size - 1, ex.size)
            // nessuna da un altro valico fra le tenute: la sola fonte V deve comunque arrivare in fondo
            // (con le strade d'accesso della fonte M una deviazione M può prendere il posto di un valico)
            if (fs.drop(1).none { ((it["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"].let { v -> (v as Map<*, *>)["source"] == "crossing" } }) {
                val only = features(cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")).withNavAlternatives(
                    NavAlternativesOptions(count = 3, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)), countries))
                assertEquals(withoutVariant(plain), withoutVariant(only[0]))
                checkTitles(only)
                for (f in only.drop(1)) {
                    val v = ((f["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"] as Map<*, *>
                    assertEquals("crossing", v["source"]); checkCrossing(v); crossingOnly++
                }
            }
        }
        println("varianti tra Paesi per fonte: $bySource in $trips viaggi, da un altro valico con la sola fonte V: $crossingOnly")
        assertTrue(trips >= 4, "viaggi: $trips")
        assertTrue((bySource["crossing"] ?: 0) > 0, "nessuna variante da un altro valico: $bySource")
        assertTrue(bySource.keys.any { it != "crossing" }, "nessuna variante a valichi fissi: $bySource")
    }

    // ------------------------------------------------------------------ cinque Paesi finti: a-b-c-d e a-b-e-d

    /** Per ogni valico di [list] un altro punto della rete a 0,3-1,5 km: i valichi dell'altra sequenza, vicini (pesi simili). */
    private fun nearby(list: List<Transfer>, seed: Long): List<Transfer> {
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val rnd = Random(seed)
        return list.map { t ->
            for (attempt in 0 until 40000) {
                val n0 = rnd.nextInt(bg.nodeCount)
                if (DistanceCalc.haversine(t.lat, t.lon, bg.lat(n0), bg.lon(n0)) !in 300.0..1_500.0) continue
                val node = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
                val d = DistanceCalc.haversine(t.lat, t.lon, bg.lat(node), bg.lon(node))
                if (d in 300.0..1_500.0) return@map Transfer(bg.lat(node), bg.lon(node), t.rank)
            }
            error("nessun punto vicino al valico ${t.lat},${t.lon}")
        }
    }

    /**
     * Cinque Paesi sullo stesso grafo, valichi disgiunti per coppia a-b, b-c, c-d, b-e, e-d: due
     * sequenze a → d con 3 confini, la seconda (per e) coi valichi a pochi km da quelli della prima
     * (pesi simili: spesso entro il fattore delle varianti).
     */
    private class Five(service: DirectionsService, seed: Long = 0) {
        val graphs = listOf("a", "b", "c", "d", "e").map { CountryGraph(it, service) }
        val pairs: Map<Set<String>, List<Transfer>>
        init {
            val test = CrossBorderVariantsTest()
            val bc = test.crossings(22 + seed * 10, 3); val cd = test.crossings(23 + seed * 10, 3)
            pairs = mapOf(setOf("a", "b") to test.crossings(21 + seed * 10, 3), setOf("b", "c") to bc, setOf("c", "d") to cd,
                setOf("b", "e") to test.nearby(bc, 24 + seed * 10), setOf("e", "d") to test.nearby(cd, 25 + seed * 10))
        }
        val transfers: (CountryGraph, CountryGraph) -> List<Transfer> = { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }
        fun cb(log: ((String) -> Unit)? = null, oneSearch: Boolean = true) = CrossBorderDirections(graphs, transfers, log, oneSearch)
        /** Il motore ristretto ai grafi di [ids] (il riferimento del peso esatto di quella sequenza). */
        fun only(ids: List<String>) = CrossBorderDirections(graphs.filter { it.id in ids }, transfers)
    }

    private fun variant(f: Map<*, *>) = ((f["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"] as Map<*, *>

    /** Le strade nominate da un titolo «via X + Y, per …» (vuota per «altro valico …»). */
    private fun titleRefs(label: String): List<String> =
        if (!label.startsWith("via ")) emptyList() else label.removePrefix("via ").substringBefore(", ").split(" + ")

    /**
     * Per ogni ref (normalizzato) della feature [f] i metri e la quota di autostrada o superstrada,
     * dai fatti sulle strade (`navmaster.roads`, classe e ref per arco) e dalla geometria.
     */
    private fun refMeters(f: Map<*, *>): Map<String, Pair<Double, Double>> {
        val coords = ((f["geometry"] as Map<*, *>)["coordinates"] as List<*>).map { c ->
            when (c) { is DoubleArray -> c[1] to c[0]; is List<*> -> (c[1] as Number).toDouble() to (c[0] as Number).toDouble(); else -> error("coordinata $c") }
        }
        val roads = ((f["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["roads"] as List<*>
        val m = HashMap<String, Double>(); val mw = HashMap<String, Double>()
        for (r in roads) {
            r as Map<*, *>
            val ref = (r["ref"] as? String)?.let { EdgeInfo.normalizeRef(it) } ?: continue
            val from = r["from"] as Int; val to = r["to"] as Int
            var d = 0.0
            for (i in from until minOf(to, coords.size - 1)) d += DistanceCalc.haversine(coords[i].first, coords[i].second, coords[i + 1].first, coords[i + 1].second)
            m.merge(ref, d, Double::plus)
            if (r["class"] == "motorway" || r["class"] == "trunk") mw.merge(ref, d, Double::plus)
        }
        return m.mapValues { (ref, d) -> d to (if (d > 0) (mw[ref] ?: 0.0) / d else 0.0) }
    }

    /**
     * Il titolo di una V per altri Paesi: nomina le autostrade nuove più lunghe, non le prime strade
     * nuove nell'ordine della strada (una statale corta). Le strade nominate sono nuove, autostrade
     * (ref da autostrada o metà dei km su archi di autostrada o superstrada), al più due nell'ordine
     * della strada, e nessuna autostrada nuova non nominata è più lunga di una nominata; senza
     * autostrade nuove «altro valico». Le quote vicine a 0,5 (metri dalla geometria) non si giudicano.
     * Vero se il titolo nomina strade.
     */
    private fun checkOtherTitle(f: Map<*, *>, v: Map<*, *>): Boolean {
        val label = v["label"] as String
        val named = titleRefs(label)
        val newRefs = (v["new_refs"] as List<*>).map { it as String }
        val rm = refMeters(f)
        fun km(r: String) = rm[r]?.first ?: 0.0
        fun share(r: String) = rm[r]?.second ?: 0.0
        fun surelyMw(r: String) = CorridorAnalyzer.isMotorwayRef(r) || share(r) >= 0.55
        fun maybeMw(r: String) = CorridorAnalyzer.isMotorwayRef(r) || share(r) >= 0.45
        val old = newRefs.take(2)
        println("  titolo per altri Paesi: «$label»; strade nuove ${newRefs.joinToString { "%s %.1f km (%.0f %%)".format(it, km(it) / 1000, share(it) * 100) }}; con le prime strade nuove sarebbe ${if (old.isEmpty()) "altro valico" else "via " + old.joinToString(" + ")}")
        assertTrue(named.size <= 2, "più di due strade: $label")
        for (r in named) {
            assertTrue(r in newRefs, "«$label»: $r non è una strada nuova ($newRefs)")
            assertTrue(maybeMw(r), "«$label»: $r non è un'autostrada (quota ${share(r)})")
        }
        assertEquals(named, newRefs.filter { it in named }, "«$label»: strade fuori dall'ordine della strada ($newRefs)")
        val shortest = named.minOfOrNull { km(it) } ?: 0.0
        for (r in newRefs) if (r !in named && surelyMw(r)) {
            assertTrue(named.size == 2 && km(r) <= shortest * 1.02 + 50.0, "«$label»: l'autostrada nuova $r (%.1f km) è più lunga di una nominata o manca".format(km(r) / 1000))
        }
        if (named.isEmpty()) assertTrue(label.startsWith("altro valico"), "titolo senza strade: $label")
        return named.isNotEmpty()
    }
    private fun countriesOf(v: Map<*, *>) = (v["countries"] as? List<*>)?.map { it as String }
    private fun crossingsLine(rc: RequestContext) = rc.stats.crossings
    /** «Q ricerche (H dalla memoria[, X per le varianti])» → (Q, H, X). */
    private fun queries(line: String): Triple<Int, Int, Int> {
        val m = Regex("""(\d+) ricerche \((\d+) dalla memoria(?:, (\d+) per le varianti)?\)""").find(line) ?: error("riga senza ricerche: $line")
        return Triple(m.groupValues[1].toInt(), m.groupValues[2].toInt(), m.groupValues[3].ifEmpty { "0" }.toInt())
    }
    private fun searched(line: String): Int = Regex("""(\d+) cercati""").find(line)?.groupValues?.get(1)?.toInt() ?: 0
    private fun legsOf(stats: RequestStats, names: Set<String>) = synchronized(stats.children) { stats.children.filter { it.first in names }.flatMap { (_, s) -> synchronized(s.legs) { s.legs.toList() } } }

    @Test
    fun `variants across borders - another country sequence`() {
        val service = DirectionsService(g)
        val logs = ArrayList<String>()
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0; var others = 0; var withinRef = 0; var exactMatch = 0; var keptOther = 0; var reservedChecks = 0; var namedTitles = 0
        for (seed in 1L..8L) {
            // valichi diversi a ogni viaggio (con gli stessi una sequenza vincerebbe sempre)
            val five = Five(service, seed)
            val cb = five.cb(log = { logs += it })
            val pts = Heidelberg.points(2, seed + 900)
            val countries = listOf("a", "d")
            val body = Heidelberg.body(pts, ""","navmaster":{"roads":true}""")
            val plainDoc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries) } catch (e: ApiException) { continue }
            trips++
            val plain = features(plainDoc)[0]
            val bestW = cb.lastWeight; val bestX = cb.lastCrossings.map { it.lat to it.lon }
            val bestSeq = cb.lastChains.first().countries
            assertEquals(listOf(bestSeq), cb.lastChains.map { it.countries }, "con topK 1 solo il migliore")
            val otherSeq = if (bestSeq == listOf("a", "b", "c", "d")) listOf("a", "b", "e", "d") else listOf("a", "b", "c", "d")
            assertTrue(bestSeq == listOf("a", "b", "c", "d") || bestSeq == listOf("a", "b", "e", "d"), "sequenza $bestSeq")
            // il riferimento: lo stesso viaggio coi soli grafi dell'altra sequenza
            val ref = five.only(otherSeq)
            val refW = try { ref.handle(DirectionsRequest.fromJson("driving-hgv", body), countries); ref.lastWeight } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            val cap = bestW * opt.maxFactor
            // la richiesta delle varianti (sola fonte V)
            val rc = RequestContext()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries, rc)
            val fs = features(doc)
            // (1) il migliore identico con topK 1 e 4
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]))
            assertEquals(bestW, cb.lastWeight, "peso con topK 4"); assertEquals(bestX, cb.lastCrossings.map { it.lat to it.lon }, "valichi con topK 4")
            assertEquals(bestSeq, cb.lastChains.first().countries)
            // (2) la catena dell'altra sequenza entro il fattore: c'è quando il riferimento sta sotto il tetto, mai se no
            val otherChain = cb.lastChains.drop(1).firstOrNull { it.countries == otherSeq }
            println("seme $seed: migliore ${bestSeq.joinToString("-")} peso %.0f; riferimento ${otherSeq.joinToString("-")} peso %.0f (tetto %.0f); catena trovata: %s".format(
                bestW, refW, cap, otherChain?.let { "peso %.0f via %s".format(it.exact, it.via.joinToString { t -> "%.4f,%.4f".format(t.lat, t.lon) }) } ?: "nessuna"))
            if (refW < cap * (1 - 1e-9)) {
                withinRef++
                assertTrue(otherChain != null, "seme $seed: il riferimento ${otherSeq.joinToString("-")} pesa $refW < $cap ma nessuna catena trovata")
                assertTrue(otherChain.exact >= refW - 1e-6 && otherChain.exact < cap, "seme $seed: catena ${otherChain.exact} fuori da [$refW, $cap)")
                if (Math.abs(otherChain.exact - refW) <= 1e-6 * refW) exactMatch++ else println("  combinazione a luogo diverso: ${otherChain.exact} invece di $refW")
            } else assertTrue(otherChain == null, "seme $seed: catena ${otherChain?.exact} oltre il tetto $cap (riferimento $refW)")
            // (3) ogni V: livello, fattore, valichi, way_points; titoli diversi
            val labels = fs.map { variant(it)["label"] as String }
            assertEquals(labels.size, labels.toSet().size, "titoli ripetuti: $labels")
            val vs = fs.drop(1).map { variant(it) }
            for ((i, f) in fs.withIndex()) {
                assertEquals(2, ((f["properties"] as Map<*, *>)["way_points"] as List<*>).size)
                if (i == 0) continue
                val v = variant(f)
                assertEquals("crossing", v["source"])
                assertTrue((v["tier"] as Int) in 1..3, "livello ${v["tier"]}")
                assertTrue((v["factor"] as Double) <= opt.maxFactor + 1e-3 && (v["km_factor"] as Double) <= opt.maxKmFactor + 1e-3, "fattori $v")
                assertEquals(3, (v["crossings"] as List<*>).size, "valichi di $v")
                val cs = countriesOf(v)!!
                assertEquals(4, cs.size)
                if (cs != bestSeq) {
                    others++; assertTrue(v["label"].toString().contains(", per "), "titolo senza il Paese: ${v["label"]}")
                    if (checkOtherTitle(f, v)) namedTitles++
                }
            }
            // (4) due V mai con tutti i valichi entro 5 km l'uno dall'altro
            for (i in vs.indices) for (j in i + 1 until vs.size) {
                val a = vs[i]["crossings"] as List<Map<*, *>>; val b = vs[j]["crossings"] as List<Map<*, *>>
                val same = countriesOf(vs[i]) == countriesOf(vs[j]) && a.indices.all { k -> DistanceCalc.haversine(a[k]["lat"] as Double, a[k]["lon"] as Double, b[k]["lat"] as Double, b[k]["lon"] as Double) < Transfers.PLACE }
                assertTrue(!same, "due V nello stesso luogo: ${vs[i]} / ${vs[j]}")
            }
            // (5) i tratti della fonte V tutti dalla cache dei tratti
            val vStats = synchronized(rc.stats.children) { rc.stats.children.filter { it.first == "varianti valichi" }.map { it.second } }
            val vLegs = vStats.flatMap { s -> synchronized(s.children) { s.children.flatMap { (_, c) -> synchronized(c.legs) { c.legs.toList() } } } }
            assertTrue(vLegs.all { it.source == LegStats.Source.CACHE }, "tratti della fonte V cercati: ${vLegs.joinToString { it.source.name }}")
            val meta = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            println("  ${fs.size - 1} varianti: ${vs.joinToString { "${it["label"]} ${it["diff"]} livello ${it["tier"]}" }}; ${meta["candidates"]}; ${meta["note"]}")
            // la scelta golosa: una V di livello 1 per altri Paesi è fra le tenute anche con le fonti M e P
            if (vs.any { countriesOf(it) != bestSeq && it["tier"] == 1 }) {
                reservedChecks++
                val all = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
                val mixed = features(cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(all), countries)).drop(1).map { variant(it) }
                val kept = mixed.filter { it["source"] == "crossing" && countriesOf(it) != bestSeq }
                println("  con M e P: ${mixed.joinToString { "${it["source"]} ${it["label"]} livello ${it["tier"]}" }}")
                assertTrue(kept.isNotEmpty(), "la V per altri Paesi non è fra le tenute: ${mixed.map { "${it["source"]} ${it["label"]}" }}")
                keptOther++
            }
            // (6) col tempo a 1 ms: nessuna eccezione, nota «budget», migliore invariato
            val short = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, budgetMs = 1, sources = opt.sources, minDetourScale = 0.02)
            logs.clear()
            val docShort = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(short), countries)
            println("  tempo 1 ms: ${logs.filter { it.contains("varianti") }.joinToString(" / ")}")
            assertEquals(withoutVariant(plain), withoutVariant(features(docShort)[0]))
            assertEquals(bestW, cb.lastWeight)
            val metaShort = ((docShort["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            // la nota «budget» (o, se tutto era in memoria e il passo 2 è finito entro il millisecondo, nessuna variante)
            assertTrue(metaShort["note"] == "budget" || (metaShort["variants"] == 0 && logs.none { it.contains("tempo finito") }), "nota col tempo finito: $metaShort")
        }
        println("altra sequenza: $trips viaggi, riferimento entro il fattore in $withinRef (peso uguale in $exactMatch), $others varianti per altri Paesi ($namedTitles titoli con strade), posto riservato $keptOther/$reservedChecks")
        assertTrue(trips >= 4, "viaggi: $trips")
        assertTrue(others > 0, "nessuna variante per un'altra sequenza di Paesi")
    }

    @Test
    fun `the second request of the same trip reuses the quick weights`() {
        val service = DirectionsService(g)
        val five = Five(service)
        val cb = five.cb()
        val opt = NavAlternativesOptions(count = 4, tier1Factor = 1.1, maxFactor = 1.2, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING), minDetourScale = 0.02)
        var trips = 0
        for (seed in 1L..6L) {
            val pts = Heidelberg.points(2, seed + 950)
            val body = Heidelberg.body(pts, ""","navmaster":{"roads":true}""")
            service.clearCaches()
            val rc0 = RequestContext()
            val plain = try { features(cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf("a", "d"), rc0))[0] } catch (e: ApiException) { continue }
            trips++
            val (q0, h0, _) = queries(crossingsLine(rc0))
            assertEquals(0, h0, "prima richiesta a memoria vuota: ${crossingsLine(rc0)}")
            assertTrue(q0 > 0, crossingsLine(rc0))
            // la seconda, identica: nessuna ricerca, i pezzi tutti dalla cache
            val rc1 = RequestContext()
            val doc1 = cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf("a", "d"), rc1)
            val line1 = crossingsLine(rc1)
            val (q1, h1, _) = queries(line1)
            assertEquals(withoutVariant(plain), withoutVariant(features(doc1)[0]))
            assertEquals(0, q1, "ricerche rifatte: $line1")
            assertEquals(0, searched(line1), "pezzi ricercati: $line1")
            assertTrue(h1 >= q0 - searched(crossingsLine(rc0)), "colpi $h1, ricerche della prima $q0: $line1")
            // la terza, con le varianti: la scelta del migliore senza ricerche, solo quelle del passo 2 (le altre catene)
            val rc = RequestContext()
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), listOf("a", "d"), rc)
            val line = crossingsLine(rc)
            val (q, h, x) = queries(line)
            println("seme $seed: prima ${crossingsLine(rc0)} | seconda $line1 | con varianti $line")
            assertEquals(withoutVariant(plain), withoutVariant(features(doc)[0]))
            assertEquals(0, q - x, "ricerche della scelta rifatte: $line")
            assertTrue(h >= h1, "colpi $h: $line")
            assertTrue(legsOf(rc.stats, setOf("a", "b", "c", "d", "e")).all { it.source == LegStats.Source.CACHE }, rc.stats.line())
            // senza cache: le ricerche come oggi, stesso risultato
            val rcNo = RequestContext(useCache = false)
            val docNo = cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf("a", "d"), rcNo)
            val (qNo, hNo, _) = queries(crossingsLine(rcNo))
            assertEquals(0, hNo); assertTrue(qNo > 0)
            assertEquals(withoutVariant(plain), withoutVariant(features(docNo)[0]))
            // dopo clearCaches le ricerche si rifanno e il risultato è identico
            service.clearCaches()
            val rcC = RequestContext()
            val docC = cb.handle(DirectionsRequest.fromJson("driving-hgv", body), listOf("a", "d"), rcC)
            val (qC, hC, _) = queries(crossingsLine(rcC))
            assertEquals(0, hC); assertEquals(q0, qC, "ricerche dopo clearCaches: ${crossingsLine(rcC)}")
            assertEquals(withoutVariant(plain), withoutVariant(features(docC)[0]))
        }
        assertTrue(trips >= 3, "viaggi: $trips")
    }

    @Test
    fun `a via point recomputes only its piece, the first border from the cache`() {
        val service = DirectionsService(g)
        val five = Five(service)
        // un valico alla volta: anche fra due Paesi la scelta passa dalle ricerche rapide (quelle in memoria)
        val cb = five.cb(oneSearch = false)
        var trips = 0; var viaB = 0; var viaA = 0
        for (seed in 1L..6L) {
            val pts = Heidelberg.points(2, seed + 980)
            service.clearCaches()
            val rc0 = RequestContext()
            try { cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")), listOf("a", "d"), rc0) } catch (e: ApiException) { continue }
            trips++
            val chain = cb.lastChains.first()
            // una tappa nel Paese b (a metà del tratto di b): il confine a/b si sceglie coi pesi in memoria
            val pieceB = cb.lastPieces[1].second
            val viaPoint = pieceB.coordinates[0].let { (la, lo) -> la + (pieceB.coordinates[1].first - la) * 0.5 to lo + (pieceB.coordinates[1].second - lo) * 0.5 }
            fun body(mid: Pair<Double, Double>) = Heidelberg.body(listOf(pts[0], mid, pts[1]), ""","navmaster":{"roads":true}""")
            val rcB = RequestContext()
            val okB = runCatching { cb.handle(DirectionsRequest.fromJson("driving-hgv", body(viaPoint)), listOf("a", "b", "d"), rcB) }.isSuccess
            if (okB) {
                val (q, h, _) = queries(crossingsLine(rcB))
                println("seme $seed, tappa in b: ${crossingsLine(rcB)}")
                assertTrue(h > 0, "nessun peso dalla memoria con la tappa in b: ${crossingsLine(rcB)}")
                assertTrue(q > 0 || h > 0)
                viaB++
                // la stessa via come la manda l'app in navigazione (pass_through e direzione): continue_straight
                // non cambia il router col core preparato, quindi la chiave dei pesi rapidi (routerKey) è la stessa
                val rcP = RequestContext()
                val bodyP = Heidelberg.body(listOf(pts[0], viaPoint, pts[1]), ""","navmaster":{"roads":true,"pass_through":[false,true,false]},"bearings":[[],[90,45],[]]""")
                val okP = runCatching { cb.handle(DirectionsRequest.fromJson("driving-hgv", bodyP), listOf("a", "b", "d"), rcP) }.isSuccess
                println("seme $seed, via (pass_through) in b: ${crossingsLine(rcP)}")
                if (okP) assertTrue(queries(crossingsLine(rcP)).second > 0, "nessun peso dalla memoria con la via pass_through in b: ${crossingsLine(rcP)}")
            }
            // una via nel Paese a, vicino alla partenza: si ricalcola solo il primo pezzo
            val viaA0 = pts[0].first + (chain.via[0].lat - pts[0].first) * 0.3 to pts[0].second + (chain.via[0].lon - pts[0].second) * 0.3
            val rcA = RequestContext()
            val okA = runCatching { cb.handle(DirectionsRequest.fromJson("driving-hgv", body(viaA0)), listOf("a", "a", "d"), rcA) }.isSuccess
            if (okA) {
                val line = crossingsLine(rcA)
                println("seme $seed, via in a: $line; ${legsOf(rcA.stats, setOf("a", "b", "c", "d", "e")).joinToString { it.source.name.lowercase() }}")
                assertTrue(queries(line).second > 0, "nessun peso dalla memoria con la via in a: $line")
                viaA++
            }
        }
        println("via: $trips viaggi, in b $viaB, in a $viaA")
        assertTrue(trips >= 3 && viaB >= 2 && viaA >= 2, "viaggi $trips, via in b $viaB, in a $viaA")
    }

    /**
     * Le strade del titolo di una V per altri Paesi ([CrossBorderDirections.otherCountriesTitleRefs])
     * sui dati veri di Canneto sull'Oglio → Bielsko-Biała (CI): la variante per la Cechia comincia con
     * SS11 e SR450 (statali del Garda, 14 e 13 km) e poi prende A22, A1 austriaca e le D ceche; il
     * titolo nomina le due autostrade nuove più lunghe nell'ordine della strada, non le statali.
     */
    @Test
    fun `the title of a variant through other countries names its longest new motorways`() {
        // le strade nuove nell'ordine della strada, km e quota di autostrada o superstrada (D1: già nel migliore, la slovacca)
        val roads = listOf("SS11" to 14.0, "SR450" to 13.0, "A22" to 205.0, "A13" to 34.0, "A12" to 69.0, "B173" to 10.0, "B178" to 17.0, "B164" to 36.0,
            "B311" to 58.0, "A10" to 42.0, "A1" to 241.0, "S33" to 27.0, "S5" to 35.0, "S3" to 35.0, "B303" to 10.0, "D52" to 17.0, "D48" to 33.0, "S52" to 32.0)
        val refM = roads.associate { it.first to it.second * 1000 }
        val mwShare = roads.associate { (r, _) -> r to if (r.startsWith("D") || CorridorAnalyzer.isMotorwayRef(r)) 1.0 else 0.0 }
        assertEquals(listOf("A22", "A1"), CrossBorderDirections.otherCountriesTitleRefs(roads.map { it.first }, refM, mwShare))
        // la via di Tarvisio → Mikulov → Brno → Ostrava: le D ceche dagli archi (nessun prefisso da autostrada)
        val tarvisio = listOf("SS11" to 14.0, "A5" to 60.0, "D52" to 40.0, "D2" to 30.0, "D1" to 150.0, "D48" to 33.0, "I/11" to 20.0)
        val tm = tarvisio.associate { it.first to it.second * 1000 }
        val tw = tarvisio.associate { (r, _) -> r to if (r.startsWith("D")) 0.9 else 0.0 }
        assertEquals(listOf("A5", "D1"), CrossBorderDirections.otherCountriesTitleRefs(tarvisio.map { it.first }, tm, tw))
        // una sola autostrada nuova: solo quella; nessuna: nessuna strada («altro valico, per la Cechia»)
        assertEquals(listOf("D1"), CrossBorderDirections.otherCountriesTitleRefs(listOf("SS11", "SR450", "D1"), mapOf("SS11" to 14e3, "SR450" to 13e3, "D1" to 9e3), mapOf("D1" to 0.8)))
        assertEquals(emptyList<String>(), CrossBorderDirections.otherCountriesTitleRefs(listOf("SS11", "SR450", "B37"), mapOf("SS11" to 14e3, "SR450" to 13e3, "B37" to 30e3), mapOf("B37" to 0.3)))
    }

    /**
     * Il titolo di una V per altri Paesi su Heidelberg: quattro Paesi finti sullo stesso grafo, a-c-d
     * coi valichi sul percorso migliore (a un terzo e a due terzi) e a-e-d coi valichi su una via più
     * lenta (+12 %) che prende prima la L596 (8 km, una statale) e poi la A5 (6 km): con le prime strade nuove
     * il titolo sarebbe «via L596 + A5»; deve nominare solo l'autostrada nuova, «via A5, per e».
     */
    @Test
    fun `on Heidelberg the title of a variant through other countries names the new motorway, not a short state road`() {
        val service = DirectionsService(g)
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val s = 49.4604735 to 8.8199303; val t = 49.384662 to 8.6202951
        val w1 = 49.4637988 to 8.7620039; val w2 = 49.3697004 to 8.6326278
        // i valichi a-c e c-d sul percorso migliore (i nodi a un terzo e a due terzi della strada)
        val best = core.route(core.snap(s.first, s.second, 400.0, 0), core.snap(t.first, t.second, 400.0, 1))
        fun nodeAt(share: Double): Transfer {
            var m = 0.0
            for (e in best.edges) { m += e.distance; if (m >= share * best.distance) return Transfer(g.graph.lat(e.adjNode), g.graph.lon(e.adjNode), 3) }
            error("percorso corto")
        }
        val pairs = mapOf(setOf("a", "c") to listOf(nodeAt(1 / 3.0)), setOf("c", "d") to listOf(nodeAt(2 / 3.0)),
            setOf("a", "e") to listOf(Transfer(w1.first, w1.second, 3)), setOf("e", "d") to listOf(Transfer(w2.first, w2.second, 3)))
        val graphs = listOf("a", "c", "d", "e").map { CountryGraph(it, service) }
        val logs = ArrayList<String>()
        val cb = CrossBorderDirections(graphs, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
        val body = Heidelberg.body(listOf(s, t), ""","navmaster":{"roads":true}""")
        val opt = NavAlternativesOptions(count = 3, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING), minDetourScale = 0.02)
        val fs = features(cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), listOf("a", "d")))
        assertEquals(listOf("a", "c", "d"), cb.lastChains.first().countries, "il migliore per c: ${cb.lastChains.map { it.countries }}")
        val labels = fs.map { variant(it)["label"] as String }
        println("titoli: $labels; catene ${cb.lastChains.joinToString { "${it.countries.joinToString("-")} %.0f".format(it.exact) }}")
        logs.filter { "varianti" in it }.forEach { println("  $it") }
        val other = fs.drop(1).firstOrNull { countriesOf(variant(it)) == listOf("a", "e", "d") }
        assertTrue(other != null, "nessuna V per e: $labels")
        val v = variant(other)
        val label = v["label"] as String
        val newRefs = (v["new_refs"] as List<*>).map { it as String }
        // la variante che serve: con le prime strade nuove il titolo nominerebbe la L596
        assertEquals("L596", newRefs.firstOrNull(), "strade nuove: $newRefs")
        assertTrue("A5" in newRefs, "strade nuove: $newRefs")
        assertTrue(checkOtherTitle(other, v), "titolo senza strade: $label")
        assertTrue(label.startsWith("via ") && "A5" in titleRefs(label) && "L596" !in titleRefs(label), "titolo: $label")
        assertTrue(label.endsWith(", per e"), "titolo: $label")
        assertEquals(labels.size, labels.toSet().size, "titoli ripetuti: $labels")
    }

    /**
     * Il caso Tarvisio: il primo valico è condiviso fra le due sequenze di Paesi e il suo luogo ha
     * per primo punto ([Transfers.places]: rank, poi deviazione) un punto più caro di quello che usa
     * il migliore. Nella struttura [Five] al valico a/b del migliore X2 si aggiunge nello stesso luogo
     * un punto X1 di rank 1 a 1,5-4 km (X2 almeno rank 2: X1 è il primo del luogo), raggiungibile ma
     * più caro (la catena dell'altra sequenza per X1 almeno l'1 % sopra quella per X2). Semi validi:
     * il migliore passa da X2, la sua sequenza è la prima esaminata (il passo 1 non pesa l'altra
     * sequenza sotto un tetto più alto), la catena dell'altra sequenza (per X2) fra 1,0× e 1,1× del
     * migliore. Con le varianti (sola fonte V, count 4, max 1,2): il migliore identico a topK 1 e fra
     * le catene quella dell'altra sequenza col primo valico entro [Transfers.PLACE] da X2 e il peso del
     * riferimento (lo stesso viaggio coi soli grafi di quella sequenza). Col passo 2 che pesava solo
     * il primo punto di ogni luogo la catena passava da X1: peso più alto, o nessuna catena.
     */
    @Test
    fun `step 2 weighs every point of a place, the first crossing shared by two sequences`() {
        val service = DirectionsService(g)
        val core = g.router(RouteOptions(profile = "driving-hgv"))
        val bg = g.graph
        val opt = NavAlternativesOptions(count = 4, maxFactor = 1.2, sources = setOf(VariantSource.CROSSING))
        val seqs = listOf(listOf("a", "b", "c", "d"), listOf("a", "b", "e", "d"))
        var valid = 0; var tried = 0
        for (seed in 1L..80L) {
            if (valid >= 4) break
            val five = Five(service, seed)
            val pts = Heidelberg.points(2, seed + 1700)
            val body = Heidelberg.body(pts, ""","navmaster":{"roads":true}""")
            val countries = listOf("a", "d")
            val cb0 = five.cb()
            try { cb0.handle(DirectionsRequest.fromJson("driving-hgv", body), countries) } catch (e: ApiException) { continue }
            val ab = five.pairs.getValue(setOf("a", "b"))
            val x2 = cb0.lastCrossings.firstOrNull() ?: continue
            if (ab.none { it.lat == x2.lat && it.lon == x2.lon }) continue
            // X1: un punto della rete a 1,5-4 km da X2, lontano dagli altri valichi a/b (nello stesso luogo solo con X2)
            val rnd = Random(seed * 31 + 7)
            var near: Transfer? = null
            for (attempt in 0 until 40000) {
                val n0 = rnd.nextInt(bg.nodeCount)
                if (DistanceCalc.haversine(x2.lat, x2.lon, bg.lat(n0), bg.lon(n0)) !in 1_500.0..4_000.0) continue
                val node = try { core.snap(bg.lat(n0), bg.lon(n0), 400.0, 0).closestNode } catch (e: PointNotFoundException) { continue }
                val la = bg.lat(node); val lo = bg.lon(node)
                if (DistanceCalc.haversine(x2.lat, x2.lon, la, lo) !in 1_500.0..4_000.0) continue
                if (ab.any { it !== x2 && DistanceCalc.haversine(it.lat, it.lon, la, lo) < Transfers.PLACE + 500.0 }) continue
                near = Transfer(la, lo, 1); break
            }
            val x1 = near ?: continue
            val ab2 = ab.map { if (it === x2) Transfer(it.lat, it.lon, maxOf(2, it.rank)) else it } + x1
            fun pairsWith(abList: List<Transfer>) = five.pairs + (setOf("a", "b") to abList)
            fun engine(ids: List<String>, abList: List<Transfer>, log: ((String) -> Unit)? = null): CrossBorderDirections {
                val pairs = pairsWith(abList)
                return CrossBorderDirections(five.graphs.filter { it.id in ids }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, log)
            }
            tried++
            val logs = ArrayList<String>()
            val cb = engine(listOf("a", "b", "c", "d", "e"), ab2) { logs += it }
            val plain = try { features(cb.handle(DirectionsRequest.fromJson("driving-hgv", body), countries))[0] } catch (e: ApiException) { continue }
            val bestW = cb.lastWeight; val bestX = cb.lastCrossings.map { it.lat to it.lon }
            val bestSeq = cb.lastChains.first().countries
            if (bestSeq !in seqs) continue
            val otherSeq = seqs.first { it != bestSeq }
            // le sequenze nell'ordine d'esame: quella del migliore prima
            val order = logs.firstOrNull { it.startsWith("sequenze ") }?.substringAfter(": ")?.split(" | ")?.map { it.split("-") }.orEmpty()
            val why = when {
                bestX.first() != (x2.lat to x2.lon) -> "il migliore non passa da X2"
                order.firstOrNull() != bestSeq -> "la sequenza del migliore non è la prima (${order})"
                else -> null
            }
            // il riferimento: l'altra sequenza da sola, per X2; poi con X1 come unico valico a/b (più caro di almeno l'1 %)
            val ref = engine(otherSeq, ab2)
            val refW = try { ref.handle(DirectionsRequest.fromJson("driving-hgv", body), countries); ref.lastWeight } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            val refX = ref.lastCrossings.firstOrNull()
            val ref1 = engine(otherSeq, listOf(x1))
            val refW1 = try { ref1.handle(DirectionsRequest.fromJson("driving-hgv", body), countries); ref1.lastWeight } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            val why2 = why ?: when {
                !refW.isFinite() || refX == null || refX.lat != x2.lat || refX.lon != x2.lon -> "il riferimento non passa da X2"
                refW < bestW || refW > 1.1 * bestW -> "riferimento %.3f× del migliore".format(refW / bestW)
                !refW1.isFinite() -> "X1 non raggiungibile"
                refW1 < 1.01 * refW -> "X1 non abbastanza più caro (%.4f×)".format(refW1 / refW)
                else -> null
            }
            println("seme $seed: migliore ${bestSeq.joinToString("-")} peso %.0f, altra ${otherSeq.joinToString("-")} per X2 %.0f (%.3f×), per X1 %.0f (%.3f×): %s".format(
                bestW, refW, refW / bestW, refW1, refW1 / refW, why2 ?: "valido"))
            if (why2 != null) continue
            valid++
            // la richiesta con le varianti: il migliore identico, la catena dell'altra sequenza per X2 col peso del riferimento
            val doc = cb.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(opt), countries)
            assertEquals(withoutVariant(plain), withoutVariant(features(doc)[0]), "seme $seed: migliore diverso con le varianti")
            assertEquals(bestW, cb.lastWeight, "seme $seed: peso con topK 4")
            assertEquals(bestX, cb.lastCrossings.map { it.lat to it.lon }, "seme $seed: valichi con topK 4")
            val chains = cb.lastChains.joinToString(" | ") { c -> "${c.countries.joinToString("-")} via ${c.via.joinToString { "%.4f,%.4f".format(it.lat, it.lon) }} peso %.0f".format(c.exact) }
            println("  catene: $chains")
            val other = cb.lastChains.drop(1).firstOrNull { it.countries == otherSeq }
            assertTrue(other != null, "seme $seed: nessuna catena per ${otherSeq.joinToString("-")} (riferimento $refW, tetto ${bestW * opt.maxFactor}): $chains")
            assertTrue(DistanceCalc.haversine(other.via[0].lat, other.via[0].lon, x2.lat, x2.lon) < Transfers.PLACE, "seme $seed: primo valico lontano da X2: $chains")
            assertTrue(Math.abs(other.exact - refW) <= 1e-6 * refW, "seme $seed: catena ${other.exact} invece del riferimento $refW (per X1 $refW1): $chains")
        }
        println("valico condiviso col primo punto del luogo più caro: $valid semi validi su $tried provati")
        assertTrue(valid >= 2, "semi validi: $valid su $tried")
    }
}
