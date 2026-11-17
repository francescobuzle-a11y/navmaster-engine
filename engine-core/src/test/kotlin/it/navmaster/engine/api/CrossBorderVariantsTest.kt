package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
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
                if (i > 0 && v["source"] == "crossing") {
                    val tier = v["tier"] as Int
                    assertTrue(tier in 1..3, "livello $tier")
                    assertTrue(v["longer"] != true || (tier >= 2 && (v["factor"] as Double) >= opt.tier1Factor), "più lunga: $v")
                }
                assertTrue(((props["summary"] as Map<*, *>)["duration"] as Double) <= 1.5 * d0 + 1, "durata ${(props["summary"] as Map<*, *>)["duration"]} oltre 1,5 × $d0")
                assertTrue((v["factor"] as Double) <= 1.5 + 1e-3, "fattore ${v["factor"]}")
            }
            val meta = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            assertEquals(3, meta["asked"]); assertEquals(fs.size, meta["found"])
            println("tra Paesi $n punti, seme $seed: ${fs.size - 1} varianti (${fs.drop(1).joinToString { ((it["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"].let { v -> "${(v as Map<*, *>)["source"]} ${v["label"]}" } }}; ${meta["candidates"]}; ${meta["note"]})")
            // exclude_best: le stesse varianti senza il migliore
            val ex = features(cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}""")).withNavAlternatives(
                NavAlternativesOptions(count = 3, excludeBest = true, sources = opt.sources, minDetourScale = 0.02)), countries))
            assertEquals(fs.size - 1, ex.size)
        }
        println("varianti tra Paesi per fonte: $bySource in $trips viaggi")
        assertTrue(trips >= 4, "viaggi: $trips")
        assertTrue((bySource["crossing"] ?: 0) > 0, "nessuna variante da un altro valico: $bySource")
        assertTrue(bySource.keys.any { it != "crossing" }, "nessuna variante a valichi fissi: $bySource")
    }
}
