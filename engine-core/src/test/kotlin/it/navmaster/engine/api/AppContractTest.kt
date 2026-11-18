package it.navmaster.engine.api

import java.util.concurrent.Executors
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertTrue

/**
 * Il contratto con l'app: i corpi che `NmRouting.request` dell'app produce oggi (copiati lettera per
 * lettera; la prova gemella dell'app, NmRoutingRequestTest, controlla che siano ancora quelli) passano
 * da [DirectionsService.handle] come fa NmEngine, e la risposta si legge come la legge l'app
 * (`NmRouting.parse`): N way_points per feature, `properties.navmaster.variant`, la nota in metadata.
 */
class AppContractTest {
    private val g get() = Heidelberg.graph

    /** Varianti automatiche (VariantAsk.AUTO), 3 punti con la via in mezzo, camion dell'app. */
    private val auto = """{"coordinates":[[8.6757,49.4036],[8.6937,49.4094],[8.7606,49.4186]],"preference":"recommended","units":"m","instructions":true,"language":"it","maneuvers":true,"extra_info":["tollways","waycategory","waytype"],"geometry":true,"navmaster":{"roads":true,"avoid_penalty":true,"pass_through":[false,true,false],"alternatives":{"count":4,"tier1_factor":1.1,"tier1_km_factor":1.1,"max_factor":1.2,"max_km_factor":1.2,"budget_ms":24000,"exclude_best":true}},"options":{"avoid_features":["ferries"],"vehicle_type":"hgv","profile_params":{"restrictions":{"height":4.0,"width":2.55,"length":16.5,"weight":27.0,"axleload":11.5}}},"maximum_speed":80.0}"""

    /** «Altri percorsi» (VariantAsk.MORE), con la direzione del mezzo alla partenza. */
    private val more = """{"coordinates":[[8.6757,49.4036],[8.6937,49.4094],[8.7606,49.4186]],"preference":"recommended","units":"m","instructions":true,"language":"it","maneuvers":true,"extra_info":["tollways","waycategory","waytype"],"geometry":true,"navmaster":{"roads":true,"avoid_penalty":true,"pass_through":[false,true,false],"alternatives":{"count":6,"tier1_factor":1.25,"tier1_km_factor":1.25,"max_factor":1.5,"max_km_factor":1.6,"allow_fewer_km":true,"exclude_best":true}},"options":{"avoid_features":["ferries"],"vehicle_type":"hgv","profile_params":{"restrictions":{"height":4.0,"width":2.55,"length":16.5,"weight":27.0,"axleload":11.5}}},"bearings":[[90.0,45],[],[]],"maximum_speed":80.0}"""

    @Suppress("UNCHECKED_CAST")
    private fun parse(s: String) = Json.parse(s) as Map<String, Any?>

    /** Il corpo [body] con altri punti ([lat, lon]) al posto dei tre fissi. */
    private fun at(body: String, pts: List<Pair<Double, Double>>): String =
        body.replace("[[8.6757,49.4036],[8.6937,49.4094],[8.7606,49.4186]]", "[" + pts.joinToString(",") { "[${it.second},${it.first}]" } + "]")

    /** Lo stesso corpo senza `navmaster.alternatives`: la richiesta del migliore che l'app fa prima. */
    private fun plain(body: String): String = body.replace(Regex(""","alternatives":\{[^}]*\}"""), "")

    /** Come NmEngine: il testo della risposta, con i posti di ricerca e il thread aiutante dell'app. */
    private fun answer(service: DirectionsService, body: String, nav: ((DirectionsRequest) -> DirectionsRequest)? = null): Map<String, Any?> {
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        try {
            val rc = RequestContext(executor = pool, slots = SearchSlots(2))
            var req = DirectionsRequest.fromJson("driving-hgv", parse(body))
            if (nav != null) req = nav(req)
            return parse(Json.stringify(service.handle(req, ResponseFormat.GEOJSON, rc)))
        } finally { pool.shutdownNow() }
    }

    /**
     * Le feature lette come `NmRouting.parse` dell'app (le stesse chiavi, gli stessi tipi); [variants]:
     * ognuna deve dire la sua variante (rank ≥ 1 con exclude_best). Rende (durata s, metri) per feature.
     */
    private fun readAsTheApp(doc: Map<String, Any?>, nPoints: Int, variants: Boolean): List<Pair<Double, Double>> {
        assertTrue(doc["error"] == null, "errore: ${doc["error"]}")
        val features = assertNotNull(doc["features"] as? List<*>)
        val out = ArrayList<Pair<Double, Double>>()
        for (f in features) {
            val fo = f as Map<*, *>
            val coords = ((fo["geometry"] as Map<*, *>)["coordinates"] as List<*>)
            assertTrue(coords.size >= 2)
            for (c in coords) { val p = c as List<*>; assertTrue(p[0] is Number && p[1] is Number) }
            val props = fo["properties"] as Map<*, *>
            val wp = props["way_points"] as List<*>
            assertEquals(nPoints, wp.size, "way_points per feature")
            assertTrue(wp.all { it is Int || it is Long })
            val summary = props["summary"] as Map<*, *>
            val distance = (summary["distance"] as Number).toDouble(); val duration = (summary["duration"] as Number).toDouble()
            if (variants) {
                val v = assertNotNull((props["navmaster"] as? Map<*, *>)?.get("variant") as? Map<*, *>, "properties.navmaster.variant")
                // il testo riletto: interi come Long (l'app li legge con intOrNull)
                assertTrue((v["rank"] as Long) >= 1)
                assertTrue(v["tier"] is Long)
                for (k in listOf("source", "label", "diff", "signature")) assertTrue(v[k] is String, "variant.$k: ${v[k]}")
                assertTrue((v["label"] as String).isNotBlank())
                for (r in v["refs"] as List<*>) { val rm = r as Map<*, *>; assertTrue(rm["ref"] is String && rm["km"] is Number) }
                assertTrue(v["extra_s"] is Number && v["extra_m"] is Number)
                assertTrue(v["longer"] is Boolean)
            }
            out += duration to distance
        }
        return out
    }

    private fun alternativesMeta(doc: Map<String, Any?>): Map<*, *> =
        assertNotNull(((doc["metadata"] as? Map<*, *>)?.get("navmaster") as? Map<*, *>)?.get("alternatives") as? Map<*, *>)

    @Test
    fun `the app bodies are accepted and read back as the app reads them`() {
        val service = DirectionsService(g)
        for ((body, count) in listOf(auto to 4, more to 6)) {
            val best = readAsTheApp(answer(service, plain(body)), 3, variants = false)
            assertEquals(1, best.size)
            val doc = answer(service, body)
            val routes = readAsTheApp(doc, 3, variants = true)
            val meta = alternativesMeta(doc)
            assertEquals(count.toLong(), meta["asked"]); assertEquals(1L + routes.size, meta["found"])
            assertTrue(meta["note"] == null || meta["note"] is String)
            assertTrue(meta["candidates"] is String)
            println("contratto app count=$count: ${routes.size} varianti (${meta["candidates"]}; ${meta["note"]})")
        }
    }

    @Test
    fun `variants of the app bodies on seeded trips keep the shape and the automatic limits`() {
        val service = DirectionsService(g)
        var found = 0
        for (seed in 1L..12L) {
            val pts = Heidelberg.points(3, seed + 700)
            for (body in listOf(auto, more)) {
                val b = at(body, pts)
                val (d0, m0) = readAsTheApp(answer(service, plain(b)), 3, variants = false).single()
                // le soglie di distanza delle regole scalate per Heidelberg (tutto corto), il resto come lo chiede l'app
                val doc = answer(service, b) { r ->
                    val o = r.navAlternatives!!
                    r.withNavAlternatives(NavAlternativesOptions(o.count, o.tier1Factor, o.maxFactor, o.budgetMs, o.excludeBest,
                        o.tier1KmFactor, o.maxKmFactor, o.allowFewerKm, o.fill, minDetourScale = 0.02))
                }
                val routes = readAsTheApp(doc, 3, variants = true)
                val o = DirectionsRequest.fromJson("driving-hgv", parse(b)).navAlternatives!!
                for ((d, m) in routes) {
                    assertTrue(d <= o.maxFactor * d0 + 1, "durata $d oltre ${o.maxFactor}× $d0 (seme $seed)")
                    // automatiche: anche i km entro il tetto; «altri percorsi» può tenerne di più corte ma più lente
                    if (body === auto) assertTrue(m <= o.maxKmFactor * m0 + 1, "metri $m oltre ${o.maxKmFactor}× $m0 (seme $seed)")
                }
                assertTrue(routes.size <= o.count - 1)
                found += routes.size
            }
        }
        println("contratto app su 12 viaggi: $found varianti")
        assertTrue(found > 0, "nessuna variante dai corpi dell'app su Heidelberg")
    }
}
