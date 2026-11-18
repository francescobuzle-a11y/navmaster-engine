package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.weighting.EdgePenalties
import java.io.File
import java.util.Random
import java.util.concurrent.Executors
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Diagnosi delle varianti (`navmaster.alternatives`) su grafi veri (CI, flusso «Valichi su grafi veri»).
 *
 * Variabili: `NM_CB="italy=/…[,austria=/…]"` (un Paese: [DirectionsService]; più Paesi:
 * [CrossBorderDirections]); `NM_TRIPS="lat,lon;lat,lon[;…]|…"` (predefiniti: [DEFAULT_TRIPS]);
 * `NM_VIAS="5,12,24"` (vie sintetiche lungo il migliore ogni L/(n+1), con la direzione, come il
 * rientro dell'app; predefinito nessuna); `NM_ALT="count,max_factor[,tier1_factor[,max_km_factor[,fewer]]]"`
 * (predefinito "4,1.2,1.1": la richiesta automatica dell'app; "6,1.5,1.25,1.6,fewer": «Altri percorsi»);
 * `NM_AVOID="tollways,ferries"` (vuoto: solo i traghetti, come prima; non vuoto: anche `avoid_penalty`
 * come l'app); `NM_AVOID` e `NM_ALT` possono essere liste separate da «;»: tutte le combinazioni in un
 * lavoro solo (i grafi si aprono una volta), ogni combinazione col suo riepilogo; `NM_PROFILE=car`
 * (auto e camper: driving-car senza misure del camion); `NM_SOURCES="motorway,penalty[,crossing]"`.
 * Senza `NM_CB` la prova non fa nulla (e passa).
 *
 * Sonde (`NM_PROBES`, solo un Paese e viaggi di due punti): voci separate da `§`, ognuna
 * `viaggio|REF@lat,lon[;REF@lat,lon…]`, con il viaggio scritto come nel registro (`Torino→Mestre`,
 * anche `Torino->Mestre`). Per ogni punto l'arco più vicino con quel ref entro [PROBE_RADIUS_M]
 * (percorribile dal camion); il percorso migliore del viaggio per quei punti (punti di passaggio
 * senza inversione, provati i due versi della carreggiata, tenuto il più leggero): tempo, km, r e
 * rk sul migliore, la firma, il verdetto del giudice sul solo migliore coi valori. `NM_TRACE=1`
 * (predefinito: acceso se ci sono sonde) accende la traccia delle varianti ([VariantTrace]): righe
 * `varianti: traccia …` del motore, le sonde giudicate sulle tenute e sorvegliate nella fonte M.
 *
 * La richiesta è quella camion dell'app (4 m, 2,55 m, 16,5 m, 27 t, 11,5 t per asse, maximum_speed
 * 80, roads, evita traghetti, pass_through sulle vie), con un aiutante e 2 posti di ricerca come sul
 * tablet. Per viaggio: ms del migliore (fonti dei tratti) e delle varianti, il registro delle fonti
 * (rete, deviazioni, candidati P), per percorso minuti, km, r, livello, fonte, firma; la memoria.
 *
 * Asserzioni (FAIL k/n): (a) almeno 3 alternative oltre al migliore (min(count, 4) percorsi) in ogni viaggio,
 * tappe comprese (la regola del prodotto è «sempre»: prima bastava il 90 % e un viaggio a 2 alternative
 * restava nascosto); un viaggio senza percorso (punto non agganciabile, rete chiusa al camion) è un FAIL, non un viaggio saltato; (b) le firme attese
 * in almeno metà dei viaggi noti (dentro i tetti 1,20: Torino→Mestre la A21, Bolzano→Milano la A22
 * fino all'interconnessione di Verona e poi la A4, senza SR450 né SS11; Milano→Bologna nessuna, A21/A22 sono oltre 1,20×); (c) livello 1 entro tier1, tutte entro max_factor in tempo e km;
 * (d) la prima feature è la risposta senza varianti; (e) le varianti subito dopo il migliore: 0
 * ricerche per il migliore; (f) esattezza delle penalità: core-ALT penalizzato = Dijkstra pieno
 * penalizzato su 20 coppie; (g) una via in più: ≤ 2 tratti calcolati (un Paese); (h) i titoli delle
 * varianti tutti diversi e mai il nome di una galleria o di un ponte; (i) tra Paesi: la richiesta
 * delle varianti rifà la scelta dei valichi senza ricerche («0 ricerche (N dalla memoria)», i pezzi
 * del migliore tutti dalla cache: i «cercati» meno quelli «per le varianti», le catene del passo 2), coi migliori di 3 o più
 * grafi (anche i rami esatti del transito, ora in memoria; con 2 grafi stampata); (j) tra Paesi: le varianti entro `NM_VARIANTS_MS` (12 000 ms, il proxy in CI dei
 * 25 s del tablet), col tempo del motore `NM_BUDGET_MS` (predefinito 10 000, sotto il limite di (j): il motore sta
 * dentro il suo tempo, minimi garantiti compresi; 2 s restano alle parti che il tempo non ferma, la preparazione della
 * rete di un pezzo e la risposta. Non è il tablet: i suoi 25 s, a ~5×, sono ~5 000 ms in CI, da provare con
 * `NM_BUDGET_MS=5000`, input «budget» del flusso); (l) tra Paesi: il migliore entro `NM_BEST_MS`, non nella prima
 * combinazione (grafi a freddo): solo stampato, un FAIL solo con `NM_BEST_MS` dato (i migliori lenti, «evita
 * autostrade» senza landmark, sono un lavoro a parte); (k) con «highways» in `NM_AVOID`: nei viaggi oltre 20 km almeno una variante
 * (su altre strade: la regola chiede dMin1 di deviazione dal migliore) e nessuna in autostrada, in
 * almeno l'80 % dei viaggi. Per i viaggi tra Paesi stampa le sequenze, le catene esatte oltre il migliore
 * (Paesi, valichi, peso) e i ms della fonte V.
 */
class AlternativesRealDiagTest {
    private class Place(val name: String, val lat: Double, val lon: Double)

    private val places = listOf(
        Place("Milano", 45.4642, 9.1900), Place("Bologna", 44.4949, 11.3426), Place("Torino", 45.0703, 7.6869), Place("Mestre", 45.4847, 12.2386),
        Place("Bolzano", 46.4983, 11.3548), Place("Brescia", 45.5416, 10.2118), Place("Genova", 44.4056, 8.9463),
        // Firenze: viale Guidoni (Novoli), la via di scorrimento fra l'uscita Firenze Nord dell'A1/A11 e la città, a ~4 km
        // dal Duomo; il centro (43.7696, 11.2558) è zona a traffico limitato, nessuna strada per il camion entro 400 m
        Place("Firenze", 43.7981, 11.2149),
        // i viaggi tra Paesi (flusso «Valichi su grafi veri» con più Paesi)
        Place("Canneto sull'Oglio", 45.1502, 10.3792), Place("Bielsko-Biała", 49.8224, 19.0444), Place("Soncino", 45.4103, 9.9428),
        Place("Innsbruck", 47.2692, 11.4041), Place("Komjatice", 48.1536, 18.1872),
        // Graz: la Hauptplatz (47.0707, 15.4395) è zona pedonale, nessuna strada per il camion entro 400 m: Graz-Webling, sul Gürtel
        Place("Graz", 47.0450, 15.4000), Place("Trieste", 45.6495, 13.7768),
        // Padova: il centro (45.4064, 11.8768) è zona a traffico limitato, nessuna strada per il camion entro 400 m: l'Interporto
        Place("Padova", 45.3920, 11.9350),
    )

    private fun placeOf(p: Pair<Double, Double>): String? = places.firstOrNull { DistanceCalc.haversine(it.lat, it.lon, p.first, p.second) < 3_000.0 }?.name

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = doc["features"] as List<Map<String, Any?>>

    @Suppress("UNCHECKED_CAST")
    private fun props(f: Map<String, Any?>) = f["properties"] as Map<String, Any?>

    @Suppress("UNCHECKED_CAST")
    private fun variant(f: Map<String, Any?>) = (props(f)["navmaster"] as? Map<String, Any?>)?.get("variant") as? Map<String, Any?>

    private fun summary(f: Map<String, Any?>) = props(f)["summary"] as Map<*, *>

    /** La feature senza `properties.navmaster.variant` (come la dà una richiesta senza varianti). */
    @Suppress("UNCHECKED_CAST")
    private fun withoutVariant(f: Map<String, Any?>): String {
        val p = LinkedHashMap(props(f))
        val nm = (p["navmaster"] as? Map<String, Any?>)?.let { LinkedHashMap(it) }
        nm?.remove("variant")
        if (nm == null || nm.isEmpty()) p.remove("navmaster") else p["navmaster"] = nm
        return Json.stringify(LinkedHashMap(f).also { it["properties"] = p })
    }

    /** "A4 87 → A22 100 → A1 40" da `variant.refs`. */
    private fun signature(v: Map<String, Any?>?): String =
        (v?.get("refs") as? List<*>)?.joinToString(" → ") { r -> (r as Map<*, *>).let { "${it["ref"]} ${Math.round((it["km"] as Number).toDouble())}" } }.orEmpty().ifEmpty { "-" }

    private fun refsOf(v: Map<String, Any?>?): Set<String> = (v?.get("refs") as? List<*>)?.map { (it as Map<*, *>)["ref"] as String }?.toSet().orEmpty()

    /** I km di [ref] nella firma di [v] (tutte le corse). */
    private fun kmOf(v: Map<String, Any?>?, ref: String): Double =
        (v?.get("refs") as? List<*>).orEmpty().map { it as Map<*, *> }.filter { it["ref"] == ref }.sumOf { (it["km"] as Number).toDouble() }

    /** Una sonda di `NM_PROBES`: il viaggio, i punti (ref, lat, lon), il testo dei punti (il suo nome). */
    private class ProbeSpec(val trip: String, val points: List<Triple<String, Double, Double>>, val name: String)

    private fun probeSpecs(text: String?): List<ProbeSpec> = text.orEmpty().split('§').map { it.trim() }.filter { it.isNotEmpty() }.map { e ->
        val (t, ps) = e.split('|', limit = 2).also { require(it.size == 2) { "sonda senza «|»: $e" } }
        val points = ps.split(';').map { it.trim() }.filter { it.isNotEmpty() }.map { q ->
            val (ref, ll) = q.split('@').also { require(it.size == 2) { "punto senza «@»: $q" } }
            val (la, lo) = ll.split(',')
            Triple(ref.trim(), la.trim().toDouble(), lo.trim().toDouble())
        }
        ProbeSpec(t.trim().replace("->", "→"), points, ps.trim())
    }

    private fun usedMb(): Long { val r = Runtime.getRuntime(); return (r.totalMemory() - r.freeMemory()) / (1024 * 1024) }

    @Test
    fun diagnose() {
        val spec = System.getenv("NM_CB") ?: return
        val graphs = spec.split(',').map { it.split('=') }.map { (id, path) ->
            val t0 = System.currentTimeMillis()
            val g = RoutingGraph.load(File(path), true)
            println("$id aperto in ${System.currentTimeMillis() - t0} ms: ${g.graph.nodeCount} nodi, ${g.graph.edgeCount} archi")
            CountryGraph(id, DirectionsService(g, DirectionsLimits(50, 6_000_000.0, 6_000_000.0, 400.0)))
        }
        val single = graphs.size == 1
        val cache = HashMap<String, List<Transfer>>()
        val transfers = { a: CountryGraph, b: CountryGraph ->
            cache.getOrPut(listOf(a.id, b.id).sorted().joinToString("-")) { Transfers.find(a.graph, b.graph).also { println("passaggi ${a.id}-${b.id}: ${it.size}") } }
        }
        val log: (String) -> Unit = { println("  $it") }
        val cb = if (single) null else CrossBorderDirections(graphs, transfers, log)
        val trips = (System.getenv("NM_TRIPS")?.takeIf { it.isNotBlank() } ?: DEFAULT_TRIPS).split('|').map { t ->
            t.split(';').map { p -> p.split(',').let { it[0].trim().toDouble() to it[1].trim().toDouble() } }
        }
        val viaCounts = System.getenv("NM_VIAS")?.split(',')?.mapNotNull { it.trim().toIntOrNull() }?.filter { it > 0 }.orEmpty()
        val sources = (System.getenv("NM_SOURCES")?.takeIf { it.isNotBlank() } ?: "motorway,penalty,crossing").split(',').map { s -> VariantSource.values().first { it.json == s.trim() } }.toSet()
        val probes = probeSpecs(System.getenv("NM_PROBES"))
        val traceOn = System.getenv("NM_TRACE")?.trim()?.takeIf { it.isNotEmpty() }?.let { it == "1" || it.equals("true", true) } ?: probes.isNotEmpty()
        // il profilo: il camion dell'app (predefinito) o l'auto/camper (NM_PROFILE=car)
        val car = System.getenv("NM_PROFILE")?.trim()?.lowercase() == "car"
        val profile = if (car) "driving-car" else "driving-hgv"
        // le combinazioni della matrice in un lavoro solo (i grafi si scaricano e si aprono una volta):
        // NM_AVOID e NM_ALT sono liste separate da «;», ogni voce come prima
        val avoidSpecs = (System.getenv("NM_AVOID") ?: "").split(';').map { it.trim() }.distinct()
        val altSpecs = (System.getenv("NM_ALT")?.takeIf { it.isNotBlank() } ?: "4,1.2,1.1").split(';').map { it.trim() }.filter { it.isNotEmpty() }.distinct()

        // come sul tablet: 2 posti di ricerca, un thread aiutante
        val slots = SearchSlots(2)
        val pool = Executors.newFixedThreadPool(1) { r -> Thread(r, "nm-leg").also { it.isDaemon = true } }
        // il tempo del motore per le varianti, sotto il limite di (j): il motore ci sta dentro coi minimi garantiti;
        // 2 s restano alle parti che il tempo non ferma (preparazione della rete di un pezzo, risposta). Il tablet
        // (25 s, CI ~5× più veloce) è NM_BUDGET_MS=5000
        val variantsMs = System.getenv("NM_VARIANTS_MS")?.trim()?.toLongOrNull() ?: 12_000L
        val budgetMs = System.getenv("NM_BUDGET_MS")?.trim()?.toLongOrNull() ?: (variantsMs - 2_000L)
        // (l) un FAIL solo se il limite è dato; se no solo stampato (12 s)
        val bestMsSet = System.getenv("NM_BEST_MS")?.trim()?.toLongOrNull()
        val bestMs = bestMsSet ?: 12_000L
        println("tempo del motore per le varianti $budgetMs ms, limite delle varianti $variantsMs ms, del migliore tra Paesi $bestMs ms")
        // i passaggi di tutti i confini prima dei viaggi (l'app li ha già scritti): il tempo del migliore non comprende la loro ricerca
        if (cb != null) for (i in graphs.indices) for (j in i + 1 until graphs.size) {
            if (graphs[i].graph.graph.bounds.intersects(graphs[j].graph.graph.bounds)) transfers(graphs[i], graphs[j])
        }
        val allFailed = ArrayList<String>()
        val summaries = ArrayList<String>()
        var firstCombo = true
        for (avoidSpec in avoidSpecs) for (altSpec in altSpecs) {
            // NM_ALT: count,max_factor[,tier1_factor[,max_km_factor[,fewer]]] («6,1.5,1.25,1.6,fewer»: «Altri percorsi» dell'app)
            val alt = altSpec.split(',').map { it.trim() }
            val count = alt[0].toInt(); val maxFactor = alt.getOrNull(1)?.toDouble() ?: 1.2
            val tier1 = minOf(maxFactor, alt.getOrNull(2)?.toDouble() ?: if (maxFactor <= 1.2) 1.1 else 1.3)
            val maxKm = maxOf(tier1, alt.getOrNull(3)?.toDouble() ?: maxFactor)
            val fewer = alt.getOrNull(4)?.lowercase()?.let { it == "1" || it == "true" || it == "fewer" } ?: false
            val opt = NavAlternativesOptions(count, tier1, maxFactor, budgetMs, false, tier1, maxKm, fewer, sources = sources)
            // NM_AVOID: le strade da evitare (vuoto: solo i traghetti, come prima); non vuoto: anche avoid_penalty come l'app (NmRouting)
            val avoidList = if (avoidSpec.isEmpty()) listOf("ferries") else avoidSpec.split(',').map { it.trim() }.filter { it.isNotEmpty() }
            val avoidPenalty = avoidSpec.isNotEmpty()
            val noMotorways = "highways" in avoidList
            val combo = "avoid=${avoidList.joinToString(",")}${if (avoidPenalty) " (penalità)" else ""} alt=$altSpec $profile"
            println("######## combinazione $combo")
            println("varianti: count $count, livello 1 ≤ $tier1, tutte ≤ $maxFactor in tempo e $maxKm in km${if (fewer) ", anche più corte" else ""}, fonti ${sources.joinToString { it.json }}; vie $viaCounts" +
                "; traccia ${if (traceOn) "sì" else "no"}; sonde ${probes.size}")
            fun context() = RequestContext(CancelToken(), RequestContext.MAIN, pool, slots, log)
            fun handle(req: DirectionsRequest, rc: RequestContext): Map<String, Any?> =
                if (cb == null) graphs[0].service.handle(req, ResponseFormat.GEOJSON, rc) else cb.handle(req, List(req.coordinates.size) { null }, rc)

            fun body(pts: List<Pair<Double, Double>>, via: List<Boolean>, bearings: List<Double?>?): Map<String, Any?> {
                val nm = linkedMapOf<String, Any?>("roads" to true)
                if (avoidPenalty) nm["avoid_penalty"] = true
                if (via.any { it }) nm["pass_through"] = via
                return linkedMapOf<String, Any?>(
                    "coordinates" to pts.map { listOf(it.second, it.first) }, "preference" to "recommended", "instructions" to true, "language" to "it",
                    "options" to (if (car) linkedMapOf<String, Any?>("avoid_features" to avoidList) else linkedMapOf<String, Any?>("avoid_features" to avoidList, "vehicle_type" to "hgv",
                        "profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "width" to 2.55, "length" to 16.5, "weight" to 27.0, "axleload" to 11.5)))),
                    "navmaster" to nm,
                ).apply {
                    if (!car) put("maximum_speed", 80.0)
                    if (bearings != null) put("bearings", bearings.map { b -> if (b == null) emptyList<Double>() else listOf(b, 45.0) })
                }
            }
            fun request(pts: List<Pair<Double, Double>>, via: List<Boolean>, bearings: List<Double?>?, variants: Boolean, o: NavAlternativesOptions = opt): DirectionsRequest {
                val r = DirectionsRequest.fromJson(profile, body(pts, via, bearings))
                return if (variants) r.withNavAlternatives(o) else r
            }

            /**
             * La sonda [spec] del viaggio [pts] (un Paese): i punti agganciati agli archi col loro ref, il
             * percorso per quei punti (i due versi di ogni carreggiata, il più leggero), stampato e
             * giudicato sul solo migliore; null se un punto non si aggancia o nessun percorso.
             */
            fun probe(pts: List<Pair<Double, Double>>, spec: ProbeSpec): VariantTrace.Probe? {
                val svc = graphs[0].service; val rg = svc.routingGraph
                val req0 = request(pts, pts.map { false }, null, false)
                val router = rg.router(svc.routeOptions(req0))
                val acc = router.access(rg.encoding.shared)
                val info = EdgeInfo(rg.graph, rg.encoding.shared, rg.guidance)
                val tag = "sonda ${spec.name}"
                class P(val lat: Double, val lon: Double, val bearing: Double, val ref: String, val edge: Int, val dist: Double)
                val snapped = spec.points.map { (ref, lat, lon) ->
                    val s = rg.index.findClosest(lat, lon, EdgeFilter { e ->
                        info.ref(e.edge) == ref && acc.filter.accept(e) && (acc.weighting.edgeWeight(e, false).isFinite() || acc.weighting.edgeWeight(e, true).isFinite())
                    }, PROBE_RINGS)
                    if (!s.isValid || s.distance > PROBE_RADIUS_M) { println("  $tag: nessun arco $ref entro ${(PROBE_RADIUS_M / 1000).toInt()} km da $lat,$lon"); return null }
                    val pl = rg.graph.fetchWayGeometry(s.edge, false, FetchMode.ALL)
                    val i = s.wayIndex.coerceIn(0, maxOf(0, pl.size - 2))
                    val b = if (pl.size < 2) 0.0 else (DistanceCalc.bearing(pl.lat(i), pl.lon(i), pl.lat(i + 1), pl.lon(i + 1)) + 360) % 360
                    P(s.snappedLat, s.snappedLon, b, ref, s.edge, s.distance)
                }
                val bestSolved = svc.solveAsHandled(req0, context())
                var chosen: DirectionsService.Solved? = null; var chosenH = ""
                val n = snapped.size
                for (mask in 0 until (1 shl n)) {
                    val headings = snapped.mapIndexed { k, p -> if ((mask shr k) and 1 == 0) p.bearing else (p.bearing + 180) % 360 }
                    val vp = listOf(pts.first()) + snapped.map { it.lat to it.lon } + listOf(pts.last())
                    val via = vp.indices.map { it in 1 until vp.size - 1 }
                    val bearings = listOf<Double?>(null) + headings + listOf(null)
                    val sol = try { svc.solveAsHandled(request(vp, via, bearings, false), context()) } catch (e: Exception) { println("  $tag versi ${headings.map { Math.round(it) }}: ${e.javaClass.simpleName} ${e.message}"); null } ?: continue
                    println("  $tag versi ${headings.map { Math.round(it) }}: %.0f min, %.1f km, peso %.0f".format(sol.legs.sumOf { it.millis } / 60000.0, sol.legs.sumOf { it.distance } / 1000, sol.legs.sumOf { it.weight }))
                    if (chosen == null || sol.legs.sumOf { it.weight } < chosen.legs.sumOf { it.weight }) { chosen = sol; chosenH = headings.joinToString { "${Math.round(it)}°" } }
                }
                val sol = chosen ?: run { println("  $tag: nessun percorso"); return null }
                val corridors = CorridorAnalyzer(rg.graph, info, rg.guidance)
                val bestD = Draft(bestSolved.legs, VariantSource.BEST)
                val v = Draft(sol.legs, VariantSource.PENALTY, tag = tag)
                val judge = VariantJudge(bestD, info, corridors, opt, rg.graph)
                val verdict = judge.evaluate(v, listOf(bestD), relaxed = false)
                val at = snapped.mapIndexed { k, p -> "${p.ref} a %.0f m (arco ${p.edge}, preso ${info.ref(sol.snaps[k + 1].edge) ?: "senza ref"} arco ${sol.snaps[k + 1].edge})".format(p.dist) }
                println("  $tag: punti ${at.joinToString("; ")}, versi $chosenH")
                println("  $tag: %.0f min, %.1f km, r %.3f, km %.3f; firma %s".format(v.millis / 60000.0, v.meters / 1000, v.millis.toDouble() / bestD.millis, v.meters / bestD.meters,
                    corridors.signature(sol.legs).joinToString(" → ") { it.first + " " + Math.round(it.second) }))
                println("  $tag: verdetto col solo migliore: " + (if (verdict.tier > 0) "livello ${verdict.tier}" + (if (verdict.reason.isNotEmpty()) " (${verdict.reason})" else "") else "scartata (${verdict.reason})") +
                    "; " + judge.explain(v, listOf(bestD), listOf("migliore")))
                return VariantTrace.Probe(spec.name, sol.legs)
            }

            // i risultati delle asserzioni: per ognuna i casi e quelli falliti
            class Check(val name: String) { var n = 0; var fail = 0; val lines = ArrayList<String>() }
            val checks = linkedMapOf("a" to Check("(a) ≥ 3 alternative in ogni viaggio"), "b" to Check("(b) firme attese"), "c" to Check("(c) fattori"), "d" to Check("(d) prima feature = migliore"),
                "e" to Check("(e) 0 ricerche per il migliore"), "f" to Check("(f) esattezza delle penalità"), "g" to Check("(g) via in più: ≤ 2 tratti"),
                "h" to Check("(h) titoli diversi"), "i" to Check("(i) tra Paesi: scelta dalla memoria"), "j" to Check("(j) tra Paesi: varianti entro il tempo"),
                "k" to Check("(k) senza autostrade: varianti su altre strade"), "l" to Check("(l) tra Paesi: migliore entro il tempo"))
            fun check(key: String, ok: Boolean, what: String) {
                val c = checks.getValue(key); c.n++
                if (!ok) { c.fail++; c.lines += what; println("  FAIL ${c.name}: $what") }
            }
            // le coppie per (f): punti lungo i migliori, 20-60 km lungo il percorso
            val pairs = ArrayList<Pair<Pair<Double, Double>, Pair<Double, Double>>>()
            val rnd = Random(42)

            /** Il percorso migliore di un viaggio: la geometria (lat, lon) per le vie; [multi]: in più Paesi (`metadata.countries`). */
            class Best(val geo: List<Pair<Double, Double>>, val multi: Boolean)
            @Suppress("UNCHECKED_CAST")
            fun countriesOf(doc: Map<String, Any?>) = ((doc["metadata"] as? Map<String, Any?>)?.get("countries") as? List<*>)?.map { it.toString() }.orEmpty()
            fun countries(doc: Map<String, Any?>) = countriesOf(doc).size.coerceAtLeast(1)

            /** Il migliore e le varianti di [pts]. */
            fun trip(pts: List<Pair<Double, Double>>, via: List<Boolean>, bearings: List<Double?>?, name: String): Best? {
                println("== $name: ${pts.size} punti")
                val m0 = usedMb()
                // il migliore, come lo mostra l'app
                val rc0 = context()
                val t0 = System.currentTimeMillis()
                val plainDoc = try { handle(request(pts, via, bearings, false), rc0) } catch (e: ApiException) {
                    println("  nessun percorso: ${e.message}")
                    // un viaggio senza percorso (coordinata non agganciabile, rete chiusa al camion o isolata) non si prova
                    // mai: conta come FAIL di (a), non sparisce (Padova nel centro: il gruppo B verde senza provare niente)
                    check("a", false, "$name: nessun percorso (${e.code}: ${e.message})")
                    return null
                }
                val plain = features(plainDoc).first()
                // un viaggio in un Paese solo (anche con più grafi aperti) ha i tratti nelle misure della richiesta
                val multi = countries(plainDoc) > 1
                val bestTook = System.currentTimeMillis() - t0
                println("  migliore in $bestTook ms: ${rc0.stats.line()}")
                // (l) il migliore tra Paesi entro il tempo (la prima combinazione apre i grafi a freddo: solo stampato)
                if (multi) {
                    if (firstCombo) println("  (l) prima combinazione (grafi a freddo): migliore in $bestTook ms")
                    else if (bestMsSet == null) println("  (l) migliore in $bestTook ms (${if (bestTook <= bestMs) "entro" else "OLTRE"} $bestMs ms, solo stampato)")
                    else check("l", bestTook <= bestMs, "$name: migliore in $bestTook ms (limite $bestMs)")
                }
                val d0 = (summary(plain)["duration"] as Number).toDouble(); val l0 = (summary(plain)["distance"] as Number).toDouble()
                // le sonde del viaggio (un Paese, due punti), poi la traccia delle varianti che le sorveglia
                val tripName = "${placeOf(pts.first())}→${placeOf(pts.last())}"
                val tripProbes = if (single && pts.size == 2) probes.filter { it.trip == tripName }.mapNotNull { probe(pts, it) } else emptyList()
                // i percorsi delle sonde possono aver tolto il migliore dalla cache dei tratti (60 000 archi in
                // tutto): di nuovo la richiesta senza varianti, perché le varianti partano come senza sonde, (e) compresa
                if (single && pts.size == 2 && probes.any { it.trip == tripName }) handle(request(pts, via, bearings, false), context())
                val optTrip = if (traceOn) opt.withTrace(VariantTrace(tripProbes)) else opt
                // le varianti subito dopo
                val rc = context()
                val t1 = System.currentTimeMillis()
                val doc = try { handle(request(pts, via, bearings, true, optTrip), rc) } catch (e: Exception) {
                    println("  varianti: errore ${e.javaClass.simpleName} ${e.message}")
                    check("a", false, "$name: errore delle varianti ${e.javaClass.simpleName} ${e.message}")
                    @Suppress("UNCHECKED_CAST")
                    return Best(((plain["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>).map { it[1] to it[0] }, multi)
                }
                val ms = System.currentTimeMillis() - t1
                val fs = features(doc)
                @Suppress("UNCHECKED_CAST")
                val meta = ((doc["metadata"] as Map<String, Any?>)["navmaster"] as? Map<String, Any?>)?.get("alternatives") as? Map<String, Any?>
                println("  varianti in $ms ms: ${fs.size} percorsi (${meta?.get("candidates")}; nota ${meta?.get("note")})")
                println("  ${rc.stats.line()}")
                for ((i, f) in fs.withIndex()) {
                    val v = variant(f)
                    val d = (summary(f)["duration"] as Number).toDouble(); val l = (summary(f)["distance"] as Number).toDouble()
                    println("   $i: %.0f min, %.1f km, r %.3f, km %.3f, livello ${v?.get("tier")}, ${v?.get("source")}, «${v?.get("label")}» ${v?.get("diff") ?: ""}; ${signature(v)}".format(d / 60, l / 1000, d / d0, l / l0))
                }
                println("  memoria: $m0 → ${usedMb()} MB")
                // (a) almeno 3 alternative oltre al migliore (la regola del prodotto; con count 3 i percorsi chiesti)
                check("a", fs.size >= minOf(count, 4), "$name: ${fs.size} percorsi (almeno ${minOf(count, 4)})")
                // (c) livello 1 entro tier1, tutte entro max_factor (tempo e km), sul migliore
                for (f in fs.drop(1)) {
                    val v = variant(f) ?: continue
                    val r = (v["factor"] as Number).toDouble(); val rk = (v["km_factor"] as Number).toDouble(); val tier = (v["tier"] as Number).toInt()
                    val shorter = fewer && rk < 1.0
                    val ok = r <= maxFactor + 1e-3 && (rk <= maxKm + 1e-3 || shorter) && (tier != 1 || (r <= tier1 + 1e-3 && rk <= tier1 + 1e-3) || shorter)
                    check("c", ok, "$name: livello $tier, tempo $r, km $rk")
                }
                // (k) senza autostrade: almeno una variante (strade non autostradali diverse dal migliore, la regola
                // chiede dMin1 di deviazione) nei viaggi oltre 20 km, e nessuna variante in autostrada
                if (noMotorways && l0 >= 20_000.0) {
                    @Suppress("UNCHECKED_CAST")
                    val onMotorway = fs.drop(1).count { f -> ((props(f)["navmaster"] as? Map<String, Any?>)?.get("roads") as? List<*>).orEmpty().any { (it as Map<*, *>)["class"] == "motorway" } }
                    check("k", fs.size >= 2 && onMotorway == 0, "$name: ${fs.size - 1} varianti, $onMotorway in autostrada")
                }
                // (d) la prima feature è la risposta senza varianti
                check("d", fs.isNotEmpty() && withoutVariant(fs[0]) == withoutVariant(plain), name)
                // (e) il migliore dalla cache: nessun tratto cercato (un Paese: i tratti della richiesta; più Paesi: i pezzi finali)
                // (un Paese coi giri per i Paesi vicini, A-B-A: il pezzo finale del Paese, come tra Paesi)
                val pieceLegs = synchronized(rc.stats.children) { rc.stats.children.filter { (n, _) -> graphs.any { it.id == n } }.flatMap { (_, s) -> synchronized(s.legs) { s.legs.toList() } } }
                val legs = if (!multi) synchronized(rc.stats.legs) { rc.stats.legs.toList() }.ifEmpty { pieceLegs } else pieceLegs
                check("e", legs.isNotEmpty() && legs.all { it.source == LegStats.Source.CACHE }, "$name: ${legs.joinToString { it.source.name.lowercase() }}")
                // (h) i titoli: tutti diversi, mai il nome di una galleria o di un ponte
                val titles = fs.mapNotNull { variant(it)?.get("label") as? String }
                check("h", titles.size == titles.toSet().size && titles.none { VariantRules.isStructureName(it.removePrefix("via ").substringBefore(", per ")) }, "$name: ${titles.joinToString(" | ")}")
                // tra Paesi: la scelta dei valichi dalla memoria (i), le catene oltre il migliore, i ms della fonte V, il tempo (j)
                if (cb != null) {
                    val line = rc.stats.crossings
                    val m = Regex("""(\d+) ricerche \((\d+) dalla memoria(?:, (\d+) per le varianti)?\)""").find(line)
                    val redone = m?.let { it.groupValues[1].toInt() - it.groupValues[3].ifEmpty { "0" }.toInt() } ?: -1
                    // i pezzi del migliore: i cercati meno quelli per le varianti (le catene del passo 2, nuove per costruzione)
                    val sm = Regex("""(\d+) cercati(?: \((\d+) per le varianti\))?""").find(line)
                    val searchedAll = sm?.groupValues?.get(1)?.toInt() ?: 0
                    val searchedV = sm?.groupValues?.get(2)?.ifEmpty { null }?.toInt() ?: 0
                    val searched = searchedAll - searchedV
                    println("  valichi (seconda richiesta): $line")
                    // (i) coi migliori di 3 o più grafi: la scelta dei due valichi del transito (chooseTwo) è nella memoria del
                    // servizio, i percorsi mirati nella memoria dei pesi rapidi. Con 2 grafi solo stampata: chooseExactTop
                    // con le varianti cerca anche oltre il migliore (contato per le varianti) e le sue ricerche molti a uno
                    // non sono contate, ma un percorso mirato con un tetto più alto di quello della prima richiesta si rifà
                    val nCountries = countries(plainDoc)
                    if (multi && nCountries >= 3) check("i", redone == 0 && searched == 0, "$name: $redone ricerche rifatte, $searched pezzi del migliore ricercati ($searchedV per le varianti): $line")
                    else println("  (i) non controllata: $nCountries grafi, $redone ricerche, $searched pezzi del migliore ricercati ($searchedV per le varianti)")
                    println("  catene: " + cb.lastChains.joinToString(" | ") { c -> "${c.countries.joinToString("-")} via ${c.via.joinToString(" ") { "%.4f,%.4f".format(it.lat, it.lon) }} peso %.0f".format(c.exact) }.ifEmpty { "nessuna" })
                    val vMs = Regex("""V \d+ \([^)]*\) in (\d+) ms""").find(meta?.get("candidates")?.toString().orEmpty())?.groupValues?.get(1)
                    println("  fonte V: ${vMs?.let { "$it ms" } ?: "non eseguita"}; varianti in tutto ${meta?.get("ms")} ms")
                    val others = fs.drop(1).mapNotNull { variant(it) }.filter { v -> (v["countries"] as? List<*>)?.let { cs -> cs != countriesOf(plainDoc) } == true }
                    println("  per altri Paesi: ${others.joinToString { "${it["label"]} (${it["countries"]})" }.ifEmpty { "nessuna" }}")
                    val total = (meta?.get("ms") as? Number)?.toLong() ?: ms
                    check("j", total <= variantsMs, "$name: varianti in $total ms (limite $variantsMs)")
                }
                // (b) le firme attese
                val from = placeOf(pts.first()); val to = placeOf(pts.last())
                val vs = fs.map { variant(it) }
                val others = vs.drop(1)
                val expected: Boolean? = when ("$from→$to") {
                    // Milano→Bologna: A21 e A22 sono oltre 1,20× (tempo o km), nessuna attesa realistica coi tetti dell'app
                    "Torino→Mestre" -> others.any { v -> "A21" in refsOf(v) }
                    // la A22 oltre Affi fino all'interconnessione con la A4 di Verona, poi tutta A4: niente SR450 né SS11
                    // (la forma con l'uscita a Peschiera e la SS11 non conta), almeno 10 km di A22 in più del migliore
                    "Bolzano→Milano" -> others.any { v -> "SR450" !in refsOf(v) && "SS11" !in refsOf(v) && kmOf(v, "A22") >= kmOf(vs[0], "A22") + 10.0 }
                    "Brescia→Milano" -> others.any { v -> "A35" in refsOf(v) }
                    "Genova→Milano" -> vs.any { v -> "A7" in refsOf(v) } && vs.any { v -> "A26" in refsOf(v) }
                    // un'altra A1 (Panoramica o Direttissima): una deviazione di almeno 20 km sulla A1
                    "Bologna→Firenze" -> others.any { v -> (v?.get("detours") as? List<*>).orEmpty().any { d ->
                        (d as Map<*, *>).let { ((it["km"] as Number).toDouble() >= 20.0) && "A1" in (it["refs"] as? List<*>).orEmpty() } } }
                    else -> null
                }
                // solo senza «evita» (le firme attese sono autostrade)
                if (expected != null && pts.size == 2 && !avoidPenalty) check("b", expected, "$from→$to: ${vs.joinToString(" | ") { signature(it) }}")
                @Suppress("UNCHECKED_CAST")
                val coords = (plain["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
                return Best(coords.map { it[1] to it[0] }, multi)
            }

            /** I punti a [at] metri lungo [geo] (lat, lon) con la direzione del percorso lì. */
            fun along(geo: List<Pair<Double, Double>>, at: List<Double>): List<Pair<Pair<Double, Double>, Double>> {
                val out = ArrayList<Pair<Pair<Double, Double>, Double>>()
                var acc = 0.0; var k = 0
                for (i in 1 until geo.size) {
                    val (a, b) = geo[i - 1] to geo[i]
                    val d = DistanceCalc.haversine(a.first, a.second, b.first, b.second)
                    while (k < at.size && acc + d >= at[k]) {
                        val t = if (d > 0) (at[k] - acc) / d else 0.0
                        val p = a.first + (b.first - a.first) * t to a.second + (b.second - a.second) * t
                        out += p to ((DistanceCalc.bearing(a.first, a.second, b.first, b.second) + 360) % 360)
                        k++
                    }
                    acc += d
                }
                return out
            }
            fun length(geo: List<Pair<Double, Double>>): Double { var s = 0.0; for (i in 1 until geo.size) s += DistanceCalc.haversine(geo[i - 1].first, geo[i - 1].second, geo[i].first, geo[i].second); return s }

            for (pts in trips) {
                val name = pts.joinToString(" → ") { p -> placeOf(p) ?: "%.4f,%.4f".format(p.first, p.second) }
                val geo = trip(pts, pts.map { false }, null, name)?.geo ?: continue
                val len = length(geo)
                // coppie per (f) lungo il migliore (20-60 km lungo il percorso)
                repeat(3) {
                    if (len < 25_000) return@repeat
                    val a = rnd.nextDouble() * (len - 22_000); val b = a + 20_000 + rnd.nextDouble() * minOf(40_000.0, len - a - 20_000)
                    val ps = along(geo, listOf(a, b))
                    if (ps.size == 2) pairs += ps[0].first to ps[1].first
                }
                // vie sintetiche lungo il migliore (un viaggio di due punti: le vie fra i due)
                if (pts.size != 2) continue
                for (n in viaCounts) {
                    val at = along(geo, (1..n).map { len * it / (n + 1) })
                    if (at.size != n) continue
                    val vp = listOf(pts[0]) + at.map { it.first } + listOf(pts[1])
                    val via = vp.indices.map { it in 1..n }
                    val bearings = listOf<Double?>(null) + at.map { it.second } + listOf(null)
                    val withVias = trip(vp, via, bearings, "$name con $n vie") ?: continue
                    // (g) una via in più a metà fra due: solo i 2 tratti nuovi si cercano (un viaggio in un Paese)
                    if (withVias.multi) { println("  (g) solo per i viaggi in un Paese"); continue }
                    val k = n / 2
                    val extra = along(geo, listOf(len * (k + 0.5) / (n + 1))).single()
                    val vp2 = vp.take(k + 1) + extra.first + vp.drop(k + 1)
                    val via2 = vp2.indices.map { it in 1 until vp2.size - 1 }
                    val b2 = bearings.take(k + 1) + extra.second + bearings.drop(k + 1)
                    val rc = context()
                    val t0 = System.currentTimeMillis()
                    try { handle(request(vp2, via2, b2, false), rc) } catch (e: ApiException) { println("  via in più: ${e.message}"); continue }
                    val legs = synchronized(rc.stats.legs) { rc.stats.legs.toList() }
                    val calc = legs.count { it.source != LegStats.Source.CACHE }
                    println("  via in più (tratto $k): ${System.currentTimeMillis() - t0} ms, $calc tratti cercati: ${rc.stats.line()}")
                    check("g", calc <= 2, "$name con $n vie + 1: $calc tratti cercati (${legs.joinToString { it.source.name.lowercase() }})")
                }
            }

            // (f) esattezza: le penalità sugli archi esposti al core (×3 sul migliore) = la ricerca piena con lo stesso peso
            // (una volta sola, nella prima combinazione)
            val req0 = request(trips.first(), trips.first().map { false }, null, false)
            var verified = 0
            if (firstCombo) for ((a, b) in pairs.take(20)) {
                val cg = graphs.firstOrNull { g -> g.graph.graph.bounds.let { it.contains(a.first, a.second) && it.contains(b.first, b.second) } } ?: continue
                val g = cg.graph
                val opts = cg.service.routeOptions(req0).copy(hasBearings = false)
                val base = g.router(opts)
                val s0 = try { base.snap(a.first, a.second, 400.0, 0) } catch (e: Exception) { continue }
                val s1 = try { base.snap(b.first, b.second, 400.0, 1) } catch (e: Exception) { continue }
                val best = try { base.route(s0, s1) } catch (e: RouteNotFoundException) { continue }
                val full = opts.copy(allowCH = false, maxVisitedNodes = 50_000_000)
                val bf = try { g.router(full).route(s0, s1) } catch (e: RouteNotFoundException) { continue }
                // solo dove core e ricerca piena danno lo stesso peso senza penalità
                if (Math.abs(bf.weight - best.weight) > 1e-6 * best.weight) { println("  (f) coppia saltata: senza penalità core ${best.weight}, piena ${bf.weight}"); continue }
                val expo = g.router(opts, EdgePenalties(1))
                val p = EdgePenalties().also { pe -> for (e in best.edges) if (e.edge !in pe && expo.isCoreExposed(e.edge)) pe.multiply(e.edge, 3.0) }
                val core = g.router(opts, p)
                if (!core.algorithm.startsWith("core_alt")) { println("  (f) ${core.algorithm}: non core"); continue }
                val t0 = System.currentTimeMillis()
                val wc = try { core.route(s0, s1).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
                val t1 = System.currentTimeMillis()
                val wf = try { g.router(full, p).route(s0, s1).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
                println("  (f) %.1f km: core %.1f (${t1 - t0} ms), piena %.1f (${System.currentTimeMillis() - t1} ms), ${p.size} archi penalizzati".format(best.distance / 1000, wc, wf))
                check("f", (wc.isInfinite() && wf.isInfinite()) || Math.abs(wc - wf) <= 1e-6 * wf, "core $wc, piena $wf")
                verified++
            }
            // con coppie disponibili almeno una verificata (non core o pesi diversi già senza penalità: niente da provare)
            if (firstCombo && pairs.isNotEmpty() && verified == 0) check("f", false, "${minOf(20, pairs.size)} coppie, nessuna verificata")
            firstCombo = false

            // il riepilogo della combinazione: (b) almeno metà, (k) almeno l'80 % (dove la rete non ha altre strade
            // non c'è variante), le altre tutte; (a) in ogni viaggio: la regola del prodotto è «sempre»
            println("== riepilogo $combo")
            for ((key, c) in checks) {
                val ok = when (key) {
                    "b" -> c.n == 0 || (c.n - c.fail) * 2 >= c.n
                    "k" -> c.n == 0 || (c.n - c.fail) >= 0.8 * c.n
                    else -> c.fail == 0
                }
                println("${c.name}: ${c.n - c.fail}/${c.n} ${if (ok) "OK" else "FAIL ${c.fail}/${c.n}"}")
                summaries += "$combo | ${c.name}: ${c.n - c.fail}/${c.n} ${if (ok) "OK" else "FAIL"}"
                if (!ok) allFailed += "[$combo] ${c.name}: FAIL ${c.fail}/${c.n}\n    " + c.lines.joinToString("\n    ")
            }
        }
        pool.shutdownNow()
        println("== riepilogo di tutte le combinazioni")
        for (l in summaries) println(l)
        assertTrue(allFailed.isEmpty(), "asserzioni sotto soglia:\n" + allFailed.joinToString("\n"))
    }

    companion object {
        /** Il raggio di aggancio di un punto di sonda all'arco col suo ref, e gli anelli di celle dell'indice guardati al più. */
        const val PROBE_RADIUS_M = 6_000.0
        const val PROBE_RINGS = 60
        /** Milano→Bologna, Torino→Mestre, Bolzano→Milano, Brescia→Milano, Genova→Milano, Genova→Brescia, Soncino→Bologna, Bologna→Firenze, Verona→Torino, Milano→Roma, Milano→Bergamo e due viaggi con una tappa. */
        const val DEFAULT_TRIPS = "45.4642,9.1900;44.4949,11.3426|45.0703,7.6869;45.4847,12.2386|46.4983,11.3548;45.4642,9.1900|45.5416,10.2118;45.4642,9.1900|" +
            "44.4056,8.9463;45.4642,9.1900|44.4056,8.9463;45.5416,10.2118|45.4103,9.9428;44.4949,11.3426|44.4949,11.3426;43.7981,11.2149|45.4384,10.9916;45.0703,7.6869|" +
            "45.4642,9.1900;41.9028,12.4964|45.4642,9.1900;45.6983,9.6773|45.1502,10.3792;45.1333,10.0227;45.0526,9.6930|45.4103,9.9428;44.8015,10.3279;44.4949,11.3426"
    }
}
