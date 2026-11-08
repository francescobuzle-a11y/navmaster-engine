package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * I viaggi tra Paesi contro il riferimento esaustivo ([CrossBorderReference]): stesso peso o
 * differenza segnalata. Su Heidelberg lo stesso grafo fa da tre Paesi, con valichi finti a gruppi
 * (più punti nello stesso luogo, strade di importanza diversa). Casi: senza opzioni, avoid_features,
 * maximum_speed, bearings, viaggi corti vicino al confine, tappe, due passaggi di confine,
 * continue_straight, transito per un terzo Paese.
 */
class CrossBorderReferenceTest {
    private fun same(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || abs(x - y) <= 0.01 + abs(y) * 1e-6

    @Test
    fun `cross-border trips weigh as the exhaustive reference`() {
        val dir = Files.createTempDirectory("nme-cbr").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { it.prepareCH() }
        CrossBorderDirections.compareOneByOne = false
        RoutingGraph.load(dir, mmap = false).use { g ->
            val bg = g.graph
            val core = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 4.0, weight = 12.0)))
            val rnd = Random(2026)
            fun randomSnap(r: Router): Snap {
                while (true) {
                    val n = rnd.nextInt(bg.nodeCount)
                    val d = if (rnd.nextBoolean()) 0.0 else 0.0006
                    try { return r.snap(bg.lat(n) + (rnd.nextDouble() - 0.5) * d, bg.lon(n) + (rnd.nextDouble() - 0.5) * d, 400.0, 0) } catch (e: PointNotFoundException) { }
                }
            }
            fun near(lat: Double, lon: Double, maxMeters: Double): Snap {
                while (true) {
                    val s = randomSnap(core)
                    if (DistanceCalc.haversine(lat, lon, s.queryLat, s.queryLon) <= maxMeters) return s
                }
            }
            // valichi finti: luoghi a più di 5,5 km, ciascuno con 1-3 punti entro 4 km di importanza diversa
            fun crossings(count: Int): List<Transfer> {
                val centers = ArrayList<Snap>()
                for (attempt in 0 until 4000) {
                    if (centers.size >= count) break
                    val s = randomSnap(core)
                    if (centers.none { DistanceCalc.haversine(it.queryLat, it.queryLon, s.queryLat, s.queryLon) < 5_500.0 }) centers += s
                }
                val out = ArrayList<Transfer>()
                for ((k, c) in centers.withIndex()) {
                    val n = c.closestNode
                    out += Transfer(bg.lat(n), bg.lon(n), 1 + k % 3)
                    repeat(k % 3) { j ->
                        val m = near(c.queryLat, c.queryLon, 4_000.0).closestNode
                        out += Transfer(bg.lat(m), bg.lon(m), if (j == 0) 1 + (k + 1) % 3 else 5)
                    }
                }
                return out.distinctBy { it.lat to it.lon }
            }
            val ab = crossings(5); val ac = crossings(3); val cb = crossings(3)
            println("valichi a/b ${ab.size}, a/c ${ac.size}, c/b ${cb.size}")
            val service = DirectionsService(g)
            val a = CountryGraph("a", service); val b = CountryGraph("b", service); val c = CountryGraph("c", service)
            fun key(x: CountryGraph, y: CountryGraph) = listOf(x.id, y.id).sorted().joinToString("")
            val direct: (CountryGraph, CountryGraph) -> List<Transfer> = { x, y -> if (key(x, y) == "ab") ab else emptyList() }
            val transit: (CountryGraph, CountryGraph) -> List<Transfer> = { x, y -> when (key(x, y)) { "ac" -> ac; "bc" -> cb; else -> emptyList() } }

            class Case(val name: String, val ids: List<String>, val transfers: (CountryGraph, CountryGraph) -> List<Transfer>, val body: (List<Snap>) -> Map<String, Any?>, val points: () -> List<Snap>,
                       val profile: String = "driving-hgv")
            fun body(pts: List<Snap>, extra: Map<String, Any?> = emptyMap(), options: Map<String, Any?> = emptyMap()) = linkedMapOf<String, Any?>(
                "coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false,
                "options" to linkedMapOf<String, Any?>("profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 12.0))).apply { putAll(options) },
            ).apply { putAll(extra) }
            val two = { listOf(randomSnap(core), randomSnap(core)) }
            val cases = listOf(
                Case("senza opzioni", listOf("a", "b"), direct, { body(it) }, two),
                Case("evita autostrade", listOf("a", "b"), direct, { body(it, options = mapOf("avoid_features" to listOf("highways"))) }, two),
                Case("evita pedaggi e traghetti", listOf("a", "b"), direct, { body(it, options = mapOf("avoid_features" to listOf("tollways", "ferries"))) }, two),
                Case("maximum_speed 80", listOf("a", "b"), direct, { body(it, extra = mapOf("maximum_speed" to 80.0)) }, two),
                Case("bearings", listOf("a", "b"), direct, { body(it, extra = mapOf("bearings" to listOf(listOf(rnd.nextInt(360).toDouble(), 45.0), listOf(rnd.nextInt(360).toDouble(), 45.0)))) }, two),
                Case("corto vicino a un valico", listOf("a", "b"), direct, { body(it) }, {
                    val t = ab[rnd.nextInt(ab.size)]
                    listOf(near(t.lat, t.lon, 2_000.0), near(t.lat, t.lon, 2_000.0))
                }),
                Case("una tappa prima del confine", listOf("a", "a", "b"), direct, { body(it) }, { List(3) { randomSnap(core) } }),
                Case("due passaggi di confine", listOf("a", "b", "a"), direct, { body(it) }, { List(3) { randomSnap(core) } }),
                Case("continue_straight", listOf("a", "a", "b"), direct, { body(it, extra = mapOf("continue_straight" to true)) }, { List(3) { randomSnap(core) } }),
                Case("transito per un terzo Paese", listOf("a", "b"), transit, { body(it) }, two),
                Case("continue_straight e due passaggi", listOf("a", "b", "a"), direct, { body(it, extra = mapOf("continue_straight" to true)) }, { List(3) { randomSnap(core) } }),
                Case("evita autostrade e maximum_speed 80", listOf("a", "b"), direct, { body(it, extra = mapOf("maximum_speed" to 80.0), options = mapOf("avoid_features" to listOf("highways"))) }, two),
                Case("bearings e una tappa", listOf("a", "a", "b"), direct, { body(it, extra = mapOf("bearings" to List(3) { listOf(rnd.nextInt(360).toDouble(), 45.0) })) }, { List(3) { randomSnap(core) } }),
                Case("camion senza misure (CH)", listOf("a", "b"), direct, { pts -> linkedMapOf("coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false) }, two),
                Case("auto", listOf("a", "b"), direct, { pts -> linkedMapOf("coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false) }, two, "driving-car"),
                Case("auto, evita pedaggi", listOf("a", "b"), direct, { pts -> linkedMapOf("coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false,
                    "options" to mapOf("avoid_features" to listOf("tollways", "highways"))) }, two, "driving-car"),
                Case("auto, transito", listOf("a", "b"), transit, { pts -> linkedMapOf("coordinates" to pts.map { listOf(it.queryLon, it.queryLat) }, "instructions" to false) }, two, "driving-car"),
            )
            val problems = ArrayList<String>()
            // due volte: con la regola del motore e con la ricerca mirata su ogni passaggio (Reach): sui grafi
            // piccoli della prova la mirata non scatterebbe da sola, e i pesi devono restare quelli esatti
            for ((mode, focus) in listOf("" to 0.0, " (mirate)" to -1.0)) for (case in cases) {
                val cbd = CrossBorderDirections(listOf(a, b, c), case.transfers).also { it.focusBeyond = focus }
                val ref = CrossBorderReference(listOf(a, b, c), case.transfers)
                var equal = 0; var noRoute = 0; var multiBorder = 0; val diffs = ArrayList<String>()
                repeat(20) { k ->
                    val pts = case.points()
                    val req = DirectionsRequest.fromJson(case.profile, case.body(pts))
                    val of = case.ids.map { id -> listOf(a, b, c).first { it.id == id } }
                    val r = ref.best(req, of)
                    val got = runCatching { cbd.handle(req, case.ids) }
                    val w = if (got.isSuccess) ref.weightOf(cbd.lastPieces) else Double.POSITIVE_INFINITY
                    val crossings = if (got.isSuccess) cbd.lastPieces.drop(1).map { (_, pr) -> pr.coordinates[0] } else emptyList()
                    when {
                        same(w, r.weight) -> if (w.isInfinite()) noRoute++ else equal++
                        // più confini: ogni valico è scelto da solo (l'approssimazione nota E): un viaggio in cui
                        // la scelta congiunta dei valichi pesa meno si segnala, non è un errore
                        r.crossings.size >= 3 && w.isFinite() && w > r.weight -> {
                            multiBorder++
                            println("  (più confini) viaggio $k: peso ${"%.1f".format(w)} invece di ${"%.1f".format(r.weight)} (+%.1f %%): valichi scelti uno alla volta".format(100 * (w - r.weight) / r.weight))
                        }
                        else -> {
                            diffs += "viaggio $k: peso ${"%.1f".format(w)} invece di ${"%.1f".format(r.weight)} (${if (w.isFinite()) "+%.1f %%".format(100 * (w - r.weight) / r.weight) else got.exceptionOrNull()?.message?.take(60)}); " +
                                "valichi $crossings invece di ${r.crossings.map { it.lat to it.lon }}"
                            // diagnostica: la scelta con il registro, e i pezzi del riferimento
                            println("  --- richiesta: ${Json.stringify(req.echo)}")
                            val logged = CrossBorderDirections(listOf(a, b, c), case.transfers, { println("  | $it") }).also { it.focusBeyond = focus }
                            runCatching { logged.handle(req, case.ids) }
                            println("  --- riferimento: valichi ${r.crossings.map { "%.5f,%.5f".format(it.lat, it.lon) }}, ${r.pieceCount} pezzi")
                        }
                    }
                }
                println("${case.name}$mode: $equal uguali, $noRoute senza percorso in entrambi, $multiBorder con più confini scelti uno alla volta, ${cbd.twoChoices} scelte di due valichi insieme, ${diffs.size} diversi${if (diffs.isEmpty()) "" else "\n    " + diffs.joinToString("\n    ")}")
                problems += diffs.map { "${case.name}$mode: $it" }
            }
            assertTrue(problems.isEmpty(), "differenze dal riferimento:\n" + problems.joinToString("\n"))
        }
    }
}
