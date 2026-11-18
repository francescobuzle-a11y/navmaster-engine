package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.util.DistanceCalc
import java.util.Collections
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Una tappa nel Paese di transito (Canneto → Drasenhofen «passa di qui» → Bielsko-Biała: sul tablet 202 s,
 * in CI 472 ricerche e 519 tratti contro 115 e 4 senza la tappa). La scelta dei valichi dopo la tappa parte
 * dal pezzo [valico prima, tappa]: il tetto (il peso della catena intera) contiene il tratto valico → tappa,
 * i limiti inferiori no, e quasi ogni uscita passava il filtro. Con i tratti fissi dei capi nei limiti
 * ([CrossBorderDirections.prefixBase]) le uscite esaminate, le ricerche e i tratti pesati tornano quelli del
 * viaggio che parte dalla tappa; il migliore è lo stesso di prima (e del riferimento esaustivo, senza
 * pass_through). Su Heidelberg con Paesi finti (lo stesso grafo, valichi finti).
 */
class CrossBorderStopTest {
    private val g get() = Heidelberg.graph

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = Json.stringify(doc["features"])

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

    private class Run(val cb: CrossBorderDirections, val logs: List<String>, val doc: Map<String, Any?>?, val rc: RequestContext) {
        /** Le ricerche della scelta dei valichi («N ricerche»). */
        val queries: Int get() = Regex("""(\d+) ricerche""").find(rc.stats.crossings)?.groupValues?.get(1)?.toInt() ?: 0
        /** I tratti dei pesi dei pezzi («pesi dei pezzi: N tratti»). */
        val legs: Int get() = Regex("""pesi dei pezzi: (\d+) tratti""").find(rc.stats.crossings)?.groupValues?.get(1)?.toInt() ?: 0
        /** Le uscite esaminate dall'ultimo transito [path] («b→c→d»); null se non c'è. */
        fun examined(path: String): Int? = logs.lastOrNull { "transito ($path)" in it }?.let { Regex("""(\d+) uscite esaminate""").find(it)?.groupValues?.get(1)?.toInt() }
    }

    /** Il viaggio [pts] (il Paese di ogni punto in [of]) coi valichi [pairs]; [passThrough] alla tappa di mezzo; [base]: [CrossBorderDirections.prefixBase]. */
    private fun run(ids: List<String>, pairs: Map<Set<String>, List<Transfer>>, pts: List<Pair<Double, Double>>, of: List<String>, passThrough: Boolean, base: Boolean): Run {
        val logs = Collections.synchronizedList(ArrayList<String>())
        val service = DirectionsService(g)
        val cb = CrossBorderDirections(ids.map { CountryGraph(it, service) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
        cb.prefixBase = base
        val via = if (passThrough) ""","pass_through":[${pts.indices.joinToString(",") { (it in 1 until pts.size - 1).toString() }}]""" else ""
        val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true$via}"""))
        val rc = RequestContext(useCache = false)
        val doc = try { cb.handle(req, of, rc) } catch (e: ApiException) { null }
        return Run(cb, synchronized(logs) { logs.toList() }, doc, rc)
    }

    private fun close(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || Math.abs(x - y) <= 1e-6 * Math.abs(y) + 1e-9

    /** Lo stesso migliore con e senza i tratti fissi: risposta, peso, valichi. */
    private fun assertSameBest(old: Run, new: Run, what: String) {
        assertEquals(features(old.doc!!), features(new.doc!!), "$what: migliore diverso")
        assertTrue(close(new.cb.lastWeight, old.cb.lastWeight), "$what: peso ${new.cb.lastWeight} invece di ${old.cb.lastWeight}")
        assertEquals(old.cb.lastCrossings.map { it.lat to it.lon }, new.cb.lastCrossings.map { it.lat to it.lon }, "$what: valichi diversi")
    }

    /**
     * a → b (tappa) → d per il transito b-c-d (i due valichi insieme, come austria-czech-poland): con la tappa
     * le uscite esaminate non sono più di quelle del viaggio che parte dalla tappa (con pass_through al più una in
     * più: il verso d'arrivo può rendere la ripartenza più cara), e mai più di prima; ricerche e tratti pesati
     * non più di prima; il migliore lo stesso (senza pass_through anche quello del riferimento esaustivo).
     */
    @Test
    fun `a stop before the border does not widen the transit`() {
        var trips = 0; var fewer = 0
        var qOld = 0; var qNew = 0; var lOld = 0; var lNew = 0; var eOld = 0; var eNew = 0; var eFrom = 0
        for (seed in 1L..16L) {
            val pairs = mapOf(setOf("a", "b") to crossings(5100 + seed, 4), setOf("b", "c") to crossings(5200 + seed, 5, 3_000.0),
                setOf("c", "d") to crossings(5300 + seed, 12, 1_500.0))
            val pts = Heidelberg.points(3, seed + 5400)
            val ids = listOf("a", "b", "c", "d")
            for (pt in listOf(false, true)) {
                val what = "seme $seed${if (pt) " (pass_through)" else ""}"
                val old = run(ids, pairs, pts, listOf("a", "b", "d"), pt, base = false)
                if (old.doc == null) continue
                val new = run(ids, pairs, pts, listOf("a", "b", "d"), pt, base = true)
                assertSameBest(old, new, what)
                val o = old.examined("b→c→d") ?: continue
                val n = new.examined("b→c→d")!!
                // il viaggio che parte dalla tappa: lo stesso transito senza il tratto prima
                val from = run(ids, pairs, pts.drop(1), listOf("b", "d"), false, base = true)
                val f = from.examined("b→c→d") ?: continue
                trips++
                println("$what: uscite esaminate prima $o, ora $n, dalla tappa $f; ricerche ${old.queries} → ${new.queries}, tratti dei pezzi ${old.legs} → ${new.legs}")
                assertTrue(n <= o, "$what: $n uscite esaminate, prima $o")
                assertTrue(n <= f + (if (pt) 1 else 0), "$what: $n uscite esaminate con la tappa, $f dalla tappa")
                // più al massimo il tratto fisso della tappa, pesato una volta per scelta (senza la cache dei tratti: nell'app
                // viene dalla cache, è il tratto del pezzo scelto al confine prima, o lo pesano comunque i pezzi finali)
                assertTrue(new.queries <= old.queries + 1, "$what: ${new.queries} ricerche, prima ${old.queries}")
                assertTrue(new.legs <= old.legs + 1, "$what: ${new.legs} tratti dei pezzi, prima ${old.legs}")
                if (n < o) fewer++
                qOld += old.queries; qNew += new.queries; lOld += old.legs; lNew += new.legs; eOld += o; eNew += n; eFrom += f
                if (!pt) {
                    // senza pass_through: il peso dei pezzi è quello del riferimento esaustivo
                    val gs = ids.map { CountryGraph(it, DirectionsService(g)) }
                    val ref = CrossBorderReference(gs, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() })
                    val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","navmaster":{"roads":true}"""))
                    val r = ref.best(req, listOf("a", "b", "d").map { id -> gs.first { it.id == id } })
                    val w = ref.weightOf(new.cb.lastPieces.map { (cg, pr) -> gs.first { it.id == cg.id } to pr })
                    assertTrue(Math.abs(w - r.weight) <= 0.01 + 1e-6 * r.weight, "$what: peso $w, riferimento ${r.weight}")
                }
            }
        }
        println("tappa prima del transito: $trips viaggi, $fewer con meno uscite; uscite $eOld → $eNew (dalla tappa $eFrom), ricerche $qOld → $qNew, tratti dei pezzi $lOld → $lNew")
        assertTrue(trips >= 10, "viaggi: $trips")
        assertTrue(fewer >= 3, "solo $fewer viaggi con meno uscite esaminate")
        assertTrue(qNew < qOld && eNew < eOld, "nessun guadagno: ricerche $qOld → $qNew, uscite $eOld → $eNew")
    }

    /**
     * a → b (tappa) → d per la sequenza b-c-e-d (programmazione dinamica sui confini, come austria-czech-slovakia-poland):
     * con la tappa le ricerche e i tratti dei pesi dei pezzi non più di prima, e in qualche viaggio meno; il migliore lo stesso.
     */
    @Test
    fun `a stop before the border does not widen the layers of a longer sequence`() {
        var trips = 0; var fewer = 0
        var qOld = 0; var qNew = 0; var lOld = 0; var lNew = 0
        for (seed in 1L..12L) {
            val pairs = mapOf(setOf("a", "b") to crossings(6100 + seed, 4), setOf("b", "c") to crossings(6200 + seed, 4, 3_000.0),
                setOf("c", "e") to crossings(6300 + seed, 5, 3_000.0), setOf("e", "d") to crossings(6400 + seed, 6, 2_500.0))
            val pts = Heidelberg.points(3, seed + 6500)
            val ids = listOf("a", "b", "c", "e", "d")
            for (pt in listOf(false, true)) {
                val what = "seme $seed${if (pt) " (pass_through)" else ""}"
                val old = run(ids, pairs, pts, listOf("a", "b", "d"), pt, base = false)
                if (old.doc == null) continue
                if (old.logs.none { it.startsWith("  b-c-e-d confine") }) continue
                val new = run(ids, pairs, pts, listOf("a", "b", "d"), pt, base = true)
                assertSameBest(old, new, what)
                trips++
                println("$what: ricerche ${old.queries} → ${new.queries}, tratti dei pezzi ${old.legs} → ${new.legs}")
                // più al massimo il tratto fisso della tappa (pesato per la programmazione dinamica e per ogni scelta esatta, senza la cache dei tratti)
                assertTrue(new.queries <= old.queries + 2, "$what: ${new.queries} ricerche, prima ${old.queries}")
                assertTrue(new.legs <= old.legs + 2, "$what: ${new.legs} tratti dei pezzi, prima ${old.legs}")
                if (new.queries < old.queries || new.legs < old.legs) fewer++
                qOld += old.queries; qNew += new.queries; lOld += old.legs; lNew += new.legs
            }
        }
        println("tappa prima di quattro Paesi: $trips viaggi, $fewer con meno lavoro; ricerche $qOld → $qNew, tratti dei pezzi $lOld → $lNew")
        assertTrue(trips >= 8, "viaggi: $trips")
        assertTrue(fewer >= 2, "solo $fewer viaggi con meno lavoro")
        assertTrue(qNew < qOld && lNew < lOld, "nessun guadagno: ricerche $qOld → $qNew, tratti $lOld → $lNew")
    }

    /**
     * I tratti fissi a entrambi i capi, con più punti della richiesta prima del primo confine o dopo l'ultimo (il
     * suffisso: [CrossBorderDirections] endBase), in due Paesi (scelta esatta), in transito (due valichi insieme) e
     * su quattro Paesi (programmazione dinamica), senza nulla, con pass_through a tutti i punti di mezzo, solo
     * all'ultimo di mezzo e con continue_straight: il migliore è lo stesso di prima (e del riferimento esaustivo
     * dove lo sa calcolare: senza pass_through né continue_straight, al più un Paese di mezzo).
     */
    @Test
    fun `fixed legs at both ends keep the best`() {
        class Case(val name: String, val ids: List<String>, val of: List<String>)
        val cases = listOf(
            Case("due Paesi, due punti dopo", listOf("a", "b"), listOf("a", "b", "b")),
            Case("due Paesi, due punti prima", listOf("a", "b"), listOf("a", "a", "b")),
            Case("transito, tre punti dopo", listOf("a", "b", "c"), listOf("a", "c", "c", "c")),
            Case("transito, due prima e due dopo", listOf("a", "b", "c"), listOf("a", "a", "c", "c")),
            Case("quattro Paesi, tre punti dopo", listOf("a", "b", "c", "d"), listOf("a", "d", "d", "d")))
        val modes = listOf("", "pass_through", "pass_through solo all'ultimo di mezzo", "continue_straight")
        var trips = 0; var applied = 0; var refs = 0
        for (seed in 1L..5L) for ((ci, case) in cases.withIndex()) {
            val pairs = case.ids.zipWithNext().withIndex().associate { (k, xy) -> setOf(xy.first, xy.second) to crossings(7000 + 100 * ci + 10 * k + seed, 5, 2_500.0) }
            val pts = Heidelberg.points(case.of.size, seed + 7500 + 10 * ci)
            for (mode in modes) {
                val what = "${case.name}, seme $seed${if (mode.isEmpty()) "" else " ($mode)"}"
                val extra = when (mode) {
                    "pass_through" -> ""","navmaster":{"roads":true,"pass_through":[${pts.indices.joinToString(",") { (it in 1 until pts.size - 1).toString() }}]}"""
                    "pass_through solo all'ultimo di mezzo" -> ""","navmaster":{"roads":true,"pass_through":[${pts.indices.joinToString(",") { (it == pts.size - 2).toString() }}]}"""
                    "continue_straight" -> ""","continue_straight":true,"navmaster":{"roads":true}"""
                    else -> ""","navmaster":{"roads":true}"""
                }
                fun go(base: Boolean): Run {
                    val logs = Collections.synchronizedList(ArrayList<String>())
                    val cb = CrossBorderDirections(case.ids.map { CountryGraph(it, DirectionsService(g)) }, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() }, { logs += it })
                    cb.prefixBase = base
                    val rc = RequestContext(useCache = false)
                    val doc = try { cb.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, extra)), case.of, rc) } catch (e: ApiException) { null }
                    return Run(cb, synchronized(logs) { logs.toList() }, doc, rc)
                }
                val old = go(false)
                if (old.doc == null) continue
                val new = go(true)
                assertSameBest(old, new, what)
                trips++
                if (new.logs.any { "tratti fissi dei capi" in it }) applied++
                if (mode.isEmpty() && case.ids.size <= 3) {
                    val gs = case.ids.map { CountryGraph(it, DirectionsService(g)) }
                    val ref = CrossBorderReference(gs, { x, y -> pairs[setOf(x.id, y.id)] ?: emptyList() })
                    val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, extra))
                    val r = ref.best(req, case.of.map { id -> gs.first { it.id == id } })
                    assertTrue(Math.abs(new.cb.lastWeight - r.weight) <= 0.01 + 1e-6 * r.weight, "$what: peso ${new.cb.lastWeight}, riferimento ${r.weight}")
                    refs++
                }
            }
        }
        println("tratti fissi ai due capi: $trips viaggi, $applied coi tratti fissi nel registro, $refs contro il riferimento")
        assertTrue(trips >= 60, "viaggi: $trips")
        assertTrue(applied >= 10, "tratti fissi nel registro solo in $applied viaggi")
    }
}
