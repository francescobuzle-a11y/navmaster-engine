package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.Detour
import it.navmaster.engine.routing.DetourSearch
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.weighting.EdgePenalties
import it.navmaster.engine.weighting.EdgePenaltyWeighting
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.util.Random
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertSame
import kotlin.test.assertTrue

/** [EdgeInfo.normalizeRef] e la cache dei ref per puntatore. */
class EdgeInfoTest {
    @Test
    fun `refs are normalized`() {
        assertEquals("A1", EdgeInfo.normalizeRef("A1, E35"))
        assertEquals("A5", EdgeInfo.normalizeRef("A 5"))
        assertEquals("A4", EdgeInfo.normalizeRef("A4;A22"))
        assertEquals("E35", EdgeInfo.normalizeRef("E35"))
        assertEquals("E35", EdgeInfo.normalizeRef("E 35; E45"))
        assertEquals("RA6", EdgeInfo.normalizeRef("RA6"))
        assertEquals("SS9", EdgeInfo.normalizeRef("ss 9"))
        assertNull(EdgeInfo.normalizeRef(""))
        assertNull(EdgeInfo.normalizeRef("Tangenziale"))
    }

    @Test
    fun `refs are read once per edge and the same string has the same id`() {
        val g = Heidelberg.graph
        val info = EdgeInfo(g.graph, g.encoding.shared, g.guidance)
        val a5 = (0 until g.graph.edgeCount).filter { info.ref(it) == "A5" }
        assertTrue(a5.size > 10, "archi A5: ${a5.size}")
        // ogni arco A5 è autostrada, e tutti hanno lo stesso indice (anche con puntatori diversi)
        assertTrue(a5.all { info.isMotorway(it) })
        assertEquals(1, a5.map { info.refId(it) }.toSet().size)
        assertEquals(info.idOf("A5"), info.refId(a5[0]))
        assertSame(info.ref(a5[0]), info.ref(a5[1]))
        assertEquals("A5", g.guidance!!.string(g.guidance!!.refPointer(a5[0])).replace(" ", "").take(2))
    }
}

/**
 * Le penalità sugli archi esposti al core sono esatte: core-ALT penalizzato = ricerca piena edge-based
 * con lo stesso peso; stesso nome del peso, stessi tempi; penalizzando anche archi non esposti no.
 */
class EdgePenaltyExactnessTest {
    private val g get() = Heidelberg.graph
    private val opts = RouteOptions(profile = "driving-hgv")

    private fun pairs(n: Int): List<Pair<Pair<Double, Double>, Pair<Double, Double>>> {
        val pts = Heidelberg.points(30, 42)
        val rnd = Random(42)
        return List(n) { val i = rnd.nextInt(pts.size); var j = rnd.nextInt(pts.size); if (j == i) j = (i + 1) % pts.size; pts[i] to pts[j] }
    }

    /** Pesi core e pieni con [penalties] (null: nessuna) sui percorsi di [n] coppie; conta quelle che differiscono. */
    private fun compare(n: Int, penalties: (RoutePath) -> EdgePenalties, mustAgree: Boolean): Int {
        val base = g.router(opts)
        val fullBase = g.router(opts.copy(allowCH = false))
        var compared = 0; var differ = 0
        for ((a, b) in pairs(n)) {
            val s0 = base.snap(a.first, a.second, 400.0, 0); val s1 = base.snap(b.first, b.second, 400.0, 1)
            val best = try { base.route(s0, s1) } catch (e: RouteNotFoundException) { continue }
            // solo dove core e ricerca piena danno lo stesso peso senza penalità
            val bf = fullBase.route(s0, s1)
            if (Math.abs(bf.weight - best.weight) > 1e-6 * best.weight) continue
            val p = penalties(best)
            val core = g.router(opts, p)
            assertTrue(core.algorithm.startsWith("core_alt"), core.algorithm)
            val full = g.router(opts.copy(allowCH = false), p)
            assertTrue(full.algorithm.startsWith("dijkstra"), full.algorithm)
            val wc = core.route(s0, s1).weight; val wf = full.route(s0, s1).weight
            compared++
            if (Math.abs(wc - wf) > 1e-6 * wf) { differ++; if (mustAgree) throw AssertionError("core $wc, piena $wf") }
        }
        assertTrue(compared > n / 2, "coppie confrontate: $compared")
        return differ
    }

    @Test
    fun `exposed edges of the best route x3 - core equals full search`() {
        val expo = g.router(opts, EdgePenalties(1))
        compare(200, { best -> EdgePenalties().also { p -> for (e in best.edges) if (e.edge !in p && expo.isCoreExposed(e.edge)) p.multiply(e.edge, 3.0) } }, mustAgree = true)
    }

    @Test
    fun `motorways x10 - core equals full search`() {
        val info = EdgeInfo(g.graph, g.encoding.shared, g.guidance)
        val expo = g.router(opts, EdgePenalties(1))
        val p = EdgePenalties()
        for (e in 0 until g.graph.edgeCount) if (info.isMotorway(e) && info.ref(e) in setOf("A5", "A656") && expo.isCoreExposed(e)) p.multiply(e, 10.0)
        assertTrue(p.size > 10, "archi A5/A656 penalizzati: ${p.size}")
        compare(200, { p }, mustAgree = true)
    }

    @Test
    fun `penalizing edges hidden in shortcuts is not exact`() {
        // controprova: anche gli archi non esposti (dentro gli shortcut la penalità non si vede)
        val differ = compare(200, { best -> EdgePenalties().also { p -> for (e in best.edges) if (e.edge !in p) p.multiply(e.edge, 3.0) } }, mustAgree = false)
        assertTrue(differ > 0, "nessuna coppia differisce penalizzando archi non esposti")
    }

    @Test
    fun `the penalized weighting keeps name and times`() {
        val pe = g.encoding.profile("heavyvehicle")
        val base = WeightingFactory.create(g.encoding, pe, WeightingMethod.RECOMMENDED)
        val p = EdgePenalties().also { it.multiply(5, 4.0); it.multiply(5, 2.0) }
        assertEquals(8.0, p.factor(5)); assertEquals(1.0, p.factor(6))
        val w = EdgePenaltyWeighting(base, p)
        assertEquals(base.name, w.name)
        val it = g.graph.createEdgeIterator()
        for (e in 0 until 50) {
            it.setEdge(e, g.graph.nodeA(e))
            assertEquals(base.edgeMillis(it, false), w.edgeMillis(it, false))
            val b = base.edgeWeight(it, false)
            if (b.isFinite()) assertEquals(b * p.factor(e), w.edgeWeight(it, false), 1e-9)
        }
        // stessi archi e fattori in altro ordine: stessa impronta; covers, touches, times
        val q = EdgePenalties().also { it.multiply(5, 2.0); it.multiply(5, 4.0) }
        assertEquals(p.fingerprint, q.fingerprint)
        val small = EdgePenalties().also { it.multiply(5, 3.0) }
        assertTrue(p.covers(small)); assertTrue(!small.covers(p))
        assertTrue(p.touches(intArrayOf(1, 5))); assertTrue(!p.touches(intArrayOf(1, 6)))
        assertEquals(24.0, p.times(small).factor(5))
    }
}

/** Il contesto delle varianti per [body] (Heidelberg, camion) e la sua fonte P. */
private fun penaltyContext(body: Map<String, Any?>, opt: NavAlternativesOptions = NavAlternativesOptions(sources = setOf(VariantSource.PENALTY), minDetourScale = 0.02)):
        Pair<VariantContext, PenaltyCandidates> {
    val service = DirectionsService(Heidelberg.graph)
    val rc = RequestContext(useCache = false)
    val ctx = NavAlternativesEngine(service, rc).prepare(DirectionsRequest.fromJson("driving-hgv", body), opt)
    val pc = PenaltyCandidates(service, ctx.solved, rc, {}, ctx)
    assertTrue(pc.supported())
    return ctx to pc
}

/** Fonte P: i tratti senza archi penalizzati si riusano (stessi oggetti, nessuna ricerca); la regola di dedup esatta. */
class LegReuseTest {
    @Test
    fun `legs without penalized edges are reused - one search, same result as a full recompute`() {
        var done = 0
        for (seed in 1L..40L) {
            val pts = Heidelberg.points(4, seed)
            val (ctx, pc) = penaltyContext(Heidelberg.body(pts))
            val legs = ctx.solved.legs
            val expo = Heidelberg.graph.router(ctx.solved.options, EdgePenalties(1))
            val others = (legs[0].edges + legs[2].edges).map { it.edge }.toSet()
            val p = EdgePenalties()
            for (e in legs[1].edges) if (e.edge !in others && e.edge !in p && expo.isCoreExposed(e.edge)) p.multiply(e.edge, 3.0)
            if (p.size == 0) continue
            val r = pc.run(PenaltyCandidates.Candidate(PenaltyCandidates.Kind.SP, p, "prova", spFactor = 3.0), CancelToken.NEVER)
            val d = assertNotNull(r.draft, r.line)
            assertSame(legs[0], d.legs[0]); assertSame(legs[2], d.legs[2])
            assertEquals(1, r.legs.count { it.source == LegStats.Source.PENALIZED }, r.line)
            assertEquals(2, r.legs.count { it.source == LegStats.Source.REUSED })
            // = tutti i tratti ricalcolati con le penalità
            val router = Heidelberg.graph.router(ctx.solved.options, p)
            for (j in 0 until 3) {
                val full = router.route(ctx.solved.snaps[j], ctx.solved.snaps[j + 1])
                assertEquals(full.weight, d.legs[j].weight, 1e-6 * full.weight, "tratto $j")
            }
            // il tratto leggero, completato, ha la geometria e i nomi
            val m = ctx.router.materialize(d.legs[1])
            assertTrue(m.edges.all { it.geometry.size >= 2 }); assertEquals(d.legs[1].weight, m.weight)
            if (++done >= 3) break
        }
        assertTrue(done > 0, "nessun viaggio con archi esposti nel tratto 2")
    }

    @Test
    fun `a superset of penalties whose path avoids them all gives the same path`() {
        var done = 0
        for (seed in 1L..60L) {
            val pts = Heidelberg.points(2, seed + 100)
            val (ctx, pc) = penaltyContext(Heidelberg.body(pts))
            val expo = Heidelberg.graph.router(ctx.solved.options, EdgePenalties(1))
            val p0 = EdgePenalties()
            for (e in ctx.solved.legs[0].edges) if (e.edge !in p0 && expo.isCoreExposed(e.edge)) p0.multiply(e.edge, 10.0)
            if (p0.size == 0) continue
            val q = pc.run(PenaltyCandidates.Candidate(PenaltyCandidates.Kind.SP, p0, "P0", spFactor = 10.0), CancelToken.NEVER).draft ?: continue
            if (p0.touches(q.edgeIds)) continue
            // P ⊇ P0, con altri archi esposti che Q non usa
            val extra = EdgePenalties()
            val qs = q.edgeIds.toSet()
            for (e in 0 until Heidelberg.graph.graph.edgeCount) if (e !in qs && e !in p0 && expo.isCoreExposed(e) && extra.size < 500) extra.multiply(e, 2.0)
            val p = p0.times(extra)
            assertTrue(p.covers(p0)); assertTrue(!p.touches(q.edgeIds))
            val q2 = assertNotNull(pc.run(PenaltyCandidates.Candidate(PenaltyCandidates.Kind.SP, p, "P", spFactor = 10.0), CancelToken.NEVER).draft)
            assertEquals(q.weight, q2.weight, 1e-6 * q.weight)
            if (++done >= 3) break
        }
        assertTrue(done > 0, "nessun caso per la regola di dedup")
    }
}

/** La regola di accettazione su percorsi sintetici (archi veri di Heidelberg, tempi e lunghezze finti). */
class VariantRulesTest {
    private val g get() = Heidelberg.graph
    private val info by lazy { EdgeInfo(g.graph, g.encoding.shared, g.guidance) }
    private val plain by lazy { (0 until g.graph.edgeCount).filter { !info.isMotorwayOrTrunk(it) && info.ref(it) == null } }
    private val a5 by lazy { (0 until g.graph.edgeCount).filter { info.ref(it) == "A5" && info.isMotorway(it) } }
    private val l536 by lazy { (0 until g.graph.edgeCount).filter { info.ref(it) == "L536" } }

    private fun path(ids: List<Int>, ms: (Int) -> Long = { 36_000L }, m: Double = 1000.0): Draft =
        Draft(listOf(RoutePath(ids.mapIndexed { i, e -> PathEdge(e, false, i, i + 1, PointList(0), m, ms(i), ms(i) / 1000.0, "") }, ids.size * m,
            ids.indices.sumOf { ms(it) }, ids.indices.sumOf { ms(it) / 1000.0 })), VariantSource.PENALTY)

    /** [best] con gli archi [from]..[to] (indici) sostituiti da [ids], ognuno di [msEach] ms. */
    private fun replace(best: List<Int>, from: Int, to: Int, ids: List<Int>, msEach: Long): Draft {
        val all = best.subList(0, from) + ids + best.subList(to + 1, best.size)
        return path(all, { i -> if (i in from until from + ids.size) msEach else 36_000L })
    }

    private fun judge(best: Draft, opt: NavAlternativesOptions = NavAlternativesOptions()) =
        VariantJudge(best, info, CorridorAnalyzer(g.graph, info, g.guidance), opt, g.graph)

    @Test
    fun `dMin1`() {
        assertEquals(15_000.0, VariantRules.dMin1(100_000.0))
        assertEquals(40_000.0, VariantRules.dMin1(1_000_000.0))
        assertEquals(2_500.0, VariantRules.dMin1(10_000.0))
        assertEquals(25_000.0, VariantRules.dMin1(500_000.0))
    }

    @Test
    fun `time thresholds 1_3 and 1_5`() {
        val b = plain.take(100)
        val best = path(b)
        val j = judge(best)
        val fresh = plain.subList(100, 130)
        // 30 km nuovi al posto di 30 km: +1044 s (r 1.29), +1116 s (1.31), +1836 s (1.51)
        assertEquals(1, j.evaluate(replace(b, 30, 59, fresh, 36_000 + 34_800), listOf(best), false).tier)
        assertEquals(2, j.evaluate(replace(b, 30, 59, fresh, 36_000 + 37_200), listOf(best), false).tier)
        val v = j.evaluate(replace(b, 30, 59, fresh, 36_000 + 61_200), listOf(best), true)
        assertEquals(0, v.tier); assertEquals("tempo", v.reason)
        // i km: 1.10 di km come limite del livello 1
        val t = NavAlternativesOptions(tier1Factor = 1.1, maxFactor = 1.2, tier1KmFactor = 1.1, maxKmFactor = 1.2)
        val longer = path(b.subList(0, 30) + plain.subList(100, 145) + b.subList(60, 100))  // +15 km, +540 s (r 1.15)
        assertEquals(2, judge(best, t).evaluate(longer, listOf(best), false).tier)
    }

    @Test
    fun `short deviations, loops and duplicates`() {
        val b = plain.take(100)
        val best = path(b)
        val j = judge(best)
        // 10 km di deviazione: sotto dMin1 (15 km), livello 2
        assertEquals(2, j.evaluate(replace(b, 30, 39, plain.subList(200, 210), 40_000), listOf(best), false).tier)
        // un anello di 30 km fra due archi vicini: scartato per la sanità locale
        val loop = replace(b, 50, 51, plain.subList(300, 330), 36_000)
        val v = j.evaluate(loop, listOf(best), true)
        assertEquals(0, v.tier); assertEquals("locale", v.reason)
        // doppioni: del migliore e di una tenuta
        val good = replace(b, 30, 59, plain.subList(100, 130), 40_000)
        assertEquals("doppione", j.evaluate(path(b), listOf(best), false).reason)
        assertEquals("doppione", j.evaluate(replace(b, 30, 59, plain.subList(100, 130), 40_000), listOf(best, good), false).reason)
        // una seconda variante sullo stesso tratto della tenuta non è abbastanza diversa da essa
        val near = replace(b, 30, 59, plain.subList(100, 128) + plain.subList(400, 402), 40_000)
        val nv = j.evaluate(near, listOf(best, good), false)
        assertEquals(0, nv.tier); assertEquals("diversità", nv.reason)
    }

    private fun leg(ids: List<Int>, ms: (Int) -> Long = { 36_000L }, nodes: (Int) -> Pair<Int, Int> = { i -> i to i + 1 }) =
        RoutePath(ids.mapIndexed { i, e -> PathEdge(e, false, nodes(i).first, nodes(i).second, PointList(0), 1000.0, ms(i), ms(i) / 1000.0, "") }, ids.size * 1000.0,
            ids.indices.sumOf { ms(it) }, ids.indices.sumOf { ms(it) / 1000.0 })

    @Test
    fun `a round trip anchors each deviation on its own leg`() {
        val go = plain.take(100); val back = go.reversed()
        val best = Draft(listOf(leg(go), leg(back)), VariantSource.BEST)
        val j = judge(best)
        // il ritorno lascia il migliore per 30 km nuovi (+10 min): le ancore stanno nel ritorno, non nell'andata
        val backV = back.subList(0, 30) + plain.subList(100, 130) + back.subList(60, 100)
        val v = Draft(listOf(leg(go), leg(backV, { i -> if (i in 30 until 60) 56_000L else 36_000L })), VariantSource.PENALTY)
        val run = j.runsFromBest(v).single()
        assertEquals(129, run.anchorFrom); assertEquals(160, run.anchorTo)
        assertEquals(30 * 36_000L, j.bestMsBetween(run))
        assertEquals(1, j.evaluate(v, listOf(best), false).tier)
    }

    @Test
    fun `the node behind a partial first edge and beyond a partial last edge are not repeats`() {
        val b = plain.take(10)
        val best = Draft(listOf(leg(b)), VariantSource.BEST)
        val j = judge(best)
        // il primo arco parte da 50 (dietro la partenza), l'ultimo arriva a 3 (oltre l'arrivo): il veicolo non ci passa due volte
        val v = Draft(listOf(leg(plain.subList(20, 30), nodes = { i -> when (i) { 0 -> 50 to 100; 9 -> 108 to 103; 5 -> 104 to 50; else -> 100 + i - 1 to 100 + i } })), VariantSource.PENALTY)
        assertTrue(j.valid(v))
        // un nodo davvero ripetuto dentro il tratto
        val loop = Draft(listOf(leg(plain.subList(20, 30), nodes = { i -> if (i == 6) 105 to 102 else 100 + i - 1 to 100 + i })), VariantSource.PENALTY)
        assertTrue(!j.valid(loop))
    }

    @Test
    fun `a non-motorway deviation from a motorway best route goes to tier 2`() {
        assertTrue(a5.size >= 40, "archi A5: ${a5.size}")
        val b = a5.take(40) + plain.take(60)
        val best = path(b)
        val j = judge(best)
        assertTrue(j.bestMwShare >= 0.3)
        val v = j.evaluate(replace(b, 5, 34, plain.subList(200, 230), 37_000), listOf(best), false)
        assertEquals(2, v.tier); assertEquals("autostrade", v.reason)
        assertTrue(v.label.isNotEmpty())
    }

    @Test
    fun `new roads name the variant, only new motorways come first`() {
        // (prima ogni strada nuova dava il bonus, anche la L536: ora solo le autostrade)
        assertTrue(l536.size >= 6, "archi L536: ${l536.size}")
        assertTrue(a5.size >= 30, "archi A5: ${a5.size}")
        val b = plain.take(100)
        val best = path(b)
        val j = judge(best)
        val plainV = replace(b, 30, 59, plain.subList(100, 130), 37_000)
        val withRef = replace(b, 30, 59, l536.take(6) + plain.subList(500, 524), 37_000)
        val v = j.evaluate(withRef, listOf(best), false)
        val pv = j.evaluate(plainV, listOf(best), false)
        // la strada nuova dà il nome, ma la L536 non è un'autostrada: nessun bonus
        assertEquals(listOf("L536"), v.newRefs); assertEquals("via L536", v.label)
        assertEquals(emptyList(), j.motorwayRefs(withRef, v.newRefs))
        assertEquals(pv.score, v.score, 1e-12)
        // l'A5 al posto dello stesso pezzo, stessi tempi e km: il bonus dell'autostrada nuova e il premio della quota autostradale
        val withA5 = replace(b, 30, 59, a5.take(30), 37_000)
        val va = j.evaluate(withA5, listOf(best), false)
        assertEquals(listOf("A5"), va.newRefs); assertEquals("via A5", va.label)
        assertEquals(pv.score - VariantRules.NEW_REF_BONUS - VariantRules.MW_SWAP_BONUS, va.score, 1e-9)
        // la scelta: una sola variante, quella sull'autostrada nuova
        val pool = VariantPool(j, NavAlternativesOptions(count = 2))
        pool.drafts += plainV; pool.drafts += withRef; pool.drafts += withA5
        pool.admit(1)
        assertEquals(1, pool.accepted.size); assertSame(withA5, pool.accepted[0].first)
    }

    @Test
    fun `a trunk road is not a new motorway`() {
        // le superstrade con un ref (statali come la SS42 della prova Bolzano→Milano): strade nuove sì, bonus no
        // (una superstrada con un ref da autostrada, RA/T/E45, conta invece come autostrada)
        val trunk = (0 until g.graph.edgeCount).filter { info.isMotorwayOrTrunk(it) && !info.isMotorway(it) && info.ref(it)?.let { r -> !CorridorAnalyzer.isMotorwayRef(r) } == true }
        val byRef = trunk.groupBy { info.ref(it)!! }.maxByOrNull { it.value.size }
        println("superstrade con ref su Heidelberg: " + trunk.groupingBy { info.ref(it) }.eachCount())
        if (byRef == null || byRef.value.size < 6) return
        val b = plain.take(100)
        val best = path(b)
        val j = judge(best)
        val plainV = replace(b, 30, 59, plain.subList(100, 130), 37_000)
        val withTrunk = replace(b, 30, 59, byRef.value.take(6) + plain.subList(500, 524), 37_000)
        val v = j.evaluate(withTrunk, listOf(best), false)
        assertEquals(listOf(byRef.key), v.newRefs)
        assertEquals(emptyList(), j.motorwayRefs(withTrunk, v.newRefs))
        assertEquals(j.evaluate(plainV, listOf(best), false).score, v.score, 1e-12)
    }

    @Test
    fun `motorway in place of plain road earns the swap premium, the reverse pays it`() {
        assertTrue(a5.size >= 40, "archi A5: ${a5.size}")
        // il migliore ha già 10 km di A5: l'A5 della variante non è una strada nuova, resta solo il premio
        val b = a5.take(10) + plain.take(90)
        val best = path(b)
        val j = judge(best)
        val d1 = VariantRules.dMin1(best.meters)
        val plainV = replace(b, 40, 69, plain.subList(100, 130), 37_000)
        val mwV = replace(b, 40, 69, a5.subList(10, 40), 37_000)
        val vp = j.evaluate(plainV, listOf(best), false); val vm = j.evaluate(mwV, listOf(best), false)
        assertEquals(emptyList(), vm.newRefs)
        assertEquals(1.0, j.motorwayGain(j.runsFromBest(mwV), d1), 1e-9)
        assertEquals(vp.score - VariantRules.MW_SWAP_BONUS, vm.score, 1e-9)
        // un pezzo sostituito corto (6 km su dMin1 15 km) vale in proporzione
        val shortV = replace(b, 40, 45, a5.subList(10, 16), 37_000)
        assertEquals(6_000.0 / d1, j.motorwayGain(j.runsFromBest(shortV), d1), 1e-9)
        // una statale al posto dell'autostrada: nessun premio, lo stesso come malus
        val best2 = path(a5.take(40) + plain.take(60))
        val j2 = judge(best2)
        val down = replace(a5.take(40) + plain.take(60), 5, 34, plain.subList(200, 230), 36_000)
        val d2 = VariantRules.dMin1(best2.meters)
        assertEquals(0.0, j2.motorwayGain(j2.runsFromBest(down), d2), 1e-12)
        val loss = j2.motorwayGain(j2.runsFromBest(down), d2, loss = true)
        assertTrue(loss > 0.5, "perdita $loss")
        val r = down.millis.toDouble() / best2.millis; val rk = down.meters / best2.meters
        assertEquals(0.6 * r + 0.4 * rk + VariantRules.MW_SWAP_BONUS * loss, j2.evaluate(down, listOf(best2), false).score, 1e-9)
        // la scelta fra le due varianti di pari tempo e km: quella che porta sull'autostrada
        val pool = VariantPool(j, NavAlternativesOptions(count = 2))
        pool.drafts += plainV; pool.drafts += mwV
        pool.admit(1)
        assertSame(mwV, pool.accepted.single().first)
    }
}

/** `navmaster.alternatives` attraverso il servizio: forma della risposta, il migliore invariato, errori. */
class NavAlternativesServiceTest {
    private val g get() = Heidelberg.graph
    private val opt = NavAlternativesOptions(count = 3, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02)

    @Suppress("UNCHECKED_CAST")
    private fun features(doc: Map<String, Any?>) = doc["features"] as List<Map<String, Any?>>

    /** La feature senza `properties.navmaster.variant` (come la dà una richiesta senza varianti). */
    @Suppress("UNCHECKED_CAST")
    private fun withoutVariant(f: Map<String, Any?>): String {
        val props = LinkedHashMap(f["properties"] as Map<String, Any?>)
        val nm = (props["navmaster"] as? Map<String, Any?>)?.let { LinkedHashMap(it) }
        nm?.remove("variant")
        if (nm == null || nm.isEmpty()) props.remove("navmaster") else props["navmaster"] = nm
        return Json.stringify(LinkedHashMap(f).also { it["properties"] = props })
    }

    @Test
    fun `variants with 2, 3 and 4 points, with and without pass_through`() {
        val service = DirectionsService(g)
        var variants = 0
        for (n in 2..4) for (pt in listOf(false, true)) for (seed in listOf(5L, 11L)) {
            val pts = Heidelberg.points(n, seed + n)
            val extra = if (pt) ""","navmaster":{"pass_through":[${pts.indices.joinToString(",") { (it > 0 && it < n - 1).toString() }}]}""" else ""
            val plain = features(service.handle(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, extra)), ResponseFormat.GEOJSON))[0]
            val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, extra)).withNavAlternatives(opt)
            val doc = service.handle(req, ResponseFormat.GEOJSON, RequestContext())
            val fs = features(doc)
            assertTrue(fs.isNotEmpty())
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]), "la prima feature è il migliore")
            val d0 = ((plain["properties"] as Map<*, *>)["summary"] as Map<*, *>)["duration"] as Double
            for ((i, f) in fs.withIndex()) {
                val props = f["properties"] as Map<*, *>
                assertEquals(n, (props["way_points"] as List<*>).size)
                val v = (props["navmaster"] as Map<*, *>)["variant"] as Map<*, *>
                assertEquals(i, v["rank"])
                assertTrue(((props["summary"] as Map<*, *>)["duration"] as Double) <= 1.5 * d0 + 1)
            }
            variants += fs.size - 1
            val meta = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            assertEquals(3, meta["asked"]); assertEquals(fs.size, meta["found"])
            println("varianti $n punti pass_through=$pt seme $seed: ${fs.size - 1} (${meta["candidates"]}; ${meta["note"]})")
        }
        assertTrue(variants > 0, "nessuna variante su Heidelberg")
    }

    @Test
    fun `the variants trace changes nothing in the answer`() {
        var traced = 0; var printed = 0
        for (seed in 1L..12L) {
            val pts = Heidelberg.points(2, seed + 300)
            val body = Heidelberg.body(pts)
            fun run(o: NavAlternativesOptions, log: ((String) -> Unit)? = null): Map<String, Any?> {
                val doc = DirectionsService(g).handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(o), ResponseFormat.GEOJSON, RequestContext(log = log, useCache = false))
                // l'ora e il tempo della richiesta sono le uniche differenze ammesse
                @Suppress("UNCHECKED_CAST")
                val meta = LinkedHashMap((doc["metadata"] as Map<String, Any?>)).also { it.remove("timestamp") }; val nm = LinkedHashMap(meta["navmaster"] as Map<String, Any?>)
                nm["alternatives"] = LinkedHashMap(nm["alternatives"] as Map<String, Any?>).also { it.remove("ms") }
                meta["navmaster"] = nm
                return LinkedHashMap(doc).also { it["metadata"] = meta }
            }
            val plain = run(opt)
            // una sonda: il percorso per un punto in più
            val extra = Heidelberg.points(1, seed + 900)[0]
            val probeLegs = try { DirectionsService(g).solve(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(listOf(pts[0], extra, pts[1])))).legs } catch (e: Exception) { null }
            val lines = ArrayList<String>()
            val withTrace = run(opt.withTrace(VariantTrace(listOfNotNull(probeLegs?.let { VariantTrace.Probe("prova", it) }))), { lines += it })
            assertEquals(Json.stringify(plain), Json.stringify(withTrace), "seme $seed")
            // il primo viaggio con la traccia nel registro della prova, come esempio
            if (lines.any { it.startsWith("varianti: traccia") }) traced++
            if (lines.any { it.startsWith("varianti: traccia M rete") } && printed++ == 0) for (l in lines) println(l)
            if (probeLegs != null && lines.any { it.startsWith("varianti: traccia M rete") }) assertTrue(lines.any { it.startsWith("varianti: traccia sonda prova:") }, lines.joinToString("\n"))
        }
        assertTrue(traced > 0, "nessuna riga di traccia")
    }

    @Test
    fun `motorway detours alone - spliced variants, no searches, off when avoiding motorways`() {
        val service = DirectionsService(g)
        // soglie a 0,3: a 0,02 la sanità locale del giudice (2·migliore + 1200 s in scala: 24 s) scarta ogni
        // deviazione di Heidelberg, e la fonte M non le offre più
        val mOnly = NavAlternativesOptions(count = 3, sources = setOf(VariantSource.MOTORWAY), minDetourScale = 0.3)
        var variants = 0; var withDetours = 0
        for (seed in 1L..40L) {
            val pts = Heidelberg.points(if (seed % 3 == 0L) 3 else 2, seed + 300)
            val body = Heidelberg.body(pts)
            val plain = features(service.handle(DirectionsRequest.fromJson("driving-hgv", body), ResponseFormat.GEOJSON))[0]
            val d0 = ((plain["properties"] as Map<*, *>)["summary"] as Map<*, *>)["duration"] as Double
            val rc = RequestContext()
            val doc = service.handle(DirectionsRequest.fromJson("driving-hgv", body).withNavAlternatives(mOnly), ResponseFormat.GEOJSON, rc)
            val fs = features(doc)
            assertEquals(withoutVariant(plain), withoutVariant(fs[0]))
            val meta = ((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            val cand = meta["candidates"] as String
            if (!cand.startsWith("M 0 ")) { withDetours++; println("fonte M, seme $seed: $cand; ${meta["note"]}") }
            // le deviazioni cucite sono percorsi validi (svolte, nodi, way_points): nessuna scartata come non valida
            assertTrue("non valido" !in cand, cand)
            for (f in fs.drop(1)) {
                val props = f["properties"] as Map<*, *>
                val v = (props["navmaster"] as Map<*, *>)["variant"] as Map<*, *>
                assertEquals("motorway", v["source"])
                assertEquals(pts.size, (props["way_points"] as List<*>).size)
                assertTrue(((props["summary"] as Map<*, *>)["duration"] as Double) <= 1.5 * d0 + 1)
                variants++
            }
            // nessuna ricerca oltre al migliore: la fonte M cuce soltanto
            assertTrue(rc.stats.children.none { (name, st) -> name == "varianti" && st.legs.any { it.source == LegStats.Source.PENALIZED } })
            // evitando le autostrade la fonte M non vale
            val avoid = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts, ""","options":{"avoid_features":["highways"]}""")).withNavAlternatives(mOnly)
            val am = ((service.handle(avoid, ResponseFormat.GEOJSON)["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>
            assertEquals("nessuna fonte", (am["candidates"] as String).substringBefore(" |"))
        }
        println("fonte M su Heidelberg: $withDetours viaggi con deviazioni, $variants varianti")
        assertTrue(withDetours > 0, "nessuna deviazione autostradale su Heidelberg")
    }

    @Test
    fun `a kept motorway detour is materialized, legal and in the JSON`() {
        val service = DirectionsService(g)
        // soglie a 0,3: su Heidelberg alcune deviazioni autostradali passano la regola
        val mOnly = NavAlternativesOptions(count = 3, tier1Factor = 1.5, maxFactor = 2.0, sources = setOf(VariantSource.MOTORWAY), minDetourScale = 0.3)
        var kept = 0
        for (seed in 1L..40L) {
            val pts = Heidelberg.points(if (seed % 3 == 0L) 3 else 2, seed + 300)
            val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts)).withNavAlternatives(mOnly)
            val r = NavAlternativesEngine(service, RequestContext()).run(req, mOnly)
            val router = service.routingGraph.router(r.best.options)
            for (a in r.variants) {
                assertEquals(VariantSource.MOTORWAY, a.draft.source)
                assertEquals(r.best.legs.size, a.legs.size)
                for ((j, leg) in a.legs.withIndex()) {
                    assertTrue(router.isLegal(leg), "seme $seed tratto $j")
                    assertTrue(leg.edges.all { it.geometry.size >= 2 })
                    // gli estremi del tratto sono quelli del migliore
                    assertEquals(r.best.legs[j].edges.first().edge, leg.edges.first().edge)
                    assertEquals(r.best.legs[j].edges.last().edge, leg.edges.last().edge)
                    assertEquals(leg.edges.sumOf { it.millis }, leg.millis)
                }
                val v = r.json(a)
                assertEquals("motorway", v["source"])
                assertTrue((v["detours"] as List<*>).isNotEmpty(), "$v")
                assertTrue((v["extra_s"] as Double) >= 0.0)
                kept++
            }
            if (r.variants.isNotEmpty()) {
                // lo stesso attraverso il servizio: le feature delle varianti con la fonte M
                val fs = features(service.handle(req, ResponseFormat.GEOJSON))
                assertEquals(1 + r.variants.size, fs.size)
                for (f in fs.drop(1)) {
                    val props = f["properties"] as Map<*, *>
                    assertEquals("motorway", ((props["navmaster"] as Map<*, *>)["variant"] as Map<*, *>)["source"])
                    assertTrue(((f["geometry"] as Map<*, *>)["coordinates"] as List<*>).size >= 2)
                }
            }
        }
        println("varianti M tenute su Heidelberg (soglie 0,3): $kept")
        assertTrue(kept > 0, "nessuna variante M tenuta")
    }

    @Test
    fun `the motorway net is never stopped by the candidates time and is cached`() {
        val service = DirectionsService(g)
        val opt0 = NavAlternativesOptions(count = 3, budgetMs = 0, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02)
        // un viaggio con una rete autostradale; senza la cache della richiesta la rete non entra in quella del servizio
        val req = (301L..380L).asSequence().map { DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(Heidelberg.points(2, it))) }.first { rq ->
            val ls = ArrayList<String>()
            NavAlternativesEngine(service, RequestContext(log = { ls += it }, useCache = false)).run(rq, opt0)
            ls.any { "M rete" in it }
        }
        assertEquals(0, service.netCache.size)
        // tempo dei candidati finito subito (budget_ms 0: nemmeno il minimo delle fonti): nessuna variante, ma la rete è costruita per intero e resta
        val lines = java.util.Collections.synchronizedList(ArrayList<String>())
        val r = NavAlternativesEngine(service, RequestContext(log = { lines += it })).run(req, opt0)
        assertEquals("budget", r.note); assertEquals(0, r.variants.size)
        assertEquals(1, service.netCache.size)
        assertTrue(lines.any { "M rete" in it && "autostrade costruite" in it && "troncata" !in it }, "$lines")
        // la richiesta dopo la trova in cache
        lines.clear()
        NavAlternativesEngine(service, RequestContext(log = { lines += it })).run(req, NavAlternativesOptions(count = 3, sources = setOf(VariantSource.MOTORWAY), minDetourScale = 0.02))
        assertTrue(lines.any { "M rete" in it && "autostrade dalla cache" in it }, "$lines")
    }

    @Test
    fun `with a helper thread the answer has the same shape and the slots are all released`() {
        val service = DirectionsService(g)
        val pool = java.util.concurrent.Executors.newFixedThreadPool(1)
        val slots = SearchSlots(2)
        try {
            for (seed in listOf(5L, 9L)) {
                val pts = Heidelberg.points(4, seed)
                val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts)).withNavAlternatives(opt)
                val seq = features(service.handle(req, ResponseFormat.GEOJSON, RequestContext(useCache = false)))
                val par = features(service.handle(req, ResponseFormat.GEOJSON, RequestContext(executor = pool, slots = slots, useCache = false)))
                // le stesse varianti con e senza aiutante (le coppie di candidati non dipendono dal posto libero)
                assertEquals(seq.size, par.size)
                for (i in seq.indices) assertEquals(Json.stringify(seq[i]), Json.stringify(par[i]), "feature $i, seme $seed")
                for ((i, f) in par.withIndex()) {
                    val props = f["properties"] as Map<*, *>
                    assertEquals(4, (props["way_points"] as List<*>).size)
                    assertEquals(i, ((props["navmaster"] as Map<*, *>)["variant"] as Map<*, *>)["rank"])
                }
                assertEquals(0, slots.inUse)
            }
        } finally { pool.shutdownNow() }
    }

    @Test
    fun `exclude_best, budget and cancellation`() {
        val service = DirectionsService(g)
        val pts = Heidelberg.points(2, 5)
        val base = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts))
        val ex = features(service.handle(base.withNavAlternatives(NavAlternativesOptions(count = 3, excludeBest = true, sources = setOf(VariantSource.PENALTY), minDetourScale = 0.02)), ResponseFormat.GEOJSON))
        for (f in ex) assertTrue((((f["properties"] as Map<*, *>)["navmaster"] as Map<*, *>)["variant"] as Map<*, *>)["rank"] as Int >= 1)
        // tempo finito subito: solo il migliore, con la nota
        val doc = service.handle(base.withNavAlternatives(NavAlternativesOptions(count = 3, budgetMs = 0, sources = setOf(VariantSource.PENALTY))), ResponseFormat.GEOJSON)
        assertEquals(1, features(doc).size)
        assertEquals("budget", (((doc["metadata"] as Map<*, *>)["navmaster"] as Map<*, *>)["alternatives"] as Map<*, *>)["note"])
        // annullata: esce come annullamento, non come errore ORS
        val t = CancelToken().also { it.cancel() }
        assertFailsWith<SearchCancelledException> { service.handle(base.withNavAlternatives(opt), ResponseFormat.GEOJSON, RequestContext(cancel = t)) }
    }

    @Test
    fun `errors`() {
        val service = DirectionsService(g)
        val coords = """{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]]"""
        fun parse(s: String) = DirectionsRequest.fromJson("driving-hgv", Json.parse(s) as Map<String, Any?>)
        assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, assertFailsWith<ApiException> { parse("""$coords,"navmaster":{"alternatives":{"count":7}}}""") }.code)
        assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, assertFailsWith<ApiException> { parse("""$coords,"navmaster":{"alternatives":{"count":3,"tier1_factor":1.6}}}""") }.code)
        assertEquals(RoutingErrorCodes.INVALID_PARAMETER_VALUE, assertFailsWith<ApiException> { parse("""$coords,"navmaster":{"alternatives":{"tier1_factor":1.3,"max_factor":1.2}}}""") }.code)
        assertEquals(RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, assertFailsWith<ApiException> {
            parse("""$coords,"alternative_routes":{"target_count":2},"navmaster":{"alternatives":{"count":3}}}""") }.code)
        assertEquals(RoutingErrorCodes.INCOMPATIBLE_PARAMETERS, assertFailsWith<ApiException> {
            service.handle(parse("""$coords,"navmaster":{"alternatives":{"count":3}}}"""), ResponseFormat.JSON) }.code)
        val ok = parse("""$coords,"navmaster":{"alternatives":{"count":4,"tier1_factor":1.1,"max_factor":1.2,"tier1_km_factor":1.1,"max_km_factor":1.2,"budget_ms":25000,"exclude_best":true}}}""")
        val o = assertNotNull(ok.navAlternatives)
        assertEquals(4, o.count); assertEquals(1.2, o.maxKmFactor); assertTrue(o.excludeBest); assertTrue(!o.allowFewerKm)
        assertNull(ok.withoutNavAlternatives().navAlternatives)
        assertNull(ok.withPoints(ok.coordinates, null).navAlternatives)
        assertSame(o, ok.withAvoidPenalty(true).navAlternatives)
    }
}

/**
 * Il tempo e la memoria delle varianti: il tempo delle fonti è budget_ms meno la preparazione (con un
 * minimo), il riempimento si ferma alla sua scadenza tenendo i tenuti fin lì, la fonte M offre al più
 * [NavAlternativesEngine.MAX_M_SETS] insiemi per richiesta.
 */
class NavAlternativesBudgetTest {
    private val g get() = Heidelberg.graph

    @Test
    fun `the sources get budget_ms minus the preparation, at least 3 s`() {
        assertEquals(25_000, NavAlternativesEngine.sourceBudgetMs(25_000, 0))
        assertEquals(20_000, NavAlternativesEngine.sourceBudgetMs(25_000, 5_000))
        // una preparazione lunga (la rete ricostruita): alle fonti resta il minimo
        assertEquals(NavAlternativesEngine.MIN_SOURCE_BUDGET_MS, NavAlternativesEngine.sourceBudgetMs(25_000, 24_000))
        assertEquals(NavAlternativesEngine.MIN_SOURCE_BUDGET_MS, NavAlternativesEngine.sourceBudgetMs(25_000, 40_000))
        // un budget_ms più breve del minimo vale tutto (0: nessun tempo, come prima)
        assertEquals(1_500, NavAlternativesEngine.sourceBudgetMs(1_500, 1_000))
        assertEquals(0, NavAlternativesEngine.sourceBudgetMs(0, 300))
    }

    private fun detour(i: Int) = Detour(i % 5, i, i + 1, IntArray(0), 1.0 + i, 1000L * i, 10.0 * i, 0.0, 0L, 0L, score = i.toDouble())

    @Test
    fun `the motorway sets are capped over the whole trip - combos and the first 8 kept`() {
        val singles = List(100) { detour(it) }
        val combos = List(40) { listOf(singles[it % 8], singles[(it + 1) % 8]) }
        val sets = NavAlternativesEngine.offeredSets(singles, combos)
        assertEquals(NavAlternativesEngine.MAX_M_SETS, sets.size)
        // le singole per prime, nell'ordine di punteggio, poi tutte le combinazioni
        val n = NavAlternativesEngine.MAX_M_SETS - combos.size
        assertEquals(singles.take(n).map { listOf(it) }, sets.take(n))
        assertEquals(combos, sets.drop(n))
        // poche combinazioni: più singole
        assertEquals(NavAlternativesEngine.MAX_M_SETS, NavAlternativesEngine.offeredSets(singles, combos.take(10)).size)
        assertEquals(NavAlternativesEngine.MAX_M_SETS - 10, NavAlternativesEngine.offeredSets(singles, combos.take(10)).count { it.size == 1 })
        // un tetto stretto tiene comunque le prime 8 (le combinazioni vengono da quelle)
        assertEquals(DetourSearch.COMBO_TOP, NavAlternativesEngine.offeredSets(singles, combos, max = 20).count { it.size == 1 })
        // sotto il tetto nulla cambia
        val few = singles.take(12)
        assertEquals(few.map { listOf(it) } + combos.take(5), NavAlternativesEngine.offeredSets(few, combos.take(5)))
    }

    @Test
    fun `the cap orders the detours over the whole trip, not over their own leg`() {
        // un viaggio di 100 min con un tratto di 40 e uno di 60 (peso = minuti): +3 min sul tratto di
        // 40 e +4 min su quello di 60, stessa quota di plateau
        val tripW = 100.0
        val shortLeg = DetourSearch.score(3.0, 2.0, 3.0 + 5.0, 40.0)
        val longLeg = DetourSearch.score(4.0, 2.0, 4.0 + 4.0, 60.0)
        // sul proprio tratto la deviazione del tratto corto valeva peggio (0,075 contro 0,067)...
        assertTrue(shortLeg > longLeg)
        // ...sul viaggio pesa meno (+3 min contro +4): ora viene prima, e oltre il tetto resta lei
        val shortTrip = DetourSearch.score(3.0, 2.0, 3.0 + 5.0, tripW)
        val longTrip = DetourSearch.score(4.0, 2.0, 4.0 + 4.0, tripW)
        assertTrue(shortTrip < longTrip)
    }

    @Test
    fun `the fill stops at the deadline keeping what was admitted, a cancellation leaves`() {
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 3, tier1Factor = 1.5, maxFactor = 2.0, sources = setOf(VariantSource.MOTORWAY), minDetourScale = 0.3)
        var checked = 0
        for (seed in 1L..40L) {
            val pts = Heidelberg.points(if (seed % 3 == 0L) 3 else 2, seed + 300)
            val req = DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts)).withNavAlternatives(opt)
            val r = NavAlternativesEngine(service, RequestContext()).run(req, opt)
            if (r.variants.isEmpty()) continue
            // le stesse varianti in un insieme nuovo, non ancora scelte
            val pool = NavAlternativesEngine(service, RequestContext()).prepare(req, opt).pool
            for (a in r.variants) pool.drafts += a.draft
            val expired = CancelToken(System.nanoTime() - 1)
            val e = assertFailsWith<SearchCancelledException> { pool.admit(2, expired) }
            assertEquals(SearchCancelledException.Reason.DEADLINE, e.reason)
            assertEquals(0, pool.accepted.size, "seme $seed")
            assertTrue("non valutati ${r.variants.size}" in pool.discards(expired), pool.discards(expired))
            assertEquals(SearchCancelledException.Reason.CANCELLED,
                assertFailsWith<SearchCancelledException> { pool.discards(CancelToken().also { it.cancel() }) }.reason)
            // senza scadenza la scelta riprende da lì
            pool.admit(2)
            assertTrue(pool.accepted.isNotEmpty(), "seme $seed")
            assertTrue("non valutati" !in pool.discards())
            if (++checked >= 3) break
        }
        assertTrue(checked > 0, "nessuna variante M su Heidelberg")
    }

    @Test
    fun `with the whole time nothing is cut`() {
        // una richiesta con tutto il tempo: su Heidelberg nessun tetto superato e nessuna scadenza
        val service = DirectionsService(g)
        val opt = NavAlternativesOptions(count = 3, sources = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY), minDetourScale = 0.02)
        for (seed in listOf(5L, 9L)) {
            val lines = java.util.Collections.synchronizedList(ArrayList<String>())
            val r = NavAlternativesEngine(service, RequestContext(log = { lines += it })).run(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(Heidelberg.points(4, seed))), opt)
            assertTrue(lines.none { "oltre il tetto" in it }, "$lines")
            assertTrue("non valutati" !in r.summary, r.summary)
            assertTrue(r.note != "budget", "seme $seed: ${r.summary}")
        }
    }
}
