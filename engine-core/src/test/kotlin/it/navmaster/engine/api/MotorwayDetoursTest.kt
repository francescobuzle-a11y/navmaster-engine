package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.routing.BestTrace
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.Detour
import it.navmaster.engine.routing.DetourSearch
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.Ellipse
import it.navmaster.engine.routing.MotorwayNet
import it.navmaster.engine.routing.MotorwayNetCache
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.Region
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.routing.TurnCheck
import it.navmaster.engine.storage.PointList
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNotNull
import kotlin.test.assertNotSame
import kotlin.test.assertSame
import kotlin.test.assertTrue

/**
 * [DetourSearch] su una rete giocattolo: il migliore A1 (nodi 0..8, archi 0..7, peso 10 ciascuno),
 * l'A21 a doppio senso (1→10→11→3, peso 8 per arco), l'A22 a senso unico (4→20→21→7, peso 12) e il
 * piolo 11→20 (peso 30) fra le due.
 */
class DetourSearchTest {
    private class Arc(val tail: Int, val head: Int, val edge: Int, val w: Double)

    private val arcs = (0..7).map { Arc(it, it + 1, it, 10.0) } +
        listOf(Arc(1, 10, 100, 8.0), Arc(10, 11, 101, 8.0), Arc(11, 3, 102, 8.0), Arc(10, 1, 100, 8.0), Arc(11, 10, 101, 8.0), Arc(3, 11, 102, 8.0)) +
        listOf(Arc(4, 20, 200, 12.0), Arc(20, 21, 201, 12.0), Arc(21, 7, 202, 12.0)) +
        listOf(Arc(11, 20, 300, 30.0))
    private val graphNodes = intArrayOf(0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 20, 21)

    private fun net(restricted: Set<Int> = emptySet()): MotorwayNet {
        val idx = graphNodes.withIndex().associate { it.value to it.index }
        return MotorwayNet.of(graphNodes, BooleanArray(graphNodes.size) { graphNodes[it] in restricted },
            IntArray(arcs.size) { idx.getValue(arcs[it].tail) }, IntArray(arcs.size) { idx.getValue(arcs[it].head) }, IntArray(arcs.size) { arcs[it].edge },
            BooleanArray(arcs.size) { arcs[it].tail > arcs[it].head }, DoubleArray(arcs.size) { arcs[it].w }, IntArray(arcs.size) { (arcs[it].w * 1000).toInt() },
            FloatArray(arcs.size) { (arcs[it].w * 100).toFloat() }, Region(listOf(Ellipse(0.0, 0.0, 0.0, 0.1, 50_000.0))))
    }

    private val best = RoutePath((0..7).map { PathEdge(it, false, it, it + 1, PointList(0), 1000.0, 10_000, 10.0, "") }, 8000.0, 80_000, 80.0)
    private val trace = BestTrace(listOf(best))

    private fun edges(n: MotorwayNet, d: Detour) = d.arcs.map { n.arcEdge[it] }

    @Test
    fun `finds A21, A22 and the rung with hand-made deltas`() {
        val n = net()
        val ds = DetourSearch(n, trace, null as TurnCheck?).detours(0, 2.0)
        val byEdges = ds.associateBy { edges(n, it) }
        assertEquals(setOf(listOf(100, 101, 102), listOf(200, 201, 202), listOf(100, 101, 300, 201, 202)), byEdges.keys)
        val a21 = byEdges.getValue(listOf(100, 101, 102))
        assertEquals(0, a21.fromPos); assertEquals(2, a21.toPos)
        assertEquals(4.0, a21.dW, 1e-9); assertEquals(4_000L, a21.dMs); assertEquals(24.0, a21.plateauW, 1e-9)
        assertEquals(20_000L, a21.bestMsBetween); assertEquals(24_000L, a21.detourMs)
        val a22 = byEdges.getValue(listOf(200, 201, 202))
        assertEquals(3, a22.fromPos); assertEquals(6, a22.toPos); assertEquals(6.0, a22.dW, 1e-9); assertEquals(6_000L, a22.dMs)
        val rung = byEdges.getValue(listOf(100, 101, 300, 201, 202))
        assertEquals(0, rung.fromPos); assertEquals(6, rung.toPos); assertEquals(10.0, rung.dW, 1e-9); assertEquals(30.0, rung.plateauW, 1e-9)
        // per punteggio: A21, A22, il piolo
        assertEquals(listOf(a21, a22, rung), ds)
        for (d in ds) {
            // rientro più avanti, nessun arco del migliore, nessuna inversione, nessun nodo ripetuto
            assertTrue(d.toPos > d.fromPos)
            val es = edges(n, d)
            assertTrue(es.none { it in 0..7 })
            assertTrue(es.zipWithNext().none { (a, b) -> a == b })
            val nodes = listOf(n.arcTail[d.arcs[0]]) + d.arcs.map { n.arcHead[it] }
            assertEquals(nodes.size, nodes.toSet().size)
            // ancorata al migliore: esce dal nodo della posizione, rientra in quello del rientro
            assertEquals(trace.node[d.fromPos], n.nodeGraph[n.arcTail[d.arcs.first()]])
            assertEquals(trace.node[d.toPos], n.nodeGraph[n.arcHead[d.arcs.last()]])
        }
        // perLeg e quota di plateau
        assertEquals(listOf(a21.arcs.toList()), DetourSearch(n, trace, null as TurnCheck?).detours(0, 2.0, perLeg = 1).map { it.arcs.toList() })
        assertEquals(2, DetourSearch(n, trace, null as TurnCheck?).detours(0, 2.0, minPlateauShare = 0.5).size)
        // il tetto di peso: 1,06 × 80 = 84,8 lascia solo l'A21 (84)
        assertEquals(listOf(listOf(100, 101, 102)), DetourSearch(n, trace, null as TurnCheck?).detours(0, 1.06).map { edges(n, it) })
    }

    @Test
    fun `the A21 backwards is never a detour - re-entry only further on`() {
        val n = net()
        val ds = DetourSearch(n, trace, null as TurnCheck?).detours(0, 3.0, minPlateauShare = 0.0)
        assertTrue(ds.none { d -> edges(n, d).let { it.contains(102) && it.indexOf(102) < it.indexOf(101) } })
        assertTrue(ds.all { it.toPos > it.fromPos })
    }

    @Test
    fun `a turn restriction is respected`() {
        // in 11 vietata la svolta 101 → 102: niente A21; il piolo resta (101 → 300 è lecita)
        val n = net(restricted = setOf(11))
        var asked = 0
        val ds = DetourSearch(n, trace, TurnCheck { i, node, o -> asked++; node == 11 && i == 101 && o == 102 }).detours(0, 2.0)
        assertTrue(asked > 0)
        val sets = ds.map { edges(n, it) }
        assertTrue(listOf(100, 101, 102) !in sets, "$sets")
        assertTrue(listOf(100, 101, 300, 201, 202) in sets, "$sets")
        assertTrue(sets.none { s -> s.zipWithNext().any { it == 101 to 102 } })
        // senza nodi con divieti la svolta non si chiede mai
        var never = 0
        DetourSearch(net(), trace, TurnCheck { _, _, _ -> never++; true }).detours(0, 2.0)
        assertEquals(0, never)
    }

    @Test
    fun `combos of disjoint detours add up`() {
        val n = net()
        val search = DetourSearch(n, trace, null as TurnCheck?)
        val ds = search.detours(0, 2.0)
        val combos = search.combos(ds)
        assertEquals(1, combos.size)
        val c = combos.single()
        assertEquals(listOf(listOf(100, 101, 102), listOf(200, 201, 202)), c.map { edges(n, it) })
        assertEquals(10.0, c.sumOf { it.dW }, 1e-9)
        // un tratto solo: il punteggio sul viaggio è quello sul tratto, e le combinazioni non cambiano
        for (d in ds) assertEquals(d.score, search.tripScore(d, trace.legW.sum()), 1e-9)
        assertEquals(combos, search.combos(ds, key = { search.tripScore(it, trace.legW.sum()) }))
        // cucite: il peso è il migliore più la somma dei Δ (le cuciture leggono il grafo: qui solo i pezzi)
        val cuts = c.map { n.cut(it) }
        assertEquals(listOf(0, 3), cuts.map { it.fromPos }); assertEquals(listOf(2, 6), cuts.map { it.toPos })
        assertEquals(listOf(1, 10, 11), cuts[0].baseNodes.toList())
    }

    @Test
    fun `short junction loops do not use up the chains rebuilt`() {
        // 30 giri di svincolo da 3 m (punteggio migliore di ogni via vera) fra le posizioni 4 e 5 del migliore
        val loops = (0 until 30).flatMap { i -> listOf(Arc(4, 1000 + i, 5000 + 2 * i, 0.6), Arc(1000 + i, 5, 5001 + 2 * i, 10.0)) }
        val all = arcs + loops
        val nodes = graphNodes + IntArray(30) { 1000 + it }
        val idx = nodes.withIndex().associate { it.value to it.index }
        val n = MotorwayNet.of(nodes, BooleanArray(nodes.size), IntArray(all.size) { idx.getValue(all[it].tail) }, IntArray(all.size) { idx.getValue(all[it].head) },
            IntArray(all.size) { all[it].edge }, BooleanArray(all.size), DoubleArray(all.size) { all[it].w }, IntArray(all.size) { (all[it].w * 1000).toInt() },
            FloatArray(all.size) { if (all[it].edge >= 5000) 1.5f else (all[it].w * 100).toFloat() }, Region(listOf(Ellipse(0.0, 0.0, 0.0, 0.1, 50_000.0))))
        // i giri hanno punteggio −0,49: senza la lunghezza sulle etichette occuperebbero le 20 ricostruite di perLeg = 1
        val ds = DetourSearch(n, trace, null as TurnCheck?).detours(0, 2.0, perLeg = 1, minDetourM = 100.0)
        assertEquals(listOf(listOf(100, 101, 102)), ds.map { edges(n, it) })
        assertTrue(DetourSearch(n, trace, null as TurnCheck?).detours(0, 2.0, perLeg = 1).single().arcs.all { n.arcEdge[it] >= 5000 })
    }

    @Test
    fun `a node mask keeps the arcs with both ends allowed`() {
        val n = net()
        // senza il nodo 21 l'A22 e il piolo non ci sono più
        val allowed = BooleanArray(n.nodeCount) { n.nodeGraph[it] != 21 }
        assertEquals(listOf(listOf(100, 101, 102)), DetourSearch(n, trace, null as TurnCheck?, allowed).detours(0, 2.0).map { edges(n, it) })
    }
}

/** La rete autostradale vera (Heidelberg): solo autostrade e superstrade, sensi unici rispettati, la cache per regione. */
class MotorwayNetTest {
    private val g get() = Heidelberg.graph

    /** Un viaggio di Heidelberg che usa l'autostrada, il suo contesto delle varianti e la rete. */
    internal fun motorwayTrip(skip: Int = 0): Triple<VariantContext, MotorwayNet, IntArray>? {
        val service = DirectionsService(g)
        var found = 0
        for (seed in 1L..80L) {
            val pts = Heidelberg.points(2, seed + 300)
            val ctx = NavAlternativesEngine(service, RequestContext(useCache = false))
                .prepare(DirectionsRequest.fromJson("driving-hgv", Heidelberg.body(pts)), NavAlternativesOptions(minDetourScale = 0.02))
            val t = ctx.trace
            val seeds = t.node.indices.filter { ctx.info.isMotorwayOrTrunk(t.inEdge[it]) || ctx.info.isMotorwayOrTrunk(t.outEdge[it]) }.map { t.node[it] }.toIntArray()
            if (seeds.size < 5) continue
            if (found++ < skip) continue
            val s = ctx.solved.snaps
            val region = Region(listOf(Ellipse(s[0].snappedLat, s[0].snappedLon, s[1].snappedLat, s[1].snappedLon, 1.25 * ctx.solved.legs[0].distance + 20_000)))
            return Triple(ctx, MotorwayNet.build(ctx.router.access(g.encoding.shared), seeds, region), seeds)
        }
        return null
    }

    @Test
    fun `only motorway and trunk arcs, one-ways respected`() {
        val (ctx, net, seeds) = motorwayTrip()!!
        assertTrue(net.arcCount > 50, "archi: ${net.arcCount}")
        for (s in seeds) assertTrue(net.netNode(s) >= 0)
        val access = ctx.router.access(g.encoding.shared)
        val it = g.graph.createEdgeIterator()
        val keys = HashSet<Long>()
        var oneWay = 0
        for (a in 0 until net.arcCount) {
            val e = net.arcEdge[a]
            assertTrue(ctx.info.isMotorwayOrTrunk(e), "arco $e")
            it.setEdge(e, net.nodeGraph[net.arcTail[a]])
            assertEquals(net.nodeGraph[net.arcHead[a]], it.adjNode)
            assertEquals(it.reverse, net.arcRev[a])
            assertEquals(access.weighting.edgeWeight(it, false), net.arcW[a])
            assertEquals(access.timeWeighting.edgeMillis(it, false).toInt(), net.arcMs[a])
            assertTrue(keys.add(e.toLong() shl 1 or (if (net.arcRev[a]) 1L else 0L)), "arco diretto doppio")
            if (access.weighting.edgeWeight(it, true).isInfinite()) oneWay++
        }
        // le carreggiate delle autostrade sono a senso unico: il verso opposto non c'è
        assertTrue(oneWay > 0)
        for (a in 0 until net.arcCount) {
            it.setEdge(net.arcEdge[a], net.nodeGraph[net.arcTail[a]])
            val back = net.arcEdge[a].toLong() shl 1 or (if (net.arcRev[a]) 0L else 1L)
            if (access.weighting.edgeWeight(it, true).isInfinite()) assertTrue(back !in keys)
        }
        // adiacenze compatte coerenti
        for (n in 0 until net.nodeCount) {
            for (k in net.outStart[n] until net.outStart[n + 1]) assertEquals(n, net.arcTail[net.outArcs[k]])
            for (k in net.inStart[n] until net.inStart[n + 1]) assertEquals(n, net.arcHead[net.inArcs[k]])
        }
        // troncata oltre il tetto
        val small = MotorwayNet.build(access, seeds, net.region, maxArcs = 10)
        assertTrue(small.truncated); assertTrue(small.arcCount in 10..net.arcCount)
    }

    @Test
    fun `regions and the cache`() {
        val a = Ellipse(49.40, 8.60, 49.42, 8.75, 15_000.0)
        val bigger = Ellipse(49.40, 8.60, 49.42, 8.75, 25_000.0)
        val inner = Ellipse(49.405, 8.65, 49.415, 8.70, 6_000.0)
        val far = Ellipse(49.60, 8.60, 49.62, 8.75, 15_000.0)
        assertTrue(Region(listOf(a)).covers(Region(listOf(a))))
        assertTrue(Region(listOf(bigger)).covers(Region(listOf(a))))
        assertTrue(!Region(listOf(a)).covers(Region(listOf(bigger))))
        assertTrue(Region(listOf(a)).covers(Region(listOf(inner))))
        assertTrue(!Region(listOf(a)).covers(Region(listOf(a, far))))
        assertTrue(Region(listOf(far, a)).covers(Region(listOf(a, far))))
        for (p in a.boundary()) assertTrue(a.contains(p[0], p[1], 1.0) && !a.contains(p[0], p[1], -10.0))

        val (ctx, net, seeds) = motorwayTrip()!!
        val cache = MotorwayNetCache(2)
        val key = g.routerKey(ctx.solved.options)
        var builds = 0
        fun get(r: Region, k: RouteOptions = key) = cache.get(k, r, fits = { n -> seeds.all { n.netNode(it) >= 0 } }) { builds++; MotorwayNet.build(ctx.router.access(g.encoding.shared), seeds, r) }
        val e = net.region.ellipses[0]
        val n1 = get(net.region)
        assertEquals(1, builds)
        // la stessa regione, e una più piccola: dalla cache
        assertSame(n1, get(net.region))
        assertSame(n1, get(Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM * 0.9)))))
        assertEquals(1, builds)
        // più grande, o altre opzioni: si ricostruisce
        val n2 = get(Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM * 1.5))))
        assertNotSame(n1, n2); assertEquals(2, builds)
        get(net.region, key.copy(avoidPenalty = 2.0))
        assertEquals(3, builds); assertEquals(2, cache.size)
        // i semi del migliore non tutti nella rete: si ricostruisce
        cache.get(key, net.region, fits = { false }) { builds++; n1 }
        assertEquals(4, builds)
    }

    @Test
    fun `a truncated net is reused only for its own region`() {
        val (ctx, net, seeds) = motorwayTrip()!!
        val access = ctx.router.access(g.encoding.shared)
        val cache = MotorwayNetCache(2)
        val key = g.routerKey(ctx.solved.options)
        var builds = 0
        fun get(r: Region, maxArcs: Int) = cache.get(key, r, fits = { n -> seeds.all { n.netNode(it) >= 0 } }) { builds++; MotorwayNet.build(access, seeds, r, maxArcs = maxArcs) }
        val e = net.region.ellipses[0]
        val t = get(net.region, 10)
        assertTrue(t.truncated)
        assertSame(t, get(Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM))), 10))
        assertEquals(1, builds)
        // una regione più piccola: una rete nuova (completa), non quella troncata
        val small = get(Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM * 0.9))), 250_000)
        assertEquals(2, builds); assertTrue(!small.truncated)
    }

    @Test
    fun `a net from the cache built for a larger region gives the detours of a new one`() {
        var compared = 0; var masked = 0
        for (skip in 0 until 6) {
            val (ctx, net, seeds) = motorwayTrip(skip) ?: break
            val access = ctx.router.access(g.encoding.shared)
            val e = net.region.ellipses[0]
            val big = MotorwayNet.build(access, seeds, Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM * 2))))
            for (f in listOf(1.0, 0.7, 0.5)) {
                val need = Region(listOf(Ellipse(e.aLat, e.aLon, e.bLat, e.bLon, e.sumM * f)))
                val cache = MotorwayNetCache(2)
                cache.get(g.routerKey(ctx.solved.options), need) { big }
                val hit = cache.get(g.routerKey(ctx.solved.options), need) { error("dalla cache") }
                assertSame(big, hit)
                val mask = hit.mask(need, seeds)
                if (mask != null) masked++
                val fresh = MotorwayNet.build(access, seeds, need)
                fun ds(n: MotorwayNet, m: BooleanArray?) = DetourSearch(n, ctx.trace, access.turnCosts, m).detours(0, 2.0, minPlateauShare = 0.1)
                    .map { d -> listOf(d.fromPos, d.toPos, Math.round(d.dW * 1000), d.dMs, d.arcs.map { n.arcEdge[it] }) }
                assertEquals(ds(fresh, null), ds(hit, mask), "viaggio $skip, regione × $f")
                compared++
            }
        }
        println("rete grande dalla cache: $compared confronti, $masked con nodi esclusi")
        assertTrue(compared > 0 && masked > 0)
    }

    @Test
    fun `a cancelled build leaves the cache untouched, a cancelled waiter leaves`() {
        val (ctx, net, seeds) = motorwayTrip()!!
        val access = ctx.router.access(g.encoding.shared)
        val key = g.routerKey(ctx.solved.options)
        val cache = MotorwayNetCache(2)
        val gone = CancelToken().also { it.cancel() }
        assertFailsWith<SearchCancelledException> { cache.get(key, net.region) { MotorwayNet.build(access, seeds, net.region, cancel = gone) } }
        assertEquals(0, cache.size)
        // un'altra richiesta costruisce: chi aspetta la stessa rete si ferma col suo annullamento
        val release = java.util.concurrent.CountDownLatch(1)
        val started = java.util.concurrent.CountDownLatch(1)
        var built: MotorwayNet? = null
        val th = Thread { built = cache.get(key, net.region) { started.countDown(); release.await(); MotorwayNet.build(access, seeds, net.region) } }
        th.start()
        started.await()
        val waiter = CancelToken()
        val t0 = System.nanoTime()
        val w = Thread { Thread.sleep(100); waiter.cancel() }.also { it.start() }
        assertFailsWith<SearchCancelledException> { cache.get(key, net.region, cancel = waiter) { error("non deve costruire") } }
        assertTrue(System.nanoTime() - t0 < 5_000_000_000L)
        w.join()
        release.countDown(); th.join()
        assertNotNull(built)
        assertSame(built, cache.get(key, net.region) { error("dalla cache") })
        assertEquals(1, cache.size)
    }
}

/** La cucitura delle deviazioni nel tratto ([Router.splice]): pesi e tempi esatti, svolte lecite, estremi del tratto invariati. */
class SpliceTest {
    private val g get() = Heidelberg.graph

    private fun check(router: Router, leg: RoutePath, s: RoutePath, cuts: List<Router.Cut>) {
        assertSame(leg.edges.first(), s.edges.first()); assertSame(leg.edges.last(), s.edges.last())
        assertTrue(router.isLegal(s))
        assertEquals(s.edges.sumOf { it.weight }, s.weight, 1e-9)
        assertEquals(s.edges.sumOf { it.millis }, s.millis)
        // = prefisso + archi + suffisso
        var w = 0.0; var ms = 0L; var next = 0
        for (c in cuts.sortedBy { it.fromPos }) {
            for (i in next..c.fromPos) { w += leg.edges[i].weight; ms += leg.edges[i].millis }
            for (k in c.edges.indices) { val e = s.edges.first { e -> e.edge == c.edges[k] && e.baseNode == c.baseNodes[k] }; w += e.weight; ms += e.millis }
            next = c.toPos + 1
        }
        for (i in next until leg.edges.size) { w += leg.edges[i].weight; ms += leg.edges[i].millis }
        assertEquals(w, s.weight, 1e-6); assertEquals(ms, s.millis)
    }

    @Test
    fun `a spliced detour has the weight and time of prefix + arcs + suffix`() {
        var spliced = 0
        for (skip in 0 until 6) {
            val (ctx, net, _) = MotorwayNetTest().motorwayTrip(skip) ?: break
            val router = ctx.router
            val leg = ctx.solved.legs[0]
            val ds = DetourSearch(net, ctx.trace, router.access(g.encoding.shared).turnCosts).detours(0, 2.0, minPlateauShare = 0.1)
            for (d in ds) {
                val cut = net.cut(d)
                val light = router.splice(leg, listOf(cut), light = true)
                val full = router.splice(leg, listOf(cut))
                check(router, leg, full, listOf(cut))
                assertEquals(leg.weight + d.dW, full.weight, 1e-6 * leg.weight)
                assertEquals(leg.millis + d.dMs, full.millis)
                assertEquals(full.weight, light.weight); assertEquals(full.millis, light.millis)
                assertTrue(full.edges.all { it.geometry.size >= 2 })
                assertEquals(full.points().size, router.materialize(light).points().size)
                spliced++
            }
            println("cucitura: viaggio $skip, rete ${net.arcCount} archi, ${ds.size} deviazioni")
        }
        // sempre: un pezzo del migliore cucito al posto di sé stesso dà il migliore
        val (ctx, _, _) = MotorwayNetTest().motorwayTrip()!!
        val leg = ctx.solved.legs[0]
        val p = leg.edges.size / 4; val q = leg.edges.size / 2
        val same = Router.Cut(p, q, IntArray(q - p) { leg.edges[p + 1 + it].edge }, IntArray(q - p) { leg.edges[p + 1 + it].baseNode })
        val s = ctx.router.splice(leg, listOf(same))
        check(ctx.router, leg, s, listOf(same))
        assertEquals(leg.weight, s.weight, 1e-6); assertEquals(leg.millis, s.millis); assertEquals(leg.edges.size, s.edges.size)
        println("deviazioni cucite su Heidelberg: $spliced")
    }
}
