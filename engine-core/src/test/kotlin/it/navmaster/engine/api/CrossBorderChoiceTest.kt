package it.navmaster.engine.api

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import java.nio.file.Files
import java.util.Random
import kotlin.math.abs
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

/**
 * La scelta del valico con una ricerca per lato ([Router.manyToOne], [Router.best]) contro le
 * ricerche una per valico: stessi pesi, stesso valico. Su Heidelberg, lo stesso grafo nei panni di
 * due Paesi e valichi finti sparsi sull'area.
 */
class CrossBorderChoiceTest {
    private fun close(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || abs(x - y) <= 0.01 + abs(y) * 1e-5

    @Test
    fun `one search per side gives the weights of one search per crossing`() {
        val dir = Files.createTempDirectory("nme-cbc").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        built.use { it.prepareCH() }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val params = VehicleParameters(height = 4.0, weight = 12.0)
            val core = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            assertEquals("core_alt_edge_based", core.algorithm)
            val rnd = Random(7)
            val bg = g.graph
            fun randomSnap(r: Router): Snap {
                while (true) {
                    val n = rnd.nextInt(bg.nodeCount)
                    // a metà arco o su un nodo
                    val d = if (rnd.nextBoolean()) 0.0 else 0.0006
                    try { return r.snap(bg.lat(n) + (rnd.nextDouble() - 0.5) * d, bg.lon(n) + (rnd.nextDouble() - 0.5) * d, 400.0, 0) } catch (e: PointNotFoundException) { }
                }
            }
            fun exact(r: Router, a: Snap, b: Snap) = try { r.route(a, b, exact = false).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }

            // molti a uno e uno a molti: limiti inferiori mai sopra il peso, pesi definitivi uguali, in ordine
            var finalEarly = 0
            repeat(12) { round ->
                val root = randomSnap(core)
                val others = List(8) { if (it == 0 && round % 3 == 0) root else randomSnap(core) }
                for (rootIsTarget in listOf(true, false)) {
                    val want = others.map { if (rootIsTarget) exact(core, it, root) else exact(core, root, it) }
                    val m = assertNotNull(core.manyToOne(root, others, rootIsTarget))
                    m.advance { m.anyReached }
                    for (i in others.indices) {
                        assertTrue(m.lowerBound(i) <= want[i] + 0.01 + want[i] * 1e-5, "limite ${m.lowerBound(i)} sopra il peso ${want[i]} (radice ${if (rootIsTarget) "arrivo" else "partenza"})")
                        if (m.isFinal(i)) { finalEarly++; assertTrue(close(m.weight(i), want[i]), "definitivo ${m.weight(i)} invece di ${want[i]}") }
                    }
                    for (i in others.indices.sortedBy { want[it] }) {
                        m.advance { m.isFinal(i) }
                        assertTrue(close(m.weight(i), want[i]), "peso ${m.weight(i)} invece di ${want[i]} (radice ${if (rootIsTarget) "arrivo" else "partenza"}): ${m.stats}")
                    }
                }
            }
            assertTrue(finalEarly > 12, "pochi pesi definitivi alla prima fermata: $finalEarly")

            // la ricerca verso molti punti, ciascuno col suo peso in più: il minimo delle ricerche una per volta
            val routers = listOf(core, g.router(RouteOptions(profile = "driving-hgv")), g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false)))
            assertEquals(listOf("core_alt_edge_based", "ch", "dijkstra_edge_based"), routers.map { it.algorithm })
            for (r in routers) repeat(8) {
                val one = randomSnap(r)
                val many = List(6) { randomSnap(r) }
                val extra = many.map { rnd.nextDouble() * 900 }
                val wantTo = many.indices.minOf { exact(r, one, many[it]) + extra[it] }
                val to = r.best(listOf(one to 0.0), many.zip(extra))
                if (wantTo.isInfinite()) assertNull(to) else {
                    assertNotNull(to, "${r.algorithm} verso molti: nessuno invece di $wantTo (${many.map { it.position }}, ${one.position}) ${r.quickStats()}")
                    assertTrue(close(to.weight, wantTo), "${r.algorithm} verso molti: ${to.weight} invece di $wantTo")
                    assertTrue(close(exact(r, one, many[to.to]) + extra[to.to], wantTo), "${r.algorithm}: il punto scelto non è il migliore")
                }
                val wantFrom = many.indices.minOf { extra[it] + exact(r, many[it], one) }
                val from = r.best(many.zip(extra), listOf(one to 0.0))
                if (wantFrom.isInfinite()) assertNull(from) else {
                    assertNotNull(from, "${r.algorithm} da molti: nessuno invece di $wantFrom")
                    assertTrue(close(from.weight, wantFrom), "${r.algorithm} da molti: ${from.weight} invece di $wantFrom")
                    assertTrue(close(extra[from.from] + exact(r, many[from.from], one), wantFrom), "${r.algorithm}: il punto scelto non è il migliore")
                }
                // con un limite più basso del migliore: nessuno
                if (wantTo.isFinite() && wantTo > 1.0) assertNull(r.best(listOf(one to 0.0), many.zip(extra), wantTo * 0.99))
            }

            // il viaggio "tra Paesi": il valico scelto in una ricerca per lato e uno alla volta
            val service = DirectionsService(g)
            val a = CountryGraph("a", service); val b = CountryGraph("b", service)
            val crossings = ArrayList<Transfer>()
            for (attempt in 0 until 4000) {
                if (crossings.size >= 10) break
                val s = randomSnap(core)
                val n = s.closestNode
                if (crossings.any { c -> DistanceCalc.haversine(c.lat, c.lon, bg.lat(n), bg.lon(n)) < 3_500.0 }) continue
                // su un nodo della strada, come i passaggi veri; strade principali e minori
                crossings += Transfer(bg.lat(n), bg.lon(n), if (crossings.size % 3 == 2) 5 else 1 + crossings.size % 3)
            }
            assertTrue(crossings.size >= 8, "valichi finti: ${crossings.size}")
            val logs = ArrayList<String>()
            val together = CrossBorderDirections(listOf(a, b), { _, _ -> crossings }, { logs += it })
            val oneByOne = CrossBorderDirections(listOf(a, b), { _, _ -> crossings }, null, oneSearch = false)
            var same = 0
            repeat(14) { k ->
                val p = randomSnap(core); val q = randomSnap(core)
                val body = linkedMapOf<String, Any?>("coordinates" to listOf(listOf(p.queryLon, p.queryLat), listOf(q.queryLon, q.queryLat)), "instructions" to false,
                    "options" to linkedMapOf<String, Any?>("profile_params" to mapOf("restrictions" to mapOf("height" to 4.0, "weight" to 12.0))))
                // anche con avoid_features (la ricerca rapida senza, come per un valico alla volta)
                if (k % 4 == 3) (body["options"] as MutableMap<String, Any?>)["avoid_features"] = listOf("tollways")
                val req = DirectionsRequest.fromJson("driving-hgv", body)
                logs.clear()
                val r1 = runCatching { together.handle(req, listOf("a", "b")) }
                val r2 = runCatching { oneByOne.handle(req, listOf("a", "b")) }
                assertEquals(r2.isSuccess, r1.isSuccess, "esito diverso: ${r1.exceptionOrNull()} / ${r2.exceptionOrNull()}")
                assertTrue(close(together.lastWeight, oneByOne.lastWeight), "peso ${together.lastWeight} invece di ${oneByOne.lastWeight}")
                assertTrue(logs.any { it.contains("valico in una ricerca per lato") }, "la scelta non è passata dalla ricerca per lato: $logs")
                if (together.lastCrossings.map { it.lat to it.lon } == oneByOne.lastCrossings.map { it.lat to it.lon }) {
                    same++
                    if (r1.isSuccess) assertEquals(Json.stringify(summary(r2.getOrThrow())), Json.stringify(summary(r1.getOrThrow())))
                }
                println("viaggio $k: peso %.1f / %.1f, valico ${together.lastCrossings.map { it.lat to it.lon }}; ${logs.lastOrNull { it.contains("valico in una ricerca") }?.trim()}".format(together.lastWeight, oneByOne.lastWeight))
            }
            // valichi diversi solo a pari peso
            assertTrue(same >= 12, "valico diverso in ${14 - same} viaggi")
        }
    }

    @Suppress("UNCHECKED_CAST")
    private fun summary(res: Map<String, Any?>) = ((res["features"] as List<Map<String, Any?>>)[0]["properties"] as Map<String, Any?>)["summary"]
}
