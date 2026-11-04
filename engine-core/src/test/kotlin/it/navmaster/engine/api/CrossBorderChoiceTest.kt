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
import kotlin.test.assertTrue

/**
 * La scelta del valico coi limiti dei landmark e una ricerca sola dal lato vicino ([Router.manyToOne])
 * contro le ricerche una per valico: stessi pesi, stesso valico. Su Heidelberg, lo stesso grafo nei panni di
 * due Paesi e valichi finti sparsi sull'area.
 */
class CrossBorderChoiceTest {
    private fun close(x: Double, y: Double) = (x.isInfinite() && y.isInfinite()) || abs(x - y) <= 0.01 + abs(y) * 1e-5

    @Test
    fun `one search per side gives the weights of one search per crossing`() {
        val dir = Files.createTempDirectory("nme-cbc").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(HgvProfile()))
        built.use { it.prepareCH() }
        CrossBorderDirections.compareOneByOne = false
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
            var finalEarly = 0; var boundSum = 0.0; var weightSum = 0.0
            repeat(12) { round ->
                val root = randomSnap(core)
                val others = List(8) { if (it == 0 && round % 3 == 0) root else randomSnap(core) }
                for (rootIsTarget in listOf(true, false)) {
                    val want = others.map { if (rootIsTarget) exact(core, it, root) else exact(core, root, it) }
                    val m = assertNotNull(core.manyToOne(root, others, rootIsTarget))
                    // subito, dai landmark: mai sopra il peso
                    for (i in others.indices) {
                        assertTrue(m.lowerBound(i) <= want[i] + 0.01 + want[i] * 1e-5, "limite dei landmark ${m.lowerBound(i)} sopra il peso ${want[i]}")
                        if (want[i].isFinite()) { boundSum += minOf(m.lowerBound(i), want[i]); weightSum += want[i] }
                    }
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
            println("limiti dei landmark: %.0f %% dei pesi".format(100 * boundSum / weightSum))
            assertTrue(boundSum > 0.3 * weightSum, "limiti dei landmark troppo deboli: $boundSum su $weightSum")

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
                assertTrue(logs.any { it.contains("valico scelto") }, "la scelta non è passata dalla ricerca per lato: $logs")
                if (together.lastCrossings.map { it.lat to it.lon } == oneByOne.lastCrossings.map { it.lat to it.lon }) {
                    same++
                    if (r1.isSuccess) assertEquals(Json.stringify(summary(r2.getOrThrow())), Json.stringify(summary(r1.getOrThrow())))
                }
                println("viaggio $k: peso %.1f / %.1f, valico ${together.lastCrossings.map { it.lat to it.lon }}; ${logs.lastOrNull { it.contains("valico scelto") }?.trim()}".format(together.lastWeight, oneByOne.lastWeight))
            }
            // valichi diversi solo a pari peso
            assertTrue(same >= 12, "valico diverso in ${14 - same} viaggi")
        }
    }

    @Test
    fun `one crossing per place keeps both carriageways of a motorway`() {
        // il Brennero: i capi delle due carreggiate (sensi unici) ai due margini degli estratti, la statale; Resia
        val north1 = Transfer(47.01760, 11.50650, 1); val south1 = Transfer(47.01790, 11.50700, 1)
        val north2 = Transfer(47.00210, 11.50480, 1); val south2 = Transfer(47.00230, 11.50530, 1)
        val road = Transfer(47.01500, 11.50300, 3)
        val resia = Transfer(46.83405, 10.51050, 3); val nauders = Transfer(46.86843, 10.50890, 3)
        val canneto = 45.150 to 10.380; val innsbruck = 47.269 to 11.404
        fun detour(t: Transfer) = DistanceCalc.haversine(canneto.first, canneto.second, t.lat, t.lon) + DistanceCalc.haversine(t.lat, t.lon, innsbruck.first, innsbruck.second)
        val all = listOf(road, north1, resia, south1, north2, nauders, south2)
        val kept = Transfers.onePerPlace(all, ::detour, 6)
        val brenner = kept.filter { it.lon > 11.0 }
        assertEquals(2, brenner.size, "al Brennero: $brenner")
        assertTrue(brenner.all { it.rank == 1 } && DistanceCalc.haversine(brenner[0].lat, brenner[0].lon, brenner[1].lat, brenner[1].lon) < 500.0)
        assertTrue(road !in kept && resia in kept && nauders in kept)
        // il numero di luoghi, non di punti
        assertEquals(brenner, Transfers.onePerPlace(all, ::detour, 1))
    }

    @Suppress("UNCHECKED_CAST")
    private fun summary(res: Map<String, Any?>) = ((res["features"] as List<Map<String, Any?>>)[0]["properties"] as Map<String, Any?>)["summary"]
}
