package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.instructions.InstructionType
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Le alternative coi punti di passaggio ([LongAlternatives]) oltre la distanza del plateau: qui la
 * soglia è abbassata a 1 km perché il grafo di prova (Heidelberg) è piccolo. Su 5 km di città i
 * candidati sono quasi sempre scartati (il metodo è per i viaggi lunghi): si controlla che il metodo
 * giri, che le regole di scarto valgano e che ogni percorso sia intero, senza la sosta al punto di
 * passaggio. La prova vera è [LongAlternativesRealTest] sui grafi dei Paesi.
 */
class LongAlternativesTest {
    @Test
    @Suppress("UNCHECKED_CAST")
    fun `via point alternatives are whole routes, distinct and not much slower`() {
        val dir = Files.createTempDirectory("nme-longalt").toFile().also { it.deleteOnExit() }
        val (built, _) = RoutingGraph.build(File("../data/heidelberg.test.pbf"), dir, listOf(CarProfile(), HgvProfile()))
        built.use { it.prepareCH() }
        RoutingGraph.load(dir, mmap = false).use { g ->
            val service = DirectionsService(g, DirectionsLimits(maximumDistanceAlternativeRoutes = 1_000.0))
            val lines = ArrayList<String>()
            service.alternativesLog = { lines += it; println(it) }
            val body = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7606,49.4186]],"language":"it",
                "alternative_routes":{"target_count":3,"weight_factor":1.4,"share_factor":0.6},"navmaster":{"roads":true},
                "options":{"profile_params":{"restrictions":{"height":4.0,"weight":18,"hazmat":true,"adr_tunnel_code":"D"}}}}""") as Map<String, Any?>
            val res = service.handle(DirectionsRequest.fromJson("driving-hgv", body), ResponseFormat.GEOJSON)
            val features = res["features"] as List<Map<String, Any?>>
            assertTrue(features.isNotEmpty() && lines.any { it.startsWith("alternative dei viaggi lunghi") }, lines.joinToString(" | "))
            // the candidates themselves, joined at the via point: one route each (checked below with the kept ones)
            val viaBody = Json.parse("""{"coordinates":[[8.6937,49.4094],[8.7200,49.4300],[8.7606,49.4186]],"language":"it","continue_straight":true}""") as Map<String, Any?>
            val viaRes = service.handle(DirectionsRequest.fromJson("driving-car", viaBody), ResponseFormat.GEOJSON)
            val joined = LongAlternatives.joinVia((viaRes["features"] as List<Map<String, Any?>>)[0], "it")
            val jp = joined["properties"] as Map<String, Any?>
            assertEquals(1, (jp["segments"] as List<*>).size)
            val js = ((jp["segments"] as List<Map<String, Any?>>)[0]["steps"] as List<Map<String, Any?>>)
            assertEquals(1, js.count { it["type"] == InstructionType.FINISH.code })
            assertEquals(LongAlternatives.duration(joined), ((jp["segments"] as List<Map<String, Any?>>)[0]["duration"] as Number).toDouble(), 0.2)
            val d0 = LongAlternatives.duration(features[0])
            val segs = features.map { LongAlternatives.segments(it) }
            for ((i, f) in features.withIndex()) {
                val props = f["properties"] as Map<String, Any?>
                // one route, no stop at the via point
                assertEquals(1, (props["segments"] as List<*>).size)
                assertEquals(2, (props["way_points"] as List<*>).size)
                val steps = ((props["segments"] as List<Map<String, Any?>>)[0]["steps"] as List<Map<String, Any?>>)
                assertEquals(InstructionType.FINISH.code, steps.last()["type"])
                assertEquals(1, steps.count { it["type"] == InstructionType.FINISH.code })
                assertEquals(1, steps.count { it["type"] == InstructionType.DEPART.code })
                if (i > 0) {
                    assertTrue(LongAlternatives.duration(f) <= d0 * 1.3 + 1)
                    assertTrue(LongAlternatives.shared(segs[i], segs[0]) <= 0.6 + 1e-9)
                }
            }
        }
    }

    @Test
    fun `offset and along`() {
        // a trip west to east: +10 km is to the north
        val p = LongAlternatives.offset(45.0 to 9.5, 45.0, 9.0, 45.0, 10.0, 10_000.0)
        assertTrue(p.first > 45.08 && p.first < 45.1 && kotlin.math.abs(p.second - 9.5) < 1e-6)
        val mid = LongAlternatives.along(listOf(doubleArrayOf(9.0, 45.0), doubleArrayOf(10.0, 45.0)), 0.5)
        assertTrue(kotlin.math.abs(mid.second - 9.5) < 1e-6)
        assertEquals(0.75, LongAlternatives.shareLimit(0.6, 400_000.0))
        assertEquals(0.6, LongAlternatives.shareLimit(0.6, 40_000.0))
    }
}
