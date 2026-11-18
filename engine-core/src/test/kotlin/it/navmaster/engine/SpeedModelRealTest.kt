package it.navmaster.engine

import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import kotlin.test.Test
import kotlin.test.assertTrue

/**
 * Il modello delle velocità sui grafi veri (CI, flusso «Prova delle velocità», `.github/workflows/prova-velocita.yml`):
 * `NM_SPEED_NAV="austria=/…,czech-republic=/…"` i grafi col modello NAVMASTER, `NM_SPEED_ORS` (facoltativo)
 * gli stessi Paesi col modello ORS per il confronto. Senza `NM_SPEED_NAV` la prova non fa nulla (e passa).
 *
 * Controlla:
 * 1. la statale 52 fra Mikulov e Pohořelice: camion ≥ 60 km/h dove il cartello è 90 (auto ≥ 80), ≥ 55 dove è 70;
 * 2. Wien → Brno del camion per il valico di Mikulov (A5 → 52 → D52), senza L3050 né 422;
 * 3. durata Wien → Brno del camion fra 105 e 135 minuti;
 * e stampa (righe «velocità») i tempi di una decina di viaggi nei due Paesi coi due modelli.
 */
class SpeedModelRealTest {
    /** Le misure del camion dell'app: 4 m, 2,55 m, 16,5 m, 27 t, 11,5 t per asse. */
    private val truck = RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 4.0, width = 2.55, length = 16.5, weight = 27.0, axleLoad = 11.5))
    private val car = RouteOptions(profile = "driving-car", weighting = WeightingMethod.FASTEST)

    private val wien = 48.2082 to 16.3738
    private val brno = 49.1951 to 16.6068

    /** I valichi austria/czech-republic (punti della sonda prova-44315230266, sulla strada che passa il confine). */
    private val crossings = listOf(
        "Mikulov/Drasenhofen (52)" to (48.78889 to 16.63681),
        "Hatě/Kleinhaugsdorf (38)" to (48.76244 to 16.06473),
        "Hatě (B303)" to (48.74228 to 16.06031),
        "Břeclav/Reintal (B47)" to (48.70555 to 16.82553),
        "Valtice (L3050)" to (48.71315 to 16.66612),
        "Valtice/Schrattenberg (L22)" to (48.73148 to 16.71999),
        "Hevlín/Laa (397)" to (48.75275 to 16.23559),
    )

    private val trips = mapOf(
        "austria" to listOf(
            "Wien → Linz" to ((48.2082 to 16.3738) to (48.3069 to 14.2858)),
            "Wien → Graz" to ((48.2082 to 16.3738) to (47.0707 to 15.4395)),
            "Innsbruck → Salzburg" to ((47.2692 to 11.4041) to (47.8095 to 13.0550)),
            "Wien → Drasenhofen" to ((48.2082 to 16.3738) to (48.7550 to 16.6400)),
            "Linz → Graz" to ((48.3069 to 14.2858) to (47.0707 to 15.4395)),
        ),
        "czech-republic" to listOf(
            "Mikulov → Pohořelice" to ((48.8056 to 16.6378) to (48.9813 to 16.5246)),
            "Brno → Znojmo" to ((49.1951 to 16.6068) to (48.8555 to 16.0488)),
            "Brno → Praha" to ((49.1951 to 16.6068) to (50.0755 to 14.4378)),
            "Brno → Ostrava" to ((49.1951 to 16.6068) to (49.8209 to 18.2625)),
            "Praha → Plzeň" to ((50.0755 to 14.4378) to (49.7384 to 13.3736)),
        ),
    )

    private fun graphs(env: String): Map<String, RoutingGraph> {
        val spec = System.getenv(env) ?: return emptyMap()
        return spec.split(',').filter { it.contains('=') }.associate { s -> val (id, path) = s.split('='); id to RoutingGraph.load(File(path), true) }
    }

    private fun route(rg: RoutingGraph, o: RouteOptions, a: Pair<Double, Double>, b: Pair<Double, Double>, radius: Double = 1000.0): RoutePath? = try {
        val r = rg.router(o)
        r.route(r.snap(a.first, a.second, radius, 0), r.snap(b.first, b.second, radius, 1))
    } catch (e: Exception) { println("velocità: nessun percorso $a → $b (${e.javaClass.simpleName}: ${e.message})"); null }

    /** I nomi/ref delle strade del percorso in ordine, senza ripetizioni consecutive. */
    private fun roads(p: RoutePath): List<String> {
        val out = ArrayList<String>()
        for (e in p.edges) if (e.name.isNotEmpty() && out.lastOrNull() != e.name) out += e.name
        return out
    }

    private fun tokens(p: RoutePath): Set<String> = p.edges.flatMap { it.name.split(", ", ";").map(String::trim) }.toSet()

    private fun line(label: String, p: RoutePath?) = if (p == null) "$label: nessuno" else
        "$label %.1f km, %.1f min (%.1f km/h)".format(p.distance / 1000, p.millis / 60000.0, p.distance / 1000 / (p.millis / 3_600_000.0))

    @Test
    fun real() {
        val nav = graphs("NM_SPEED_NAV")
        if (nav.isEmpty()) return
        val ors = graphs("NM_SPEED_ORS")
        try {
            for ((id, g) in nav) println("velocità: $id NAVMASTER speed_model=${g.speedModel} country=${g.country}, ${g.graph.edgeCount} archi")
            for ((id, g) in ors) println("velocità: $id ORS speed_model=${g.speedModel}, ${g.graph.edgeCount} archi")
            val problems = ArrayList<String>()

            // confronto dei due modelli su una decina di viaggi
            for ((country, list) in trips) {
                val n = nav[country] ?: continue
                for ((label, ab) in list) for ((pl, o) in listOf("camion" to truck, "auto" to car)) {
                    val pn = route(n, o, ab.first, ab.second)
                    val po = ors[country]?.let { route(it, o, ab.first, ab.second) }
                    val ratio = if (pn != null && po != null) " NAVMASTER/ORS %.3f".format(pn.millis.toDouble() / po.millis) else ""
                    println("velocità: $country $label $pl | ${line("NAVMASTER", pn)} | ${line("ORS", po)}$ratio")
                    pn?.let { println("velocità:   strade NAVMASTER ${roads(it).take(25)}") }
                    po?.let { println("velocità:   strade ORS ${roads(it).take(25)}") }
                }
            }

            // 1) la statale 52 fra Mikulov e Pohořelice
            nav["czech-republic"]?.let { cz -> problems += road52(cz) }

            // 2-3) Wien → Brno: il pezzo austriaco fino al valico e quello ceco dal valico, per ogni valico
            val at = nav["austria"]; val cz = nav["czech-republic"]
            if (at != null && cz != null) {
                class Via(val name: String, val atLeg: RoutePath, val czLeg: RoutePath) {
                    val weight get() = atLeg.weight + czLeg.weight
                    val millis get() = atLeg.millis + czLeg.millis
                    val km get() = (atLeg.distance + czLeg.distance) / 1000
                }
                val vias = crossings.mapNotNull { (name, x) ->
                    val a = route(at, truck, wien, x, 200.0) ?: return@mapNotNull null
                    val b = route(cz, truck, x, brno, 200.0) ?: return@mapNotNull null
                    Via(name, a, b)
                }
                for (v in vias.sortedBy { it.weight }) println("velocità: Wien → Brno camion per ${v.name}: %.1f km, %.1f min, peso %.0f".format(v.km, v.millis / 60000.0, v.weight))
                val best = vias.minByOrNull { it.weight }
                if (best == null) problems += "Wien → Brno: nessun valico percorribile"
                else {
                    println("velocità: Wien → Brno camion migliore per ${best.name}: strade AT ${roads(best.atLeg)}, CZ ${roads(best.czLeg)}")
                    val t = tokens(best.atLeg) + tokens(best.czLeg)
                    if (!best.name.startsWith("Mikulov")) problems += "Wien → Brno: il camion passa per ${best.name}, non per Mikulov"
                    if ("52" !in t || "D52" !in t) problems += "Wien → Brno: mancano la 52 o la D52 (${t.filter { it.length <= 6 }})"
                    if ("L3050" in t || "422" in t) problems += "Wien → Brno: passa per L3050 o 422"
                    val min = best.millis / 60000.0
                    if (min < 105 || min > 135) problems += "Wien → Brno: %.1f min, atteso fra 105 e 135".format(min)
                }
            }
            for (p in problems) println("velocità: PROBLEMA $p")
            assertTrue(problems.isEmpty(), problems.joinToString("\n"))
        } finally {
            nav.values.forEach { it.close() }; ors.values.forEach { it.close() }
        }
    }

    /** Gli archi della 52 (primary, fuori dalle rotatorie) fra Mikulov e Pohořelice: le velocità del camion. */
    private fun road52(cz: RoutingGraph): List<String> {
        val g = cz.graph; val enc = cz.encoding
        val hgv = enc.profile("heavyvehicle"); val carPe = enc.profiles.firstOrNull { it.profile.name == "car-ors" }
        val it = g.createEdgeIterator()
        val out = ArrayList<String>()
        var n = 0; var km = 0.0; var weighted = 0.0
        for (e in 0 until g.edgeCount) {
            val lat = g.lat(g.nodeA(e)); val lon = g.lon(g.nodeA(e))
            if (lat !in 48.80..48.98 || lon !in 16.50..16.70) continue
            if ("52" !in g.name(e).split(", ", ";").map(String::trim)) continue
            it.setEdge(e, g.nodeA(e))
            if (it.get(enc.shared.roadClass) != RoadClass.PRIMARY || it.get(enc.shared.roundabout)) continue
            for (rev in listOf(false, true)) {
                val f = g.getFlags(e)
                if (!hgv.access.get(f, rev)) continue
                val v = hgv.speed.get(f, rev); val vc = carPe?.speed?.get(f, rev) ?: Double.NaN
                val need = when { vc >= 80 -> 60.0; vc >= 60 -> 55.0; else -> 0.0 }
                n++; km += g.distance(e) / 1000; weighted += g.distance(e) / 1000 * v
                if (v < need) out += "52: arco $e (%.5f,%.5f, %.0f m) camion %.0f km/h, auto %.0f: atteso ≥ %.0f".format(lat, lon, g.distance(e), v, vc, need)
            }
        }
        println("velocità: 52 Mikulov–Pohořelice: $n sensi d'arco, %.1f km, camion in media %.1f km/h".format(km, if (km > 0) weighted / km else 0.0))
        if (n == 0) out += "52: nessun arco trovato fra Mikulov e Pohořelice"
        return out
    }
}
