package it.navmaster.engine.routing

import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.weighting.VehicleParameters
import java.io.File
import kotlin.test.Test

/**
 * Diagnostica su grafi veri (`NM_CB="ukraine=/…"`, `NM_TRIPS="lat,lon;lat,lon|…"`): un viaggio
 * interno col core e con la ricerca piena edge-based, il camion delle prove per Paese. Per ognuno
 * peso, legalità, nodi ripetuti (il giro attorno a un isolato) e dove i due percorsi si separano.
 */
class InternalPairDiagTest {
    @Test
    fun compare() {
        val spec = System.getenv("NM_CB") ?: return
        val trips = (System.getenv("NM_TRIPS") ?: return).split('|').map { t -> t.split(';').map { p -> p.split(',').let { it[0].trim().toDouble() to it[1].trim().toDouble() } } }
        val params = VehicleParameters(height = 4.0, width = 2.55, length = 16.5, weight = 27.0, axleLoad = 11.5)
        for ((id, path) in spec.split(',').map { it.split('=') }) {
            val g = RoutingGraph.load(File(path), true)
            val core = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params))
            val full = g.router(RouteOptions(profile = "driving-hgv", vehicleParameters = params, allowCH = false))
            for (t in trips) {
                val (a, b) = t
                val pc = runCatching { core.route(core.snap(a.first, a.second, 400.0, 0), core.snap(b.first, b.second, 400.0, 1)) }.getOrNull()
                val pf = runCatching { full.route(full.snap(a.first, a.second, 400.0, 0), full.snap(b.first, b.second, 400.0, 1)) }.getOrNull()
                println("$id %.4f,%.4f → %.4f,%.4f".format(a.first, a.second, b.first, b.second))
                for ((name, p) in listOf("core" to pc, "piena" to pf)) {
                    if (p == null) { println("  $name: nessun percorso"); continue }
                    val nodes = p.edges.map { it.baseNode }
                    val repeated = nodes.groupBy { it }.filter { it.value.size > 1 }.keys
                    val st = g.chStorages.getValue("heavyvehicle_recommended_core"); val n = g.graph.nodeCount
                    println("  $name: peso %.1f, %.2f km, %d archi, legale ${full.isLegal(p)}, nodi ripetuti ${repeated.size} %s".format(p.weight, p.distance / 1000, p.edges.size,
                        repeated.take(6).map { "$it${if (st.level(it) >= n) " (core)" else ""}" }))
                    // per ogni nodo ripetuto: il giro fra la prima e l'ultima volta, e quanti nodi fuori dal core
                    for (r in repeated.take(3)) {
                        val i = nodes.indexOf(r); val j = nodes.lastIndexOf(r)
                        val loop = nodes.subList(i, j + 1)
                        println("    giro da $r: ${loop.size} nodi, ${loop.count { st.level(it) < n }} fuori dal core, %.0f m".format(p.edges.subList(i, j).sumOf { it.distance }))
                    }
                }
                if (pc != null && pf != null) {
                    val ec = pc.edges.map { it.edge }; val ef = pf.edges.map { it.edge }
                    var k = 0; while (k < minOf(ec.size, ef.size) && ec[k] == ef[k]) k++
                    val sep = pc.edges.getOrNull(k) ?: pc.edges.last()
                    println("  uguali fino all'arco $k di ${ec.size}/${ef.size}, poi core ${pc.edges.drop(k).take(4).joinToString { "${it.edge}(${it.baseNode}→${it.adjNode} ${it.name})" }}")
                    println("    piena ${pf.edges.drop(k).take(4).joinToString { "${it.edge}(${it.baseNode}→${it.adjNode} ${it.name})" }}; dove: %.5f,%.5f".format(g.graph.lat(sep.baseNode), g.graph.lon(sep.baseNode)))
                    // il peso del percorso della piena ricalcolato arco per arco, e la parte comune
                    println("    pesi archi: core ${"%.1f".format(pc.edges.sumOf { it.weight })}, piena ${"%.1f".format(pf.edges.sumOf { it.weight })}")
                }
            }
            g.close()
        }
    }
}
