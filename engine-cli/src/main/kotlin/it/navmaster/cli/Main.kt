package it.navmaster.cli

import it.navmaster.engine.Engine
import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingMethod
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.TurnCostEncoding
import it.navmaster.engine.storage.TurnCostStorage
import java.io.File
import kotlin.system.exitProcess

private const val USAGE = """
navmaster-engine

  import <file.osm.pbf> <cartella-grafo> [--profiles=car,hgv] [--no-ch] [--no-width-from-carriageway] [--mmap] [--core=narrow]   costruisce il grafo (--mmap: file su disco invece che in RAM; --core=narrow: core ridotto, senza autostrade e strade veloci)
  info <cartella-grafo>                                          riepilogo del grafo
  route <cartella-grafo> <lat,lon> <lat,lon> [--profile=driving-hgv] [--weighting=fastest|shortest|recommended]
        [--avoid=highways,tollways,ferries,fords] [--height=4.0 --weight=40 --width=2.55 --length=18.75 --hazmat] [--no-turn-costs] [--no-ch]
"""

fun main(args: Array<String>) {
    if (args.isEmpty()) { println(USAGE.trimIndent()); exitProcess(1) }
    when (args[0]) {
        "import" -> importGraph(args.drop(1))
        "info" -> info(args.drop(1))
        "route" -> route(args.drop(1))
        "bench" -> bench(args.drop(1))
        else -> { println("comando sconosciuto: ${args[0]}\n${USAGE.trimIndent()}"); exitProcess(1) }
    }
}

/** Misura i tempi medi di calcolo a JVM calda su coppie casuali di nodi, per ciascun algoritmo. */
private fun bench(args: List<String>) {
    val positional = args.filter { !it.startsWith("--") }
    require(positional.size == 1) { "uso: bench <cartella-grafo> [--n=200]" }
    val n = opt(args, "n")?.toInt() ?: 200
    RoutingGraph.load(File(positional[0])).use { rg ->
        val g = rg.graph
        val rnd = java.util.Random(42)
        val pairs = (0 until n).map {
            val a = rnd.nextInt(g.nodeCount); val b = rnd.nextInt(g.nodeCount)
            doubleArrayOf(g.lat(a), g.lon(a), g.lat(b), g.lon(b))
        }
        val scenarios = listOf(
            "camion, CH (nessuna restrizione)" to RouteOptions(profile = "driving-hgv", turnCosts = false),
            "camion, Core edge-based (4 m, 40 t, divieti di svolta)" to RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 4.0, weight = 40.0)),
            "camion, Dijkstra edge-based (stesse restrizioni, senza preparazioni)" to RouteOptions(profile = "driving-hgv", vehicleParameters = VehicleParameters(height = 4.0, weight = 40.0), allowCH = false),
            "auto, CH" to RouteOptions(profile = "driving-car", weighting = WeightingMethod.FASTEST, turnCosts = false),
        )
        for ((label, options) in scenarios) {
            val router = rg.router(options)
            var ok = 0; var totalNs = 0L; var totalKm = 0.0; var maxNs = 0L
            for ((i, p) in pairs.withIndex()) {
                val t0 = System.nanoTime()
                try {
                    val path = router.route(router.snap(p[0], p[1], 1000.0, 0), router.snap(p[2], p[3], 1000.0, 1))
                    val dt = System.nanoTime() - t0
                    if (i >= n / 10) { ok++; totalNs += dt; totalKm += path.distance / 1000; if (dt > maxNs) maxNs = dt } // il primo 10 % scalda la JVM
                } catch (e: Exception) { /* punto non agganciabile o non raggiungibile */ }
            }
            println("%-70s %4d percorsi, media %.1f ms, max %.0f ms, lunghezza media %.0f km (%s)".format(label, ok, totalNs / 1e6 / maxOf(1, ok), maxNs / 1e6, totalKm / maxOf(1, ok), router.algorithm))
        }
    }
}

private fun opt(args: List<String>, name: String): String? = args.firstOrNull { it.startsWith("--$name=") }?.substringAfter('=')

private fun route(args: List<String>) {
    val positional = args.filter { !it.startsWith("--") }
    require(positional.size == 3) { "uso: route <cartella-grafo> <lat,lon> <lat,lon>" }
    val (fromLat, fromLon) = positional[1].split(',').map { it.trim().toDouble() }
    val (toLat, toLon) = positional[2].split(',').map { it.trim().toDouble() }
    val params = VehicleParameters(
        height = opt(args, "height")?.toDouble() ?: 0.0, weight = opt(args, "weight")?.toDouble() ?: 0.0,
        width = opt(args, "width")?.toDouble() ?: 0.0, length = opt(args, "length")?.toDouble() ?: 0.0,
        axleLoad = opt(args, "axleload")?.toDouble() ?: 0.0, hazmat = args.contains("--hazmat"),
    )
    val options = RouteOptions(
        profile = opt(args, "profile") ?: "driving-hgv",
        weighting = WeightingMethod.valueOf((opt(args, "weighting") ?: "recommended").uppercase()),
        avoidFeatures = (opt(args, "avoid") ?: "").split(',').filter { it.isNotBlank() }.fold(0) { acc, s -> acc or AvoidFeatureFlags.fromString(s) },
        vehicleParameters = if (params.hasAttributes) params else null,
        maximumSpeed = opt(args, "max-speed")?.toDouble() ?: Double.POSITIVE_INFINITY,
        turnCosts = !args.contains("--no-turn-costs"),
        allowCH = !args.contains("--no-ch"),
    )
    val t0 = System.nanoTime()
    RoutingGraph.load(File(positional[0])).use { rg ->
        val loadMs = (System.nanoTime() - t0) / 1e6
        val router = rg.router(options)
        val t1 = System.nanoTime()
        val from = router.snap(fromLat, fromLon, options.maximumSnappingRadius, 0); val to = router.snap(toLat, toLon, options.maximumSnappingRadius, 1)
        val alternatives = opt(args, "alternatives")?.toInt() ?: 0
        val paths = if (alternatives > 0) router.alternatives(from, to, 1 + alternatives, 1.4, 0.6) else listOf(router.route(from, to))
        val routeMs = (System.nanoTime() - t1) / 1e6
        for ((i, path) in paths.withIndex()) {
            println("percorso ${i + 1}/${paths.size}, profilo ${options.profile} (${options.weighting.name.lowercase()}, ${if (alternatives > 0) "alternative" else router.algorithm}): %.1f km, %d min %d s, peso %.1f, %d archi".format(
                path.distance / 1000, path.millis / 60000, path.millis / 1000 % 60, path.weight, path.edges.size))
            if (!args.contains("--quiet")) {
                var last = ""
                for (e in path.edges) if (e.name != last && e.name.isNotEmpty()) { println("  - ${e.name}"); last = e.name }
            }
        }
        println("caricamento %.0f ms, calcolo %.1f ms".format(loadMs, routeMs))
    }
}

fun profilesFromArgs(args: List<String>): List<VehicleProfile> {
    val spec = args.firstOrNull { it.startsWith("--profiles=") }?.substringAfter('=') ?: "car,hgv"
    return spec.split(',').map {
        when (it.trim()) {
            "car", "driving-car" -> CarProfile()
            "hgv", "driving-hgv", "truck" -> HgvProfile()
            else -> error("profilo sconosciuto: $it")
        }
    }
}

private fun importGraph(args: List<String>) {
    val positional = args.filter { !it.startsWith("--") }
    require(positional.size == 2) { "uso: import <file.osm.pbf> <cartella-grafo>" }
    val pbf = File(positional[0]); val dir = File(positional[1])
    val profiles = profilesFromArgs(args)
    val importOptions = it.navmaster.engine.osm.ImportOptions(widthFromCarriageway = !args.contains("--no-width-from-carriageway"))
    println("${Engine.NAME}: import di $pbf → $dir, profili ${profiles.map { it.name }}" + if (!importOptions.widthFromCarriageway) " (tag width ignorato)" else "")
    val (rg, stats) = RoutingGraph.build(pbf, dir, profiles, mmap = args.contains("--mmap"), importOptions = importOptions)
    rg.use {
        println("nodi OSM %,d, way %,d (accettate %,d)".format(stats.osmNodes, stats.osmWays, stats.acceptedWays))
        println("grafo: %,d nodi, %,d archi (%,d pillar, %,d archi barriera), divieti di svolta %,d applicati su %,d, %.1f s".format(
            stats.towers, stats.edges, stats.pillars, stats.barrierEdges, stats.turnRestrictions, stats.turnRestrictionRelations, stats.millis / 1000.0))
        println("bbox: ${it.graph.bounds}")
        if (!args.contains("--no-ch")) it.prepareCH(log = { msg -> println(msg) }, coreWide = opt(args, "core") != "narrow")
    }
}

private fun info(args: List<String>) {
    require(args.size == 1) { "uso: info <cartella-grafo>" }
    RoutingGraph.load(File(args[0])).use { rg ->
        val graph = rg.graph
        println("profili: ${rg.profileNames}")
        println("%,d nodi, %,d archi, %,d voci di costo di svolta, bbox ${graph.bounds}".format(graph.nodeCount, graph.edgeCount, rg.turnCosts.count))
        println("valori codificati (${graph.intsForFlags} int per arco): ${graph.encodedValues.all.joinToString { it.name }}")
        println("indice spaziale: ${rg.index.cells} celle, ${rg.index.entries} voci")
    }
}
