package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgeFilter
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.roundToLong

/**
 * Percorsi alternativi per i viaggi lunghi e per quelli tra Paesi, dove il metodo del plateau di ORS
 * (ricerca piena bidirezionale) non sta nella memoria del tablet e non attraversa i confini.
 *
 * Il metodo dei punti di passaggio: dopo il percorso migliore, si calcolano candidati che passano per
 * un punto spostato di lato a metà del viaggio (±[OFFSETS] della distanza in linea d'aria), agganciato
 * alla strada principale più vicina, nella direzione del viaggio (`bearings`) e senza inversione al
 * punto (`continue_straight`). Ogni candidato usa la ricerca normale (core, landmark, valichi): costa
 * quanto un percorso. Un candidato vale se dura al più [MAX_FACTOR] volte il migliore (o il
 * `weight_factor` della richiesta, se più stretto) e se ha in comune con quelli già tenuti al più la
 * parte [shareLimit] della sua lunghezza. TomTom fa qualcosa di simile con le penalità (metodo A delle
 * sue alternative, vedi `docs/analisi/tomtom-go-expert-offline-routing.md`).
 *
 * [route] calcola una richiesta senza alternative e restituisce la FeatureCollection GeoJSON; il
 * secondo argomento è il Paese di ogni punto (null per il punto di passaggio).
 */
class LongAlternatives(
    private val graphs: List<RoutingGraph>,
    private val log: ((String) -> Unit)? = null,
    /** Oltre questo tempo (ms) non si comincia un altro candidato. */
    private val budgetMs: Long = 35_000,
    private val route: (DirectionsRequest, List<String?>) -> Map<String, Any?>,
) {
    @Suppress("UNCHECKED_CAST")
    fun handle(request: DirectionsRequest, pointGraph: List<String?>): Map<String, Any?> {
        val alt = request.alternatives ?: return route(request, pointGraph)
        require(request.coordinates.size == 2) { "alternative solo fra due punti" }
        val started = System.currentTimeMillis()
        val base = request.withPoints(request.coordinates, request.radiuses, request.exactPoints, request.bearings)
        val first = route(base, pointGraph)
        val f0 = (first["features"] as List<Map<String, Any?>>).first()
        val (sLat, sLon) = request.coordinates[0]; val (eLat, eLon) = request.coordinates[1]
        val crow = DistanceCalc.haversine(sLat, sLon, eLat, eLon)
        val d0 = duration(f0)
        val maxDuration = d0 * min(alt.weightFactor, MAX_FACTOR)
        val shareLimit = shareLimit(alt.shareFactor, crow)
        val kept = arrayListOf(f0)
        val keptSegments = arrayListOf(segments(f0))
        var tried = 0
        val mid = along(coordinates(f0), 0.5)
        val vias = ArrayList<Pair<Double, Double>>()
        for (fraction in OFFSETS) {
            if (kept.size >= alt.targetCount || System.currentTimeMillis() - started > budgetMs) break
            val target = offset(mid, sLat, sLon, eLat, eLon, fraction * crow)
            // the main road nearest to the target, not too far from it, and not one already tried
            val via = viaPoint(request, target.first, target.second, maxOf(MIN_VIA_SEARCH, 0.6 * kotlin.math.abs(fraction) * crow)) ?: continue
            if (vias.any { DistanceCalc.haversine(it.first, it.second, via.first, via.second) < maxOf(2_000.0, 0.03 * crow) }) continue
            vias += via
            tried++
            val pts = listOf(request.coordinates[0], via, request.coordinates[1])
            val radiuses = doubleArrayOf(request.radiuses?.getOrNull(0) ?: -1.0, VIA_RADIUS, request.radiuses?.getOrNull(1) ?: -1.0)
            // at the via point no direction is imposed: continue_straight keeps it from being a u-turn
            val bearings = listOf(request.bearings?.getOrNull(0), null, request.bearings?.getOrNull(1))
            val viaRequest = DirectionsRequest(
                request.profile, pts, request.preference, request.units, request.language, request.instructions, request.htmlFormat,
                request.geometry, request.maneuvers, request.avoidFeatures, request.vehicleType, request.vehicleParameters, request.avoidPolygons,
                request.maximumSpeed, continueStraight = true, navmasterRoads = request.navmasterRoads, alternatives = null,
                radiuses = radiuses,
                id = request.id, attributes = request.attributes, extraInfo = request.extraInfo, echo = request.echo,
                exactPoints = null, bearings = bearings.takeIf { b -> b.any { it != null } }, avoidPenalty = request.avoidPenalty,
            )
            val t0 = System.currentTimeMillis()
            val fc = try { route(viaRequest, listOf(pointGraph.getOrNull(0), null, pointGraph.getOrNull(1))) } catch (e: ApiException) {
                log?.invoke("alternativa ${pct(fraction)}: nessun percorso (${e.message?.take(80)})"); continue
            }
            val f = joinVia((fc["features"] as List<Map<String, Any?>>).first(), request.language)
            val d = duration(f)
            val segs = segments(f)
            val share = keptSegments.maxOf { k -> shared(segs, k) }
            val ok = d <= maxDuration && share <= shareLimit
            log?.invoke("alternativa ${pct(fraction)}: ${(d / 60).roundToLong()} min contro ${(d0 / 60).roundToLong()}, in comune ${(share * 100).roundToLong()} %, " +
                "${System.currentTimeMillis() - t0} ms${if (ok) "" else " — scartata"}")
            if (!ok) continue
            kept += f; keptSegments += segs
        }
        log?.invoke("alternative dei viaggi lunghi: ${kept.size - 1} su $tried candidati in ${System.currentTimeMillis() - started} ms")
        val bbox = BBox.empty()
        for (f in kept) { val b = f["bbox"] as DoubleArray; bbox.include(b[1], b[0]); bbox.include(b[3], b[2]) }
        val out = LinkedHashMap(first)
        out["features"] = kept
        out["bbox"] = doubleArrayOf(DirectionsService.r6(bbox.minLon), DirectionsService.r6(bbox.minLat), DirectionsService.r6(bbox.maxLon), DirectionsService.r6(bbox.maxLat))
        return out
    }

    /**
     * Il punto di passaggio: il punto della strada principale (autostrada, superstrada, statale,
     * rampe escluse) più vicino a ([lat], [lon]) entro circa [VIA_SEARCH] m, in uno dei grafi.
     */
    private fun viaPoint(request: DirectionsRequest, lat: Double, lon: Double, maxDistance: Double): Pair<Double, Double>? {
        var best: Pair<Double, Double>? = null; var bestDist = Double.POSITIVE_INFINITY
        for (g in graphs) {
            val b = g.graph.bounds
            if (!b.isValid || lat < b.minLat - 0.3 || lat > b.maxLat + 0.3 || lon < b.minLon - 0.4 || lon > b.maxLon + 0.4) continue
            val pe = try { g.encoding.profile(g.encoderName(request.profile)) } catch (e: Exception) { continue }
            val sh = g.encoding.shared
            val major = EdgeFilter { e ->
                e.get(sh.roadClass) in MAJOR && !e.get(sh.roadClassLink) && (e.get(pe.access) || e.getReverse(pe.access))
            }
            val s = g.index.findClosest(lat, lon, major, maxRings = 60)
            if (s.isValid && s.distance < minOf(VIA_SEARCH, maxDistance) && s.distance < bestDist) { bestDist = s.distance; best = s.snappedLat to s.snappedLon }
        }
        return best
    }

    companion object {
        /** Gli spostamenti laterali del punto di passaggio, in frazioni della distanza in linea d'aria. */
        val OFFSETS = doubleArrayOf(0.12, -0.12, 0.22, -0.22)
        /** Al più tanto più lungo (in tempo) del percorso migliore. */
        const val MAX_FACTOR = 1.3
        const val VIA_RADIUS = 300.0
        const val VIA_SEARCH = 30_000.0
        const val MIN_VIA_SEARCH = 3_000.0
        private val MAJOR = setOf(RoadClass.MOTORWAY, RoadClass.TRUNK, RoadClass.PRIMARY)

        /** Sui viaggi lunghi l'inizio e la fine sono quasi sempre in comune: la soglia si allarga fino a 0,75. */
        fun shareLimit(requested: Double, crow: Double): Double = if (crow > 150_000) maxOf(requested, 0.75) else requested

        @Suppress("UNCHECKED_CAST")
        fun duration(f: Map<String, Any?>): Double = ((f["properties"] as Map<String, Any?>)["summary"] as Map<String, Any?>)["duration"].let { (it as Number).toDouble() }

        @Suppress("UNCHECKED_CAST")
        fun coordinates(f: Map<String, Any?>): List<DoubleArray> = (f["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>

        /** I tratti della geometria con la loro lunghezza, per chiave (i due capi arrotondati, in qualunque verso). */
        fun segments(f: Map<String, Any?>): Map<Long, Double> {
            val c = coordinates(f)
            val out = HashMap<Long, Double>(c.size * 2)
            for (i in 1 until c.size) {
                val a = key(c[i - 1]); val b = key(c[i])
                if (a == b) continue
                out[if (a < b) a * 31 + b else b * 31 + a] = DistanceCalc.haversine(c[i - 1][1], c[i - 1][0], c[i][1], c[i][0])
            }
            return out
        }

        private fun key(p: DoubleArray): Long = (Math.round(p[0] * 1e5) shl 26) xor Math.round(p[1] * 1e5)

        /** La parte della lunghezza di [a] che è anche in [b]. */
        fun shared(a: Map<Long, Double>, b: Map<Long, Double>): Double {
            var total = 0.0; var common = 0.0
            for ((k, len) in a) { total += len; if (k in b) common += len }
            return if (total > 0) common / total else 1.0
        }

        /** Il punto a [t] (0..1) della lunghezza della linea (lat, lon). */
        fun along(c: List<DoubleArray>, t: Double): Pair<Double, Double> {
            if (c.size < 2) return c.firstOrNull()?.let { it[1] to it[0] } ?: (0.0 to 0.0)
            var total = 0.0
            for (i in 1 until c.size) total += DistanceCalc.haversine(c[i - 1][1], c[i - 1][0], c[i][1], c[i][0])
            var acc = 0.0; val goal = total * t
            for (i in 1 until c.size) {
                val d = DistanceCalc.haversine(c[i - 1][1], c[i - 1][0], c[i][1], c[i][0])
                if (acc + d >= goal && d > 0) {
                    val k = (goal - acc) / d
                    return (c[i - 1][1] + (c[i][1] - c[i - 1][1]) * k) to (c[i - 1][0] + (c[i][0] - c[i - 1][0]) * k)
                }
                acc += d
            }
            return c.last()[1] to c.last()[0]
        }

        /** [p] spostato di [meters] perpendicolarmente alla linea dalla partenza all'arrivo (positivo: a sinistra). */
        fun offset(p: Pair<Double, Double>, sLat: Double, sLon: Double, eLat: Double, eLon: Double, meters: Double): Pair<Double, Double> {
            val k = cos(Math.toRadians((sLat + eLat) / 2))
            val dx = (eLon - sLon) * k; val dy = eLat - sLat
            val n = hypot(dx, dy).takeIf { it > 0 } ?: return p
            val px = -dy / n; val py = dx / n
            val deg = meters / DistanceCalc.METERS_PER_DEGREE
            return (p.first + py * deg) to (p.second + px * deg / cos(Math.toRadians(p.first)))
        }

        /**
         * Il percorso con il punto di passaggio come un percorso solo: i due tratti uniti, via l'arrivo
         * al punto e la partenza da lì (come ai valichi, [CrossBorderDirections.joinSteps]).
         */
        @Suppress("UNCHECKED_CAST")
        fun joinVia(f: Map<String, Any?>, language: String): Map<String, Any?> {
            val props = LinkedHashMap(f["properties"] as Map<String, Any?>)
            val segs = props["segments"] as List<Map<String, Any?>>
            val wps = props["way_points"] as List<Int>
            if (segs.size != 2 || wps.size != 3) return f
            val a = LinkedHashMap(segs[0]); val b = segs[1]
            a["distance"] = (a["distance"] as Number).toDouble() + (b["distance"] as Number).toDouble()
            a["duration"] = CrossBorderDirections.round1((a["duration"] as Number).toDouble() + (b["duration"] as Number).toDouble())
            val sa = (a["steps"] as? List<Map<String, Any?>>)?.toMutableList()
            val sb = (b["steps"] as? List<Map<String, Any?>>)?.toMutableList()
            if (sa != null && sb != null) {
                CrossBorderDirections.joinSteps(sa, sb, coordinates(f), wps[1], InstructionTranslator.forLanguage(language))
                a["steps"] = sa
            }
            a.remove("percentage")
            props["segments"] = listOf(a)
            props["way_points"] = listOf(wps[0], wps[2])
            val out = LinkedHashMap(f)
            out["properties"] = props
            return out
        }

        private fun pct(f: Double) = "${if (f > 0) "+" else ""}${(f * 100).roundToLong()} %"
    }
}
