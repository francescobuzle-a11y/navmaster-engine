package it.navmaster.engine.api

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.instructions.InstructionType
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.Router
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import java.io.File

/** Il grafo di un Paese, per i viaggi tra Paesi. */
class CountryGraph(val id: String, val service: DirectionsService) {
    val graph: RoutingGraph get() = service.routingGraph
}

/**
 * Un punto in cui si passa da un grafo all'altro: lo stesso punto della stessa strada nei due grafi
 * (le strade che attraversano il confine sono in entrambi gli estratti OSM dei due Paesi). [rank]:
 * l'importanza della strada (1 autostrada … 7 strada di servizio).
 */
class Transfer(val lat: Double, val lon: Double, val rank: Int)

/** I punti di passaggio fra due grafi, trovati una volta e salvati. */
object Transfers {
    /** Quanto è durata l'ultima ricerca dei passaggi, per il registro. */
    @Volatile var timing: String = ""

    /**
     * I punti di passaggio fra [a] e [b]: i nodi senza uscita di un grafo (dove il suo estratto
     * finisce, oltre il confine) che nell'altro sono un punto di una strada che prosegue.
     */
    fun find(a: RoutingGraph, b: RoutingGraph): List<Transfer> {
        val ba = a.graph.bounds; val bb = b.graph.bounds
        if (!ba.isValid || !bb.isValid || !ba.intersects(bb)) return emptyList()
        val overlap = BBox.empty().apply {
            include(maxOf(ba.minLat, bb.minLat), maxOf(ba.minLon, bb.minLon))
            include(minOf(ba.maxLat, bb.maxLat), minOf(ba.maxLon, bb.maxLon))
        }
        val t0 = System.currentTimeMillis()
        val degA = degrees(a); val degB = degrees(b)
        val t1 = System.currentTimeMillis()
        val out = ArrayList<Transfer>()
        val qa = scan(a, degA, b, degB, overlap, out)
        val t2 = System.currentTimeMillis()
        val qb = scan(b, degB, a, degA, overlap, out)
        timing = "gradi ${t1 - t0} ms, A→B ${t2 - t1} ms ($qa ricerche), B→A ${System.currentTimeMillis() - t2} ms ($qb ricerche)"
        // un punto per luogo: i due capi di una stessa strada di confine restano (possono distare km)
        val kept = ArrayList<Transfer>()
        for (t in out.sortedBy { it.rank }) if (kept.none { DistanceCalc.haversine(it.lat, it.lon, t.lat, t.lon) < 25.0 }) kept += t
        return kept
    }

    /** Il numero di archi di ogni nodo (fino a 3). */
    private fun degrees(g: RoutingGraph): ByteArray {
        val bg = g.graph
        val deg = ByteArray(bg.nodeCount)
        for (e in 0 until bg.edgeCount) {
            val x = bg.nodeA(e); val y = bg.nodeB(e)
            if (x == y) continue
            if (deg[x] < 3) deg[x]++
            if (deg[y] < 3) deg[y]++
        }
        return deg
    }

    private fun scan(src: RoutingGraph, degSrc: ByteArray, dst: RoutingGraph, degDst: ByteArray, overlap: BBox, out: MutableList<Transfer>): Int {
        var queries = 0
        val g = src.graph
        val it = g.createEdgeIterator()
        val rc = src.encoding.shared.roadClass
        for (n in 0 until g.nodeCount) {
            if (degSrc[n].toInt() != 1) continue
            val lat = g.lat(n); val lon = g.lon(n)
            if (!overlap.contains(lat, lon)) continue
            // solo dove l'altro grafo ha strade (la sua griglia è vuota fuori dal suo estratto)
            if (!dst.index.hasEdgesNear(lat, lon)) continue
            queries++
            val s = dst.index.findClosest(lat, lon, maxRings = 1)
            if (!s.isValid || s.distance > 1.0) continue
            // senza uscita anche nell'altro: una strada chiusa nella zona coperta da entrambi, non un passaggio
            if (s.position == SnapPosition.TOWER && degDst[s.closestNode] <= 1) continue
            it.setBaseNode(n)
            if (!it.next()) continue
            out += Transfer(lat, lon, rank(it.get(rc)))
        }
        return queries
    }

    fun rank(c: RoadClass): Int = when (c) {
        RoadClass.MOTORWAY -> 1
        RoadClass.TRUNK -> 2
        RoadClass.PRIMARY -> 3
        RoadClass.SECONDARY -> 4
        RoadClass.TERTIARY -> 5
        RoadClass.UNCLASSIFIED, RoadClass.RESIDENTIAL, RoadClass.LIVING_STREET -> 6
        else -> 7
    }

    fun write(f: File, list: List<Transfer>) {
        f.parentFile?.mkdirs()
        val tmp = File(f.path + ".tmp")
        tmp.writeText(buildString { append("NMTRANSFERS 1\n"); for (t in list) append(t.lat).append(',').append(t.lon).append(',').append(t.rank).append('\n') })
        tmp.renameTo(f)
    }

    fun read(f: File): List<Transfer>? {
        if (!f.exists()) return null
        val lines = f.readLines()
        if (lines.firstOrNull() != "NMTRANSFERS 1") return null
        return lines.drop(1).filter { it.isNotBlank() }.mapNotNull { l ->
            val p = l.split(',')
            if (p.size != 3) null else Transfer(p[0].toDouble(), p[1].toDouble(), p[2].toInt())
        }
    }
}

/**
 * Viaggi tra Paesi con un grafo per Paese: ogni tratto che passa da un grafo all'altro è calcolato
 * nei grafi dei Paesi attraversati e cucito ai punti di passaggio del confine ([Transfers]). Fra
 * tutti i passaggi candidati di ciascun confine si sceglie la combinazione col peso totale minimo
 * (lo stesso peso di openrouteservice: tempo, distanza o "recommended" con le restrizioni del mezzo),
 * passando anche per Paesi intermedi (fino a due). La risposta è quella di openrouteservice
 * (GeoJSON) come per un percorso in un grafo solo: le istruzioni proseguono al confine senza un
 * arrivo intermedio, gli extra e i fatti sulle strade sono uniti.
 */
class CrossBorderDirections(
    private val graphs: List<CountryGraph>,
    /** I passaggi fra due grafi (calcolati una volta e conservati da chi chiama). */
    private val transfers: (CountryGraph, CountryGraph) -> List<Transfer>,
    private val log: ((String) -> Unit)? = null,
) {
    private class LL(val lat: Double, val lon: Double)

    /** Un pezzo del viaggio dentro un grafo: i suoi punti (le tappe e i passaggi di confine). */
    private class Piece(val graph: CountryGraph, val points: MutableList<Pair<Double, Double>>, val radiuses: MutableList<Double>,
                        val exact: MutableList<Boolean> = MutableList(points.size) { false })

    /** Lo stato di una richiesta: router e agganci per grafo, pesi già calcolati. */
    private inner class Ctx(val request: DirectionsRequest) {
        private val routers = HashMap<String, Router>()
        private val snaps = HashMap<String, Snap?>()
        private val weights = HashMap<String, Double>()
        var queries = 0

        fun router(g: CountryGraph): Router = routers.getOrPut(g.id) { g.graph.router(g.service.routeOptions(request)) }

        /** L'aggancio di un punto ([exact]: un passaggio di confine, senza il controllo delle reti isolate). */
        fun snap(g: CountryGraph, p: LL, radius: Double, exact: Boolean = radius == TRANSFER_RADIUS): Snap? = snaps.getOrPut("${g.id}:${p.lat}:${p.lon}:$radius") {
            try { router(g).snap(p.lat, p.lon, radius, 0, checkNetwork = !exact) } catch (e: PointNotFoundException) { null }
        }

        fun weight(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double): Double = weights.getOrPut("${g.id}:${a.lat}:${a.lon}>${b.lat}:${b.lon}") {
            val sa = snap(g, a, ra) ?: return@getOrPut Double.POSITIVE_INFINITY
            val sb = snap(g, b, rb) ?: return@getOrPut Double.POSITIVE_INFINITY
            queries++
            try { router(g).route(sa, sb).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
        }
    }

    /**
     * La risposta GeoJSON di openrouteservice per [request]; [pointGraph]: il Paese (id del grafo) di
     * ogni punto quando è noto, altrimenti il grafo che lo aggancia più vicino.
     */
    fun handle(request: DirectionsRequest, pointGraph: List<String?>): Map<String, Any?> {
        val started = System.currentTimeMillis()
        val ctx = Ctx(request)
        val pts = request.coordinates.map { LL(it.first, it.second) }
        val radius = { i: Int -> request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius }
        val of = pts.indices.map { i ->
            val wanted = pointGraph.getOrNull(i)?.let { id -> graphs.firstOrNull { it.id == id } }
            val g = wanted?.takeIf { ctx.snap(it, pts[i], radius(i)) != null } ?: graphs
                .filter { it.graph.graph.bounds.contains(pts[i].lat, pts[i].lon) }
                .mapNotNull { g -> ctx.snap(g, pts[i], radius(i))?.let { s -> g to s.distance } }
                .minByOrNull { it.second }?.first
            g ?: throw ApiException(404, RoutingErrorCodes.POINT_NOT_FOUND,
                "Could not find routable point within a radius of ${radius(i).toLong()} meters of specified coordinate $i: %.7f %.7f.".format(pts[i].lon, pts[i].lat))
        }
        log?.invoke("punti nei grafi: ${of.joinToString { it.id }}")
        if (of.all { it === of[0] }) return of[0].service.handle(request, ResponseFormat.GEOJSON)

        // i pezzi del viaggio: le tappe nello stesso grafo restano insieme, ai confini il passaggio migliore
        val pieces = ArrayList<Piece>()
        var cur = Piece(of[0], mutableListOf(request.coordinates[0]), mutableListOf(radius(0)))
        for (i in 1 until pts.size) {
            if (of[i] !== cur.graph) {
                val chain = chain(ctx, pts[i - 1], radius(i - 1), cur.graph, pts[i], radius(i), of[i])
                    ?: throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${i - 1} (%.7f %.7f) and $i (%.7f %.7f).".format(
                        pts[i - 1].lon, pts[i - 1].lat, pts[i].lon, pts[i].lat))
                for ((t, next) in chain) {
                    cur.points += t.lat to t.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true
                    pieces += cur
                    cur = Piece(next, mutableListOf(t.lat to t.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true))
                }
            }
            cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false
        }
        pieces += cur
        val docs = pieces.map { p ->
            val r = p.graph.service.handle(request.withPoints(p.points, p.radiuses.toDoubleArray(), p.exact.toBooleanArray()), ResponseFormat.GEOJSON)
            @Suppress("UNCHECKED_CAST")
            (r["features"] as List<Map<String, Any?>>).first()
        }
        log?.invoke("viaggio tra Paesi: ${pieces.joinToString(" → ") { it.graph.id }}, ${ctx.queries} ricerche, ${System.currentTimeMillis() - started} ms")
        val feature = merge(docs, request)
        @Suppress("UNCHECKED_CAST")
        val bbox = feature["bbox"] as DoubleArray
        val metadata = LinkedHashMap<String, Any?>()
        metadata["attribution"] = pieces[0].graph.service.attribution
        metadata["service"] = "routing"
        metadata["timestamp"] = System.currentTimeMillis()
        metadata["query"] = LinkedHashMap<String, Any?>(request.echo).apply { put("profile", request.profile); put("format", "geojson") }
        metadata["engine"] = pieces[0].graph.service.engineInfo()
        metadata["countries"] = pieces.map { it.graph.id }
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bbox, "features" to listOf(feature), "metadata" to metadata)
    }

    // ------------------------------------------------------------------------------- the borders

    /** Le sequenze di grafi da [a] a [b] (al più due intermedi) che stanno vicino alla linea p → q. */
    private fun countryPaths(a: CountryGraph, b: CountryGraph, p: LL, q: LL): List<List<CountryGraph>> {
        val crow = DistanceCalc.haversine(p.lat, p.lon, q.lat, q.lon)
        val reach = crow * 0.35 + 60_000.0
        fun near(g: CountryGraph) = distanceToBox(p, q, g.graph.graph.bounds) <= reach
        fun linked(x: CountryGraph, y: CountryGraph) = x.graph.graph.bounds.intersects(y.graph.graph.bounds) && transfers(x, y).isNotEmpty()
        val out = ArrayList<List<CountryGraph>>()
        fun walk(path: List<CountryGraph>) {
            val last = path.last()
            if (last === b) { out += path; return }
            if (path.size >= 4) return
            for (g in graphs) {
                if (g in path) continue
                if (g !== b && !near(g)) continue
                if (!last.graph.graph.bounds.intersects(g.graph.graph.bounds)) continue
                if (!linked(last, g)) continue
                walk(path + g)
            }
        }
        walk(listOf(a))
        // prima le sequenze più corte
        return out.sortedBy { it.size }.take(6)
    }

    /** Distanza (m, approssimata) fra il segmento p → q e un rettangolo. */
    private fun distanceToBox(p: LL, q: LL, b: BBox): Double {
        var best = Double.MAX_VALUE
        for (i in 0..40) {
            val t = i / 40.0
            val la = p.lat + (q.lat - p.lat) * t; val lo = p.lon + (q.lon - p.lon) * t
            val cla = la.coerceIn(b.minLat, b.maxLat); val clo = lo.coerceIn(b.minLon, b.maxLon)
            best = minOf(best, DistanceCalc.haversine(la, lo, cla, clo))
        }
        return best
    }

    /** I passaggi da provare su un confine: i migliori per deviazione dalla linea p → q, prima le strade principali. */
    private fun candidates(list: List<Transfer>, p: LL, q: LL, transit: Boolean): List<Transfer> {
        fun detour(t: Transfer) = DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon) + DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)
        val major = list.filter { it.rank <= 3 }.sortedBy { detour(it) }.take(if (transit) 6 else 8)
        // le strade minori solo vicino alla partenza o all'arrivo (un viaggio corto oltre il confine)
        val minor = list.filter { it.rank > 3 }.filter { t ->
            minOf(DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon), DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)) < 40_000.0
        }.sortedBy { detour(it) }.take(if (transit) 2 else 5)
        return (major + minor).distinct()
    }

    /**
     * I passaggi di confine del percorso migliore da p (grafo [ga]) a q (grafo [gb]), con il grafo
     * in cui si entra a ciascuno; null quando nessuna sequenza di Paesi ha un percorso.
     */
    private fun chain(ctx: Ctx, p: LL, rp: Double, ga: CountryGraph, q: LL, rq: Double, gb: CountryGraph): List<Pair<Transfer, CountryGraph>>? {
        var best = Double.POSITIVE_INFINITY
        var bestChain: List<Pair<Transfer, CountryGraph>>? = null
        val paths = countryPaths(ga, gb, p, q)
        log?.invoke("sequenze ${ga.id} → ${gb.id}: ${paths.joinToString(" | ") { pa -> pa.joinToString("-") { it.id } }}")
        for (path in paths) {
            // programmazione dinamica sui confini: il peso migliore fino a ogni passaggio
            var layer: List<Triple<Transfer, Double, List<Transfer>>> = emptyList()
            var ok = true
            for (k in 1 until path.size) {
                val from = path[k - 1]; val to = path[k]
                val cands = candidates(transfers(from, to), p, q, path.size > 2)
                    // il passaggio deve essere percorribile dal mezzo in entrambi i grafi
                    .filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } }
                log?.invoke("  ${path.joinToString("-") { it.id }} confine ${from.id}/${to.id}: ${cands.size} candidati")
                if (cands.isEmpty()) { ok = false; break }
                val next = ArrayList<Triple<Transfer, Double, List<Transfer>>>()
                for (t in cands) {
                    val tl = LL(t.lat, t.lon)
                    var w = Double.POSITIVE_INFINITY; var via: List<Transfer> = emptyList()
                    if (k == 1) {
                        w = ctx.weight(from, p, rp, tl, TRANSFER_RADIUS); via = listOf(t)
                    } else for ((pt, pw, pc) in layer) {
                        if (pw >= best || pw >= w) continue
                        val x = pw + ctx.weight(from, LL(pt.lat, pt.lon), TRANSFER_RADIUS, tl, TRANSFER_RADIUS)
                        if (x < w) { w = x; via = pc + t }
                    }
                    if (w.isFinite() && w < best) next += Triple(t, w, via)
                }
                layer = next.sortedBy { it.second }.take(if (k == path.size - 1) next.size else 8)
                log?.invoke("    pesi ${from.id}/${to.id}: ${layer.joinToString { "%.0f".format(it.second) }}")
                if (layer.isEmpty()) { ok = false; break }
            }
            if (!ok) continue
            val last = path.last()
            for ((t, w, via) in layer) {
                if (w >= best) continue
                val total = w + ctx.weight(last, LL(t.lat, t.lon), TRANSFER_RADIUS, q, rq)
                if (total < best) { best = total; bestChain = via.mapIndexed { i, tr -> tr to path[i + 1] } }
            }
        }
        return bestChain
    }

    // ------------------------------------------------------------------------------- the answer

    /** I pezzi uniti in un percorso solo, come se fosse stato calcolato in un grafo. */
    @Suppress("UNCHECKED_CAST")
    private fun merge(docs: List<Map<String, Any?>>, request: DirectionsRequest): Map<String, Any?> {
        val coords = ArrayList<DoubleArray>()
        val segments = ArrayList<MutableMap<String, Any?>>()
        val wayPoints = ArrayList<Int>().also { it += 0 }
        val extraValues = LinkedHashMap<String, MutableList<MutableList<Number>>>()
        val extraDist = LinkedHashMap<String, LinkedHashMap<Number, Double>>()
        val roads = ArrayList<Map<String, Any?>>()
        val points = ArrayList<Map<String, Any?>>()
        val bbox = BBox.empty()
        var hasRoads = false
        var cur: MutableMap<String, Any?>? = null
        val translator = InstructionTranslator.forLanguage(request.language)
        val pieceStarts = ArrayList<Int>()
        for ((pi, f) in docs.withIndex()) {
            val props = f["properties"] as Map<String, Any?>
            val geo = (f["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            val offset = if (pi == 0) 0 else coords.size - 1
            pieceStarts += offset
            for (i in (if (pi == 0) 0 else 1) until geo.size) { coords += geo[i]; bbox.include(geo[i][1], geo[i][0]) }
            val segs = props["segments"] as List<Map<String, Any?>>
            val wps = props["way_points"] as List<Int>
            for ((si, seg0) in segs.withIndex()) {
                val seg = LinkedHashMap(seg0)
                val steps: MutableList<Map<String, Any?>>? = (seg["steps"] as? List<Map<String, Any?>>)?.map<Map<String, Any?>, Map<String, Any?>> { shiftStep(it, offset) }?.toMutableList()
                if (steps != null) seg["steps"] = steps
                val c = cur
                if (si == 0 && pi > 0 && c != null) {
                    // dopo un passaggio di confine: lo stesso tratto prosegue
                    c["distance"] = (c["distance"] as Double) + (seg["distance"] as Double)
                    c["duration"] = round1((c["duration"] as Double) + (seg["duration"] as Double))
                    val cs = c["steps"] as MutableList<Map<String, Any?>>?
                    if (cs != null && steps != null) joinSteps(cs, steps, coords, offset, translator)
                } else {
                    cur?.let { segments += it }
                    cur = seg
                }
                val endsAtBorder = si == segs.size - 1 && pi < docs.size - 1
                if (!endsAtBorder) { segments += cur!!; cur = null; wayPoints += wps[si + 1] + offset }
            }
            // gli extra: gli intervalli spostati, uniti al confine quando il valore non cambia
            (props["extras"] as? Map<String, Any?>)?.forEach { (kind, v) ->
                val m = v as Map<String, Any?>
                val vals = extraValues.getOrPut(kind) { ArrayList() }
                for (r in m["values"] as List<List<Number>>) {
                    val from = r[0].toInt() + offset; val to = r[1].toInt() + offset; val value = r[2]
                    val last = vals.lastOrNull()
                    if (last != null && last[2] == value && last[1].toInt() == from) last[1] = to else vals += mutableListOf(from, to, value)
                }
                val dist = extraDist.getOrPut(kind) { LinkedHashMap() }
                for (sm in m["summary"] as List<Map<String, Any?>>) {
                    val value = sm["value"] as Number
                    dist[value] = (dist[value] ?: 0.0) + (sm["distance"] as Number).toDouble()
                }
            }
            (props["navmaster"] as? Map<String, Any?>)?.let { nm ->
                hasRoads = true
                (nm["roads"] as? List<Map<String, Any?>>)?.forEach { r ->
                    val m = LinkedHashMap(r)
                    m["from"] = (r["from"] as Int) + offset; m["to"] = (r["to"] as Int) + offset
                    (r["nodes"] as? List<Map<String, Any?>>)?.let { ns -> m["nodes"] = ns.map { n -> LinkedHashMap(n).apply { put("index", (n["index"] as Int) + offset) } } }
                    roads += m
                }
                (nm["points"] as? List<Map<String, Any?>>)?.forEach { p ->
                    if (points.none { it["lat"] == p["lat"] && it["lon"] == p["lon"] && it["type"] == p["type"] }) {
                        points += LinkedHashMap(p).apply { put("index", (p["index"] as Int) + offset); put("piece", pi) }
                    }
                }
            }
        }
        cur?.let { segments += it }
        // le distanze lungo il percorso dei punti (autovelox…): dall'inizio del percorso unito
        if (points.isNotEmpty()) {
            val cum = DoubleArray(coords.size)
            for (i in 1 until coords.size) cum[i] = cum[i - 1] + DistanceCalc.haversine(coords[i - 1][1], coords[i - 1][0], coords[i][1], coords[i][0])
            for (p in points) {
                val m = p as MutableMap<String, Any?>
                val start = pieceStarts[m.remove("piece") as Int]
                m["offset"] = Math.round(((m["offset"] as Number).toDouble() + cum[start]) * 10) / 10.0
            }
            points.sortBy { (it["offset"] as Number).toDouble() }
        }
        val totalDistance = segments.sumOf { it["distance"] as Double }
        val totalDuration = round1(segments.sumOf { it["duration"] as Double })
        if ("percentage" in request.attributes) for (s in segments) s["percentage"] = if (totalDistance > 0) Math.round((s["distance"] as Double) * 100 / totalDistance * 100) / 100.0 else 0.0
        val properties = LinkedHashMap<String, Any?>()
        properties["segments"] = segments
        properties["way_points"] = wayPoints
        properties["summary"] = linkedMapOf("distance" to DirectionsService.roundForUnits(totalDistance, request.units), "duration" to totalDuration)
        if (extraValues.isNotEmpty()) {
            properties["extras"] = extraValues.mapValuesTo(LinkedHashMap()) { (kind, vals) ->
                val dist = extraDist[kind] ?: LinkedHashMap()
                val total = dist.values.sum()
                linkedMapOf("values" to vals, "summary" to dist.entries.sortedByDescending { it.value }.map { (v, d) ->
                    linkedMapOf("value" to v, "distance" to DirectionsService.roundForUnits(d, request.units), "amount" to (if (total > 0) Math.round(d * 100 / total * 100) / 100.0 else 0.0))
                })
            }
        }
        if (hasRoads) properties["navmaster"] = linkedMapOf("roads" to roads, "points" to points)
        val bboxArr = doubleArrayOf(DirectionsService.r6(bbox.minLon), DirectionsService.r6(bbox.minLat), DirectionsService.r6(bbox.maxLon), DirectionsService.r6(bbox.maxLat))
        return linkedMapOf("bbox" to bboxArr, "type" to "Feature", "properties" to properties,
            "geometry" to linkedMapOf("coordinates" to coords, "type" to "LineString"))
    }

    @Suppress("UNCHECKED_CAST")
    private fun shiftStep(s: Map<String, Any?>, offset: Int): MutableMap<String, Any?> {
        val m = LinkedHashMap(s)
        val wp = s["way_points"] as List<Int>
        m["way_points"] = listOf(wp[0] + offset, wp[1] + offset)
        return m
    }

    /**
     * Le istruzioni al passaggio di confine: via l'arrivo del pezzo prima e la partenza di quello
     * dopo; la strada prosegue (stesso nome: un'istruzione sola) o "Continua dritto su …".
     */
    private fun joinSteps(before: MutableList<Map<String, Any?>>, after: MutableList<Map<String, Any?>>, coords: List<DoubleArray>, offset: Int,
                          t: InstructionTranslator) {
        if (before.lastOrNull()?.get("type") == InstructionType.FINISH.code) before.removeAt(before.size - 1)
        val depart = after.firstOrNull()
        if (depart == null) return
        val last = before.lastOrNull()
        val name = depart["name"] as? String ?: "-"
        val rest = after.drop(1)
        if (last != null && depart["type"] == InstructionType.DEPART.code && name != "-" && last["name"] == name) {
            // la stessa strada oltre il confine: l'istruzione di prima arriva fin dove arrivava la partenza
            val m = LinkedHashMap(last)
            m["distance"] = round1((last["distance"] as Number).toDouble() + (depart["distance"] as Number).toDouble())
            m["duration"] = round1((last["duration"] as Number).toDouble() + (depart["duration"] as Number).toDouble())
            m["way_points"] = listOf((last["way_points"] as List<*>)[0], (depart["way_points"] as List<*>)[1])
            before[before.size - 1] = m
        } else if (depart["type"] == InstructionType.DEPART.code) {
            val m = LinkedHashMap(depart)
            m["type"] = InstructionType.CONTINUE.code
            m["instruction"] = t.continueOn(InstructionType.CONTINUE, if (name == "-") "" else name)
            // la direzione d'arrivo al confine
            (depart["maneuver"] as? Map<*, *>)?.let { mv ->
                val k = offset
                val before0 = if (k >= 1) DistanceCalc.bearing(coords[k - 1][1], coords[k - 1][0], coords[k][1], coords[k][0]) else null
                m["maneuver"] = LinkedHashMap<String, Any?>().apply {
                    putAll(mv.entries.associate { it.key.toString() to it.value })
                    if (before0 != null) put("bearing_before", Math.round((before0 + 360) % 360).toInt())
                }
            }
            before += m
        } else before += depart
        before += rest
    }

    private fun round1(v: Double) = Math.round(v * 10) / 10.0

    companion object {
        /** Il raggio di aggancio di un passaggio di confine: il punto è un nodo della strada. */
        const val TRANSFER_RADIUS = 3.0
    }
}
