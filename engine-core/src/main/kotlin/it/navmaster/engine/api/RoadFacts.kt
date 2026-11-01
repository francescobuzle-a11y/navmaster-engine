package it.navmaster.engine.api

import it.navmaster.engine.index.LocationIndex
import it.navmaster.engine.profile.GuidanceAttributes
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.WayType
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.GuidanceStore
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.storage.PointStore
import it.navmaster.engine.storage.ProfileEncoding
import it.navmaster.engine.util.DistanceCalc

/**
 * Estensione NavMaster (fuori dall'API di openrouteservice, solo dati): i "fatti" sulle strade del
 * percorso, uno per arco, letti dal grafo e dal file `guidance`. Non influiscono sul calcolo.
 *
 * Ogni voce copre i punti `from..to` della geometria del percorso e contiene: `way` (id OSM),
 * `class` (highway senza `_link`), `link`, `roundabout`, `tunnel`, `bridge`, `ferry`, `toll` (per il
 * profilo della richiesta), `surface`, `oneway`, `access` (il profilo può percorrere l'arco in questo
 * verso), `maxspeed`, `maxspeed_hgv`, `lanes`, `turn_lanes`, `ref`, `name`, `destination`,
 * `destination_ref`, `exit` (svincolo "ref|nome" al nodo d'ingresso), `sign` (cartello di una
 * relazione `destination_sign` all'imbocco: destination, ref, colour), `lane_signs` (cartelli sulle
 * corsie: destination, ref, colour, campi `:lanes`), `limits` (altezza, larghezza, lunghezza, peso,
 * carico per asse in m/t, solo quelli presenti), `hazmat` (vietato alle merci pericolose),
 * `adr_tunnel` (categoria B…E), `conditional` (restrizioni a orario "chiave=valore"), `nodes`
 * (caselli, portali, dogane: tipo e indice del punto) e `branches` (le altre strade che lasciano il
 * nodo finale dell'arco: direzione iniziale, classe, link, corsie, accesso, destinazioni, svincolo).
 */
class RoadFactsBuilder(private val graph: BaseGraph, private val encoding: GraphEncoding, private val profile: ProfileEncoding, private val guidance: GuidanceStore?) {
    private val sh = encoding.shared
    private val it = graph.createEdgeIterator()
    private val branchIt = graph.createEdgeIterator()
    private val isHgv = profile.profile.name == "heavyvehicle"

    fun build(legs: List<RoutePath>): List<Map<String, Any?>> {
        val out = ArrayList<Map<String, Any?>>()
        var pointIndex = 0
        for ((li, leg) in legs.withIndex()) {
            for ((ei, e) in leg.edges.withIndex()) {
                val first = if (li == 0 && ei == 0) pointIndex else pointIndex - 1
                val last = first + e.geometry.size - 1
                it.setEdge(e.edge, e.baseNode)
                val m = facts(e.edge, it.reverse)
                m["from"] = first; m["to"] = last
                m["access"] = it.get(profile.access)
                // caselli/portali/dogane: solo se la geometria dell'arco è completa (non un tratto parziale agli estremi)
                guidance?.specialNodes(e.edge)?.takeIf { s -> s.isNotEmpty() }?.let { s ->
                    val full = graph.fetchWayGeometry(e.edge, false, FetchMode.ALL).size
                    if (full == e.geometry.size) {
                        m["nodes"] = s.split(';').mapNotNull { item ->
                            val at = item.indexOf('@'); if (at < 0) return@mapNotNull null
                            val k = item.substring(at + 1).toIntOrNull() ?: return@mapNotNull null
                            val local = if (it.reverse) full - 1 - k else k
                            linkedMapOf("type" to item.substring(0, at), "index" to first + local)
                        }
                    }
                }
                // le altre strade che lasciano il nodo finale (per la forma degli incroci e i cartelli delle altre uscite)
                val nextEdge = leg.edges.getOrNull(ei + 1)?.edge ?: legs.getOrNull(li + 1)?.edges?.firstOrNull()?.edge ?: -1
                val branches = branches(e.adjNode, e.edge, nextEdge)
                if (branches.isNotEmpty()) m["branches"] = branches
                out += m
                pointIndex += e.geometry.size - 1 + (if (li == 0 && ei == 0) 1 else 0)
            }
        }
        return out
    }

    /** Le strade che lasciano [node], salvo [edge] e [nextEdge], con la direzione dei primi metri. */
    private fun branches(node: Int, edge: Int, nextEdge: Int): List<Map<String, Any?>> {
        val out = ArrayList<Map<String, Any?>>(4)
        branchIt.setBaseNode(node)
        while (branchIt.next()) {
            val b = branchIt.edge
            if (b == edge || b == nextEdge) continue
            val pl = branchIt.fetchWayGeometry(FetchMode.ALL)
            if (pl.size < 2) continue
            var k = 1
            while (k < pl.size - 1 && DistanceCalc.haversine(pl.lat(0), pl.lon(0), pl.lat(k), pl.lon(k)) < 8.0) k++
            val m = LinkedHashMap<String, Any?>()
            m["heading"] = Math.round(DistanceCalc.bearing(pl.lat(0), pl.lon(0), pl.lat(k), pl.lon(k)))
            m["way"] = branchIt.get(sh.osmWayId)
            m["class"] = branchIt.get(sh.roadClass).osm
            m["link"] = branchIt.get(sh.roadClassLink)
            m["roundabout"] = branchIt.get(sh.roundabout)
            m["access"] = branchIt.get(profile.access)
            m["access_back"] = branchIt.getReverse(profile.access)
            val g = guidance
            if (g != null) {
                g.lanes(b, branchIt.reverse).takeIf { v -> v > 0 }?.let { v -> m["lanes"] = v }
                g.destination(b, branchIt.reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination"] = s }
                g.destinationRef(b, branchIt.reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination_ref"] = s }
                g.junctionAtStart(b, branchIt.reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["exit"] = s }
                g.relationSign(b, branchIt.reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["sign"] = sign(s) }
                val ref = g.ref(b); if (ref.isNotEmpty()) m["ref"] = ref
            }
            val name = graph.name(b); if (name.isNotEmpty()) m["name"] = name
            out += m
        }
        return out
    }

    private fun sign(s: String): Map<String, Any?> {
        val p = s.split('|')
        return linkedMapOf("destination" to p.getOrElse(0) { "" }, "ref" to p.getOrElse(1) { "" }, "colour" to p.getOrElse(2) { "" })
    }

    private fun laneSigns(s: String): Map<String, Any?> {
        val p = s.split(GuidanceAttributes.SEP_FIELD)
        return linkedMapOf("destination" to p.getOrElse(0) { "" }, "ref" to p.getOrElse(1) { "" }, "colour" to p.getOrElse(2) { "" })
    }

    /** I fatti di [edge] percorso nel verso dato (l'iteratore deve essere già posizionato sull'arco). */
    private fun facts(edge: Int, reverse: Boolean): LinkedHashMap<String, Any?> {
        val m = LinkedHashMap<String, Any?>()
        m["way"] = it.get(sh.osmWayId)
        m["class"] = it.get(sh.roadClass).osm
        m["link"] = it.get(sh.roadClassLink)
        m["roundabout"] = it.get(sh.roundabout)
        val wt = it.get(sh.wayType)
        m["ferry"] = wt == WayType.FERRY
        val toll = it.get(sh.toll)
        m["toll"] = toll == Toll.ALL || (isHgv && toll == Toll.HGV)
        m["surface"] = it.get(sh.waySurface).name.lowercase()
        val g = guidance
        val name = graph.name(edge)
        val ref = g?.ref(edge) ?: ""
        // il nome del grafo è "nome, ref" (come ORS): qui nome e ref vanno separati
        m["name"] = if (ref.isNotEmpty() && name.endsWith(", $ref")) name.dropLast(ref.length + 2) else if (ref.isNotEmpty() && name == ref) "" else name
        if (ref.isNotEmpty()) m["ref"] = ref
        if (g != null) {
            m["tunnel"] = g.tunnel(edge); m["bridge"] = g.bridge(edge)
            val ow = g.oneway(edge)
            m["oneway"] = ow != 0
            g.maxSpeed(edge, reverse).takeIf { v -> v > 0 }?.let { v -> m["maxspeed"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
            g.maxSpeedHgv(edge, reverse).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_hgv"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
            g.lanes(edge, reverse).takeIf { v -> v > 0 }?.let { v -> m["lanes"] = v }
            g.turnLanes(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["turn_lanes"] = s }
            g.destination(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination"] = s }
            g.destinationRef(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination_ref"] = s }
            g.junctionAtStart(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["exit"] = s }
            g.relationSign(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["sign"] = sign(s) }
            g.laneSigns(edge, reverse).takeIf { s -> s.isNotEmpty() }?.let { s -> m["lane_signs"] = laneSigns(s) }
            g.adrTunnel(edge).takeIf { v -> v in 2..5 }?.let { v -> m["adr_tunnel"] = "BCDE"[v - 2].toString() }
            g.conditional(edge).takeIf { s -> s.isNotEmpty() }?.let { s -> m["conditional"] = s.split(GuidanceAttributes.SEP_ITEM) }
        } else {
            m["tunnel"] = false; m["bridge"] = false; m["oneway"] = false
        }
        val limits = LinkedHashMap<String, Any?>()
        it.get(sh.maxHeight).takeIf { v -> v.isFinite() }?.let { v -> limits["height"] = v }
        it.get(sh.maxWidth).takeIf { v -> v.isFinite() }?.let { v -> limits["width"] = v }
        it.get(sh.maxLength).takeIf { v -> v.isFinite() }?.let { v -> limits["length"] = v }
        it.get(sh.maxWeight).takeIf { v -> v.isFinite() }?.let { v -> limits["weight"] = v }
        it.get(sh.maxAxleLoad).takeIf { v -> v.isFinite() }?.let { v -> limits["axleload"] = v }
        if (limits.isNotEmpty()) m["limits"] = limits
        if (it.get(sh.hazmatForbidden)) m["hazmat"] = true
        return m
    }

    /**
     * Le strade entro [radiusMeters] da un punto (per la vista degli svincoli, la strada su cui si
     * sta guidando, il punto "più avanti"): per ogni arco i fatti nei due versi e la geometria
     * `coordinates` [lon, lat] da A a B. `oneway` qui è 1 (A→B), -1 (B→A) o 0; `access_ab`/`access_ba`
     * dicono se il profilo può percorrere l'arco in quel verso; `lanes_ab`/`lanes_ba`, `turn_lanes_ab`/
     * `turn_lanes_ba`, `maxspeed_ab`/`maxspeed_ba`, `destination_ab`/`destination_ba`, `exit_ab`/`exit_ba`
     * e `sign_ab`/`sign_ba` sono per verso.
     */
    fun around(index: LocationIndex, lat: Double, lon: Double, radiusMeters: Double): List<Map<String, Any?>> {
        val dLat = radiusMeters / 111_195.0
        val dLon = radiusMeters / (111_195.0 * Math.cos(Math.toRadians(lat)))
        val edges = index.query(lat - dLat, lon - dLon, lat + dLat, lon + dLon)
        val out = ArrayList<Map<String, Any?>>(edges.size)
        val g = guidance
        for (edge in edges) {
            it.setEdge(edge, graph.nodeA(edge))
            val m = facts(edge, false)
            for (k in listOf("maxspeed", "maxspeed_hgv", "lanes", "turn_lanes", "destination", "destination_ref", "exit", "sign", "lane_signs")) m.remove(k)
            m["oneway"] = g?.oneway(edge) ?: 0
            m["access_ab"] = profile.access.get(it.flags, false)
            m["access_ba"] = profile.access.get(it.flags, true)
            if (g != null) {
                for ((suffix, rev) in listOf("ab" to false, "ba" to true)) {
                    g.maxSpeed(edge, rev).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_$suffix"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
                    g.lanes(edge, rev).takeIf { v -> v > 0 }?.let { v -> m["lanes_$suffix"] = v }
                    g.turnLanes(edge, rev).takeIf { s -> s.isNotEmpty() }?.let { s -> m["turn_lanes_$suffix"] = s }
                    g.destination(edge, rev).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination_$suffix"] = s }
                    g.destinationRef(edge, rev).takeIf { s -> s.isNotEmpty() }?.let { s -> m["destination_ref_$suffix"] = s }
                    g.junctionAtStart(edge, rev).takeIf { s -> s.isNotEmpty() }?.let { s -> m["exit_$suffix"] = s }
                    g.relationSign(edge, rev).takeIf { s -> s.isNotEmpty() }?.let { s -> m["sign_$suffix"] = sign(s) }
                }
                g.maxSpeedHgv(edge, false).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_hgv"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
            }
            val pl = graph.fetchWayGeometry(edge, false, FetchMode.ALL)
            val coords = ArrayList<DoubleArray>(pl.size)
            for (i in 0 until pl.size) coords.add(doubleArrayOf(Math.round(pl.lon(i) * 1e6) / 1e6, Math.round(pl.lat(i) * 1e6) / 1e6))
            m["coordinates"] = coords
            out += m
        }
        return out
    }

    /**
     * Autovelox, Tutor e fotocamere ai semafori entro [maxDistance] metri dalla linea del percorso:
     * `type`, `lat`, `lon`, `value` (km/h, 0 se assente), `bearing` (-1 se assente), `index` (il
     * segmento della geometria), `offset` (metri dall'inizio del percorso), `distance` (metri dalla linea).
     */
    fun pointsAlong(legs: List<RoutePath>, points: PointStore, maxDistance: Double = 30.0): List<Map<String, Any?>> {
        val pl = PointList(256)
        for ((li, leg) in legs.withIndex()) {
            val pts = leg.points()
            for (i in (if (li == 0) 0 else 1) until pts.size) pl.add(pts.lat(i), pts.lon(i))
        }
        if (pl.size < 2) return emptyList()
        var minLat = Double.MAX_VALUE; var maxLat = -Double.MAX_VALUE; var minLon = Double.MAX_VALUE; var maxLon = -Double.MAX_VALUE
        val cum = DoubleArray(pl.size)
        for (i in 0 until pl.size) {
            minLat = minOf(minLat, pl.lat(i)); maxLat = maxOf(maxLat, pl.lat(i)); minLon = minOf(minLon, pl.lon(i)); maxLon = maxOf(maxLon, pl.lon(i))
            if (i > 0) cum[i] = cum[i - 1] + DistanceCalc.haversine(pl.lat(i - 1), pl.lon(i - 1), pl.lat(i), pl.lon(i))
        }
        val margin = maxDistance / 111_195.0 * 1.5
        val candidates = points.inBBox(minLat - margin, minLon - margin, maxLat + margin, maxLon + margin)
        val out = ArrayList<Map<String, Any?>>()
        for (p in candidates) {
            val la = points.lat(p); val lo = points.lon(p)
            val k = Math.cos(Math.toRadians(la))
            var best = Double.MAX_VALUE; var bestI = -1; var bestOffset = 0.0
            for (i in 0 until pl.size - 1) {
                val ax = (pl.lon(i) - lo) * k; val ay = pl.lat(i) - la
                val bx = (pl.lon(i + 1) - lo) * k; val by = pl.lat(i + 1) - la
                // distanza (in gradi scalati) dal punto al segmento, e frazione lungo il segmento
                val dx = bx - ax; val dy = by - ay
                val len2 = dx * dx + dy * dy
                val t = if (len2 > 0) ((-ax * dx - ay * dy) / len2).coerceIn(0.0, 1.0) else 0.0
                val px = ax + dx * t; val py = ay + dy * t
                val d = Math.sqrt(px * px + py * py) * 111_195.0
                if (d < best) { best = d; bestI = i; bestOffset = cum[i] + (cum[i + 1] - cum[i]) * t }
            }
            if (bestI < 0 || best > maxDistance) continue
            out += linkedMapOf("type" to PointStore.typeName(points.type(p)), "lat" to la, "lon" to lo, "value" to points.value(p),
                "bearing" to points.bearing(p), "index" to bestI, "offset" to Math.round(bestOffset * 10) / 10.0, "distance" to Math.round(best * 10) / 10.0)
        }
        out.sortBy { (it["offset"] as Double) }
        return out
    }
}
