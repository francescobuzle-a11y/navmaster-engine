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
import it.navmaster.engine.storage.ProfileEncoding

/**
 * Estensione NavMaster (fuori dall'API di openrouteservice, solo dati): i "fatti" sulle strade del
 * percorso, uno per arco, letti dal grafo e dal file `guidance`. Non influiscono sul calcolo.
 *
 * Ogni voce copre i punti `from..to` della geometria del percorso e contiene: `way` (id OSM),
 * `class` (highway senza `_link`), `link`, `roundabout`, `tunnel`, `bridge`, `ferry`, `toll` (per il
 * profilo della richiesta), `surface`, `oneway`, `access` (il profilo può percorrere l'arco in questo
 * verso), `maxspeed`, `maxspeed_hgv`, `lanes`, `turn_lanes`, `ref`, `name`, `destination`,
 * `destination_ref`, `exit` (svincolo "ref|nome" al nodo d'ingresso), `limits` (altezza, larghezza,
 * lunghezza, peso, carico per asse in m/t, solo quelli presenti), `hazmat` (vietato alle merci
 * pericolose) e `nodes` (caselli, portali, dogane: tipo e indice del punto).
 */
class RoadFactsBuilder(private val graph: BaseGraph, private val encoding: GraphEncoding, private val profile: ProfileEncoding, private val guidance: GuidanceStore?) {
    private val sh = encoding.shared
    private val it = graph.createEdgeIterator()
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
                out += m
                pointIndex += e.geometry.size - 1 + (if (li == 0 && ei == 0) 1 else 0)
            }
        }
        return out
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
     * `turn_lanes_ba`, `maxspeed_ab`/`maxspeed_ba` sono per verso.
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
            m.remove("maxspeed"); m.remove("maxspeed_hgv"); m.remove("lanes"); m.remove("turn_lanes"); m.remove("destination"); m.remove("destination_ref"); m.remove("exit")
            m["oneway"] = g?.oneway(edge) ?: 0
            m["access_ab"] = profile.access.get(it.flags, false)
            m["access_ba"] = profile.access.get(it.flags, true)
            if (g != null) {
                g.maxSpeed(edge, false).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_ab"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
                g.maxSpeed(edge, true).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_ba"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
                g.maxSpeedHgv(edge, false).takeIf { v -> v > 0 }?.let { v -> m["maxspeed_hgv"] = if (v == GuidanceAttributes.NO_LIMIT) 0 else v }
                g.lanes(edge, false).takeIf { v -> v > 0 }?.let { v -> m["lanes_ab"] = v }
                g.lanes(edge, true).takeIf { v -> v > 0 }?.let { v -> m["lanes_ba"] = v }
                g.turnLanes(edge, false).takeIf { s -> s.isNotEmpty() }?.let { s -> m["turn_lanes_ab"] = s }
                g.turnLanes(edge, true).takeIf { s -> s.isNotEmpty() }?.let { s -> m["turn_lanes_ba"] = s }
            }
            val pl = graph.fetchWayGeometry(edge, false, FetchMode.ALL)
            val coords = ArrayList<DoubleArray>(pl.size)
            for (i in 0 until pl.size) coords.add(doubleArrayOf(Math.round(pl.lon(i) * 1e6) / 1e6, Math.round(pl.lat(i) * 1e6) / 1e6))
            m["coordinates"] = coords
            out += m
        }
        return out
    }
}
