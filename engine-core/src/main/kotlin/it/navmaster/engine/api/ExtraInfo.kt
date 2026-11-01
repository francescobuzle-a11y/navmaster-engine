package it.navmaster.engine.api

import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.RoadAccess
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.profile.WayType
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.ProfileEncoding

/**
 * `extra_info` di ORS: per ogni tipo richiesto, intervalli `[da, a, valore]` sugli indici della geometria
 * e un riepilogo `{value, distance, amount}` (percentuale della distanza). Gli archi consecutivi con lo
 * stesso valore vengono uniti in un solo intervallo (ORS `RouteExtraInfo`).
 */
class ExtraInfoBuilder(private val graph: BaseGraph, private val encoding: GraphEncoding, private val profile: ProfileEncoding) {
    private val sh = encoding.shared
    private val it = graph.createEdgeIterator()
    private val isHgv = profile.profile.name == "heavyvehicle"

    val supported = listOf("tollways", "waycategory", "waytype", "surface", "roadaccessrestrictions", "osmid", "avgspeed")

    /** Valore dell'extra per l'arco corrente dell'iteratore. */
    private fun valueOf(kind: String): Number = when (kind) {
        "tollways" -> { val t = it.get(sh.toll); if (t == Toll.ALL || (isHgv && t == Toll.HGV)) 1 else 0 }
        "waycategory" -> {
            var v = 0
            if (it.get(sh.motorway)) v = v or AvoidFeatureFlags.HIGHWAYS
            val t = it.get(sh.toll); if (t == Toll.ALL || (isHgv && t == Toll.HGV)) v = v or AvoidFeatureFlags.TOLLWAYS
            val wt = it.get(sh.wayType)
            if (wt == WayType.STEPS) v = v or AvoidFeatureFlags.STEPS
            if (wt == WayType.FERRY) v = v or AvoidFeatureFlags.FERRIES
            if (it.get(sh.ford)) v = v or AvoidFeatureFlags.FORDS
            v
        }
        "waytype" -> it.get(sh.wayType).code
        "surface" -> it.get(sh.waySurface).code
        "roadaccessrestrictions" -> when (it.get(sh.roadAccess)) { // ORS AccessRestrictionType
            RoadAccess.NO -> 1; RoadAccess.CUSTOMERS -> 2; RoadAccess.DESTINATION -> 4; RoadAccess.DELIVERY -> 8; RoadAccess.PRIVATE -> 16
            else -> 0
        }
        "osmid" -> it.get(sh.osmWayId)
        "avgspeed" -> it.get(profile.speed)
        else -> 0
    }

    /** @param legs tratti del percorso nell'ordine; la geometria complessiva salta il primo punto dei tratti successivi al primo. */
    fun build(kinds: Collection<String>, legs: List<RoutePath>, unitsPerMeter: Double, unitDecimals: Int): Map<String, Any?> {
        val result = LinkedHashMap<String, Any?>()
        for (kind in kinds) {
            if (kind !in supported) continue
            val values = ArrayList<List<Number>>()
            val distByValue = LinkedHashMap<Number, Double>()
            var total = 0.0
            var pointIndex = 0
            var curValue: Number? = null; var curFrom = 0
            for ((li, leg) in legs.withIndex()) {
                for ((ei, e) in leg.edges.withIndex()) {
                    it.setEdge(e.edge, e.baseNode)
                    val v = valueOf(kind)
                    val startIdx = pointIndex
                    // numero di nuovi punti aggiunti da questo arco alla geometria
                    val newPts = e.geometry.size - 1 + (if (li == 0 && ei == 0) 1 else 0)
                    if (curValue == null) { curValue = v; curFrom = startIdx }
                    else if (v != curValue) { values.add(listOf(curFrom, startIdx, curValue!!)); curValue = v; curFrom = startIdx }
                    distByValue[v] = (distByValue[v] ?: 0.0) + e.distance
                    total += e.distance
                    pointIndex += newPts
                }
            }
            if (curValue != null) values.add(listOf(curFrom, pointIndex - 1, curValue!!))
            val summary = distByValue.entries.sortedByDescending { it.value }.map { (v, d) ->
                linkedMapOf("value" to v, "distance" to roundTo(d * unitsPerMeter, unitDecimals), "amount" to roundTo(if (total > 0) d * 100 / total else 0.0, 2))
            }
            result[kind] = linkedMapOf("values" to values, "summary" to summary)
        }
        return result
    }

    private fun roundTo(v: Double, decimals: Int): Double { val f = Math.pow(10.0, decimals.toDouble()); return Math.round(v * f) / f }
}
