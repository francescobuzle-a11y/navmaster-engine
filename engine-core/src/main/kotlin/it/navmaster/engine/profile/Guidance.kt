package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay

/**
 * Dati di guida di una way, indipendenti dal profilo e senza effetto sul calcolo del percorso:
 * limiti di velocità, corsie e frecce, destinazioni, ref, galleria/ponte, senso unico. Servono
 * all'app (corsie, cartelli, limiti lungo il percorso) e vengono salvati nel file `guidance`.
 *
 * I valori "forward" seguono il verso della way (nodeA → nodeB dell'arco), "backward" il contrario.
 * Velocità in km/h (0 = sconosciuta, [NO_LIMIT] = cartello "nessun limite"); corsie 0 = sconosciute.
 */
class GuidanceAttributes(
    val maxSpeedFwd: Int = 0, val maxSpeedBwd: Int = 0,
    val maxSpeedHgvFwd: Int = 0, val maxSpeedHgvBwd: Int = 0,
    val lanes: Int = 0, val lanesFwd: Int = 0, val lanesBwd: Int = 0,
    val tunnel: Boolean = false, val bridge: Boolean = false,
    /** 1 = senso unico nel verso della way, -1 = nel verso opposto, 0 = doppio senso. */
    val oneway: Int = 0,
    val ref: String = "",
    val turnLanesFwd: String = "", val turnLanesBwd: String = "",
    val destinationFwd: String = "", val destinationBwd: String = "",
    val destinationRefFwd: String = "", val destinationRefBwd: String = "",
) {
    companion object {
        const val NO_LIMIT = 255
        val NONE = GuidanceAttributes()
    }
}

object GuidanceParser {
    private val onewayYes = setOf("yes", "1", "true")
    private val onewayReverse = setOf("-1", "reverse")
    private val impliedOneway = setOf("motorway", "motorway_link", "trunk_link")

    fun parse(way: OsmWay): GuidanceAttributes {
        val oneway = oneway(way)
        // `turn:lanes` senza suffisso vale per il verso di marcia di un senso unico (come la guida di GraphHopper);
        // su una strada a doppio senso contano solo `turn:lanes:forward|backward`
        val plainTurn = way.tag("turn:lanes") ?: ""
        val turnFwd = way.tag("turn:lanes:forward") ?: (if (oneway == 1) plainTurn else "")
        val turnBwd = way.tag("turn:lanes:backward") ?: (if (oneway == -1) plainTurn else "")
        val plainDest = way.tag("destination") ?: ""
        val plainDestRef = way.tag("destination:ref") ?: ""
        return GuidanceAttributes(
            maxSpeedFwd = speed(way.tag("maxspeed:forward") ?: way.tag("maxspeed")),
            maxSpeedBwd = speed(way.tag("maxspeed:backward") ?: way.tag("maxspeed")),
            maxSpeedHgvFwd = speed(way.tag("maxspeed:hgv:forward") ?: way.tag("maxspeed:hgv")),
            maxSpeedHgvBwd = speed(way.tag("maxspeed:hgv:backward") ?: way.tag("maxspeed:hgv")),
            lanes = lanes(way.tag("lanes")),
            lanesFwd = lanes(way.tag("lanes:forward")),
            lanesBwd = lanes(way.tag("lanes:backward")),
            tunnel = way.tag("tunnel").let { it != null && it != "no" },
            bridge = way.tag("bridge").let { it != null && it != "no" },
            oneway = oneway,
            ref = (way.tag("ref") ?: "").replace(";", ", "),
            turnLanesFwd = turnFwd, turnLanesBwd = turnBwd,
            destinationFwd = way.tag("destination:forward") ?: (if (oneway != -1) plainDest else ""),
            destinationBwd = way.tag("destination:backward") ?: (if (oneway != 1) plainDest else ""),
            destinationRefFwd = way.tag("destination:ref:forward") ?: (if (oneway != -1) plainDestRef else ""),
            destinationRefBwd = way.tag("destination:ref:backward") ?: (if (oneway != 1) plainDestRef else ""),
        )
    }

    fun oneway(way: OsmWay): Int {
        val v = way.tag("oneway")
        if (v != null) return when (v) { in onewayYes -> 1; in onewayReverse -> -1; else -> 0 }
        if (way.hasTag("junction", "roundabout", "circular")) return 1
        if (way.tag("highway") in impliedOneway) return 1
        return 0
    }

    /** km/h intero 1..254, [GuidanceAttributes.NO_LIMIT] per "none", 0 se assente o illeggibile. */
    fun speed(raw: String?): Int {
        if (raw == null) return 0
        if (raw.trim() == "none") return GuidanceAttributes.NO_LIMIT
        val v = OsmValues.parseKmh(raw)
        if (v.isNaN() || v <= 0) return 0
        return Math.round(v).toInt().coerceIn(1, 254)
    }

    fun lanes(raw: String?): Int {
        val v = raw?.trim()?.substringBefore(';')?.substringBefore('.')?.toIntOrNull() ?: return 0
        return v.coerceIn(0, 255)
    }

    /** Informazione di un nodo utile alla guida: svincolo autostradale ("ref|name") o nodo speciale. */
    fun junction(tags: Map<String, String>): String? {
        if (tags["highway"] != "motorway_junction") return null
        val ref = tags["ref"] ?: ""; val name = tags["name"] ?: ""
        if (ref.isEmpty() && name.isEmpty()) return null
        return "$ref|$name".replace(";", ", ")
    }

    fun specialNode(tags: Map<String, String>): String? = when {
        tags["barrier"] == "toll_booth" -> "toll_booth"
        tags["highway"] == "toll_gantry" -> "toll_gantry"
        tags["barrier"] == "border_control" -> "border_control"
        else -> null
    }
}
