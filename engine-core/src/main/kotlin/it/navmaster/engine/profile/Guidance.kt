package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay

/**
 * Dati di guida di una way, indipendenti dal profilo e senza effetto sul calcolo del percorso:
 * limiti di velocità, corsie e frecce, destinazioni, ref, galleria/ponte, senso unico, categoria
 * ADR della galleria, restrizioni a orario, cartelli sulle corsie. Servono all'app (corsie,
 * cartelli, limiti lungo il percorso) e vengono salvati nel file `guidance`.
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
    /** Categoria ADR della galleria: 0 nessuna, 2 = B … 5 = E (`hazmat:B=no` … `hazmat:E=no`, la più restrittiva). */
    val adrTunnel: Int = 0,
    /** Restrizioni a orario (`*:conditional`), "chiave=valore" separate da [SEP_ITEM]; vuoto se assenti. */
    val conditional: String = "",
    /** Cartelli sulle corsie: `destination:lanes`, `destination:ref:lanes`, `destination:colour:lanes` separati da [SEP_FIELD]. */
    val laneSignsFwd: String = "", val laneSignsBwd: String = "",
) {
    companion object {
        const val NO_LIMIT = 255
        const val SEP_ITEM = '\u001e'
        const val SEP_FIELD = '\u001f'
        val NONE = GuidanceAttributes()
    }
}

object GuidanceParser {
    private val onewayYes = setOf("yes", "1", "true")
    private val onewayReverse = setOf("-1", "reverse")
    private val impliedOneway = setOf("motorway", "motorway_link", "trunk_link")
    private val adrLetters = listOf("B", "C", "D", "E")
    /** Le restrizioni a orario che riguardano un mezzo pesante o le sue misure. */
    private val conditionalKeys = listOf("access:conditional", "vehicle:conditional", "motor_vehicle:conditional", "hgv:conditional", "goods:conditional",
        "bus:conditional", "maxweight:conditional", "maxweight:hgv:conditional", "maxheight:conditional", "maxwidth:conditional", "maxlength:conditional",
        "maxaxleload:conditional", "maxspeed:conditional", "maxspeed:hgv:conditional", "hazmat:conditional", "oneway:conditional")

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
            adrTunnel = adrTunnel(way),
            conditional = conditional(way),
            laneSignsFwd = laneSigns(way, "forward", oneway != -1),
            laneSignsBwd = laneSigns(way, "backward", oneway == -1),
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

    /** Categoria ADR della galleria: la lettera più restrittiva con `hazmat:X=no` (B = 2 … E = 5), 0 se nessuna. */
    fun adrTunnel(way: OsmWay): Int {
        way.tag("hazmat:adr_tunnel_category")?.trim()?.uppercase()?.let { c -> adrLetters.indexOf(c).takeIf { it >= 0 }?.let { return it + 2 } }
        for ((i, l) in adrLetters.withIndex()) if (way.tag("hazmat:$l") == "no") return i + 2
        return 0
    }

    fun conditional(way: OsmWay): String {
        val sb = StringBuilder()
        for (k in conditionalKeys) {
            val v = way.tag(k) ?: continue
            if (sb.isNotEmpty()) sb.append(GuidanceAttributes.SEP_ITEM)
            sb.append(k).append('=').append(v.replace(GuidanceAttributes.SEP_ITEM, ' ').replace(GuidanceAttributes.SEP_FIELD, ' '))
        }
        return sb.toString()
    }

    /** I cartelli sulle corsie nel verso [dir]: i tag con suffisso, altrimenti quelli semplici se [plainApplies]. */
    fun laneSigns(way: OsmWay, dir: String, plainApplies: Boolean): String {
        fun tag(base: String): String = way.tag("$base:$dir") ?: (if (plainApplies) way.tag(base) ?: "" else "")
        val dest = tag("destination:lanes"); val ref = tag("destination:ref:lanes"); val colour = tag("destination:colour:lanes")
        if (dest.isEmpty() && ref.isEmpty()) return ""
        return listOf(dest, ref, colour).joinToString(GuidanceAttributes.SEP_FIELD.toString()) { it.replace(GuidanceAttributes.SEP_FIELD, ' ').replace(GuidanceAttributes.SEP_ITEM, ' ') }
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

    /** Un cartello di una relazione `destination_sign`: "destinazioni|ref|colore". */
    fun relationSign(tags: Map<String, String>): String? {
        val dest = tags["destination"] ?: ""; val ref = tags["destination:ref"] ?: ""; val colour = tags["colour"] ?: tags["destination:colour"] ?: ""
        if (dest.isEmpty() && ref.isEmpty()) return null
        return "$dest|$ref|$colour"
    }
}
