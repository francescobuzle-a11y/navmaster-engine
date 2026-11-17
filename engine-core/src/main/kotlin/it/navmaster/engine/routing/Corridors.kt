package it.navmaster.engine.routing

import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GuidanceStore

/** Un pezzo di una corsa autostradale: gli archi [fromIdx]..[toIdx] (compresi) del tratto [leg]. */
data class CorridorUnit(val leg: Int, val fromIdx: Int, val toIdx: Int, val meters: Double)

/**
 * Una corsa autostradale di un percorso: archi `motorway` consecutivi con lo stesso [ref] ("MW#n"
 * dove il ref manca), divisa in [units] agli svincoli con un'altra autostrada. [touchesPoint]:
 * comincia o finisce (entro 1 km) a un punto del viaggio.
 */
data class CorridorRun(val ref: String, val leg: Int, val units: List<CorridorUnit>, val meters: Double, val touchesPoint: Boolean) {
    val fromIdx: Int get() = units.first().fromIdx
    val toIdx: Int get() = units.last().toIdx
}

/**
 * Le corse autostradali di un percorso (per la fonte P delle varianti) e la sua firma (le strade
 * con un ref, per le etichette e il confronto delle varianti). Legge classe e ref da [info]; le
 * rampe degli svincoli (fuori dal percorso) solo sui nodi delle corse.
 */
class CorridorAnalyzer(private val graph: BaseGraph, private val info: EdgeInfo, private val guidance: GuidanceStore?) {
    private val it = graph.createEdgeIterator()

    /**
     * Le corse di almeno [MIN_RUN_METERS] e del [MIN_RUN_SHARE] del viaggio ([tripMeters]),
     * dalla più lunga.
     */
    fun runs(legs: List<RoutePath>, tripMeters: Double): List<CorridorRun> {
        val out = ArrayList<CorridorRun>()
        var unnamed = 0
        for ((li, leg) in legs.withIndex()) {
            val es = leg.edges
            val n = es.size
            var i = 0
            while (i < n) {
                if (!info.isMotorway(es[i].edge)) { i++; continue }
                var j = i
                while (j + 1 < n && info.isMotorway(es[j + 1].edge)) j++
                // i pezzi con lo stesso ref (-1: nessuno)
                class Chunk(var ref: Int, val from: Int, var to: Int, var meters: Double, var name: String? = null)
                val chunks = ArrayList<Chunk>()
                for (k in i..j) {
                    val r = info.refId(es[k].edge)
                    val last = chunks.lastOrNull()
                    if (last != null && last.ref == r) { last.to = k; last.meters += es[k].distance } else chunks += Chunk(r, k, k, es[k].distance)
                }
                // un buco senza ref sotto 2 km eredita il ref vicino; più lungo, "MW#n"
                for ((ci, c) in chunks.withIndex()) if (c.ref < 0) {
                    val prev = chunks.getOrNull(ci - 1); val next = chunks.getOrNull(ci + 1)
                    val near = prev?.let { p -> if (p.ref >= 0) info.refName(p.ref) else p.name } ?: next?.takeIf { it.ref >= 0 }?.let { info.refName(it.ref) }
                    c.name = if (c.meters < GAP_METERS && near != null) near else "MW#${++unnamed}"
                }
                val merged = ArrayList<Chunk>()
                for (c in chunks) {
                    val name = c.name ?: info.refName(c.ref)
                    val last = merged.lastOrNull()
                    if (last != null && last.name == name) { last.to = c.to; last.meters += c.meters } else merged += Chunk(c.ref, c.from, c.to, c.meters, name)
                }
                for (c in merged) {
                    val before = es.subList(0, c.from).sumOf { it.distance }
                    val after = es.subList(c.to + 1, n).sumOf { it.distance }
                    out += CorridorRun(c.name!!, li, units(li, es, c.from, c.to, c.name!!), c.meters, before < TOUCH_METERS || after < TOUCH_METERS)
                }
                i = j + 1
            }
        }
        return out.filter { it.meters >= MIN_RUN_METERS && it.meters >= MIN_RUN_SHARE * tripMeters }.sortedByDescending { it.meters }
    }

    /**
     * Le unità della corsa [from]..[to] del tratto [leg]: si spezza ai nodi con una rampa fuori dal
     * percorso il cui `destination:ref` nomina un'altra autostrada; le unità sotto [MIN_UNIT_METERS]
     * si fondono con la precedente. Senza dati di guida: terzi per distanza.
     */
    private fun units(leg: Int, es: List<PathEdge>, from: Int, to: Int, ref: String): List<CorridorUnit> {
        val cuts = ArrayList<Int>() // ultimo indice di ogni unità
        val total = (from..to).sumOf { es[it].distance }
        if (guidance == null) {
            var acc = 0.0; var third = 1
            for (k in from until to) { acc += es[k].distance; if (acc >= total * third / 3.0 && third < 3) { cuts += k; third++ } }
        } else for (k in from until to) {
            val node = es[k].adjNode
            if (interchange(node, es[k].edge, es[k + 1].edge, ref)) cuts += k
        }
        cuts += to
        val raw = ArrayList<CorridorUnit>()
        var start = from
        for (c in cuts) { if (c < start) continue; raw += CorridorUnit(leg, start, c, (start..c).sumOf { es[it].distance }); start = c + 1 }
        // le unità corte si fondono (con la precedente, la prima con la seguente)
        val out = ArrayList<CorridorUnit>()
        for (u in raw) {
            val last = out.lastOrNull()
            if (last != null && (u.meters < MIN_UNIT_METERS || last.meters < MIN_UNIT_METERS)) out[out.size - 1] = CorridorUnit(leg, last.fromIdx, u.toIdx, last.meters + u.meters)
            else out += u
        }
        return out
    }

    /** Al nodo [node] esce (fuori dal percorso) una rampa autostradale verso un'altra autostrada. */
    private fun interchange(node: Int, inEdge: Int, outEdge: Int, ref: String): Boolean {
        it.setBaseNode(node)
        var degree = 0
        while (it.next()) degree++
        if (degree <= 2) return false
        it.setBaseNode(node)
        while (it.next()) {
            val e = it.edge
            if (e == inEdge || e == outEdge || !info.isMotorway(e) || !info.isLink(e)) continue
            for (part in guidance!!.destinationRef(e, it.reverse).split(';', ',', '/')) {
                val r = EdgeInfo.normalizeRef(part) ?: continue
                if (r != ref && isMotorwayRef(r)) return true
            }
        }
        return false
    }

    /**
     * La firma di un percorso: i ref (normalizzati) in ordine con i km, solo i pezzi di almeno
     * [SIGNATURE_METERS] (gli archi senza ref non interrompono un pezzo), i consecutivi uguali uniti.
     */
    fun signature(legs: List<RoutePath>): List<Pair<String, Double>> {
        val chunks = ArrayList<Pair<Int, Double>>()
        var cur = -1; var m = 0.0
        for (leg in legs) for (e in leg.edges) {
            val r = info.refId(e.edge)
            if (r < 0) continue
            if (r == cur) m += e.distance else { if (cur >= 0) chunks += cur to m; cur = r; m = e.distance }
        }
        if (cur >= 0) chunks += cur to m
        val out = ArrayList<Pair<String, Double>>()
        for ((r, meters) in chunks) {
            if (meters < SIGNATURE_METERS) continue
            val name = info.refName(r)
            val last = out.lastOrNull()
            if (last != null && last.first == name) out[out.size - 1] = name to last.second + meters / 1000 else out += name to meters / 1000
        }
        return out.map { it.first to Math.round(it.second * 10) / 10.0 }
    }

    companion object {
        const val MIN_RUN_METERS = 30_000.0
        const val MIN_RUN_SHARE = 0.10
        const val GAP_METERS = 2_000.0
        const val MIN_UNIT_METERS = 10_000.0
        const val TOUCH_METERS = 1_000.0
        const val SIGNATURE_METERS = 5_000.0

        /** Un ref da autostrada (A1, A22, RA6, M1, E35, S16…), non una statale (SS9, B37). */
        fun isMotorwayRef(r: String): Boolean {
            val letters = r.takeWhile { it.isLetter() }
            return letters in MOTORWAY_PREFIXES
        }
        private val MOTORWAY_PREFIXES = setOf("A", "AP", "RA", "M", "E", "S", "T", "BAB")
    }
}
