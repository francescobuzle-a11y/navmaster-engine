package it.navmaster.engine.instructions

import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.PathEdge
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.storage.ProfileEncoding
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max

/** Segnali di GraphHopper (`Instruction.*`). */
object Sign {
    const val UNKNOWN = -99
    const val U_TURN_UNKNOWN = -98
    const val U_TURN_LEFT = -8
    const val KEEP_LEFT = -7
    const val LEAVE_ROUNDABOUT = -6
    const val TURN_SHARP_LEFT = -3
    const val TURN_LEFT = -2
    const val TURN_SLIGHT_LEFT = -1
    const val CONTINUE_ON_STREET = 0
    const val TURN_SLIGHT_RIGHT = 1
    const val TURN_RIGHT = 2
    const val TURN_SHARP_RIGHT = 3
    const val FINISH = 4
    const val REACHED_VIA = 5
    const val USE_ROUNDABOUT = 6
    const val KEEP_RIGHT = 7
    const val U_TURN_RIGHT = 8
    const val IGNORE = Int.MIN_VALUE
}

/** Istruzione nel formato GraphHopper: segnale, nome della via, punti percorsi (ultimo escluso), distanza e tempo. */
class Instruction(var sign: Int, var name: String) {
    val points = PointList()
    var distance = 0.0
    var millis = 0L
    /** Azimut iniziale (solo prima istruzione) e finale (solo FINISH), in gradi. */
    var heading = Double.NaN
    var lastHeading = Double.NaN

    // rotatorie
    var isRoundabout = false
    var exitNumber = 0
    var exited = false
    var radian = Double.NaN
    var clockwise = false

    fun increaseExitNumber() { exitNumber++ }
}

/**
 * Porting di GraphHopper `InstructionsFromEdges` (fork ORS): produce la lista di istruzioni di un
 * percorso osservando, a ogni nodo, gli archi alternativi (nomi, classi stradali, velocità).
 */
class InstructionsBuilder(private val graph: BaseGraph, private val encoding: GraphEncoding, private val profile: ProfileEncoding) {
    private val sh = encoding.shared
    private val outIt = graph.createEdgeIterator()
    private val tmpIt = graph.createEdgeIterator()

    fun build(path: RoutePath): List<Instruction> {
        val ways = ArrayList<Instruction>()
        if (path.edges.isEmpty()) return ways
        val s = State(ways)
        for (e in path.edges) s.next(e)
        s.finish(path.edges.last())
        return ways
    }

    private inner class State(val ways: MutableList<Instruction>) {
        var prevEdge: PathEdge? = null
        var prevName: String? = null
        var prevInstruction: Instruction? = null
        var prevInRoundabout = false
        var prevOrientation = 0.0
        var prevInstructionPrevOrientation = Double.NaN
        var prevInstructionName: String? = null
        var prevNode = -1
        var prevLat = 0.0; var prevLon = 0.0
        var doublePrevLat = 0.0; var doublePrevLon = 0.0

        fun next(edge: PathEdge) {
            val adjNode = edge.adjNode; val baseNode = edge.baseNode
            if (prevNode == -1) { prevLat = graph.lat(baseNode); prevLon = graph.lon(baseNode) }
            val adjLat = graph.lat(adjNode); val adjLon = graph.lon(adjNode)
            val wayGeo = edge.geometry
            val isRoundabout = isRoundabout(edge.edge)
            val latitude: Double; val longitude: Double
            if (wayGeo.size <= 2) { latitude = adjLat; longitude = adjLon } else { latitude = wayGeo.lat(1); longitude = wayGeo.lon(1) }
            val name = edge.name

            if (prevName == null && !isRoundabout) {
                // prima istruzione
                val ins = Instruction(Sign.CONTINUE_ON_STREET, name)
                ins.heading = round2(AngleCalc.calcAzimuth(graph.lat(baseNode), graph.lon(baseNode), latitude, longitude))
                prevInstruction = ins; ways.add(ins)
                prevName = name
            } else if (isRoundabout) {
                if (!prevInRoundabout) {
                    val ins = Instruction(Sign.USE_ROUNDABOUT, name).also { it.isRoundabout = true }
                    prevInstructionPrevOrientation = prevOrientation
                    if (prevName != null) {
                        // un'uscita allo stesso nodo in cui si entra nella rotatoria?
                        outIt.setBaseNode(baseNode)
                        while (outIt.next()) {
                            if (outIt.adjNode != prevNode && !outIt.get(sh.roundabout) && outIt.get(profile.access)) { ins.increaseExitNumber(); break }
                        }
                        prevOrientation = AngleCalc.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon)
                        var orientation = AngleCalc.calcOrientation(prevLat, prevLon, latitude, longitude)
                        orientation = AngleCalc.alignOrientation(prevOrientation, orientation)
                        ins.clockwise = (orientation - prevOrientation) < 0
                    } else {
                        prevOrientation = AngleCalc.calcOrientation(prevLat, prevLon, latitude, longitude)
                        prevName = name
                    }
                    prevInstruction = ins; ways.add(ins)
                }
                // ogni nodo della rotatoria con almeno un'uscita conta come uscita
                outIt.setBaseNode(adjNode)
                while (outIt.next()) {
                    if (!outIt.get(sh.roundabout) && outIt.get(profile.access)) { prevInstruction!!.increaseExitNumber(); break }
                }
            } else if (prevInRoundabout) {
                val ra = prevInstruction!!
                ra.name = name
                var orientation = AngleCalc.calcOrientation(prevLat, prevLon, latitude, longitude)
                orientation = AngleCalc.alignOrientation(prevOrientation, orientation)
                ra.radian = orientation - prevOrientation
                val recent = AngleCalc.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon)
                orientation = AngleCalc.alignOrientation(recent, orientation)
                ra.clockwise = (orientation - recent) < 0
                ra.exited = true
                prevInstructionName = prevName; prevName = name
                // fork ORS: dopo la rotatoria un'istruzione CONTINUE con il nome della strada d'uscita
                val cont = Instruction(Sign.CONTINUE_ON_STREET, name)
                prevInstruction = cont; ways.add(cont)
            } else {
                val sign = getTurn(edge, baseNode, prevNode, adjNode, name)
                if (sign != Sign.IGNORE) {
                    var isUTurn = false; var uTurnType = Sign.U_TURN_UNKNOWN
                    val pi = prevInstruction!!
                    if (!prevInstructionPrevOrientation.isNaN() && pi.distance < MAX_U_TURN_DISTANCE &&
                        (sign < 0) == (pi.sign < 0) && abs(sign) in 1..3 && abs(pi.sign) in 1..3 &&
                        isOneway(edge) && isNameSimilar(prevInstructionName, name)
                    ) {
                        val currentOrientation = AngleCalc.calcOrientation(prevLat, prevLon, latitude, longitude, exact = false)
                        val diff = abs(prevInstructionPrevOrientation - currentOrientation)
                        if (diff > PI * 0.9 && diff < PI * 1.1) { isUTurn = true; uTurnType = if (sign < 0) Sign.U_TURN_LEFT else Sign.U_TURN_RIGHT }
                    }
                    if (isUTurn) { pi.sign = uTurnType; pi.name = name } else {
                        val ins = Instruction(sign, name)
                        prevInstructionPrevOrientation = prevOrientation
                        prevInstructionName = prevName
                        prevInstruction = ins; ways.add(ins)
                    }
                }
                prevName = name
            }
            // punti (ultimo escluso), distanza e tempo
            val pi = prevInstruction!!
            for (i in 0 until wayGeo.size - 1) pi.points.add(wayGeo.lat(i), wayGeo.lon(i))
            pi.distance += edge.distance; pi.millis += edge.millis

            if (wayGeo.size <= 2) { doublePrevLat = prevLat; doublePrevLon = prevLon } else { doublePrevLat = wayGeo.lat(wayGeo.size - 2); doublePrevLon = wayGeo.lon(wayGeo.size - 2) }
            prevInRoundabout = isRoundabout; prevNode = baseNode; prevLat = adjLat; prevLon = adjLon; prevEdge = edge
        }

        fun finish(last: PathEdge) {
            if (prevInRoundabout) {
                val ra = prevInstruction!!
                var orientation = AngleCalc.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon)
                orientation = AngleCalc.alignOrientation(prevOrientation, orientation)
                ra.radian = orientation - prevOrientation
            }
            val fin = Instruction(Sign.FINISH, "")
            fin.points.add(graph.lat(last.adjNode), graph.lon(last.adjNode))
            fin.lastHeading = round2(AngleCalc.calcAzimuth(doublePrevLat, doublePrevLon, prevLat, prevLon))
            ways.add(fin)
        }

        private fun getTurn(edge: PathEdge, baseNode: Int, prevNode: Int, adjNode: Int, name: String): Int {
            val (lat, lon) = pointForOrientation(edge)
            prevOrientation = AngleCalc.calcOrientation(doublePrevLat, doublePrevLon, prevLat, prevLon)
            val sign = calculateSign(prevLat, prevLon, lat, lon, prevOrientation)
            val outgoing = OutgoingEdges(prevEdge, edge, prevNode, baseNode, adjNode)
            val nrOfPossibleTurns = outgoing.allowedTurns
            if (nrOfPossibleTurns <= 1) {
                return if (abs(sign) > 1 && outgoing.visibleTurns > 1) sign else Sign.IGNORE
            }
            if (abs(sign) > 1) {
                return if (isNameSimilar(name, prevName) && outgoing.outgoingEdgesAreSlowerByFactor(2.0)) Sign.IGNORE else sign
            }
            val prev = prevEdge ?: return sign
            val outgoingEdgesAreSlower = outgoing.outgoingEdgesAreSlowerByFactor(1.0)
            val otherContinue = outgoing.getOtherContinue(prevLat, prevLon, prevOrientation)
            val delta = orientationDelta(prevLat, prevLon, lat, lon, prevOrientation)
            if (otherContinue >= 0) {
                val otherName = graph.name(otherContinue)
                if (!isNameSimilar(name, prevName) || isNameSimilar(otherName, prevName) || !outgoingEdgesAreSlower) {
                    val roadClass = roadClassOf(edge.edge); val prevRoadClass = roadClassOf(prev.edge); val otherRoadClass = roadClassOf(otherContinue)
                    val link = linkOf(edge.edge); val prevLink = linkOf(prev.edge); val otherLink = linkOf(otherContinue)
                    if (roadClass in MAJOR && roadClass == prevRoadClass && link == prevLink && (otherRoadClass != prevRoadClass || otherLink != prevLink)) return Sign.IGNORE
                    val (oLat, oLon) = pointForOrientation(otherContinue, baseNode)
                    val otherDelta = orientationDelta(prevLat, prevLon, oLat, oLon, prevOrientation)
                    if (abs(delta) < 0.1 && abs(otherDelta) > 0.15 && isNameSimilar(name, prevName)) return Sign.CONTINUE_ON_STREET
                    return if (otherDelta < delta) Sign.KEEP_LEFT else Sign.KEEP_RIGHT
                }
            }
            if (!outgoingEdgesAreSlower) {
                if (abs(delta) > 0.6 || outgoing.isLeavingCurrentStreet(prevName, name)) return sign
            }
            return Sign.IGNORE
        }

        /** Archi alternativi al nodo base (GraphHopper `InstructionsOutgoingEdges`). */
        private inner class OutgoingEdges(val prevEdge: PathEdge?, val current: PathEdge, prevNode: Int, val baseNode: Int, adjNode: Int) {
            val visible = ArrayList<Int>(); val allowed = ArrayList<Int>()
            init {
                tmpIt.setBaseNode(baseNode)
                while (tmpIt.next()) {
                    if (tmpIt.adjNode == prevNode || tmpIt.adjNode == adjNode) continue
                    if (!tmpIt.get(profile.access) && !tmpIt.getReverse(profile.access)) continue
                    visible.add(tmpIt.edge)
                    if (tmpIt.get(profile.access)) allowed.add(tmpIt.edge)
                }
            }
            val allowedTurns get() = 1 + allowed.size
            val visibleTurns get() = 1 + visible.size

            fun outgoingEdgesAreSlowerByFactor(factor: Double): Boolean {
                val pathSpeed = speedOf(current.edge, current.baseNode)
                val prevSpeed = if (prevEdge != null) speedOf(prevEdge.edge, prevEdge.baseNode) else pathSpeed
                if (pathSpeed != prevSpeed || pathSpeed < 1) return false
                var maxSurrounding = -1.0
                for (e in visible) maxSurrounding = max(maxSurrounding, speedOf(e, baseNode))
                return maxSurrounding * factor < pathSpeed
            }

            fun getOtherContinue(prevLat: Double, prevLon: Double, prevOrientation: Double): Int {
                for (e in allowed) {
                    val (la, lo) = pointForOrientation(e, baseNode)
                    if (abs(calculateSign(prevLat, prevLon, la, lo, prevOrientation)) <= 1) return e
                }
                return -1
            }

            fun isLeavingCurrentStreet(prevName: String?, name: String): Boolean {
                if (isNameSimilar(name, prevName)) return false
                val prev = prevEdge ?: return false
                val classOrLinkChange = !sameRoadClassAndLink(prev.edge, current.edge)
                for (e in allowed) {
                    val edgeName = graph.name(e)
                    if (isNameSimilar(prevName, edgeName) || (classOrLinkChange && sameRoadClassAndLink(prev.edge, e))) return true
                    if (isNameSimilar(name, edgeName) || (classOrLinkChange && sameRoadClassAndLink(current.edge, e))) return true
                }
                return false
            }
        }
    }

    // --- accessori sugli archi ---------------------------------------------------------------

    private fun isRoundabout(edge: Int): Boolean { tmpIt.setEdge(edge, graph.nodeA(edge)); return tmpIt.get(sh.roundabout) }
    private fun roadClassOf(edge: Int): RoadClass { tmpIt.setEdge(edge, graph.nodeA(edge)); return tmpIt.get(sh.roadClass) }
    private fun linkOf(edge: Int): Boolean { tmpIt.setEdge(edge, graph.nodeA(edge)); return tmpIt.get(sh.roadClassLink) }
    private fun sameRoadClassAndLink(e1: Int, e2: Int) = roadClassOf(e1) == roadClassOf(e2) && linkOf(e1) == linkOf(e2)
    private fun speedOf(edge: Int, base: Int): Double { tmpIt.setEdge(edge, base); return tmpIt.get(profile.speed) }
    private fun isOneway(edge: PathEdge): Boolean { tmpIt.setEdge(edge.edge, edge.baseNode); return tmpIt.get(profile.access) != tmpIt.getReverse(profile.access) }

    /** Punto per il calcolo dell'orientamento: il primo pillar, altrimenti il nodo adiacente. */
    private fun pointForOrientation(edge: PathEdge): Pair<Double, Double> =
        if (edge.geometry.size <= 2) graph.lat(edge.adjNode) to graph.lon(edge.adjNode) else edge.geometry.lat(1) to edge.geometry.lon(1)

    private fun pointForOrientation(edge: Int, base: Int): Pair<Double, Double> {
        tmpIt.setEdge(edge, base)
        val geo = tmpIt.fetchWayGeometry(it.navmaster.engine.storage.FetchMode.ALL)
        return if (geo.size <= 2) graph.lat(tmpIt.adjNode) to graph.lon(tmpIt.adjNode) else geo.lat(1) to geo.lon(1)
    }

    companion object {
        const val MAX_U_TURN_DISTANCE = 35.0
        private val MAJOR = setOf(RoadClass.MOTORWAY, RoadClass.TRUNK, RoadClass.PRIMARY, RoadClass.SECONDARY, RoadClass.TERTIARY)

        fun isNameSimilar(a: String?, b: String?): Boolean {
            if (a.isNullOrEmpty() && b.isNullOrEmpty()) return false
            return a == b
        }

        fun orientationDelta(prevLat: Double, prevLon: Double, lat: Double, lon: Double, prevOrientation: Double): Double {
            var o = AngleCalc.calcOrientation(prevLat, prevLon, lat, lon, exact = false)
            o = AngleCalc.alignOrientation(prevOrientation, o)
            return o - prevOrientation
        }

        /** Soglie di GraphHopper: 0.2 rad ≈ 11°, 0.8 ≈ 46°, 1.8 ≈ 103°. */
        fun calculateSign(prevLat: Double, prevLon: Double, lat: Double, lon: Double, prevOrientation: Double): Int {
            val delta = orientationDelta(prevLat, prevLon, lat, lon, prevOrientation)
            val a = abs(delta)
            return when {
                a < 0.2 -> Sign.CONTINUE_ON_STREET
                a < 0.8 -> if (delta > 0) Sign.TURN_SLIGHT_LEFT else Sign.TURN_SLIGHT_RIGHT
                a < 1.8 -> if (delta > 0) Sign.TURN_LEFT else Sign.TURN_RIGHT
                else -> if (delta > 0) Sign.TURN_SHARP_LEFT else Sign.TURN_SHARP_RIGHT
            }
        }

        fun round2(v: Double) = Math.round(v * 100) / 100.0
    }
}
