package it.navmaster.engine.instructions

import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.floor

/** Tipi di istruzione di ORS (`InstructionType`): il valore è quello del campo `type` nel JSON. */
enum class InstructionType(val code: Int) {
    TURN_LEFT(0), TURN_RIGHT(1), TURN_SHARP_LEFT(2), TURN_SHARP_RIGHT(3), TURN_SLIGHT_LEFT(4), TURN_SLIGHT_RIGHT(5),
    CONTINUE(6), ENTER_ROUNDABOUT(7), EXIT_ROUNDABOUT(8), UTURN(9), FINISH(10), DEPART(11), KEEP_LEFT(12), KEEP_RIGHT(13),
    PT_ENTER(14), PT_TRANSFER(15), PT_EXIT(16), UNKNOWN(17);

    val isSlight get() = this == TURN_SLIGHT_LEFT || this == TURN_SLIGHT_RIGHT
}

enum class ArrivalDirection { UNKNOWN, LEFT, RIGHT, STRAIGHT_AHEAD }

enum class CardinalDirection { N, NE, E, SE, S, SW, W, NW }

data class Maneuver(val lat: Double, val lon: Double, val bearingBefore: Int, val bearingAfter: Int) {
    /** ORS `isContinue`: differenza di bearing sotto 6°, senza gestire il passaggio per 360. */
    val isContinue get() = abs(bearingAfter - bearingBefore) < 6
}

class RouteStep(
    val type: InstructionType,
    val instruction: String,
    val name: String,
    val distance: Double,
    val duration: Double,
    val wayPointStart: Int,
    val wayPointEnd: Int,
    val exitNumber: Int,
    val maneuver: Maneuver?,
)

enum class DistanceUnit(val ors: String, val perMeter: Double, val decimals: Int) {
    METERS("m", 1.0, 1), KILOMETERS("km", 0.001, 3), MILES("mi", 1 / 1609.344, 3)
}

/** Opzioni di conversione (dalla richiesta ORS). */
data class StepOptions(
    val language: String = "en",
    val units: DistanceUnit = DistanceUnit.METERS,
    val maneuvers: Boolean = false,
    val htmlFormat: Boolean = false,
    /** Se vero, le inversioni restano `UTURN (9)` invece di diventare `CONTINUE` come in ORS. */
    val emitUTurns: Boolean = false,
)

/**
 * Conversione delle istruzioni GraphHopper nei passi di ORS (`RouteResultBuilder.convertRouteSteps`):
 * tipo, testo localizzato, manovra, indici `way_points`, arrotondamenti.
 */
class OrsStepBuilder(private val options: StepOptions) {
    private val t = InstructionTranslator.forLanguage(options.language)

    /**
     * @param instructions istruzioni del segmento
     * @param wayPointOffset indice del primo punto del segmento nella geometria complessiva del percorso
     * @param destination coordinata di destinazione richiesta (per la direzione d'arrivo)
     * @param nextSegmentFirstPoints punti della prima istruzione del segmento successivo, se esiste
     */
    fun build(instructions: List<Instruction>, wayPointOffset: Int, destination: Pair<Double, Double>, nextSegmentFirstPoints: PointList? = null): List<RouteStep> {
        val steps = ArrayList<RouteStep>(instructions.size)
        var start = wayPointOffset
        for (ii in instructions.indices) {
            val ins = instructions[ii]
            var type = typeOf(ins, ii)
            val prev = instructions.getOrNull(ii - 1)
            val next = if (ii + 1 < instructions.size) instructions[ii + 1].points else nextSegmentFirstPoints
            var maneuver: Maneuver? = null
            if (options.maneuvers || type.isSlight) {
                maneuver = calcManeuver(prev?.points, ins.points, next, type)
                if (type.isSlight && maneuver != null && maneuver.isContinue) type = InstructionType.CONTINUE
            }
            val name = ins.name
            val roadName = if (options.htmlFormat && name.isNotEmpty()) "<b>$name</b>" else name
            val text = when (type) {
                InstructionType.DEPART -> {
                    val target = when {
                        ins.points.size > 1 -> ins.points.lat(1) to ins.points.lon(1)
                        next != null && next.size > 0 -> next.lat(0) to next.lon(0)
                        else -> ins.points.lat(0) to ins.points.lon(0)
                    }
                    t.depart(calcDirection(ins.points.lat(0), ins.points.lon(0), target.first, target.second), roadName)
                }
                InstructionType.ENTER_ROUNDABOUT -> t.roundabout(ins.exitNumber, roadName)
                InstructionType.FINISH -> t.arrive(arrivalDirection(prevPointsOf(instructions, ii), destination), prev?.name ?: "")
                InstructionType.CONTINUE -> t.continueOn(type, roadName)
                InstructionType.KEEP_LEFT, InstructionType.KEEP_RIGHT -> t.keep(type, roadName)
                InstructionType.UTURN -> t.uturn(roadName)
                else -> t.turn(type, roadName)
            }
            var end = start + ins.points.size
            if (type == InstructionType.FINISH || (type == InstructionType.DEPART && ins.distance == 0.0 && ins.points.size == 1)) end = start
            steps += RouteStep(
                type, text, name,
                distance = roundForUnits(ins.distance * options.units.perMeter),
                duration = round1(ins.millis / 1000.0),
                wayPointStart = start, wayPointEnd = end,
                exitNumber = if (ins.isRoundabout) ins.exitNumber else -1,
                maneuver = if (options.maneuvers) maneuver else null,
            )
            start = end
        }
        return steps
    }

    /** Tutti i punti fino all'istruzione [ii] esclusa, più il primo punto di FINISH (l'ultimo nodo). */
    private fun prevPointsOf(instructions: List<Instruction>, ii: Int): PointList {
        val pl = PointList()
        for (k in 0..ii) for (i in 0 until instructions[k].points.size) pl.add(instructions[k].points.lat(i), instructions[k].points.lon(i))
        return pl
    }

    private fun typeOf(ins: Instruction, ii: Int): InstructionType {
        if (ii == 0) return InstructionType.DEPART
        if (ins.isRoundabout) return InstructionType.ENTER_ROUNDABOUT
        return when (ins.sign) {
            Sign.TURN_LEFT -> InstructionType.TURN_LEFT
            Sign.TURN_RIGHT -> InstructionType.TURN_RIGHT
            Sign.TURN_SHARP_LEFT -> InstructionType.TURN_SHARP_LEFT
            Sign.TURN_SHARP_RIGHT -> InstructionType.TURN_SHARP_RIGHT
            Sign.TURN_SLIGHT_LEFT -> InstructionType.TURN_SLIGHT_LEFT
            Sign.TURN_SLIGHT_RIGHT -> InstructionType.TURN_SLIGHT_RIGHT
            Sign.USE_ROUNDABOUT -> InstructionType.ENTER_ROUNDABOUT
            Sign.LEAVE_ROUNDABOUT -> InstructionType.EXIT_ROUNDABOUT
            Sign.FINISH -> InstructionType.FINISH
            Sign.KEEP_LEFT -> InstructionType.KEEP_LEFT
            Sign.KEEP_RIGHT -> InstructionType.KEEP_RIGHT
            Sign.U_TURN_LEFT, Sign.U_TURN_RIGHT, Sign.U_TURN_UNKNOWN -> if (options.emitUTurns) InstructionType.UTURN else InstructionType.CONTINUE
            else -> InstructionType.CONTINUE
        }
    }

    /** ORS `calcManeuver`: bearing prima/dopo in gradi interi, posizione = primo punto del passo. */
    private fun calcManeuver(prev: PointList?, cur: PointList, next: PointList?, type: InstructionType): Maneuver? {
        if (cur.size == 0) return null
        val lat0 = cur.lat(0); val lon0 = cur.lon(0)
        var before = 0; var after = 0
        if (type == InstructionType.DEPART) {
            after = bearingAfter(cur, next)
        } else {
            if (prev != null && prev.size > 0) before = Math.round(AngleCalc.calcAzimuth(prev.lat(prev.size - 1), prev.lon(prev.size - 1), lat0, lon0)).toInt()
            if (type != InstructionType.FINISH) after = bearingAfter(cur, next)
        }
        return Maneuver(lat0, lon0, before, after)
    }

    private fun bearingAfter(cur: PointList, next: PointList?): Int {
        val (la, lo) = when {
            cur.size > 1 -> cur.lat(1) to cur.lon(1)
            next != null && next.size > 0 -> next.lat(0) to next.lon(0)
            else -> return 0
        }
        return Math.round(AngleCalc.calcAzimuth(cur.lat(0), cur.lon(0), la, lo)).toInt()
    }

    private fun roundForUnits(v: Double): Double {
        val f = Math.pow(10.0, options.units.decimals.toDouble())
        return Math.round(v * f) / f
    }

    companion object {
        fun round1(v: Double) = Math.round(v * 10) / 10.0

        /** ORS `calcDirection`: punto cardinale della direzione p0→p1. */
        fun calcDirection(lat0: Double, lon0: Double, lat1: Double, lon1: Double): CardinalDirection {
            var orientation = AngleCalc.round4(-AngleCalc.calcOrientation(lat0, lon0, lat1, lon1) + PI / 2)
            if (orientation < 0) orientation += 2 * PI
            val deg = Math.toDegrees(orientation)
            val idx = floor(((deg + 22.5) % 360) / 45).toInt()
            return CardinalDirection.entries[idx.coerceIn(0, 7)]
        }

        /** ORS `getArrivalDirection`: prodotto vettoriale fra l'ultimo tratto e la destinazione richiesta. */
        fun arrivalDirection(points: PointList, destination: Pair<Double, Double>): ArrivalDirection {
            if (points.size < 2) return ArrivalDirection.UNKNOWN
            val lat1 = points.lat(points.size - 1); val lon1 = points.lon(points.size - 1)
            val lat0 = points.lat(points.size - 2); val lon0 = points.lon(points.size - 2)
            if (DistanceCalc.haversine(lat1, lon1, destination.first, destination.second) < 1) return ArrivalDirection.STRAIGHT_AHEAD
            val s = Math.signum((lon1 - lon0) * (destination.first - lat0) - (lat1 - lat0) * (destination.second - lon0))
            return when { s == 0.0 -> ArrivalDirection.STRAIGHT_AHEAD; s > 0 -> ArrivalDirection.LEFT; else -> ArrivalDirection.RIGHT }
        }
    }
}

/** Testi delle istruzioni (ORS `InstructionTranslator` + `ors_*.resources`). */
class InstructionTranslator private constructor(
    private val numerals: List<String>, private val directions: List<String>, private val maneuvers: Map<String, String>,
    private val departDefault: String, private val departName: String,
    private val turnDefault: String, private val turnName: String,
    private val continueDefault: String, private val continueName: String,
    private val keepDefault: String, private val keepName: String,
    private val roundaboutDefault: String, private val roundaboutName: String,
    private val uturnDefault: String, private val uturnName: String,
    private val arrive: List<Pair<String, String>>, // per ArrivalDirection: (default, name)
) {
    fun depart(dir: CardinalDirection, roadName: String): String =
        (if (roadName.isEmpty()) departDefault else departName).replace("{direction}", directions[dir.ordinal]).replace("{way_name}", roadName)

    fun turn(type: InstructionType, roadName: String): String =
        (if (roadName.isEmpty()) turnDefault else turnName).replace("{turn_maneuver}", maneuverOf(type)).replace("{way_name}", roadName)

    fun continueOn(type: InstructionType, roadName: String): String =
        (if (roadName.isEmpty()) continueDefault else continueName).replace("{turn_maneuver}", maneuverOf(type)).replace("{way_name}", roadName)

    fun keep(type: InstructionType, roadName: String): String =
        (if (roadName.isEmpty()) keepDefault else keepName).replace("{turn_maneuver}", maneuverOf(type)).replace("{way_name}", roadName)

    fun uturn(roadName: String): String = (if (roadName.isEmpty()) uturnDefault else uturnName).replace("{way_name}", roadName)

    fun roundabout(exit: Int, roadName: String): String {
        val n = when { exit == 0 -> "UNKNOWN"; exit > 10 -> exit.toString(); else -> numerals[exit - 1] }
        return (if (roadName.isEmpty()) roundaboutDefault else roundaboutName).replace("{exit_number}", n).replace("{way_name}", roadName)
    }

    fun arrive(dir: ArrivalDirection, roadName: String): String {
        val (d, n) = arrive[dir.ordinal]
        return (if (roadName.isEmpty()) d else n).replace("{way_name}", roadName)
    }

    private fun maneuverOf(type: InstructionType): String = when (type) {
        InstructionType.TURN_LEFT, InstructionType.KEEP_LEFT -> maneuvers.getValue("left")
        InstructionType.TURN_RIGHT, InstructionType.KEEP_RIGHT -> maneuvers.getValue("right")
        InstructionType.TURN_SHARP_LEFT -> maneuvers.getValue("sharp_left")
        InstructionType.TURN_SHARP_RIGHT -> maneuvers.getValue("sharp_right")
        InstructionType.TURN_SLIGHT_LEFT -> maneuvers.getValue("slight_left")
        InstructionType.TURN_SLIGHT_RIGHT -> maneuvers.getValue("slight_right")
        InstructionType.CONTINUE -> maneuvers.getValue("straight")
        InstructionType.UTURN -> maneuvers.getValue("uturn")
        else -> maneuvers.getValue("left")
    }

    companion object {
        private val EN = InstructionTranslator(
            numerals = listOf("1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th", "9th", "10th"),
            directions = listOf("north", "northeast", "east", "southeast", "south", "southwest", "west", "northwest"),
            maneuvers = mapOf("left" to "left", "right" to "right", "sharp_left" to "sharp left", "sharp_right" to "sharp right",
                "slight_left" to "slight left", "slight_right" to "slight right", "straight" to "straight", "uturn" to "U-turn"),
            departDefault = "Head {direction}", departName = "Head {direction} on {way_name}",
            turnDefault = "Turn {turn_maneuver}", turnName = "Turn {turn_maneuver} onto {way_name}",
            continueDefault = "Continue {turn_maneuver}", continueName = "Continue {turn_maneuver} onto {way_name}",
            keepDefault = "Keep {turn_maneuver}", keepName = "Keep {turn_maneuver} onto {way_name}",
            roundaboutDefault = "Enter the roundabout and take the {exit_number} exit", roundaboutName = "Enter the roundabout and take the {exit_number} exit onto {way_name}",
            uturnDefault = "Make a U-turn", uturnName = "Make a U-turn onto {way_name}",
            arrive = listOf(
                "Arrive at your destination" to "Arrive at {way_name}",
                "Arrive at your destination, on the left" to "Arrive at {way_name}, on the left",
                "Arrive at your destination, on the right" to "Arrive at {way_name}, on the right",
                "Arrive at your destination, straight ahead" to "Arrive at {way_name}, straight ahead",
            ),
        )
        private val IT = InstructionTranslator(
            numerals = listOf("prima", "seconda", "terza", "quarta", "quinta", "sesta", "settima", "ottava", "nona", "decima"),
            directions = listOf("Nord", "Nord-Est", "Est", "Sud-Est", "Sud", "Sud-Ovest", "Ovest", "Nord-Ovest"),
            maneuvers = mapOf("left" to "a sinistra", "right" to "a destra", "sharp_left" to "svolta sinistra", "sharp_right" to "svolta destra",
                "slight_left" to "sulla sinistra", "slight_right" to "sulla destra", "straight" to "dritto", "uturn" to "inversione di marcia"),
            departDefault = "Avanti {direction}", departName = "Avanti {direction} su {way_name}",
            turnDefault = "Gira {turn_maneuver}", turnName = "Gira {turn_maneuver} su {way_name}",
            continueDefault = "Continua {turn_maneuver}", continueName = "Continua {turn_maneuver} su {way_name}",
            keepDefault = "Gira {turn_maneuver}", keepName = "Gira {turn_maneuver} su {way_name}",
            roundaboutDefault = "Entra nella rotatoria e prendi la {exit_number} uscita", roundaboutName = "Entra nella rotatoria e prendi la {exit_number} uscita su {way_name}",
            uturnDefault = "Fai un'inversione a U", uturnName = "Fai un'inversione a U su {way_name}",
            arrive = listOf(
                "Arrivo a destinazione" to "Arrivo a {way_name}",
                "Arrivo a destinazione, a sinistra" to "Arrivo a {way_name}, a sinistra",
                "Arrivo a destinazione, sulla destra" to "Arrivo a {way_name}, sulla destra",
                "Arrivo a destinazione, dritto" to "Arrivo a {way_name}, dritto",
            ),
        )

        fun forLanguage(lang: String): InstructionTranslator = when (lang.lowercase().substringBefore('-').substringBefore('_')) {
            "it" -> IT
            else -> EN
        }

        val supportedLanguages = listOf("en", "en-US", "it", "it-IT")
    }
}
