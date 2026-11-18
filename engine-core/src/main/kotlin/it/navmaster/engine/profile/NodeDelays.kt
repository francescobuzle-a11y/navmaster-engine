package it.navmaster.engine.profile

import kotlin.math.min

/**
 * Le perdite di tempo nei nodi del modello [SpeedModel.NAVMASTER]: dove il mezzo si ferma o rallenta
 * davvero (semafori, stop, dare precedenza, rotatorie, caselli, passaggi a livello), invece del modello
 * di accelerazione per way OSM di ORS. I secondi si salvano per arco e per senso (`<profilo>_delay`).
 *
 * Codice di un nodo ([classify]): il tipo ([Control.ordinal]) nei bit 0–3, la direzione nei bit 4–5
 * ([DIR_BOTH], [DIR_FORWARD], [DIR_BACKWARD], rispetto all'ordine dei nodi della way), [ALL_WAY] nel bit 6.
 */
object NodeDelays {
    enum class Control { NONE, SIGNALS, STOP, GIVE_WAY, TOLL, RAIL, CROSSING_SIGNALS, MINI_ROUNDABOUT, IMPLIED_GIVE_WAY, ROUNDABOUT_ENTRY }

    const val DIR_BOTH = 0
    const val DIR_FORWARD = 1
    const val DIR_BACKWARD = 2
    /** `stop=all`: lo stop vale per tutti gli accessi. */
    const val ALL_WAY = 1 shl 6

    /** Il ritardo massimo salvabile per arco e per senso (6 bit, passo 1 s). */
    const val MAX_SECONDS = 63

    /** Sopra questa velocità d'arrivo il calcolo non cresce più (un incrocio non si affronta a 90). */
    const val SPEED_CAP_KMH = 70.0

    /**
     * Un semaforo senza `traffic_signals:direction` a meno di tanti metri da un incrocio è quello di
     * un accesso (in OSM spesso mappato su ogni accesso, prima della linea d'arresto): ferma solo chi
     * va verso l'incrocio, non chi ne esce. Più lontano (semaforo pedonale a metà strada) vale nei due sensi.
     */
    const val SIGNAL_APPROACH_M = 30.0

    /** Velocità a cui si affronta una precedenza o una rotatoria senza fermarsi. */
    const val YIELD_SPEED_KMH = 20.0

    private val CONTROLS = Control.entries.toTypedArray()

    fun control(code: Int): Control = CONTROLS[code and 15]
    fun direction(code: Int): Int = (code shr 4) and 3
    fun allWay(code: Int): Boolean = code and ALL_WAY != 0

    /** Il codice dei tag di un nodo, 0 se il nodo non rallenta. */
    fun classify(tags: Map<String, String>?): Int {
        if (tags == null) return 0
        val highway = tags["highway"]
        val c = when {
            highway == "traffic_signals" -> Control.SIGNALS
            highway == "stop" -> Control.STOP
            highway == "give_way" -> Control.GIVE_WAY
            highway == "mini_roundabout" -> Control.MINI_ROUNDABOUT
            tags["barrier"] == "toll_booth" -> Control.TOLL
            tags["railway"] == "level_crossing" -> Control.RAIL
            highway == "crossing" && (tags["crossing"] == "traffic_signals" || tags["crossing:signals"] == "yes") -> Control.CROSSING_SIGNALS
            else -> return 0
        }
        val dirTag = when (c) {
            Control.SIGNALS -> tags["traffic_signals:direction"] ?: tags["direction"]
            Control.STOP, Control.GIVE_WAY -> tags["direction"]
            else -> null
        }
        val dir = when (dirTag?.trim()?.lowercase()) {
            "forward" -> DIR_FORWARD
            "backward" -> DIR_BACKWARD
            else -> DIR_BOTH
        }
        var code = c.ordinal or (dir shl 4)
        if (c == Control.STOP && tags["stop"] == "all") code = code or ALL_WAY
        return code
    }

    /**
     * I secondi persi a un controllo di tipo [c], arrivando a [vKmh] (limitata a [SPEED_CAP_KMH]) con
     * accelerazione [a] e decelerazione [d] in m/s².
     */
    fun seconds(c: Control, vKmh: Double, a: Double, d: Double): Double {
        if (c == Control.NONE || vKmh.isNaN() || vKmh <= 0) return 0.0
        val v = min(vKmh, SPEED_CAP_KMH)
        val k = (1 / a + 1 / d) / 2
        val stop = stopLoss(v, k)
        return when (c) {
            Control.NONE -> 0.0
            Control.SIGNALS -> 0.5 * stop + 8
            Control.STOP -> stop + 2
            Control.GIVE_WAY, Control.IMPLIED_GIVE_WAY, Control.ROUNDABOUT_ENTRY, Control.MINI_ROUNDABOUT ->
                0.3 * stop + 0.7 * slowLoss(v, YIELD_SPEED_KMH, k) + 1
            Control.TOLL -> 0.5 * stop + 10
            Control.RAIL -> slowLoss(v, YIELD_SPEED_KMH, k) + 5
            Control.CROSSING_SIGNALS -> 0.2 * stop + 3
        }
    }

    /** Tempo perso fermandosi e ripartendo da [vKmh]: v·(1/a + 1/d)/2. */
    private fun stopLoss(vKmh: Double, k: Double): Double = vKmh / 3.6 * k

    /** Tempo perso rallentando da [vKmh] a [vsKmh] e riaccelerando. */
    private fun slowLoss(vKmh: Double, vsKmh: Double, k: Double): Double {
        if (vKmh <= vsKmh) return 0.0
        val v = vKmh / 3.6; val vs = vsKmh / 3.6
        return (v - vs) * (v - vs) / v * k
    }
}
