package it.navmaster.engine.profile

import kotlin.math.pow

/**
 * Come il peso «recommended» usa la priorità delle strade ([PriorityCode]): il tempo di un arco
 * viene moltiplicato per 2^((codice − 4) / (4 − 7) × scala).
 *
 * - [ORS]: quello di openrouteservice 10 (scala 1): ottimo ×0,50, autostrada ×0,63, statale ×0,79,
 *   neutro ×1, strada locale ×1,26 … pessimo ×2,52. Con le velocità di ORS, tutte simili, è la
 *   priorità a scegliere la strada: una statale costa il 26 % in più di un'autostrada con lo stesso
 *   tempo, e la I/52 Mikulov–Brno perdeva contro Znojmo pur facendo arrivare 9 minuti prima.
 * - [NAVMASTER]: per i grafi col modello di velocità NAVMASTER, dove i tempi sono già quelli veri
 *   (limiti legali del camion, tempo perso ai nodi): come TomTom, iGO e Sygic sceglie col tempo, con un
 *   premio leggero alle strade migliori (scala 1/4: ottimo ×0,84, autostrada ×0,89, statale ×0,94,
 *   neutro ×1; statale contro autostrada +6 % invece di +26 %). Le penalità delle strade da evitare col
 *   camion (locali, residenziali, solo per arrivare, sterrate) restano quelle di ORS.
 *
 * Il modello è scritto nel grafo (proprietà `priority_model` del file `profiles`) quando lo si prepara,
 * perché CH, core e landmark sono calcolati con questo peso: un grafo si usa sempre col suo.
 */
enum class PriorityModel(private val bonusScale: Double, private val penaltyScale: Double) {
    ORS(1.0, 1.0),
    NAVMASTER(0.25, 1.0);

    val id: String get() = name.lowercase()

    /**
     * Il fattore del peso per la priorità salvata nell'arco ([PriorityCode.factor], 0..1). Con [ORS]
     * il calcolo è proprio quello di prima (stessi bit), per la parità con openrouteservice.
     */
    fun factor(priority: Double): Double {
        val normalized = priority * BEST - UNCHANGED
        val exponent = normalized / (UNCHANGED - BEST)
        return 2.0.pow(if (normalized > 0) exponent * bonusScale else exponent * penaltyScale)
    }

    companion object {
        private val BEST = PriorityCode.BEST.value.toDouble()
        private val UNCHANGED = PriorityCode.UNCHANGED.value.toDouble()

        /** Il modello di un grafo letto dal disco: [ORS] se manca (grafi preparati prima, anche quelli NAVMASTER). */
        fun parse(s: String?): PriorityModel = when (s?.trim()?.lowercase()) {
            null, "", "ors" -> ORS
            "navmaster" -> NAVMASTER
            else -> error("modello di priorità sconosciuto: $s")
        }

        /** Il modello per un grafo nuovo con questi profili: [NAVMASTER] se le velocità sono NAVMASTER. */
        fun forProfiles(profiles: List<VehicleProfile>): PriorityModel =
            if (profiles.any { it.options.speedModel == SpeedModel.NAVMASTER }) NAVMASTER else ORS
    }
}
