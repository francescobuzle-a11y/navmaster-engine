package it.navmaster.engine.profile

import kotlin.math.pow

/**
 * Come il peso «recommended» usa la priorità delle strade ([PriorityCode]): il tempo di un arco
 * viene moltiplicato per un fattore del suo codice (ORS: 2^((codice − 4) / (4 − 7))).
 *
 * - [ORS]: quello di openrouteservice 10: ottimo ×0,50, autostrada ×0,63, statale ×0,79,
 *   neutro ×1, strada locale ×1,26 … pessimo ×2,52. Con le velocità di ORS, tutte simili, è la
 *   priorità a scegliere la strada: una statale costa il 26 % in più di un'autostrada con lo stesso
 *   tempo, e la I/52 Mikulov–Brno perdeva contro Znojmo pur facendo arrivare 9 minuti prima.
 * - [NAVMASTER]: per i grafi col modello di velocità NAVMASTER, dove i tempi sono già quelli veri
 *   (limiti legali del camion, tempo perso ai nodi): come TomTom, iGO e Sygic fra le strade buone
 *   decide il tempo. Fino alle statali (codice 5) i fattori sono quelli di ORS, così le strade
 *   terziarie, urbane, locali e sterrate restano penalizzate come prima rispetto alle statali; sopra,
 *   il premio è a scala 1/4: autostrada ×0,75 e percorso dei camion (`hgv=designated`) ×0,71 invece
 *   di ×0,63 e ×0,50. Statale contro autostrada: +6 % invece di +26 %.
 *
 * Il modello è scritto nel grafo (proprietà `priority_model` del file `profiles`, e la variante del
 * formato in `properties`, che i motori di prima rifiutano) quando lo si prepara, perché CH, core e
 * landmark sono calcolati con questo peso: un grafo si usa sempre col suo.
 */
enum class PriorityModel(private val bonusScale: Double) {
    ORS(1.0),
    NAVMASTER(0.25);

    val id: String get() = name.lowercase()

    /**
     * Il fattore del peso per la priorità salvata nell'arco ([PriorityCode.factor], 0..1). Con [ORS]
     * il calcolo è proprio quello di prima (stessi bit), per la parità con openrouteservice.
     */
    fun factor(priority: Double): Double {
        val normalized = priority * BEST - UNCHANGED
        // i codici sono interi: fino a PREFER (normalized 1) il fattore di ORS
        if (this == ORS || normalized < 1.5) return 2.0.pow(normalized / (UNCHANGED - BEST))
        return PREFER_FACTOR * 2.0.pow((normalized - 1.0) / (UNCHANGED - BEST) * bonusScale)
    }

    companion object {
        private val BEST = PriorityCode.BEST.value.toDouble()
        private val UNCHANGED = PriorityCode.UNCHANGED.value.toDouble()
        private val PREFER_FACTOR = 2.0.pow(1.0 / (UNCHANGED - BEST))

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
