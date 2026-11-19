package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay

/**
 * Il modello delle velocità di marcia scritto negli archi all'import.
 *
 * - [ORS]: quello di openrouteservice 10.1.0 (accelerazione per way OSM, `maxspeed` del camion
 *   scartato sopra il default del tipo di strada): i grafi del confronto con ORS.
 * - [NAVMASTER]: quello dei navigatori (TomTom, Sygic, iGO, HERE, GraphHopper): velocità =
 *   min(cartello, limite legale del mezzo per Paese e tipo di strada, massimo del mezzo) × fattore di
 *   marcia, senza accelerazione per way; il tempo perso si paga nei nodi dove il mezzo si ferma o
 *   rallenta davvero (semafori, stop, precedenze, rotatorie, caselli, passaggi a livello, vedi [NodeDelays]).
 */
enum class SpeedModel {
    ORS, NAVMASTER;

    val id: String get() = name.lowercase()

    companion object {
        fun parse(s: String): SpeedModel = when (s.trim().lowercase()) {
            "ors" -> ORS
            "navmaster" -> NAVMASTER
            else -> error("modello di velocità sconosciuto: $s")
        }
    }
}

/** Il contesto stradale per i limiti legali (gli stessi nomi di `SpeedRules.Road` dell'app, più [URBAN]). */
enum class RoadContext {
    URBAN, MINOR, RURAL, EXPRESS, MOTORWAY;

    companion object {
        /**
         * Il contesto finale di una way: quello del tag di zona se c'è, altrimenti dal tipo di strada;
         * poi un cartello ≤ 50 lo rende urbano (la regola di `SpeedRules.roadOf` dell'app).
         */
        fun of(highway: String, zoneContext: RoadContext?, signedKmh: Double): RoadContext {
            var ctx = zoneContext ?: fromHighway(highway)
            if (!signedKmh.isNaN() && signedKmh <= 50) ctx = URBAN
            return ctx
        }

        fun fromHighway(highway: String): RoadContext = when (highway) {
            "motorway", "motorway_link" -> MOTORWAY
            "trunk", "trunk_link", "motorroad" -> EXPRESS
            "primary", "primary_link" -> RURAL
            "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified" -> MINOR
            else -> URBAN
        }
    }
}

/**
 * Limiti legali del camion (autoarticolato o autotreno oltre 12 t) per Paese e contesto, in km/h.
 * Stessi numeri di `SpeedRules.legal` dell'app (oltre 12 t, autoarticolato) per IT, AT, DE, CH, CZ, NL, FR, ES, PL
 * (AT: 70 anche sulle trunk, come l'app; il limite notturno di 60 non è applicato).
 *
 * Righe controllate il 2026-11-19 sui testi di legge e sulle pagine «Guidare all'estero» dell'UE
 * (europa.eu/youreurope). Sopra i 90 km/h vale il limitatore dei mezzi pesanti (direttiva 92/6/CEE):
 * le righe si fermano a 90. EXPRESS sono le trunk e le motorroad: dove le trunk sono quasi sempre a
 * una carreggiata (ES, PL, GB) vale il limite delle strade normali, non quello delle superstrade.
 * - SK: 90 fuori dai centri sopra 3,5 t (zákon 8/2009 §16); HR: 80 fuori, 90 in autostrada (ZSPC art. 55).
 * - ES: 90 solo su autopista/autovía (motorway in OSM), 80 sulle nazionali (RGC art. 48).
 * - PL: 80 su autostrade, superstrade e doppie carreggiate (motorway in OSM), 70 altrove (art. 20 ust. 3).
 * - GB: valori dell'Inghilterra e del Galles, 50 mph su carreggiata singola e 56 mph in autostrada
 *   (la Scozia è più bassa: 40 mph); le doppie carreggiate (60 mph) restano a 80.
 * - BG: DV 64/2025 (dal 07.09.2025). DE: 80 sulle Kraftfahrstraßen solo a carreggiate separate (qui sempre 80).
 * - LT: le greitkeliai sono motorway in OSM, ma per i camion valgono 80 (qui 90).
 * - EE (solo pagina UE), AL (art. 140 non letto), MK, CY, BY: fiducia media o bassa.
 * - AD, FO, GE, MD, XK, IM, GG: nessuna fonte trovata, restano sulla riga di riserva.
 */
object HgvLegalSpeeds {
    /** Ordine: URBAN, MINOR, RURAL, EXPRESS, MOTORWAY. */
    private val table: Map<String, IntArray> = mapOf(
        "IT" to intArrayOf(50, 70, 70, 70, 80),
        // come SpeedRules.legal dell'app: 80 solo in autostrada, 70 su tutte le altre strade fuori dai centri
        "AT" to intArrayOf(50, 70, 70, 70, 80),
        "DE" to intArrayOf(50, 60, 60, 80, 80),
        "CH" to intArrayOf(50, 80, 80, 80, 80),
        "CZ" to intArrayOf(50, 80, 80, 80, 80),
        "NL" to intArrayOf(50, 80, 80, 80, 80),
        "DK" to intArrayOf(50, 80, 80, 80, 80),
        "FR" to intArrayOf(50, 60, 80, 80, 90),
        "ES" to intArrayOf(50, 80, 80, 80, 90),
        "PL" to intArrayOf(50, 70, 70, 70, 80),
        "SK" to intArrayOf(50, 90, 90, 90, 90),
        "HU" to intArrayOf(50, 70, 70, 70, 80),
        "SI" to intArrayOf(50, 70, 70, 80, 80),
        "HR" to intArrayOf(50, 80, 80, 80, 90),
        "BE" to intArrayOf(50, 60, 60, 90, 90),
        "LU" to intArrayOf(50, 75, 75, 75, 90),
        "PT" to intArrayOf(50, 80, 80, 80, 90),
        "RO" to intArrayOf(50, 70, 70, 80, 90),
        "BG" to intArrayOf(50, 80, 80, 90, 90),
        "AL" to intArrayOf(40, 60, 60, 60, 80),
        "BY" to intArrayOf(60, 70, 70, 70, 90),
        "BA" to intArrayOf(50, 70, 70, 70, 70),
        "CY" to intArrayOf(50, 64, 64, 80, 80),
        "EE" to intArrayOf(50, 90, 90, 90, 90),
        "FI" to intArrayOf(50, 80, 80, 80, 80),
        "GB" to intArrayOf(48, 80, 80, 80, 90),
        "GR" to intArrayOf(50, 70, 70, 70, 80),
        "IS" to intArrayOf(50, 80, 80, 80, 80),
        "IE" to intArrayOf(50, 80, 80, 80, 90),
        "LV" to intArrayOf(50, 80, 80, 80, 80),
        "LI" to intArrayOf(50, 80, 80, 80, 80),
        "LT" to intArrayOf(50, 80, 80, 80, 90),
        "MK" to intArrayOf(50, 70, 70, 70, 80),
        "MT" to intArrayOf(40, 60, 60, 60, 60),
        "MC" to intArrayOf(50, 50, 50, 50, 50),
        "ME" to intArrayOf(50, 70, 70, 70, 70),
        "NO" to intArrayOf(50, 80, 80, 80, 80),
        "RS" to intArrayOf(50, 70, 70, 70, 90),
        "SE" to intArrayOf(50, 80, 80, 80, 80),
        "TR" to intArrayOf(50, 80, 80, 85, 90),
        "UA" to intArrayOf(50, 90, 90, 90, 90),
    )

    /** Riga di riserva per un Paese sconosciuto o assente. */
    private val fallback = intArrayOf(50, 80, 80, 80, 80)

    fun limit(country: String?, ctx: RoadContext): Int = (country?.let { table[it.uppercase()] } ?: fallback)[ctx.ordinal]

    fun knows(country: String?): Boolean = country != null && table.containsKey(country.uppercase())

    /** Fattore di marcia sul limite legale: pieno su autostrade e superstrade, 0,9 altrove. */
    fun factor(ctx: RoadContext): Double = when (ctx) {
        RoadContext.MOTORWAY -> MOTORWAY_FACTOR
        RoadContext.EXPRESS -> 1.0
        else -> 0.9
    }

    /** La manopola della calibrazione con TomTom: 1,05 dà 84 → 85 km/h in autostrada. */
    const val MOTORWAY_FACTOR = 1.0
}

/**
 * Paese e contesto di una way dai tag di zona (`maxspeed:type`, `source:maxspeed`, `zone:maxspeed`,
 * `zone:traffic`, o lo stesso `maxspeed` nella forma `XX:valore`).
 *
 * @param implicitKmh il limite generale del Paese per quel contesto (`cz:rural` → 90), -1 se ignoto
 */
data class ZoneInfo(val country: String?, val context: RoadContext?, val implicitKmh: Int) {
    companion object {
        private val KEYS = arrayOf("maxspeed:type", "source:maxspeed", "zone:maxspeed", "zone:traffic", "maxspeed")

        /** Vero se [v] ha la forma `XX:valore` (due lettere, due punti). */
        fun isZoneValue(v: String?): Boolean = v != null && v.length > 3 && v[2] == ':' && v[0].isLetter() && v[1].isLetter()

        fun of(way: OsmWay, highway: String): ZoneInfo? {
            for (k in KEYS) {
                val v = way.tag(k)?.trim() ?: continue
                if (!isZoneValue(v)) continue
                val country = v.substring(0, 2).uppercase()
                val value = v.substring(3).lowercase()
                val ctx = when {
                    value == "urban" || value.startsWith("zone") || value == "living_street" -> RoadContext.URBAN
                    value == "rural" || value == "nsl_single" -> when (highway) {
                        "primary", "primary_link", "trunk", "trunk_link", "motorroad" -> RoadContext.RURAL
                        else -> RoadContext.MINOR
                    }
                    value == "trunk" || value == "nsl_dual" -> RoadContext.EXPRESS
                    value == "motorway" -> RoadContext.MOTORWAY
                    else -> null
                }
                return ZoneInfo(country, ctx, implicit(v))
            }
            return null
        }

        /** Il limite generale di `XX:valore`; per le zone (`DE:zone30`, `IT:zone:30`) il numero della zona. */
        private fun implicit(v: String): Int {
            CarSpeedTables.countryMaxSpeeds[v.lowercase()]?.let { return it }
            val value = v.substring(3).lowercase()
            if (value.startsWith("zone")) value.filter { it.isDigit() }.toIntOrNull()?.takeIf { it in 5..130 }?.let { return it }
            return -1
        }

        /** Un valore di `maxspeed`: `XX:valore` vale il limite generale del Paese, il resto come [OsmValues.parseKmh]. */
        fun parseSigned(raw: String?): Double {
            if (raw == null) return Double.NaN
            val v = raw.trim()
            if (isZoneValue(v)) implicit(v).takeIf { it > 0 }?.let { return it.toDouble() }
            return OsmValues.parseKmh(v)
        }
    }
}
