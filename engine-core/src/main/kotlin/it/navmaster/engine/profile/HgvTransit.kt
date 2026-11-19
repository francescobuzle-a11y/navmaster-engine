package it.navmaster.engine.profile

import it.navmaster.engine.osm.OsmValues
import it.navmaster.engine.osm.OsmWay

/**
 * Divieti di transito per i mezzi pesanti (modello NAVMASTER, novembre 2026): i tratti dove il camion
 * può entrare solo per arrivare (o partire), e i limiti di peso «di classe» (`maxweightrating`) che
 * prima non si leggevano affatto. ORS non li conosce: sui grafi del modello ORS non si usa nulla di qui.
 *
 * Il riferimento è un autoarticolato da [HgvProfile.REF_TRUCK_T] t, come per i limiti di velocità
 * condizionali: un limite N vale se N < 40 (una condizione `weight>=N` se N ≤ 40).
 *
 * **Divieto «solo destinazione»** (bit [FWD]/[BWD], per senso di marcia della way): il camion ci passa,
 * ma ogni metro pesa ×10 (LimitedAccessWeighting, solo profilo `heavyvehicle`): lo usa per arrivare o
 * partire, non per attraversare. Vale per:
 * - `hgv` (o `goods` se manca `hgv`) = `destination`, `delivery` o `local`, anche fra più valori `a;b`;
 *   non `private`, che sulle strade principali resta la corsia di dogana (HgvProfile.privateMainRoad)
 *   e altrove un divieto;
 * - le condizioni di solo peso `destination|delivery|private @ (weight>N)` (anche `weightrating`,
 *   `maxweight`, `maxweightrating`, virgola decimale, spazi, `t`) sulle chiavi `hgv`, `goods`,
 *   `motor_vehicle`, `vehicle`, `access` `:conditional`: decide la chiave più specifica che ha una parte
 *   valida, e fra le sue parti l'ultima valida (regola del wiki). Una parte con un orario, un giorno, un
 *   mese, PH, neve o bagnato (qualunque termine che non sia un peso) non vale: i divieti a orario non
 *   diventano permanenti;
 * - `hgv=no` con `hgv:conditional=yes|destination|delivery @ (destination|delivery)`: era un divieto
 *   totale, la via di consegna stessa diventava irraggiungibile;
 * - `maxweightrating[:hgv][:forward|:backward]` o `maxweight:hgv` N < 40 con la deroga
 *   `none|no @ destination|delivery` (in `<chiave>:<senso>:conditional`, poi `<chiave>:conditional`):
 *   il senso dato (i due se senza senso);
 * - `maxweight`/`maxgcweight` N < 40 con la stessa deroga: due sensi più [EXEMPT], perché il filtro del
 *   camion non blocchi sul peso (le altre misure sì).
 *
 * **Divieti veri:** `no @ (weight>N)` senza orari blocca il camion (bit HGV, come `hgv=no`);
 * `maxweightrating[:hgv][:senso]` e `maxweight:hgv` N < 40 senza deroga diventano un limite di peso
 * (il minimo con `maxweight`, [ratingLimit]): così si rispettano i ponti francesi, ormai segnati tutti
 * con `maxweightrating`. Il limite di peso è unico per i due sensi: un `:forward` senza deroga limita
 * anche l'altro senso (prudente, raro).
 *
 * Limiti noti: una deroga per zona (Fernpass B179: chi parte o arriva nei distretti di Imst, Reutte,
 * Landeck, Innsbruck e nei Landkreis vicini) non si esprime per strada: un Reutte → Imst gira largo.
 * Le chiavi `hgv` valgono anche per gli autobus del profilo heavyvehicle (stessi pesi preparati).
 */
object HgvTransit {
    /** Divieto di transito nel senso della way. */
    const val FWD = 1
    /** Divieto di transito nel senso opposto alla way. */
    const val BWD = 2
    const val BOTH = FWD or BWD
    /** Il limite di `maxweight` della way ha la deroga per destinazione: il filtro non blocca sul peso. */
    const val EXEMPT = 4

    /** Esito di una condizione di solo peso: nessuna parte valida, aperta, solo destinazione, vietata. */
    enum class Rule { NONE, OPEN, DESTINATION, NO }

    private val plainValues = setOf("destination", "delivery", "local")
    private val conditionalKeys = listOf("hgv", "goods", "motor_vehicle", "vehicle", "access").map { "$it:conditional" }
    private val ratingBases = listOf("maxweightrating:hgv", "maxweightrating", "maxweight:hgv")
    private val exemptionConditions = setOf("destination", "delivery")
    private val exemptionValues = setOf("none", "no")
    private val hgvExemptionValues = setOf("yes", "destination", "delivery")
    private val openValues = setOf("yes", "designated", "permissive")
    private val destinationValues = setOf("destination", "delivery", "private")

    /** Una condizione di peso (regex a parte da quella dei limiti di velocità, che non cambiano). */
    private val TRANSIT_WEIGHT = Regex("""^(weight|weightrating|maxweight|maxweightrating)\s*(>=?)\s*([0-9]+(?:[.,][0-9]+)?)\s*t?$""")
    private val AND = Regex("""\s+and\s+""", RegexOption.IGNORE_CASE)
    private val LIST = Regex("""\s*(?:;|,|\s+or\s+)\s*""", RegexOption.IGNORE_CASE)

    /** I bit [FWD], [BWD], [EXEMPT] della way. */
    fun flags(way: OsmWay): Int {
        var bits = 0
        // tag semplici: hgv, o goods se manca hgv
        val plainKey = if (way.hasTag("hgv")) "hgv" else if (way.hasTag("goods")) "goods" else null
        if (plainKey != null && way.tagValues(plainKey).any { it.lowercase() in plainValues }) bits = BOTH
        if (hgvNoWithDestinationExemption(way)) bits = BOTH
        if (weightRule(way) == Rule.DESTINATION) bits = BOTH
        // limiti di classe con la deroga per destinazione, per senso
        for (base in ratingBases) for ((dir, bit) in DIRS) {
            val n = limitTons(way.tag(if (dir == null) base else "$base:$dir")) ?: continue
            if (n < HgvProfile.REF_TRUCK_T && exempted(way, base, dir)) bits = bits or bit
        }
        // maxweight con la deroga: solo destinazione nei due sensi, e il filtro non blocca sul peso
        if (plainWeightExempt(way)) bits = bits or BOTH or EXEMPT
        if (bits and EXEMPT != 0 && ratingLimit(way).isFinite()) bits = bits and EXEMPT.inv()
        return bits
    }

    /**
     * Il limite di peso vero dei limiti di classe senza deroga (`maxweightrating[:hgv][:senso]`,
     * `maxweight:hgv`, N < 40), il più basso; +∞ se nessuno.
     */
    fun ratingLimit(way: OsmWay): Double {
        var limit = Double.POSITIVE_INFINITY
        for (base in ratingBases) for ((dir, _) in DIRS) {
            val n = limitTons(way.tag(if (dir == null) base else "$base:$dir")) ?: continue
            if (n < HgvProfile.REF_TRUCK_T && !exempted(way, base, dir) && n < limit) limit = n
        }
        return limit
    }

    /** `maxweight` (o `maxgcweight`) N < 40 con la deroga `none @ destination|delivery` nel suo `:conditional`. */
    fun plainWeightExempt(way: OsmWay): Boolean {
        val key = listOf("maxweight", "maxgcweight").firstOrNull { way.hasTag(it) } ?: return false
        val n = limitTons(way.tag(key)) ?: return false
        return n < HgvProfile.REF_TRUCK_T && exemption(way.tag("$key:conditional"), exemptionValues)
    }

    /** `hgv=no` con `hgv:conditional=yes|destination|delivery @ (destination|delivery)`. */
    fun hgvNoWithDestinationExemption(way: OsmWay): Boolean =
        way.hasTag("hgv", "no") && exemption(way.tag("hgv:conditional"), hgvExemptionValues)

    /** La condizione di solo peso che vale per il camion di riferimento, dalla chiave più specifica che ne ha una. */
    fun weightRule(way: OsmWay): Rule {
        for (key in conditionalKeys) {
            val r = weightRule(way.tag(key))
            if (r != Rule.NONE) return r
        }
        return Rule.NONE
    }

    /** L'ultima parte valida di un valore `:conditional` di accesso con sole condizioni di peso. */
    fun weightRule(raw: String?): Rule {
        if (raw.isNullOrBlank()) return Rule.NONE
        var rule = Rule.NONE
        for (part in parts(raw)) {
            val (value, condition) = split(part) ?: continue
            val terms = condition.split(AND).map { it.trim() }
            if (terms.isEmpty() || !terms.all { appliesToRefTruck(it) }) continue
            val r = when (value) {
                in openValues -> Rule.OPEN
                in destinationValues -> Rule.DESTINATION
                "no" -> Rule.NO
                else -> null
            } ?: continue
            rule = r
        }
        return rule
    }

    /** Vero se una parte del valore ha uno dei [values] con la sola condizione destination/delivery (anche in elenco), senza orari. */
    fun exemption(raw: String?, values: Set<String>): Boolean {
        if (raw.isNullOrBlank()) return false
        return parts(raw).any { part ->
            val (value, condition) = split(part) ?: return@any false
            if (value !in values) return@any false
            val terms = condition.split(AND).flatMap { it.split(LIST) }.map { it.trim().removePrefix("(").removeSuffix(")").trim() }.filter { it.isNotEmpty() }
            terms.isNotEmpty() && terms.all { it in exemptionConditions }
        }
    }

    /** Il limite in tonnellate di un tag (`7.5`, `7,5`, `7.5 t`), null se manca o non è un numero. */
    fun limitTons(raw: String?): Double? {
        if (raw.isNullOrBlank()) return null
        val v = OsmValues.parseTons(raw.trim().replace(',', '.').replace(" ", ""))
        return if (v.isNaN() || v <= 0) null else v
    }

    private val DIRS = listOf<Pair<String?, Int>>(null to BOTH, "forward" to FWD, "backward" to BWD)

    /** La deroga per destinazione di un limite: `<base>:<senso>:conditional`, poi `<base>:conditional`. */
    private fun exempted(way: OsmWay, base: String, dir: String?): Boolean {
        if (dir != null && exemption(way.tag("$base:$dir:conditional"), exemptionValues)) return true
        return exemption(way.tag("$base:conditional"), exemptionValues)
    }

    private fun appliesToRefTruck(term: String): Boolean {
        val m = TRANSIT_WEIGHT.find(term.trim().removePrefix("(").removeSuffix(")").trim()) ?: return false
        val n = m.groupValues[3].replace(',', '.').toDoubleOrNull() ?: return false
        return if (m.groupValues[2] == ">=") n <= HgvProfile.REF_TRUCK_T else n < HgvProfile.REF_TRUCK_T
    }

    /** Valore e condizione (senza le parentesi esterne) di una parte `valore @ condizione`, in minuscolo. */
    private fun split(part: String): Pair<String, String>? {
        val at = part.indexOf('@')
        if (at <= 0) return null
        val value = part.substring(0, at).trim().lowercase()
        var cond = part.substring(at + 1).trim().lowercase()
        if (cond.startsWith("(") && cond.endsWith(")")) cond = cond.substring(1, cond.length - 1).trim()
        return if (value.isEmpty() || cond.isEmpty()) null else value to cond
    }

    /** Le parti di un valore `:conditional`, separate dai `;` fuori dalle parentesi. */
    fun parts(raw: String): List<String> {
        val out = ArrayList<String>(2)
        var depth = 0; var start = 0
        for (i in raw.indices) when (raw[i]) {
            '(' -> depth++
            ')' -> if (depth > 0) depth--
            ';' -> if (depth == 0) { out += raw.substring(start, i); start = i + 1 }
        }
        out += raw.substring(start)
        return out.map { it.trim() }.filter { it.isNotEmpty() }
    }
}
