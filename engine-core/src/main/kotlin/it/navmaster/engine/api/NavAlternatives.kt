package it.navmaster.engine.api

import com.carrotsearch.hppc.IntDoubleHashMap
import com.carrotsearch.hppc.IntHashSet
import com.carrotsearch.hppc.IntIntHashMap
import com.carrotsearch.hppc.LongIntHashMap
import it.navmaster.engine.index.Snap
import it.navmaster.engine.routing.BestTrace
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.Circle
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.Detour
import it.navmaster.engine.routing.DetourSearch
import it.navmaster.engine.routing.DetourTrace
import it.navmaster.engine.routing.Ellipse
import it.navmaster.engine.routing.MotorwayNet
import it.navmaster.engine.routing.Region
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.storage.BaseGraph
import java.util.concurrent.CountDownLatch
import java.util.concurrent.atomic.AtomicBoolean

/** Da dove viene un percorso della risposta `navmaster.alternatives`. */
enum class VariantSource(val json: String) { BEST("best"), MOTORWAY("motorway"), PENALTY("penalty"), CROSSING("crossing") }

/**
 * Estensione NavMaster `{"navmaster": {"alternatives": {…}}}`: varianti del percorso migliore con
 * qualunque numero di punti (non le `alternative_routes` di ORS). [count]: i percorsi in tutto,
 * migliore compreso (2..6). Livello 1: durata ≤ [tier1Factor] e km ≤ [tier1KmFactor] del migliore;
 * livello 2 («di poco più lunghe»): ≤ [maxFactor] e ≤ [maxKmFactor]. [allowFewerKm]: una variante
 * con meno km del migliore vale al livello 1 fino a [maxFactor] di durata. [fill]: in mancanza,
 * anche il livello 3 (riempimento). [budgetMs]: il tempo della richiesta di varianti; alle fonti va
 * quello che resta dopo la preparazione (il migliore, la rete autostradale), almeno 3 s, poi si
 * risponde con quel che c'è. [excludeBest]: la risposta senza il migliore (l'app lo ha già).
 */
class NavAlternativesOptions(
    val count: Int = 3,
    val tier1Factor: Double = 1.3,
    val maxFactor: Double = 1.5,
    val budgetMs: Long = 30_000,
    val excludeBest: Boolean = false,
    val tier1KmFactor: Double = tier1Factor,
    val maxKmFactor: Double = maxFactor,
    val allowFewerKm: Boolean = false,
    val fill: Boolean = false,
    internal val sources: Set<VariantSource> = setOf(VariantSource.MOTORWAY, VariantSource.PENALTY, VariantSource.CROSSING),
    /** Le soglie di distanza delle regole moltiplicate per questo (solo le prove su Heidelberg, dove tutto è corto). */
    internal val minDetourScale: Double = 1.0,
) {
    /** La traccia delle varianti (solo le prove diagnostiche, [withTrace]; null: spenta, nessun conto in più). */
    internal var trace: VariantTrace? = null
        private set

    /** Le stesse opzioni con la traccia [t]. */
    internal fun withTrace(t: VariantTrace?): NavAlternativesOptions =
        NavAlternativesOptions(count, tier1Factor, maxFactor, budgetMs, excludeBest, tier1KmFactor, maxKmFactor, allowFewerKm, fill, sources, minDetourScale).also { it.trace = t }

    companion object {
        /** Da `navmaster.alternatives` del corpo; errori ORS 2003. */
        fun fromJson(v: Any?): NavAlternativesOptions {
            val m = v as? Map<*, *> ?: throw bad("Parameter 'navmaster.alternatives' must be an object.")
            for (k in m.keys) if (k !in KNOWN) throw ApiException(400, RoutingErrorCodes.UNKNOWN_PARAMETER, "Unknown parameter 'navmaster.alternatives.$k'.")
            fun num(key: String): Double? = m[key]?.let { x -> (x as? Number)?.toDouble() ?: throw bad("Parameter 'navmaster.alternatives.$key' must be a number.") }
            fun flag(key: String): Boolean? = m[key]?.let { x -> x as? Boolean ?: throw bad("Parameter 'navmaster.alternatives.$key' must be true or false.") }
            val count = num("count")?.let { c -> if (c != Math.floor(c)) throw bad("Parameter 'navmaster.alternatives.count' must be an integer."); c.toInt() } ?: 3
            if (count !in 2..6) throw bad("Parameter 'navmaster.alternatives.count' has incorrect value of '$count' (2 to 6).")
            val tier1 = num("tier1_factor") ?: 1.3
            if (tier1 !in 1.05..1.5) throw bad("Parameter 'navmaster.alternatives.tier1_factor' must be between 1.05 and 1.5.")
            val max = num("max_factor") ?: maxOf(1.5, tier1)
            if (max < tier1 || max > 2.0) throw bad("Parameter 'navmaster.alternatives.max_factor' must be between tier1_factor and 2.0.")
            val tier1Km = num("tier1_km_factor") ?: tier1
            if (tier1Km !in 1.05..2.0) throw bad("Parameter 'navmaster.alternatives.tier1_km_factor' must be between 1.05 and 2.0.")
            val maxKm = num("max_km_factor") ?: maxOf(max, tier1Km)
            if (maxKm < tier1Km || maxKm > 2.0) throw bad("Parameter 'navmaster.alternatives.max_km_factor' must be between tier1_km_factor and 2.0.")
            val budget = num("budget_ms")?.toLong() ?: 30_000L
            if (budget !in 1_000L..120_000L) throw bad("Parameter 'navmaster.alternatives.budget_ms' must be between 1000 and 120000.")
            return NavAlternativesOptions(count, tier1, max, budget, flag("exclude_best") ?: false, tier1Km, maxKm, flag("allow_fewer_km") ?: false, flag("fill") ?: false)
        }

        private val KNOWN = setOf("count", "tier1_factor", "max_factor", "tier1_km_factor", "max_km_factor", "allow_fewer_km", "fill", "budget_ms", "exclude_best")
        private fun bad(msg: String) = ApiException(400, RoutingErrorCodes.INVALID_PARAMETER_VALUE, msg)
    }
}

/**
 * La traccia delle varianti (solo le prove diagnostiche, mai dal JSON): righe `varianti: traccia …`
 * in più nel registro, sulle stesse scelte (la risposta non cambia). Con la traccia: la rete M
 * (semi, parte del migliore coperta, versi lasciati fuori), per tratto i conti della ricerca delle
 * deviazioni e le deviazioni anche oltre `perLeg` (al più [maxDetoursPerLeg], giudicate ma non
 * offerte), il giudizio di ogni candidato coi valori delle regole, la fonte P provata «in ombra»
 * ([shadowPenalty]: senza offerte) se il pool era già pieno, e le sonde [probes] (percorsi forzati
 * della prova) giudicate e sorvegliate nella ricerca M.
 */
internal class VariantTrace(val probes: List<Probe> = emptyList(), val maxDetoursPerLeg: Int = 40, val shadowPenalty: Boolean = true) {
    /** Un percorso forzato della prova (per esempio per un punto della A21), coi tratti del router. */
    class Probe(val name: String, val legs: List<RoutePath>)
}

/**
 * Un percorso candidato: un tratto per coppia di punti, [legs] leggeri (archi interni senza
 * geometria, `Router.route(light = true)`) o cuciti; si completa solo se tenuto. [parts]: quante
 * deviazioni lo compongono (le combinazioni della fonte M). [tag]: il candidato che l'ha dato (registro).
 * [turnsChecked]: ogni svolta è già stata controllata da chi l'ha costruito (le deviazioni cucite
 * della fonte M): la validità non rilegge i divieti di tutto il percorso.
 */
internal class Draft(val legs: List<RoutePath>, val source: VariantSource, val detours: List<Detour>? = null, val parts: Int = 1, val tag: String = "",
                     val turnsChecked: Boolean = false) {
    val millis: Long = legs.sumOf { it.millis }
    val meters: Double = legs.sumOf { it.distance }
    val weight: Double = legs.sumOf { it.weight }
    /** Gli archi in ordine (tutti i tratti di seguito), con tempi e metri. */
    val edgeIds: IntArray
    val edgeMs: LongArray
    val edgeM: DoubleArray
    val hash: Long
    /** L'indice in [edgeIds] del primo arco di ogni tratto (e, in fondo, la fine dell'ultimo). */
    val legStart = IntArray(legs.size + 1)

    init {
        val n = legs.sumOf { it.edges.size }
        edgeIds = IntArray(n); edgeMs = LongArray(n); edgeM = DoubleArray(n)
        var i = 0
        var h = 1125899906842597L
        for ((j, l) in legs.withIndex()) {
            legStart[j] = i
            for (e in l.edges) {
                edgeIds[i] = e.edge; edgeMs[i] = e.millis; edgeM[i] = e.distance; i++
                h = 31 * h + (e.edge.toLong() shl 1 or (if (e.reverse) 1L else 0L))
            }
            h = 31 * h - 7 // separatore fra i tratti
        }
        legStart[legs.size] = i
        hash = h
    }

    /** Il tratto dell'arco in posizione [i] di [edgeIds]. */
    fun legAt(i: Int): Int { var j = 0; while (j + 1 < legs.size && legStart[j + 1] <= i) j++; return j }

    /** L'ultima valutazione ([VariantRules]): la fonte P la guarda per la crescita degli AR scartati. */
    @Volatile var verdict: Verdict? = null
    /** I fatti per le regole, calcolati una volta ([VariantJudge.facts]). */
    internal var facts: DraftFacts? = null
    /** Il titolo senza gli altri tenuti ([VariantJudge.title]: le strade che il migliore non usa, o la strada del tratto deviato), letto una volta. */
    internal var baseTitle: String? = null
    /** Solo locale ([VariantJudge.localOnly]), calcolato una volta (legge le coordinate di pochi nodi). */
    internal var localOnly: Boolean? = null

    /** Il nodo d'arrivo dell'arco in posizione [i] di [edgeIds]. */
    fun nodeAt(i: Int): Int { val j = legAt(i); return legs[j].edges[i - legStart[j]].adjNode }
}

/**
 * Un pezzo di un viaggio tra Paesi (passo 7b): le varianti del pezzo si giudicano sui totali del
 * viaggio. [extraMillis], [extraMeters], [extraWeight]: gli altri pezzi del migliore, che restano;
 * [offsetMeters]: i metri del viaggio prima del pezzo (i km delle deviazioni nella risposta).
 */
internal class TripContext(val extraMillis: Long = 0L, val extraMeters: Double = 0.0, val extraWeight: Double = 0.0, val offsetMeters: Double = 0.0) {
    companion object { val NONE = TripContext() }
}

/**
 * Classe e ref degli archi di un [Draft]. [edgeSet] (gli archi di [edges]) si costruisce alla prima
 * lettura: serve solo per i percorsi tenuti (il migliore compreso), non per ogni candidato (memoria
 * della fonte M: circa 16 byte per arco per candidato risparmiati).
 */
internal class DraftFacts(private val edges: IntArray, val refMeters: IntDoubleHashMap, val signature: List<Pair<String, Double>>, val mw: BooleanArray, val mwMeters: Double,
                          /**
                           * Per arco: autostradale, cioè `highway=motorway` (rampe comprese) o una superstrada
                           * (`trunk`) con un ref da autostrada ([CorridorAnalyzer.isMotorwayRef]: RA, T, E45…);
                           * le superstrade statali (SS42, SR450, SS11, B37) no. La quota autostradale del punteggio.
                           */
                          val motorway: BooleanArray = BooleanArray(edges.size),
                          /** ref → i suoi metri autostradali ([motorway]): il bonus delle strade nuove vale solo per le autostrade. */
                          val refMotorwayMeters: IntDoubleHashMap = IntDoubleHashMap()) {
    val edgeSet: IntHashSet by lazy { IntHashSet(edges.size * 2).also { s -> for (e in edges) s.add(e) } }
}

/**
 * L'esito della regola per un [Draft] rispetto ai percorsi tenuti: [tier] 1, 2, 3 (0 = scartato, con
 * [reason]: tempo, km, diversità, locale, doppione, autostrade); [dMaxM]: la deviazione più lunga
 * dal tenuto più vicino; [newRefs]: le strade (ref) che nessun tenuto usa; [label]: «via A21».
 */
internal class Verdict(val tier: Int, val reason: String, val dMaxM: Double, val newRefs: List<String>, val label: String, val score: Double = 0.0,
                       /** Solo locale ([VariantJudge.localOnly]): nella scelta dopo tutte le altre. */
                       val local: Boolean = false) {
    fun withLabel(l: String) = Verdict(tier, reason, dMaxM, newRefs, l, score, local)
}

/** Un tratto contiguo di una variante fuori dagli archi di un altro percorso. */
internal class Run(val from: Int, val to: Int, val meters: Double, val ms: Long, val mwMeters: Double, val motMeters: Double = 0.0) {
    /** Le posizioni sul migliore dell'ultimo arco comune prima e del primo dopo (-1 / size se nessuno). */
    var anchorFrom = -1; var anchorTo = -1
    val mwShare: Double get() = if (meters > 0) mwMeters / meters else 0.0
}

/** Le soglie della regola di accettazione delle varianti (uguale per tutte le fonti). */
internal object VariantRules {
    /** La deviazione minima del livello 1 per un viaggio di [L] metri. */
    fun dMin1(L: Double): Double = minOf(40_000.0, maxOf(0.05 * L, minOf(15_000.0, 0.25 * L)))

    /** I tratti massimali di [v] con archi fuori da [k]; [mw]: per arco di [v], autostrada o superstrada; [mot]: solo autostrada. */
    fun runsOff(v: Draft, k: IntHashSet, mw: BooleanArray? = null, mot: BooleanArray? = null): List<Run> {
        val out = ArrayList<Run>()
        var i = 0
        val n = v.edgeIds.size
        while (i < n) {
            if (k.contains(v.edgeIds[i])) { i++; continue }
            var j = i; var m = 0.0; var ms = 0L; var mwm = 0.0; var motm = 0.0
            while (j < n && !k.contains(v.edgeIds[j])) {
                m += v.edgeM[j]; ms += v.edgeMs[j]
                if (mw != null && mw[j]) mwm += v.edgeM[j]
                if (mot != null && mot[j]) motm += v.edgeM[j]
                j++
            }
            out += Run(i, j - 1, m, ms, mwm, motm)
            i = j
        }
        return out
    }

    // livello 1
    const val T1_RUN_MS = 300_000L
    const val T1_A = 1.6
    const val T1_B_MS = 600_000L
    /** Il migliore è «autostradale» (≥ 30 % autostrada o superstrada): una variante deve deviare su autostrada o cambiare firma. */
    const val MW_BEST_SHARE = 0.3
    const val MW_RUN_SHARE = 0.6
    // livello 2 (la deviazione minima è dMin1 a ogni livello: una variante deve essere un'alternativa riconoscibile)
    const val T2_RUN_MS = 180_000L
    const val T2_A = 1.8
    const val T2_B_MS = 900_000L
    // livello 3
    const val T3_A = 2.0
    const val T3_B_MS = 1_200_000L
    /**
     * Le deviazioni più corte cercate dalla fonte M: max(5 km, 0,3·dMin1), in scala. Da sole non
     * bastano (ogni livello chiede dMin1), servono nelle combinazioni (due cambi di autostrada).
     */
    const val SEARCH_D = 0.3
    const val SEARCH_D_MIN = 5_000.0
    /** Una strada «nuova» ha almeno 5 km nella variante e meno di 1 km in ogni tenuto. */
    const val NEW_REF_M = 5_000.0
    const val ABSENT_REF_M = 1_000.0

    // il punteggio della scelta golosa (più basso = meglio):
    // 0,6·r + 0,4·rk − NEW_REF_BONUS·(autostrade nuove, al più 2) − COMBO_BONUS (2+ deviazioni, senza lasciare l'autostrada)
    // − MW_SWAP_BONUS·guadagno + MW_SWAP_BONUS·perdita
    /** Per ogni autostrada nuova (al più [NEW_REF_MAX]): una strada nuova vale solo se è autostrada per almeno [NEW_REF_MW_SHARE] dei suoi km nella variante. */
    const val NEW_REF_BONUS = 0.04
    const val NEW_REF_MAX = 2
    const val NEW_REF_MW_SHARE = 0.5
    /** Una combinazione di più deviazioni (cambi di autostrada in più punti). */
    const val COMBO_BONUS = 0.02
    /**
     * Il premio quando la variante sostituisce km non autostradali del migliore con autostrada,
     * dello stesso ordine di un'autostrada nuova: per ogni tratto deviato il guadagno di quota
     * autostradale ([DraftFacts.motorway]: quota del tratto meno quella del pezzo del migliore che
     * sostituisce, se positivo) pesato col pezzo sostituito su dMin1 (al più 1: un pezzo di 1 km
     * vale poco), sommato sui tratti e al più 1. Una deviazione su statale al posto di
     * un'autostrada (guadagno negativo) non prende nulla. Le superstrade statali (`trunk` con ref
     * SS/SR/B: SR450, SS11, SS42) non contano come autostrada; quelle con un ref da autostrada (RA,
     * T, E45) sì. Esempio Bolzano→Milano: il migliore fa SR450+SS11 fra A22 e A4, la variante A22
     * fino a Verona e A4 sostituisce ~50 km di statali con autostrada (guadagno ~0,8-1: −0,03/0,04).
     * Il caso inverso (una statale al posto dell'autostrada del migliore) paga lo stesso come malus:
     * per un camion le statali che attraversano i paesi valgono meno a parità di tempo e km.
     */
    const val MW_SWAP_BONUS = 0.04
    /** Il bonus delle combinazioni solo se la quota autostradale persa ([MW_SWAP_BONUS]) non supera questa. */
    const val COMBO_MAX_LOSS = 0.1
    /**
     * Una variante «solo locale» ([VariantJudge.localOnly]): ogni tratto deviato di almeno
     * [LOCAL_RUN_M] dentro i cerchi d'accesso dei punti (controllato ogni [LOCAL_SAMPLE_M] di strada),
     * nessuna autostrada che il migliore non usa e un guadagno di quota autostradale sotto
     * [LOCAL_MAX_GAIN]: un giro diverso per uscire dalla città, non un altro corridoio.
     */
    const val LOCAL_RUN_M = 1_000.0
    const val LOCAL_SAMPLE_M = 1_000.0
    const val LOCAL_MAX_GAIN = 0.1

    /** I nomi di gallerie e ponti (mai nel titolo di una variante se c'è un'altra strada). */
    private val STRUCTURE = Regex("^(galleria|ponte|viadotto|traforo|sottopasso|sovrappasso|cavalcavia|tunnel)\\b|(tunnel|brücke|bruecke)$", RegexOption.IGNORE_CASE)
    fun isStructureName(n: String): Boolean = STRUCTURE.containsMatchIn(n.trim())

    /** Dove una deviazione esce dal migliore, per distinguere due titoli: «dal km 75», «dalla partenza» (km 0). */
    fun fromKm(km: Double): String = Math.round(km).let { if (it == 0L) "dalla partenza" else "dal km $it" }

    /**
     * Titoli tutti diversi: [titles] nell'ordine, e per un titolo già usato (anche in [taken]) la
     * prima delle sue alternative [alternatives] libera, se no un numero ("via A1 (2)").
     */
    fun distinct(titles: List<String>, alternatives: (Int) -> List<String>, taken: Collection<String> = emptyList()): List<String> {
        val used = HashSet<String>(taken)
        return titles.mapIndexed { i, t ->
            val pick = if (t !in used) t else alternatives(i).firstOrNull { it !in used }
                ?: generateSequence(2) { it + 1 }.map { "$t ($it)" }.first { it !in used }
            used += pick
            pick
        }
    }
}

/**
 * La regola di accettazione ([VariantRules]) per le varianti di [best]: i fatti per percorso
 * (classe e ref degli archi, firma), le ancore dei tratti deviati sul migliore, la validità.
 * [router], [request] e [snaps] servono ai controlli di validità (svolte, verso ai punti di passaggio).
 * [trip]: un pezzo di un viaggio tra Paesi, tempi e km si confrontano sul viaggio intero.
 * [circles]: i cerchi d'accesso dei punti ([accessCircles]): una variante che devia solo lì è «solo locale» ([localOnly]).
 */
internal class VariantJudge(val best: Draft, private val info: EdgeInfo, private val corridors: CorridorAnalyzer, private val opt: NavAlternativesOptions,
                            private val graph: BaseGraph, private val router: Router? = null, private val request: DirectionsRequest? = null,
                            private val snaps: List<Snap>? = null, val trip: TripContext = TripContext.NONE,
                            private val circles: List<Circle> = emptyList()) {
    /** Durata e metri del viaggio del migliore (con gli altri pezzi di un viaggio tra Paesi). */
    val tripMillis: Long get() = best.millis + trip.extraMillis
    val tripMeters: Double get() = best.meters + trip.extraMeters

    /**
     * (tratto, arco) → posizione (la prima) nel migliore, e arco → posizione (la prima, in tutto il
     * viaggio) se il tratto non ce l'ha; i tempi cumulati del migliore (prefisso: prima dell'arco i).
     * Per tratto: in un'andata e ritorno lo stesso arco sta in due tratti, in ordine inverso.
     */
    private val bestLegPos = LongIntHashMap(best.edgeIds.size * 2)
    private val bestPos = IntIntHashMap(best.edgeIds.size * 2)
    private val bestPreMs = LongArray(best.edgeIds.size + 1)
    private val bestPreM = DoubleArray(best.edgeIds.size + 1)
    /** I metri cumulati autostradali del migliore ([DraftFacts.motorway], prefisso come [bestPreM]): calcolati alla prima variante. */
    private val bestPreMot: DoubleArray by lazy {
        val mot = facts(best).motorway
        DoubleArray(best.edgeIds.size + 1).also { a -> for (i in best.edgeIds.indices) a[i + 1] = a[i] + (if (mot[i]) best.edgeM[i] else 0.0) }
    }

    init {
        var leg = 0
        for (i in best.edgeIds.indices) {
            while (leg + 1 < best.legs.size && best.legStart[leg + 1] <= i) leg++
            val k = legKey(leg, best.edgeIds[i])
            if (!bestLegPos.containsKey(k)) bestLegPos.put(k, i)
            if (!bestPos.containsKey(best.edgeIds[i])) bestPos.put(best.edgeIds[i], i)
            bestPreMs[i + 1] = bestPreMs[i] + best.edgeMs[i]; bestPreM[i + 1] = bestPreM[i] + best.edgeM[i]
        }
    }

    private fun legKey(leg: Int, edge: Int): Long = (leg.toLong() shl 32) or (edge.toLong() and 0xffffffffL)

    /** La posizione nel migliore dell'arco [i] di [v]: nello stesso tratto se c'è, se no la prima; [none] se manca. */
    private fun posInBest(v: Draft, i: Int, none: Int): Int {
        val e = v.edgeIds[i]
        val k = legKey(v.legAt(i), e)
        return if (bestLegPos.containsKey(k)) bestLegPos.get(k) else bestPos.getOrDefault(e, none)
    }

    private val scale = opt.minDetourScale
    private companion object {
        /** Una strada sotto i 5 km nel titolo solo con almeno questa quota di un tratto deviato ([titleRefs]). */
        const val TITLE_RUN_SHARE = 0.2
    }
    val tier1Factor: Double get() = opt.tier1Factor
    val bestMwShare: Double get() = facts(best).mwMeters / maxOf(1.0, best.meters)

    fun facts(d: Draft): DraftFacts {
        d.facts?.let { return it }
        val refs = IntDoubleHashMap()
        val refsMot = IntDoubleHashMap()
        val mw = BooleanArray(d.edgeIds.size)
        val mot = BooleanArray(d.edgeIds.size)
        var mwm = 0.0
        // per ref d'una superstrada: 1 se è un ref da autostrada, 2 se no (letto una volta per ref in questa bozza)
        val trunkRef = com.carrotsearch.hppc.IntByteHashMap()
        for (i in d.edgeIds.indices) {
            val e = d.edgeIds[i]
            val r = info.refId(e)
            if (info.isMotorwayOrTrunk(e)) {
                mw[i] = true; mwm += d.edgeM[i]
                // autostradale: `highway=motorway`, o una superstrada con un ref da autostrada (RA, T, A, E45…)
                mot[i] = info.isMotorway(e) || r >= 0 && run {
                    var c = trunkRef.get(r)
                    if (c.toInt() == 0) { c = if (CorridorAnalyzer.isMotorwayRef(info.refName(r))) 1 else 2; trunkRef.put(r, c) }
                    c.toInt() == 1
                }
            }
            if (r >= 0) { refs.addTo(r, d.edgeM[i]); if (mot[i]) refsMot.addTo(r, d.edgeM[i]) }
        }
        return DraftFacts(d.edgeIds, refs, corridors.signature(d.legs), mw, mwm, mot, refsMot).also { d.facts = it }
    }

    /** I tratti di [v] fuori dal migliore, con le ancore sul migliore. */
    fun runsFromBest(v: Draft): List<Run> {
        val f = facts(v)
        val runs = VariantRules.runsOff(v, facts(best).edgeSet, f.mw, f.motorway)
        for (r in runs) {
            r.anchorFrom = if (r.from > 0) posInBest(v, r.from - 1, -1) else -1
            r.anchorTo = if (r.to + 1 < v.edgeIds.size) posInBest(v, r.to + 1, best.edgeIds.size) else best.edgeIds.size
        }
        return runs
    }

    /** Il tempo del migliore fra le ancore di [r] (gli archi strettamente fra le due). */
    fun bestMsBetween(r: Run): Long { val a = r.anchorFrom + 1; val b = r.anchorTo; return if (b > a) bestPreMs[b] - bestPreMs[a] else 0L }
    /** I km lungo il migliore dell'ancora d'uscita e di quella di rientro di [r]. */
    fun bestKmAt(r: Run): Pair<Double, Double> = (trip.offsetMeters + bestPreM[r.anchorFrom + 1]) / 1000 to (trip.offsetMeters + bestPreM[maxOf(r.anchorFrom + 1, r.anchorTo)]) / 1000

    /** Sanità locale: ogni tratto deviato non costa più di [a] volte il migliore fra le sue ancore più [bMs] (in scala). */
    private fun sane(runs: List<Run>, a: Double, bMs: Long): Boolean = runs.all { r -> r.ms <= a * bestMsBetween(r) + bMs * scale }

    /**
     * Vero se [v] è un percorso ammissibile come variante: un tratto per coppia di punti, ogni
     * svolta lecita, nessun nodo ripetuto in un tratto (più di quanti ne ripete il migliore), e ai
     * punti di passaggio il verso d'arrivo conservato dove il migliore lo conserva.
     */
    fun valid(v: Draft): Boolean {
        if (v.legs.size != best.legs.size) return false
        val r = router
        for ((j, leg) in v.legs.withIndex()) {
            if (r != null && !v.turnsChecked && !r.isLegal(leg)) return false
            if (repeats(leg) > repeats(best.legs[j])) return false
        }
        val rq = request; val s = snaps
        if (r != null && rq != null && s != null) for (j in 1 until v.legs.size) {
            if (!rq.passThroughAt(j) || r.directionFree(s[j])) continue
            if (turnsBack(v, j, s[j]) && !turnsBack(best, j, s[j])) return false
        }
        return true
    }

    private fun turnsBack(d: Draft, j: Int, s: Snap): Boolean {
        val a = d.legs[j - 1].edges.lastOrNull() ?: return false
        val b = d.legs[j].edges.firstOrNull() ?: return false
        return a.edge == s.edge && b.edge == s.edge && a.reverse != b.reverse
    }

    /**
     * I nodi ripetuti fra quelli dentro il tratto (l'adjNode di ogni arco tranne l'ultimo): il
     * baseNode del primo arco parziale sta dietro la partenza e l'adjNode dell'ultimo oltre l'arrivo.
     */
    private fun repeats(leg: RoutePath): Int {
        val m = leg.edges.size
        if (m < 2) return 0
        val seen = IntHashSet(m * 2)
        var n = 0
        for (i in 0 until m - 1) if (!seen.add(leg.edges[i].adjNode)) n++
        return n
    }

    /**
     * Il livello di [v] rispetto ai tenuti [kept] (il migliore compreso): 1 (equivalente), 2 (di
     * poco più lungo, o non autostradale), 3 (riempimento, solo con [relaxed] e [NavAlternativesOptions.fill]), 0 scartato.
     */
    fun evaluate(v: Draft, kept: List<Draft>, relaxed: Boolean): Verdict {
        if (kept.any { it.hash == v.hash }) return Verdict(0, "doppione", 0.0, emptyList(), "")
        if (best.millis <= 0 || best.meters <= 0) return Verdict(0, "tempo", 0.0, emptyList(), "")
        // sul viaggio intero (gli altri pezzi di un viaggio tra Paesi restano quelli del migliore)
        val r = (v.millis + trip.extraMillis).toDouble() / tripMillis
        val rk = (v.meters + trip.extraMeters) / tripMeters
        val f = facts(v)
        val d1 = VariantRules.dMin1(tripMeters) * scale
        val runsB = runsFromBest(v)
        if (runsB.isEmpty()) return Verdict(0, "doppione", 0.0, emptyList(), "")
        // la deviazione più lunga e il suo tempo, dal tenuto più vicino
        var dMax = Double.POSITIVE_INFINITY; var majorMs = Long.MAX_VALUE
        var sameSignature = false
        for (k in kept) {
            val runs = VariantRules.runsOff(v, facts(k).edgeSet)
            val major = runs.maxByOrNull { it.meters }
            dMax = minOf(dMax, major?.meters ?: 0.0); majorMs = minOf(majorMs, major?.ms ?: 0L)
            if (facts(k).signature.map { it.first } == f.signature.map { it.first }) sameSignature = true
        }
        val majorB = runsB.maxByOrNull { it.meters }!!
        val mwOk = bestMwShare < VariantRules.MW_BEST_SHARE || majorB.mwShare >= VariantRules.MW_RUN_SHARE || !sameSignature
        val newRefs = newRefs(v, kept)
        // a ogni livello la deviazione più lunga dal tenuto più vicino almeno dMin1 (in scala): un'alternativa riconoscibile
        val diverse = dMax >= d1
        val tier1Time = (r <= opt.tier1Factor && rk <= opt.tier1KmFactor) || (opt.allowFewerKm && rk < 1.0 && r <= opt.maxFactor)
        val tier1Rest = diverse && majorMs >= VariantRules.T1_RUN_MS * scale && sane(runsB, VariantRules.T1_A, VariantRules.T1_B_MS)
        val score = score(v, r, rk, newRefs, runsB, d1)
        fun ok(tier: Int, reason: String = "") = Verdict(tier, reason, dMax, newRefs, title(v), score, localOnly(v, runsB, d1))
        if (tier1Time && tier1Rest && mwOk) return ok(1)
        val withinMax = r <= opt.maxFactor && (rk <= opt.maxKmFactor || (opt.allowFewerKm && rk < 1.0))
        if (tier1Time && tier1Rest) return ok(2, "autostrade")
        if (withinMax && diverse && majorMs >= VariantRules.T2_RUN_MS * scale && sane(runsB, VariantRules.T2_A, VariantRules.T2_B_MS)) return ok(2)
        if (relaxed && opt.fill && withinMax && diverse && sane(runsB, VariantRules.T3_A, VariantRules.T3_B_MS)) return ok(3)
        val reason = when {
            r > opt.maxFactor -> "tempo"
            !withinMax -> "km"
            !diverse || majorMs < VariantRules.T2_RUN_MS * scale -> "diversità"
            !sane(runsB, VariantRules.T2_A, VariantRules.T2_B_MS) -> "locale"
            else -> "diversità"
        }
        return Verdict(0, reason, dMax, newRefs, "", score)
    }

    /**
     * Il perché del verdetto di [v] rispetto a [kept] ([names]: i loro nomi nel registro) coi valori
     * di ogni condizione di [evaluate]: solo la traccia delle varianti.
     */
    fun explain(v: Draft, kept: List<Draft>, names: List<String>): String {
        if (best.millis <= 0 || best.meters <= 0) return "migliore vuoto"
        val r = (v.millis + trip.extraMillis).toDouble() / tripMillis
        val rk = (v.meters + trip.extraMeters) / tripMeters
        val f = facts(v)
        val d1 = VariantRules.dMin1(tripMeters) * scale
        val runsB = runsFromBest(v)
        val sb = StringBuilder("r %.3f rk %.3f, d1 %.1f km".format(r, rk, d1 / 1000))
        if (runsB.isEmpty()) return sb.append(", nessun tratto fuori dal migliore").toString()
        val majorB = runsB.maxByOrNull { it.meters }!!
        val (ka, kb) = bestKmAt(majorB)
        sb.append("; dal migliore ${runsB.size} tratti, il maggiore %.1f km %.1f min (migliore %.1f min) fra i km %.1f e %.1f del migliore, ".format(
            majorB.meters / 1000, majorB.ms / 60000.0, bestMsBetween(majorB) / 60000.0, ka, kb)).append(refsIn(v, majorB).joinToString(" ").ifEmpty { "senza ref" })
        var dMax = Double.POSITIVE_INFINITY; var majorMs = Long.MAX_VALUE; var dMaxOf = ""
        var sameSignature = false
        val per = ArrayList<String>()
        for ((i, k) in kept.withIndex()) {
            val runs = VariantRules.runsOff(v, facts(k).edgeSet)
            val major = runs.maxByOrNull { it.meters }
            val mm = major?.meters ?: 0.0
            if (mm < dMax) { dMax = mm; dMaxOf = names.getOrElse(i) { "#$i" } }
            majorMs = minOf(majorMs, major?.ms ?: 0L)
            val same = facts(k).signature.map { it.first } == f.signature.map { it.first }
            if (same) sameSignature = true
            val common = 1.0 - runs.sumOf { it.meters } / maxOf(1.0, v.meters)
            per += names.getOrElse(i) { "#$i" } + ": dev. max %.1f km %.1f min, in comune %.0f %%".format(mm / 1000, (major?.ms ?: 0L) / 60000.0, common * 100) + (if (same) ", stessa firma" else "")
        }
        sb.append("; contro i tenuti [").append(per.joinToString("; ")).append("]; dMax %.1f km (da ".format(dMax / 1000)).append(dMaxOf).append("), corsa min %.1f min".format(majorMs / 60000.0))
        fun worst(a: Double, bMs: Long): String {
            val w = runsB.maxByOrNull { it.ms - (a * bestMsBetween(it) + bMs * scale) }!!
            val lim = a * bestMsBetween(w) + bMs * scale
            val (wa, wb) = bestKmAt(w)
            return (if (w.ms <= lim) "sì" else "no") + " (peggiore %.1f km %.0f s contro %.0f s, km %.1f–%.1f)".format(w.meters / 1000, w.ms / 1000.0, lim / 1000, wa, wb)
        }
        val tier1Time = (r <= opt.tier1Factor && rk <= opt.tier1KmFactor) || (opt.allowFewerKm && rk < 1.0 && r <= opt.maxFactor)
        fun yn(b: Boolean) = if (b) "sì" else "no"
        sb.append("; L1: tempo/km ${yn(tier1Time)} (≤ %.2f/%.2f)".format(opt.tier1Factor, opt.tier1KmFactor)).append(", dMax ≥ d1 ${yn(dMax >= d1)}, corsa ≥ 300 s ${yn(majorMs >= VariantRules.T1_RUN_MS * scale)}, sana 1,6·+600 s ")
            .append(worst(VariantRules.T1_A, VariantRules.T1_B_MS))
        val mwOk = bestMwShare < VariantRules.MW_BEST_SHARE || majorB.mwShare >= VariantRules.MW_RUN_SHARE || !sameSignature
        sb.append("; autostrade ${yn(mwOk)} (migliore %.0f %% MW, tratto maggiore %.0f %% MW, stessa firma ${yn(sameSignature)})".format(bestMwShare * 100, majorB.mwShare * 100))
        val withinMax = r <= opt.maxFactor && (rk <= opt.maxKmFactor || (opt.allowFewerKm && rk < 1.0))
        sb.append("; L2: entro ${yn(withinMax)} (≤ %.2f/%.2f)".format(opt.maxFactor, opt.maxKmFactor)).append(", dMax ≥ d1 ${yn(dMax >= d1)}, corsa ≥ 180 s ${yn(majorMs >= VariantRules.T2_RUN_MS * scale)}, sana 1,8·+900 s ")
            .append(worst(VariantRules.T2_A, VariantRules.T2_B_MS))
        val nr = newRefs(v, kept)
        val score = score(v, r, rk, nr, runsB, d1)
        sb.append("; strade nuove ").append(nr.ifEmpty { listOf("nessuna") }.joinToString(" ")).append(" (autostrade ").append(motorwayRefs(v, nr).ifEmpty { listOf("nessuna") }.joinToString(" "))
            .append("), guadagno autostradale %.2f, perdita %.2f, parti ${v.parts}, punteggio %.3f".format(motorwayGain(runsB, d1), motorwayGain(runsB, d1, loss = true), score))
            .append(if (circles.isEmpty()) "" else "; solo locale ${yn(localOnly(v, runsB, d1))}").append("; titolo «${title(v)}»; firma ")
            .append(f.signature.joinToString(" ") { it.first + " %.0f".format(it.second) })
        return sb.toString()
    }

    /**
     * Il punteggio della scelta golosa di [v] (più basso = meglio, valori in [VariantRules]):
     * 0,6·[r] + 0,4·[rk] − il bonus delle autostrade nuove ([motorwayRefs] fra [newRefs]) − quello
     * delle combinazioni (solo se non lasciano l'autostrada: perdita ≤ [VariantRules.COMBO_MAX_LOSS])
     * − il premio della quota autostradale guadagnata sul migliore ([motorwayGain]) + lo stesso per
     * quella persa (una statale al posto dell'autostrada).
     */
    private fun score(v: Draft, r: Double, rk: Double, newRefs: List<String>, runsB: List<Run>, d1: Double): Double {
        val loss = motorwayGain(runsB, d1, loss = true)
        return 0.6 * r + 0.4 * rk - VariantRules.NEW_REF_BONUS * minOf(VariantRules.NEW_REF_MAX, motorwayRefs(v, newRefs).size) -
            (if (v.parts >= 2 && loss <= VariantRules.COMBO_MAX_LOSS) VariantRules.COMBO_BONUS else 0.0) -
            VariantRules.MW_SWAP_BONUS * motorwayGain(runsB, d1) + VariantRules.MW_SWAP_BONUS * loss
    }

    /** Fra le strade nuove [newRefs] di [v], quelle che in [v] sono autostradali ([DraftFacts.motorway]) per almeno metà dei loro km. */
    fun motorwayRefs(v: Draft, newRefs: List<String>): List<String> {
        if (newRefs.isEmpty()) return newRefs
        val f = facts(v)
        return newRefs.filter { ref ->
            val id = info.idOf(ref)
            id >= 0 && f.refMotorwayMeters.getOrDefault(id, 0.0) >= VariantRules.NEW_REF_MW_SHARE * f.refMeters.getOrDefault(id, 0.0)
        }
    }

    /**
     * La quota autostradale guadagnata sul migliore ([VariantRules.MW_SWAP_BONUS]), in [0, 1]: per
     * ogni tratto deviato [runsB] (con le ancore, [runsFromBest]) la quota autostradale del
     * tratto meno quella del pezzo del migliore fra le ancore, se positiva, per min(1, metri del
     * pezzo sostituito / [d1]); la somma, al più 1. Con [loss] la quota persa (quella del pezzo del
     * migliore meno quella del tratto, se positiva), allo stesso modo. Solo RAM (i fatti della bozza, già letti).
     */
    fun motorwayGain(runsB: List<Run>, d1: Double, loss: Boolean = false): Double {
        if (runsB.isEmpty()) return 0.0
        var g = 0.0
        for (r in runsB) {
            if (r.meters <= 0) continue
            val a = r.anchorFrom + 1; val b = maxOf(a, r.anchorTo)
            val bestM = bestPreM[b] - bestPreM[a]
            if (bestM <= 0) continue
            val shareB = (bestPreMot[b] - bestPreMot[a]) / bestM
            val gain = if (loss) shareB - r.motMeters / r.meters else r.motMeters / r.meters - shareB
            if (gain > 0) g += gain * minOf(1.0, bestM / maxOf(1.0, d1))
        }
        return minOf(1.0, g)
    }

    /** Le strade (ref) con almeno 5 km in [v] e meno di 1 km in ogni tenuto, nell'ordine di [v]. */
    fun newRefs(v: Draft, kept: List<Draft>): List<String> {
        val f = facts(v)
        val out = ArrayList<String>()
        for ((ref, _) in f.signature) {
            if (ref in out) continue
            val id = info.idOf(ref)
            if (id < 0 || f.refMeters.getOrDefault(id, 0.0) < VariantRules.NEW_REF_M) continue
            if (kept.all { k -> facts(k).refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M }) out += ref
        }
        return out
    }

    /**
     * Le strade (ref) di [v] che il migliore non usa: almeno [minM] metri in [v] e meno di 1 km nel
     * migliore, nell'ordine di [v] (dalla firma). Solo RAM (i fatti delle bozze).
     */
    fun refsNotInBest(v: Draft, minM: Double = VariantRules.NEW_REF_M): List<String> {
        val f = facts(v); val fb = facts(best)
        val out = ArrayList<String>()
        for ((ref, _) in f.signature) {
            if (ref in out) continue
            val id = info.idOf(ref)
            if (id < 0 || f.refMeters.getOrDefault(id, 0.0) < minM) continue
            if (fb.refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M) out += ref
        }
        return out
    }

    /**
     * Le strade per il titolo: [refsNotInBest]; se nessuna ha 5 km, quelle che il migliore non usa
     * con almeno 1 km e un quinto di un tratto deviato (la SR62, 4 km su 16), non un raccordo di 2 km
     * in una deviazione di 34.
     */
    private fun titleRefs(v: Draft): List<String> {
        val refs = refsNotInBest(v)
        if (refs.isNotEmpty()) return refs
        val fb = facts(best)
        val out = ArrayList<String>()
        for (r in runsFromBest(v)) {
            val m = LinkedHashMap<Int, Double>()
            for (i in r.from..r.to) { val id = info.refId(v.edgeIds[i]); if (id >= 0) m.merge(id, v.edgeM[i], Double::plus) }
            for ((id, mm) in m) if (mm >= maxOf(VariantRules.ABSENT_REF_M, TITLE_RUN_SHARE * r.meters) && fb.refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M) {
                val name = info.refName(id)
                if (name !in out) out += name
            }
        }
        return out
    }

    /**
     * Il titolo di [v]: le strade che il migliore non usa ([titleRefs]), al più due nell'ordine della
     * strada, prima quelle che nessuno di [others] usa («via A24», «via A50 + A24»); se il migliore
     * le usa già tutte, la strada (ref) del tratto deviato più lungo «via A1, altro tracciato», o il
     * suo nome se non ha ref (mai una galleria o un ponte), o «altro tracciato». Senza [others]
     * letto una volta per bozza (i nomi sono letture del grafo).
     */
    fun title(v: Draft, others: List<Draft> = emptyList()): String {
        val base = v.baseTitle ?: titleRefs(v).let { refs -> if (refs.isNotEmpty()) "via " + refs.take(2).joinToString(" + ") else roadTitle(v) }.also { v.baseTitle = it }
        if (others.isEmpty()) return base
        val refs = titleRefs(v)
        if (refs.size <= 2) return base
        val own = refs.filter { ref -> val id = info.idOf(ref); others.all { o -> facts(o).refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M } }
        val pick = (own + refs.filter { it !in own }).take(2).toSet()
        return "via " + refs.filter { it in pick }.joinToString(" + ")
    }

    /**
     * Il titolo quando [titleRefs] è vuoto: la strada (ref) del tratto deviato più lungo, se copre
     * almeno 1 km e un quinto del tratto come in [titleRefs] («via A1, altro tracciato» se il migliore
     * la usa già), se no il nome più lungo del tratto (mai una galleria o un ponte), se no «altro
     * tracciato»: non un raccordo di 2 km in una deviazione di 34, non 80 m della SS9 attraversata.
     */
    private fun roadTitle(v: Draft): String {
        val major = runsFromBest(v).maxByOrNull { it.meters } ?: return "altro tracciato"
        val byRef = IntDoubleHashMap()
        for (i in major.from..major.to) { val r = info.refId(v.edgeIds[i]); if (r >= 0) byRef.addTo(r, v.edgeM[i]) }
        var bestRef = -1; var bestM = 0.0
        for (c in byRef) if (c.value > bestM) { bestM = c.value; bestRef = c.key }
        if (bestRef >= 0 && bestM >= maxOf(VariantRules.ABSENT_REF_M, TITLE_RUN_SHARE * major.meters)) {
            val inBest = facts(best).refMeters.getOrDefault(bestRef, 0.0) >= VariantRules.ABSENT_REF_M
            return "via ${info.refName(bestRef)}" + if (inBest) ", altro tracciato" else ""
        }
        // nessun ref: il nome più lungo del tratto che non sia una galleria o un ponte (letto solo qui)
        val byName = HashMap<String, Double>()
        for (i in major.from..major.to) { val n = graph.name(v.edgeIds[i]); if (n.isNotEmpty() && !VariantRules.isStructureName(n)) byName.merge(n, v.edgeM[i], Double::plus) }
        return byName.maxByOrNull { it.value }?.let { "via ${it.key}" } ?: "altro tracciato"
    }

    /**
     * I titoli delle varianti [vs] (nell'ordine della risposta), tutti diversi fra loro e dal
     * migliore ([bestLabel]): [title] rispetto alle altre; per un titolo già usato una strada in più
     * dei tratti deviati che né il migliore né le altre usano (nell'ordine della strada), poi il km
     * del migliore dove la deviazione più lunga esce.
     */
    fun titles(vs: List<Draft>): List<String> {
        val others = vs.map { v -> vs.filter { it !== v } }
        val base = vs.indices.map { title(vs[it], others[it]) }
        return VariantRules.distinct(base, { i -> alternativeTitles(vs[i], others[i], base[i]) }, listOf(bestLabel()))
    }

    private fun alternativeTitles(v: Draft, others: List<Draft>, t: String): List<String> {
        val out = ArrayList<String>()
        val runs = runsFromBest(v)
        if (t.startsWith("via ") && ',' !in t) {
            val shown = t.removePrefix("via ").split(" + ")
            val m = LinkedHashMap<Int, Double>()
            for (r in runs) for (i in r.from..r.to) { val id = info.refId(v.edgeIds[i]); if (id >= 0) m.merge(id, v.edgeM[i], Double::plus) }
            val fb = facts(best)
            m.entries.firstOrNull { (id, mm) -> mm >= VariantRules.ABSENT_REF_M && info.refName(id) !in shown && fb.refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M &&
                    others.all { o -> facts(o).refMeters.getOrDefault(id, 0.0) < VariantRules.ABSENT_REF_M } }
                ?.let { extra ->
                    // nell'ordine della strada (i tratti deviati sono in ordine): una strada che non c'è resta dov'era
                    val order = m.keys.map { info.refName(it) }
                    out += "via " + (shown.toList() + info.refName(extra.key)).sortedBy { r -> order.indexOf(r).let { if (it < 0) Int.MAX_VALUE else it } }.joinToString(" + ")
                }
        }
        runs.maxByOrNull { it.meters }?.let { out += "$t, ${VariantRules.fromKm(bestKmAt(it).first)}" }
        return out
    }

    /**
     * Vero se [v] devia solo vicino ai punti del viaggio: ogni suo tratto deviato dal migliore
     * ([runsB]) di almeno [VariantRules.LOCAL_RUN_M] sta dentro i cerchi d'accesso ([circles]; un
     * nodo ogni [VariantRules.LOCAL_SAMPLE_M] di strada), nessuna autostrada che il migliore non usa
     * ([motorwayRefs] fra [refsNotInBest]) e un guadagno di quota autostradale ([motorwayGain]) sotto
     * [VariantRules.LOCAL_MAX_GAIN]: un altro modo di uscire dalla città, non un altro corridoio.
     * Senza cerchi mai. Una volta per bozza: le coordinate di pochi nodi, solo se le strade non bastano.
     */
    fun localOnly(v: Draft, runsB: List<Run> = runsFromBest(v), d1: Double = VariantRules.dMin1(tripMeters) * scale): Boolean {
        v.localOnly?.let { return it }
        val local = circles.isNotEmpty() && runsB.any { it.meters >= VariantRules.LOCAL_RUN_M } &&
            motorwayRefs(v, refsNotInBest(v)).isEmpty() && motorwayGain(runsB, d1) < VariantRules.LOCAL_MAX_GAIN &&
            runsB.all { r -> r.meters < VariantRules.LOCAL_RUN_M || inCircles(v, r) }
        v.localOnly = local
        return local
    }

    private fun inCircles(v: Draft, r: Run): Boolean {
        var acc = 0.0
        for (i in r.from..r.to) {
            acc += v.edgeM[i]
            if (acc < VariantRules.LOCAL_SAMPLE_M && i < r.to) continue
            acc = 0.0
            val n = v.nodeAt(i)
            if (!Circle.inAny(circles, graph.lat(n), graph.lon(n), MotorwayNet.KIND_MAIN)) return false
        }
        return true
    }

    /** Le strade (ref) con almeno 1 km nel tratto [r] di [v], in ordine. */
    fun refsIn(v: Draft, r: Run): List<String> {
        val m = LinkedHashMap<Int, Double>()
        for (i in r.from..r.to) { val id = info.refId(v.edgeIds[i]); if (id >= 0) m.merge(id, v.edgeM[i], Double::plus) }
        return m.filter { it.value >= VariantRules.ABSENT_REF_M }.keys.map { info.refName(it) }
    }

    /** L'etichetta del migliore: le sue due strade più lunghe, in ordine. */
    fun bestLabel(): String {
        val sig = facts(best).signature
        val top = sig.sortedByDescending { it.second }.take(2).map { it.first }.toSet()
        val refs = sig.map { it.first }.filter { it in top }.distinct()
        return if (refs.isEmpty()) "percorso migliore" else "via " + refs.joinToString(" + ")
    }
}

/**
 * I candidati ammessi finora e la scelta golosa e deterministica fra loro: dentro un livello prima
 * le fonti nell'ordine (M, P, V), poi il punteggio più basso; a ogni scelta si rivalutano gli altri
 * rispetto ai nuovi tenuti. Le varianti solo locali ([Verdict.local]) solo nell'ultima passata
 * ([admit] con `local`), dopo tutte le altre: restano se non c'è altro.
 */
internal class VariantPool(val judge: VariantJudge, private val opt: NavAlternativesOptions) {
    val best: Draft get() = judge.best
    val want: Int = opt.count - 1
    private val seen = HashSet<Long>().also { it += judge.best.hash }
    val drafts = ArrayList<Draft>()
    val accepted = ArrayList<Pair<Draft, Verdict>>()
    var invalid = 0; private set
    var duplicates = 0; private set
    val kept: List<Draft> get() = listOf(best) + accepted.map { it.first }
    val full: Boolean get() = accepted.size >= want
    /** I percorsi calcolati e non tenuti. */
    val rejected: List<Draft> get() = drafts.filter { d -> accepted.none { it.first === d } }

    /** Aggiunge [d] (se valido e nuovo) e tiene quelli di livello 1 finché ne servono. */
    fun offer(d: Draft): Boolean {
        if (!seen.add(d.hash)) { duplicates++; return false }
        if (!judge.valid(d)) { invalid++; return false }
        drafts += d
        admit(1)
        return true
    }

    /** Come [offer] per più candidati insieme (una fonte che ne trova molti): una scelta sola alla fine. */
    fun offerAll(batch: List<Draft>): Int {
        var n = 0
        for (d in batch) {
            if (!seen.add(d.hash)) { duplicates++; continue }
            if (!judge.valid(d)) { invalid++; continue }
            drafts += d; n++
        }
        admit(1)
        return n
    }

    /**
     * Tiene, fra i non tenuti, quelli di livello ≤ [maxTier] finché ne servono; quelli solo locali
     * ([Verdict.local]) solo con [local], e dopo gli altri. [stop] si controlla a ogni valutazione:
     * fermata (lancia [SearchCancelledException]), restano i tenuti fino a lì.
     */
    fun admit(maxTier: Int, stop: CancelToken = CancelToken.NEVER, local: Boolean = false) {
        val tl = traceLog
        while (!full) {
            val k = kept
            var pick: Draft? = null; var pickV: Verdict? = null
            // con la traccia: i candidati in gara in questo giro (solo raccolti, la scelta è la stessa)
            val race = if (tl != null) ArrayList<Pair<Draft, Verdict>>() else null
            for (d in rejected) {
                stop.check()
                val v = judge.evaluate(d, k, relaxed = maxTier >= 3)
                d.verdict = v
                if (v.tier !in 1..maxTier) continue
                race?.add(d to v)
                if (v.local && !local) continue
                val pv = pickV
                if (pv == null || before(d, v, pick!!, pv)) { pick = d; pickV = v }
            }
            if (tl != null && race != null && race.isNotEmpty()) {
                val order = race.sortedWith(compareBy<Pair<Draft, Verdict>>({ it.second.local }, { it.second.tier }, { it.first.source.ordinal }, { it.second.score }))
                tl("varianti: traccia scelta ${accepted.size + 1} (livelli ≤ $maxTier${if (local) ", anche solo locali" else ""}, sui tenuti di ora): " + order.take(TRACE_RACE).joinToString("; ") { (d, v) ->
                    (if (d === pick) "PRESA " else "") + d.tag.ifEmpty { d.source.json } + " liv. ${v.tier} punteggio %.3f".format(v.score) + (if (v.newRefs.isEmpty()) "" else " nuove " + v.newRefs.joinToString(" ")) +
                        (if (v.local) " solo locale" else "")
                } + (if (order.size > TRACE_RACE) "; altri ${order.size - TRACE_RACE}" else ""))
            }
            if (pick == null) return
            accepted += pick to pickV!!
        }
    }

    /**
     * Le tenute nell'ordine della risposta (rank): prima le non solo locali, poi per livello, poi
     * nell'ordine di presa. Una solo locale di livello 1 presa nell'ultima passata resta dopo le
     * varianti di corridoio di livello 2, come nella scelta (e come tra Paesi).
     */
    fun ordered(): List<Pair<Draft, Verdict>> =
        accepted.withIndex().sortedWith(compareBy({ it.value.second.local }, { it.value.second.tier }, { it.index })).map { it.value }

    /** Vero se [d] ([v]) viene prima di [p] ([pv]) nella scelta: non solo locale, livello, fonte, punteggio. */
    private fun before(d: Draft, v: Verdict, p: Draft, pv: Verdict): Boolean = when {
        v.local != pv.local -> !v.local
        v.tier != pv.tier -> v.tier < pv.tier
        d.source != p.source -> d.source.ordinal < p.source.ordinal
        else -> v.score < pv.score
    }

    /** La traccia delle varianti: i giri della scelta golosa ([admit]); null: spenta. */
    internal var traceLog: ((String) -> Unit)? = null

    private companion object {
        /** I candidati scritti al più per giro della scelta nella traccia. */
        const val TRACE_RACE = 12
    }

    /**
     * "scarti: tempo 5, diversità 6, locale 1, doppione 2" con i motivi dell'ultima valutazione. Alla
     * scadenza di [stop] i candidati restanti si contano come "non valutati" (un annullamento lancia).
     */
    fun discards(stop: CancelToken = CancelToken.NEVER): String {
        val k = kept
        val counts = LinkedHashMap<String, Int>()
        val rest = rejected
        var done = 0
        try {
            for (d in rest) { stop.check(); val v = judge.evaluate(d, k, relaxed = true); counts.merge(if (v.tier == 0) v.reason else "riserva", 1, Int::plus); done++ }
        } catch (e: SearchCancelledException) {
            if (e.reason != SearchCancelledException.Reason.DEADLINE) throw e
            counts["non valutati"] = rest.size - done
        }
        if (duplicates > 0) counts.merge("doppione", duplicates, Int::plus)
        if (invalid > 0) counts["non valido"] = invalid
        return if (counts.isEmpty()) "scarti: nessuno" else "scarti: " + counts.entries.joinToString(", ") { "${it.key} ${it.value}" }
    }
}

/** Una fonte di varianti dell'orchestratore: propone [Draft] al [VariantPool] finché servono. */
internal interface VariantProvider {
    val source: VariantSource
    /** Vero se la fonte vale per questa richiesta. */
    fun supported(ctx: VariantContext): Boolean
    /**
     * Il lavoro prima del tempo dei candidati (la rete autostradale, poi in cache: una volta sola per
     * viaggio, anche se il tempo dei candidati è breve); lo ferma solo l'annullamento della richiesta.
     * Il suo tempo si toglie da quello delle fonti ([NavAlternativesEngine.sourceBudgetMs]).
     * Chiamata per le fonti chieste, anche non [supported].
     */
    fun prepare(ctx: VariantContext) {}
    /** Propone i suoi candidati a [VariantContext.pool]; si ferma quando è pieno o a [budget] scaduto (lancia). */
    fun run(ctx: VariantContext, budget: CancelToken)
    /** Il riepilogo per `candidates` ("P 4"). */
    fun summary(): String
    /** Perché la fonte ha dato meno del possibile ("rete autostradale troncata"), se lo sa. */
    fun note(): String? = null
}

/** Quello che le fonti di varianti condividono in una richiesta. */
internal class VariantContext(
    val service: DirectionsService, val rc: RequestContext, val request: DirectionsRequest, val solved: DirectionsService.Solved,
    val opt: NavAlternativesOptions, val info: EdgeInfo, val corridors: CorridorAnalyzer, val trace: BestTrace, val pool: VariantPool,
    val router: Router, val log: (String) -> Unit, val trip: TripContext = TripContext.NONE,
) {
    /** Le misure dei tratti delle varianti (fonte e ricerca). */
    val stats: RequestStats by lazy { rc.stats.child("varianti") }

}

/** Un posto da aiutante per [rc] (executor e posti presenti, uno libero; senza attesa), figlio di [token]; va chiuso. */
internal fun helperSlot(rc: RequestContext, token: CancelToken): SearchSlots.Helper? =
    if (rc.executor != null) rc.slots?.tryHelper(rc.priority, token) else null

/**
 * [a] su questo thread e, col posto da aiutante [slot] ([helperSlot]), [b] sull'executor di [rc] col
 * token del posto; senza posto [b] dopo [a], col token [token]. Un aiutante prelazionato (o mai
 * partito) lascia [b] a questo thread. Esce con l'aiutante fermo e [slot] chiuso.
 */
internal fun <A, B> onTwoThreads(rc: RequestContext, token: CancelToken, slot: SearchSlots.Helper?, a: (CancelToken) -> A, b: ((CancelToken) -> B)?): Pair<A, B?> {
    val ex = rc.executor
    if (b == null || slot == null || ex == null) {
        slot?.close()
        val ra = a(token)
        return ra to b?.invoke(token)
    }
    val claimed = AtomicBoolean(false)
    val done = CountDownLatch(1)
    var rb: Any? = null
    var err: Throwable? = null
    val submitted = try {
        ex.execute {
            if (claimed.compareAndSet(false, true)) try { rb = slot.run { b(slot.token) } } catch (t: Throwable) { err = t } finally { slot.close(); done.countDown() }
        }
        true
    } catch (e: java.util.concurrent.RejectedExecutionException) { false }
    if (!submitted) slot.close()
    val ra = try { a(token) } catch (t: Throwable) {
        slot.token.cancel()
        if (submitted && !claimed.compareAndSet(false, true)) awaitQuietly(done)
        slot.close()
        throw t
    }
    val helperRan = submitted && !claimed.compareAndSet(false, true)
    if (helperRan) awaitQuietly(done)
    slot.close()
    if (!helperRan) return ra to b(token)
    val e = err
    if (e != null) {
        if (e is SearchCancelledException && e.reason == SearchCancelledException.Reason.PREEMPTED && !token.isCancelled) return ra to b(token)
        throw e
    }
    @Suppress("UNCHECKED_CAST")
    return ra to (rb as B)
}

private fun awaitQuietly(l: CountDownLatch) {
    var interrupted = false
    while (true) { try { l.await(); break } catch (e: InterruptedException) { interrupted = true } }
    if (interrupted) Thread.currentThread().interrupt()
}

/**
 * Un cerchio d'accesso per punto del viaggio ([Circle]): raggio [NavAlternativesEngine.ACCESS_RADIUS_M],
 * al più [NavAlternativesEngine.ACCESS_LEG_SHARE] del tratto più lungo che tocca il punto (in un
 * viaggio corto le strade principali non diventano tutta la rete). Un punto ripetuto (andata e
 * ritorno) una volta; nessun cerchio per un punto agganciato a un'autostrada o una superstrada (un
 * ricalcolo in marcia: si è già sulla rete, le strade urbane attorno non servono). Per la rete della
 * fonte M e per le varianti solo locali ([VariantJudge.localOnly]).
 * Con [trip] (un pezzo di un viaggio tra Paesi) nessun cerchio sui punti di trasferimento al
 * confine (il primo se il pezzo non apre il viaggio, l'ultimo se non lo chiude): una deviazione
 * attorno a un valico non è un'uscita dalla città. Senza [trip] (la rete M) tutti i punti.
 */
internal fun accessCircles(solved: DirectionsService.Solved, info: EdgeInfo, trip: TripContext? = null): List<Circle> {
    val out = ArrayList<Circle>()
    val last = solved.snaps.size - 1
    for ((j, s) in solved.snaps.withIndex()) {
        if (trip != null && ((j == 0 && trip.offsetMeters > 0.5) || (j == last && trip.extraMeters - trip.offsetMeters > 0.5))) continue
        if (s.edge >= 0 && info.isMotorwayOrTrunk(s.edge)) continue
        val legM = maxOf(solved.legs.getOrNull(j - 1)?.distance ?: 0.0, solved.legs.getOrNull(j)?.distance ?: 0.0)
        val r = minOf(NavAlternativesEngine.ACCESS_RADIUS_M, NavAlternativesEngine.ACCESS_LEG_SHARE * legM)
        if (r < 1.0) continue
        val c = Circle(s.snappedLat, s.snappedLon, r)
        if (out.none { it.covers(c) }) out += c
    }
    return out
}

/**
 * Le varianti del percorso migliore di una richiesta (`navmaster.alternatives`): il migliore dalla
 * cache dei tratti ([DirectionsService.solve]), poi le fonti in ordine (M: deviazioni autostradali,
 * passo 6; P: penalità, [PenaltyCandidates]) finché servono varianti di livello 1, entro il tempo
 * [NavAlternativesOptions.budgetMs] meno la preparazione ([sourceBudgetMs]); poi il riempimento
 * (livello 2, e 3 con `fill`), al più [FILL_GRACE_MS] oltre quel tempo. Solo le tenute
 * si completano (geometria e nomi). Righe di registro `varianti: …`.
 */
internal class NavAlternativesEngine(private val service: DirectionsService, private val rc: RequestContext, private val trip: TripContext = TripContext.NONE) {
    companion object {
        /** La regione della rete autostradale per tratto: somma delle distanze dai punti ≤ 1,25 volte il tratto più 20 km. */
        const val REGION_FACTOR = 1.25
        const val REGION_EXTRA_M = 20_000.0
        /** Il margine di peso di una deviazione, riferito al suo tratto, al più questo (un tratto corto in un viaggio lungo). */
        const val MAX_LEG_FACTOR = 3.0
        /**
         * I cerchi d'accesso della rete M ([Circle]): attorno a ogni punto del viaggio le strade
         * primarie e secondarie entro 10 km (le terziarie entro 4 km), al più un quarto del tratto
         * più lungo che tocca il punto. Torino centro → tangenziale A55: ~8 km di strade urbane.
         */
        const val ACCESS_RADIUS_M = 10_000.0
        const val ACCESS_LEG_SHARE = 0.25
        /**
         * Una deviazione M con più di questa quota dei km su strade d'accesso (primarie, secondarie,
         * terziarie dei cerchi) è un giro urbano, non un'altra autostrada: scartata. Le strade
         * d'accesso servono a raggiungere un'altra autostrada dalla parte urbana (Torino centro → A21: 8 km su 225).
         */
        const val MAX_ACCESS_SHARE = 0.5
        /**
         * Gli insiemi di deviazioni della fonte M offerti per richiesta, al più (combinazioni comprese):
         * ogni insieme è un [Draft] con gli archi di tutto il viaggio, e con molti tratti su un viaggio
         * lungo la memoria crescerebbe troppo per il tablet.
         */
        const val MAX_M_SETS = 64
        /** Il tempo minimo delle fonti (se `budget_ms` non è più breve), anche dopo una preparazione lunga. */
        const val MIN_SOURCE_BUDGET_MS = 3_000L
        /** Il riempimento (livelli 2 e 3) e il conto degli scarti dopo la scadenza delle fonti, al più. */
        const val FILL_GRACE_MS = 2_000L
        /** Il tempo della fonte P «in ombra» della traccia ([VariantTrace.shadowPenalty]). */
        const val SHADOW_P_MS = 20_000L

        /**
         * Il tempo delle fonti: `budget_ms` meno il tempo già speso a preparare (il migliore, la rete
         * autostradale), almeno [MIN_SOURCE_BUDGET_MS] (o tutto `budget_ms`, se è più breve).
         */
        internal fun sourceBudgetMs(budgetMs: Long, preparedMs: Long): Long = maxOf(minOf(budgetMs, MIN_SOURCE_BUDGET_MS), budgetMs - preparedMs)

        /**
         * Gli insiemi offerti dalla fonte M: le combinazioni ([combos], dalle prime [DetourSearch.COMBO_TOP]
         * singole, [DetourSearch.comboTop]) e le singole [singles] (già in ordine: [Detour.rankClass],
         * poi punteggio su tutto il viaggio, [DetourSearch.tripScore])
         * fino a [max] in tutto, mai meno delle prime [DetourSearch.COMBO_TOP].
         */
        internal fun offeredSets(singles: List<Detour>, combos: List<List<Detour>>, max: Int = MAX_M_SETS): List<List<Detour>> =
            singles.take(maxOf(DetourSearch.COMBO_TOP, max - combos.size)).map { listOf(it) } + combos
    }

    class Accepted(val draft: Draft, val verdict: Verdict, val rank: Int, val legs: List<RoutePath>)
    class Result(val best: DirectionsService.Solved, val variants: List<Accepted>, val note: String?, val summary: String,
                 internal val judge: VariantJudge, internal val bestDraft: Draft, val ms: Long) {
        /** `properties.navmaster.variant` del migliore: rank 0, livello 0. */
        fun bestJson(): Map<String, Any?> {
            val sig = judge.facts(bestDraft).signature
            return linkedMapOf("rank" to 0, "tier" to 0, "source" to VariantSource.BEST.json, "label" to judge.bestLabel(), "diff" to "",
                "signature" to sig.joinToString(" · ") { it.first }, "refs" to sig.map { linkedMapOf("ref" to it.first, "km" to it.second) },
                "new_refs" to emptyList<String>(), "extra_s" to 0.0, "extra_m" to 0.0, "factor" to 1.0, "km_factor" to 1.0, "longer" to false,
                "detours" to emptyList<Any>())
        }

        /** `properties.navmaster.variant` della variante [a]. */
        fun json(a: Accepted): Map<String, Any?> {
            val d = a.draft; val b = bestDraft
            val sig = judge.facts(d).signature
            val extraS = (d.millis - b.millis) / 1000.0
            val extraM = d.meters - b.meters
            // i fattori sul viaggio intero (un pezzo di un viaggio tra Paesi)
            val t = judge.trip
            val detours = judge.runsFromBest(d).filter { it.ms >= 60_000 || it.meters >= 1_000 }.map { r ->
                val (fromKm, toKm) = judge.bestKmAt(r)
                linkedMapOf("from_km" to round1(fromKm), "to_km" to round1(toKm), "km" to round1(r.meters / 1000),
                    "refs" to judge.refsIn(d, r), "extra_s" to round1((r.ms - judge.bestMsBetween(r)) / 1000.0))
            }
            return linkedMapOf("rank" to a.rank, "tier" to a.verdict.tier, "source" to d.source.json, "label" to a.verdict.label, "diff" to diff(extraS, extraM),
                "signature" to sig.joinToString(" · ") { it.first }, "refs" to sig.map { linkedMapOf("ref" to it.first, "km" to it.second) },
                "new_refs" to a.verdict.newRefs, "extra_s" to round1(extraS), "extra_m" to round1(extraM),
                "factor" to Math.round((d.millis + t.extraMillis) * 1000.0 / maxOf(1L, judge.tripMillis)) / 1000.0,
                "km_factor" to Math.round((d.meters + t.extraMeters) * 1000.0 / maxOf(1.0, judge.tripMeters)) / 1000.0,
                "longer" to (a.verdict.tier >= 2 && d.millis + t.extraMillis > judge.tripMillis * judge.tier1Factor), "detours" to detours)
        }

        private fun round1(v: Double) = Math.round(v * 10) / 10.0

        /** "+4 min, −6 km". */
        private fun diff(extraS: Double, extraM: Double): String {
            val min = Math.round(extraS / 60).toInt(); val km = Math.round(extraM / 1000).toInt()
            fun signed(v: Int) = if (v < 0) "−${-v}" else "+$v"
            return "${signed(min)} min, ${signed(km)} km"
        }
    }

    private val log: (String) -> Unit = { line -> (rc.log ?: service.alternativesLog)?.invoke(line) }
    /**
     * Con la traccia: i candidati M giudicati ma non offerti (oltre `perLeg` o oltre il tetto degli
     * insiemi), cuciti solo alla fine ([traceEnd]) per non togliere tempo alle fonti.
     */
    private val traceDrafts = ArrayList<() -> Draft>()

    /** Le fonti nell'ordine in cui si provano: M (deviazioni autostradali in RAM), poi P (penalità). */
    private fun providers(): List<VariantProvider> = listOf(MotorwayProvider(), PenaltyProvider())

    /** Il migliore di [request] ([DirectionsService.solve]: dalla cache dei tratti se c'è) e quello che le fonti condividono. */
    internal fun prepare(request: DirectionsRequest, opt: NavAlternativesOptions): VariantContext {
        val solved = service.solve(request.withoutNavAlternatives(), rc)
        val graphs = service.routingGraph
        val info = EdgeInfo(graphs.graph, graphs.encoding.shared, graphs.guidance)
        val corridors = CorridorAnalyzer(graphs.graph, info, graphs.guidance)
        val router = graphs.router(solved.options)
        val best = Draft(solved.legs, VariantSource.BEST)
        val judge = VariantJudge(best, info, corridors, opt, graphs.graph, router, request, solved.snaps, trip, accessCircles(solved, info, trip))
        return VariantContext(service, rc, request, solved, opt, info, corridors, BestTrace(solved.legs), VariantPool(judge, opt).also { if (opt.trace != null) it.traceLog = log }, router, log, trip)
    }

    fun run(request: DirectionsRequest, opt: NavAlternativesOptions): Result = rc.searching {
        val t0 = System.nanoTime()
        // 1. il migliore: nessuna ricerca se l'app lo ha appena calcolato (cache dei tratti)
        val ctx = prepare(request, opt)
        val solved = ctx.solved; val pool = ctx.pool; val judge = pool.judge; val best = judge.best; val router = ctx.router
        // 2. la preparazione delle fonti (la rete autostradale, mai fermata dal tempo: una rete a metà
        // non resterebbe in cache), poi il tempo dei candidati: budget_ms meno la preparazione, con un minimo
        val ps = providers()
        for (p in ps) if (p.source in opt.sources) p.prepare(ctx)
        val preparedMs = (System.nanoTime() - t0) / 1_000_000
        val sourceMs = sourceBudgetMs(opt.budgetMs, preparedMs)
        val deadline = System.nanoTime() + sourceMs * 1_000_000
        val budget = rc.cancel.child(deadline)
        if (preparedMs >= 1_000) log("varianti: preparazione $preparedMs ms, alle fonti $sourceMs ms")
        var note: String? = null
        val used = ArrayList<VariantProvider>()
        // 3. le fonti, finché servono varianti di livello 1
        try {
            for ((pi, p) in ps.withIndex()) {
                if (pool.full) {
                    if (opt.trace != null) for (q in ps.drop(pi)) if (q.source in opt.sources)
                        log("varianti: traccia: ${q.source.json} non provata, pool pieno (${pool.accepted.count { it.second.tier == 1 }} di livello 1 su ${pool.want})")
                    break
                }
                if (p.source !in opt.sources || !p.supported(ctx)) continue
                used += p
                p.run(ctx, budget)
            }
        } catch (e: SearchCancelledException) {
            // il tempo dei candidati è finito: si risponde con quel che c'è; un annullamento della richiesta esce
            if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
            note = "budget"
        }
        // 4. riempimento: livello 2, poi 3, al più FILL_GRACE_MS dopo la scadenza delle fonti (poi restano i tenuti fin lì)
        val fillStop = rc.cancel.child(deadline + FILL_GRACE_MS * 1_000_000)
        try {
            pool.admit(2, fillStop)
            if (opt.fill) pool.admit(3, fillStop)
            // le varianti solo locali (un altro modo di uscire dalla città) solo se non c'è altro
            pool.admit(if (opt.fill) 3 else 2, fillStop, local = true)
        } catch (e: SearchCancelledException) {
            if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
            note = "budget"
        }
        if (pool.accepted.size < pool.want && note == null) note = used.firstNotNullOfOrNull { it.note() } ?: when {
            used.isEmpty() -> "nessuna fonte di varianti per questa richiesta"
            else -> "nessun'altra strada entro +${Math.round((opt.maxFactor - 1) * 100)} %"
        }
        // 5. le tenute complete (geometria e nomi), nell'ordine della risposta
        val ordered = pool.ordered()
        // i titoli sulle altre varianti della risposta: prima le strade che il migliore non usa, tutti diversi
        val titles = judge.titles(ordered.map { it.first })
        val variants = ordered.mapIndexed { i, (d, v) -> Accepted(d, v.withLabel(titles[i]), i + 1, d.legs.map { router.materialize(it) }) }
        val summary = (used.joinToString(", ") { it.summary() }.ifEmpty { "nessuna fonte" }) + " | " + pool.discards(fillStop)
        val ms = (System.nanoTime() - t0) / 1_000_000
        log("varianti: ${variants.size}/${pool.want} in $ms ms ($summary)" + (note?.let { "; $it" } ?: ""))
        rc.stats.variants = "${variants.size}/${pool.want} in $ms ms ($summary)"
        opt.trace?.let { traceEnd(ctx, it, used) }
        Result(solved, variants, note, summary, judge, best, ms)
    }

    private fun verdictText(v: Verdict): String =
        if (v.tier > 0) "livello ${v.tier}" + (if (v.reason.isNotEmpty()) " (${v.reason})" else "") + ", punteggio %.3f".format(v.score) else "scartata (${v.reason})"

    /**
     * La fine della traccia ([VariantTrace]), dopo la scelta: ogni candidato (offerto o no) giudicato
     * sulle tenute e sul solo migliore coi valori delle regole, la fonte P in ombra se non è stata
     * provata, le sonde. Nessun effetto sulla risposta.
     */
    private fun traceEnd(ctx: VariantContext, tr: VariantTrace, used: List<VariantProvider>) {
        val pool = ctx.pool; val judge = pool.judge; val best = judge.best
        val kept = pool.kept
        val names = listOf("migliore") + pool.accepted.mapIndexed { i, a -> "T${i + 1} ${a.second.label}" }
        for ((i, a) in pool.accepted.withIndex())
            log("varianti: traccia tenuta T${i + 1}: " + a.first.tag.ifEmpty { a.first.source.json } + " (livello ${a.second.tier}, ${a.second.label}, punteggio %.3f)".format(a.second.score))
        fun line(d: Draft, what: String) {
            val acc = pool.accepted.indexOfFirst { it.first === d }
            val esito = if (acc >= 0) "tenuta T${acc + 1}" else verdictText(judge.evaluate(d, kept, relaxed = true))
            val alone = verdictText(judge.evaluate(d, listOf(best), relaxed = false))
            // una tenuta si spiega sulle altre tenute (con sé stessa la deviazione sarebbe nulla)
            val others = kept.indices.filter { kept[it] !== d }
            log("varianti: traccia esito " + d.tag.ifEmpty { d.source.json } + what + ": " + esito + "; col solo migliore " + alone + "; " +
                judge.explain(d, others.map { kept[it] }, others.map { names[it] }))
        }
        for (d in pool.drafts) line(d, "")
        for (make in traceDrafts) line(make(), " (non offerta)")
        if (tr.shadowPenalty && VariantSource.PENALTY in ctx.opt.sources && used.none { it.source == VariantSource.PENALTY }) shadowPenalty(ctx, names)
        for (p in tr.probes) {
            if (p.legs.isEmpty()) continue
            val v = Draft(p.legs, VariantSource.PENALTY, tag = "sonda ${p.name}")
            val vk = judge.evaluate(v, kept, relaxed = true); val vb = judge.evaluate(v, listOf(best), relaxed = false)
            log("varianti: traccia sonda " + p.name + ": %.0f min, %.1f km".format(v.millis / 60000.0, v.meters / 1000) + "; con le tenute " + verdictText(vk) +
                "; col solo migliore " + verdictText(vb) + "; " + judge.explain(v, kept, names))
        }
    }

    /**
     * La fonte P «in ombra» (traccia, pool già pieno dopo M): i suoi candidati come li avrebbe
     * provati dopo M, giudicati sulle tenute ma mai offerti; al più [SHADOW_P_MS].
     */
    private fun shadowPenalty(ctx: VariantContext, names: List<String>) {
        val pc = PenaltyCandidates(service, ctx.solved, rc, log, ctx)
        if (!pc.supported()) { log("varianti: traccia P in ombra: non vale per questa richiesta"); return }
        log("varianti: traccia P in ombra (il pool era pieno: candidati giudicati sulle tenute, non offerti)")
        val pool = ctx.pool; val judge = pool.judge
        val token = rc.cancel.child(System.nanoTime() + SHADOW_P_MS * 1_000_000)
        val rejected = ArrayList(pool.rejected)
        val hashes = HashSet<Long>().also { h -> for (d in pool.kept) h += d.hash; for (d in pool.drafts) h += d.hash }
        var n = 0
        try {
            while (n < pc.maxCandidates) {
                val c = pc.next(pool.kept, rejected) ?: break
                n++
                val r = pc.run(c, token)
                pc.record(r)
                val d = r.draft
                val esito = when {
                    d == null -> ""
                    !hashes.add(d.hash) -> " → doppione"
                    !judge.valid(d) -> " → non valido"
                    else -> {
                        rejected += d
                        val v = judge.evaluate(d, pool.kept, relaxed = true); d.verdict = v
                        " → " + verdictText(v) + "; col solo migliore " + verdictText(judge.evaluate(d, listOf(judge.best), relaxed = false)) + "; " + judge.explain(d, pool.kept, names)
                    }
                }
                log("varianti: traccia P in ombra: " + r.line.removePrefix("varianti: ") + esito)
            }
        } catch (e: SearchCancelledException) {
            if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
            log("varianti: traccia P in ombra: tempo finito dopo $n candidati")
        }
    }

    /**
     * La fonte M: deviazioni del migliore sulla rete autostradale in RAM ([MotorwayNet], dalla cache
     * del servizio se copre il viaggio; [DetourSearch]), singole e combinate, cucite nel migliore
     * ([Router.splice], leggere) senza nessuna ricerca. Non vale se la richiesta evita le autostrade.
     */
    private inner class MotorwayProvider : VariantProvider {
        override val source = VariantSource.MOTORWAY
        private var singles = 0
        private var combos = 0
        private var truncated = false

        override fun supported(ctx: VariantContext): Boolean =
            ctx.request.avoidFeatures and AvoidFeatureFlags.HIGHWAYS == 0 && ctx.trace.node.isNotEmpty()

        /** La rete della richiesta, i nodi usabili ([MotorwayNet.mask]) e il tempo di [prepare]. */
        private var net: MotorwayNet? = null
        private var allowed: BooleanArray? = null
        private var t0 = 0L

        /**
         * I nodi del migliore non autostradali dentro un cerchio d'accesso, come semi: solo le
         * posizioni a non più di 3 raggi di strada dall'inizio o dalla fine di un tratto (fuori
         * nessun nodo può stare nel cerchio della partenza o dell'arrivo), una lettura delle
         * coordinate per nodo, fuori dai cicli delle ricerche.
         */
        private fun addAccessSeeds(ctx: VariantContext, circles: List<Circle>, seeds: com.carrotsearch.hppc.IntArrayList) {
            if (circles.isEmpty()) return
            val g = service.routingGraph.graph
            val trace = ctx.trace
            val have = com.carrotsearch.hppc.IntHashSet(seeds.size() * 2).also { h -> for (k in 0 until seeds.size()) h.add(seeds.get(k)) }
            val maxR = circles.maxOf { it.radiusM }
            for (l in ctx.solved.legs.indices) {
                val s = trace.legStart[l]; val e = trace.legStart[l + 1]
                val legM = ctx.solved.legs[l].distance
                for (p in s until e) {
                    val alongM = trace.preM[p]
                    if (alongM > 3 * maxR && legM - alongM > 3 * maxR) continue
                    val n = trace.node[p]
                    if (have.contains(n)) continue
                    if (Circle.inAny(circles, g.lat(n), g.lon(n), MotorwayNet.KIND_MAIN)) { have.add(n); seeds.add(n) }
                }
            }
        }

        /**
         * La rete autostradale attorno al viaggio: dalla cache del servizio se ne ha una che lo copre
         * (con [RequestContext.useCache]), se no costruita col solo annullamento della richiesta
         * (mai fermata dal tempo dei candidati, che però la conta: una costruzione fermata a metà non
         * resterebbe in cache e le richieste dopo perderebbero le varianti).
         */
        override fun prepare(ctx: VariantContext) {
            if (!supported(ctx)) return
            t0 = System.nanoTime()
            val graphs = service.routingGraph
            val solved = ctx.solved; val trace = ctx.trace
            // i semi: i nodi del migliore su un'autostrada o una superstrada (rampe comprese)
            val seeds = com.carrotsearch.hppc.IntArrayList()
            for (k in trace.node.indices) if (ctx.info.isMotorwayOrTrunk(trace.inEdge[k]) || ctx.info.isMotorwayOrTrunk(trace.outEdge[k])) seeds.add(trace.node[k])
            if (seeds.isEmpty) { log("varianti: M nessun tratto autostradale nel migliore"); return }
            val mwSeeds = seeds.size()
            val mwSeedArr = seeds.toArray()
            // i cerchi d'accesso attorno ai punti, e i nodi del migliore dentro (la parte urbana: deviazioni che escono lì)
            val circles = accessCircles(ctx.solved, ctx.info)
            addAccessSeeds(ctx, circles, seeds)
            val seedArr = seeds.toArray()
            // la regione: un'ellisse per tratto coi fuochi sui suoi punti
            val need = Region(solved.legs.indices.map { j ->
                val a = solved.snaps[j]; val b = solved.snaps[j + 1]
                Ellipse(a.snappedLat, a.snappedLon, b.snappedLat, b.snappedLon, REGION_FACTOR * solved.legs[j].distance + REGION_EXTRA_M)
            })
            val access = ctx.router.access(graphs.encoding.shared)
            var built = false; var accessBuilt = false
            val counts = if (ctx.opt.trace != null) IntArray(4) else null
            // la parte autostradale (dai soli semi autostradali): in cache per i viaggi che la regione copre
            val buildNet = { built = true; MotorwayNet.build(access, mwSeedArr, need, cancel = rc.cancel, counts = counts) }
            val base = if (!rc.useCache) buildNet()
                else service.netCache.get(graphs.routerKey(solved.options), need, fits = { c -> mwSeedArr.all { c.netNode(it) >= 0 } }, cancel = rc.cancel, build = buildNet)
            val tBase = (System.nanoTime() - t0) / 1_000_000
            // le strade d'accesso dei cerchi di questo viaggio, sopra la parte autostradale (letti solo i nodi nei cerchi)
            val buildAccess = { accessBuilt = true; MotorwayNet.withAccess(access, base, seedArr, need, circles, cancel = rc.cancel, counts = counts) }
            val n = when {
                circles.isEmpty() -> base
                !rc.useCache -> buildAccess()
                else -> service.netCache.overlay(base, need, circles, fits = { c -> seedArr.all { c.netNode(it) >= 0 } }, build = buildAccess)
            }
            // una rete della cache per una regione più grande: solo i nodi che una nuova avrebbe
            allowed = if (built) null else n.mask(need, seedArr)
            net = n
            truncated = n.truncated
            val tNet = (System.nanoTime() - t0) / 1_000_000
            log("varianti: M rete ${n.arcCount} archi (${n.accessArcs} d'accesso in ${circles.size} cerchi" + (if (n.accessCapped) ", al tetto" else "") +
                "), ${n.nodeCount} nodi (${n.accessNodes} solo d'accesso), ${seedArr.size} semi (${seedArr.size - mwSeeds} d'accesso), svolte vietate ${n.turnBans?.size() ?: 0}; autostrade " +
                (if (built) "costruite in $tBase ms" else "dalla cache") + ", strade d'accesso " +
                (if (circles.isEmpty()) "nessuna" else if (accessBuilt) "costruite in ${tNet - tBase} ms" else "dalla cache") + (if (n.truncated) " (troncata)" else ""))
            if (ctx.opt.trace != null) traceNet(ctx, n, seedArr, need, if (built) counts else null)
        }

        /** Traccia: i semi, la parte del migliore coperta dalla rete, i versi lasciati fuori, il migliore per classe di strada. */
        private fun traceNet(ctx: VariantContext, n: MotorwayNet, seeds: IntArray, need: Region, counts: IntArray?) {
            val trace = ctx.trace
            var inNet = 0; var first = -1; var last = -1
            for (k in trace.node.indices) if (n.netNode(trace.node[k]) >= 0) { inNet++; if (first < 0) first = k; last = k }
            fun km(k: Int) = if (k < 0) "-" else "%.1f".format(trace.preM[k] / 1000)
            log("varianti: traccia M rete: ${seeds.size} semi, ${trace.node.size} posizioni del migliore, $inNet nella rete (dalla posizione $first al km ${km(first)} alla $last al km ${km(last)}), " +
                "troncata ${if (n.truncated) "sì" else "no"}, maschera ${allowed?.count { !it }?.let { "$it nodi esclusi" } ?: "nessuna"}, regione " +
                need.ellipses.joinToString { "somma %.0f km".format(it.sumM / 1000) } +
                (counts?.let { c -> "; versi lasciati fuori: filtro del veicolo ${c[0]}, fuori regione ${c[1]}, peso infinito nei due versi ${c[2]}, in avanti ${c[3]}" } ?: "; versi lasciati fuori: rete dalla cache, non contati"))
            // il migliore per corse della stessa classe (MW autostrada, TR superstrada, altro), con le strade
            val runs = ArrayList<Pair<Double, String>>()
            var at = 0.0
            for (leg in ctx.solved.legs) {
                var cls = ""; var from = at; var m = 0.0
                val refs = LinkedHashMap<String, Double>()
                fun close() {
                    if (cls.isEmpty() || m <= 0) return
                    runs += m to cls + " km %.1f–%.1f (%.1f km".format(from / 1000, (from + m) / 1000, m / 1000) + (if (refs.isEmpty()) "" else ", " + refs.entries.sortedByDescending { it.value }.take(3).joinToString(" ") { it.key }) + ")"
                }
                for (e in leg.edges) {
                    val c = when { ctx.info.isMotorway(e.edge) -> "MW"; ctx.info.isMotorwayOrTrunk(e.edge) -> "TR"; else -> "altro" }
                    if (c != cls) { close(); cls = c; from = at; m = 0.0; refs.clear() }
                    m += e.distance; at += e.distance
                    ctx.info.ref(e.edge)?.let { refs.merge(it, e.distance, Double::plus) }
                }
                close()
            }
            // le corse sotto 1 km (rampe, rotonde) solo contate
            val shown = runs.filter { it.first >= 1000.0 }.map { it.second }
            log("varianti: traccia migliore per classe: " + shown.joinToString(" → ") + (if (shown.size < runs.size) " (+${runs.size - shown.size} corse sotto 1 km)" else ""))
        }

        override fun run(ctx: VariantContext, budget: CancelToken) {
            val net = net ?: return
            val graphs = service.routingGraph
            val solved = ctx.solved; val trace = ctx.trace; val router = ctx.router
            val access = router.access(graphs.encoding.shared)
            // le deviazioni per tratto: il margine di peso è quello del viaggio (1,2 volte max_factor − 1), riferito al tratto
            val search = DetourSearch(net, trace, access.turnCosts, allowed)
            // il margine e la deviazione minima sul viaggio intero (con gli altri pezzi di un viaggio tra Paesi)
            val w0 = trace.legW.sum() + ctx.trip.extraWeight
            val scale = ctx.opt.minDetourScale
            val d1 = scale * VariantRules.dMin1(ctx.pool.best.meters + ctx.trip.extraMeters)
            val minM = maxOf(scale * VariantRules.SEARCH_D_MIN, VariantRules.SEARCH_D * d1)
            val all = ArrayList<Detour>()
            val diag = ctx.opt.trace?.let { tr ->
                DetourTrace(log, { e -> ctx.info.ref(e) }, tr.probes.map { p ->
                    val es = p.legs.flatMap { it.edges }
                    DetourTrace.Watch(p.name, IntArray(es.size) { es[it].edge }, IntArray(es.size) { es[it].adjNode }, DoubleArray(es.size) { es[it].distance })
                }, maxOf(tr.maxDetoursPerLeg, DetourSearch.PER_LEG))
            }
            for (j in solved.legs.indices) {
                budget.check()
                val lw = trace.legW[j]
                if (lw <= 0) continue
                val factor = minOf(MAX_LEG_FACTOR, 1 + PenaltyCandidates.CAP_SLACK * (ctx.opt.maxFactor - 1) * w0 / lw)
                // le deviazioni più corte di dMin1 del viaggio dopo le lunghe, solo nell'ordine dei posti
                // fuori le deviazioni quasi tutte su strade d'accesso e quelle che il giudice non terrebbe mai (sanità del livello 3)
                all += search.detours(j, factor, cancel = budget, minDetourM = minM, diag = diag, rankMinM = d1, maxAccessShare = MAX_ACCESS_SHARE,
                    saneA = VariantRules.T3_A, saneBMs = VariantRules.T3_B_MS * scale)
            }
            // un tetto per richiesta: le prime per punteggio su tutto il viaggio (le combinazioni vengono
            // dalle prime 8). Prima l'ordine era quello di Detour.score, riferito al proprio tratto:
            // oltre il tetto cadevano per prime le deviazioni dei tratti corti, anche con pochi minuti
            // in più sul viaggio, e restavano quelle dei tratti lunghi con più minuti
            val tripW = trace.legW.sum()
            val onTrip = java.util.IdentityHashMap<Detour, Double>(all.size * 2)
            for (d in all) onTrip[d] = search.tripScore(d, tripW)
            val tripKey = { d: Detour -> onTrip.getValue(d) }
            // (come nella ricerca: le lunghe di corridoi diversi, poi le rimandate dello stesso corridoio,
            // poi le corte sotto dMin1, anche per il tetto degli insiemi e per le prime 8 delle combinazioni)
            all.sortWith(compareBy({ it.rankClass }, tripKey, { it.leg }, { it.fromPos }))
            val comboSets = search.combos(all, key = tripKey)
            val sets = offeredSets(all, comboSets)
            combos = comboSets.size; singles = sets.size - combos
            if (singles < all.size) log("varianti: M ${all.size - singles} deviazioni oltre il tetto di $MAX_M_SETS insiemi")
            budget.check()
            // ogni insieme cucito nel migliore, leggero; la regola sceglie
            val drafts = sets.map { parts ->
                val legs = solved.legs.mapIndexed { j, leg ->
                    val cuts = parts.filter { it.leg == j }.map { net.cut(it) }
                    if (cuts.isEmpty()) leg else router.splice(leg, cuts, light = true)
                }
                Draft(legs, VariantSource.MOTORWAY, parts, parts.size, "M " + parts.joinToString(" + ") { d ->
                    "t${d.leg} ${d.fromPos}→${d.toPos} ${"%+d".format(d.dMs / 1000)} s ${Math.round(d.detourM / 100) / 10.0} km" }, turnsChecked = true)
            }
            val before = ctx.pool.accepted.size
            ctx.pool.offerAll(drafts)
            log("varianti: M ${singles} deviazioni, ${combos} combinazioni, ${(System.nanoTime() - t0) / 1_000_000} ms")
            if (diag != null) traceDetours(ctx, net, search, all, sets, drafts, diag, tripKey)
            for ((d, v) in ctx.pool.accepted.drop(before)) log("varianti: ${d.tag} → tenuta (livello ${v.tier}, ${v.label})")
        }

        /**
         * Traccia: ogni deviazione (le tenute dalla ricerca, poi quelle oltre `perLeg`) con uscita e
         * rientro sul migliore, Δ, metri, plateau, punteggi, strade; le coppie fra le prime 8 non
         * combinabili; i candidati non offerti cuciti per il giudizio finale ([traceEnd]).
         */
        private fun traceDetours(ctx: VariantContext, net: MotorwayNet, search: DetourSearch, all: List<Detour>, sets: List<List<Detour>>,
                                 drafts: List<Draft>, diag: DetourTrace, tripKey: (Detour) -> Double) {
            val trace = ctx.trace
            val offered = java.util.Collections.newSetFromMap(java.util.IdentityHashMap<Detour, Boolean>()).also { o -> for (p in sets) if (p.size == 1) o += p[0] }
            val tripW = trace.legW.sum()
            fun refs(d: Detour): String {
                val m = LinkedHashMap<String, Double>()
                for (a in d.arcs) ctx.info.ref(net.arcEdge[a])?.let { m.merge(it, net.arcM[a].toDouble(), Double::plus) }
                return m.entries.filter { it.value >= 500 }.joinToString(" ") { it.key + " %.0f".format(it.value / 1000) }.ifEmpty { "senza ref" }
            }
            fun line(i: Int, d: Detour, what: String) {
                val s = trace.legStart[d.leg]
                val share = d.plateauW / maxOf(1e-9, d.dW + (trace.pre[s + d.toPos] - trace.pre[s + d.fromPos]))
                log("varianti: traccia M deviazione $i$what: t${d.leg} ${d.fromPos}→${d.toPos} (km %.1f→%.1f del migliore), Δ %+d s %+.1f km, deviazione %.1f km (migliore %.1f km), plateau %.2f, punteggio %.3f, sul viaggio %.3f, rango catena %s; ".format(
                    trace.preM[s + d.fromPos] / 1000, trace.preM[s + d.toPos] / 1000, d.dMs / 1000, d.dM / 1000, d.detourM / 1000, (d.detourM - d.dM) / 1000,
                    share, d.score, search.tripScore(d, tripW), diag.rank[d]?.toString() ?: "?") + refs(d))
            }
            for ((i, d) in all.withIndex()) line(i, d, if (d in offered) " (offerta)" else " (oltre il tetto degli insiemi)")
            for ((i, d) in diag.extra.withIndex()) line(all.size + i, d, " (oltre perLeg)")
            // le coppie fra le prime 8 (come le combinazioni, DetourSearch.comboTop): non disgiunte, e fra queste quelle che si toccano nello stesso nodo
            val top = search.comboTop(all, tripKey)
            var notDisjoint = 0; var touching = 0
            for (i in top.indices) for (j in i + 1 until top.size) if (!top[i].disjoint(top[j])) {
                notDisjoint++
                if (top[i].leg == top[j].leg && (top[i].toPos == top[j].fromPos || top[j].toPos == top[i].fromPos)) touching++
            }
            log("varianti: traccia M combinazioni: fra le prime ${top.size} coppie non disgiunte $notDisjoint, di cui che si toccano nello stesso nodo $touching")
            val notInPool = drafts.count { d -> ctx.pool.drafts.none { it === d } }
            if (notInPool > 0) log("varianti: traccia M: $notInPool insiemi offerti scartati come doppioni o non validi")
            // i non offerti, cuciti alla fine per il giudizio finale (mai offerti)
            val solved = ctx.solved
            for (d in all.filter { it !in offered } + diag.extra) traceDrafts += {
                val legs = solved.legs.mapIndexed { j, leg -> if (d.leg == j) ctx.router.splice(leg, listOf(net.cut(d)), light = true) else leg }
                Draft(legs, VariantSource.MOTORWAY, listOf(d), 1, "M t${d.leg} ${d.fromPos}→${d.toPos} ${"%+d".format(d.dMs / 1000)} s ${Math.round(d.detourM / 100) / 10.0} km", turnsChecked = true)
            }
        }

        override fun summary(): String = "M $singles ($combos combo)"
        override fun note(): String? = if (truncated) "rete autostradale troncata" else null
    }

    /** La fonte P (penalità sul core, [PenaltyCandidates]). */
    private inner class PenaltyProvider : VariantProvider {
        override val source = VariantSource.PENALTY
        private var pc: PenaltyCandidates? = null
        private var executed = 0
        private var offered = 0

        override fun supported(ctx: VariantContext): Boolean {
            val c = PenaltyCandidates(service, ctx.solved, rc, log, ctx)
            if (!c.supported()) return false
            pc = c
            return true
        }

        /**
         * I candidati a coppie fisse (C1-C2, C3-C4, C5-C6, C7): il secondo si sceglie prima di cercare
         * il primo, con o senza aiutante, così le varianti non dipendono dal posto libero. Con
         * l'aiutante i due si cercano insieme; senza, il secondo dopo il primo (e non se il primo ha
         * già riempito: le tenute sarebbero le stesse). A tempo scaduto si offre comunque il
         * candidato finito dell'altro thread, poi DEADLINE.
         */
        override fun run(ctx: VariantContext, budget: CancelToken) {
            val c = pc ?: return
            val pool = ctx.pool
            fun attempt(cc: PenaltyCandidates.Candidate, t: CancelToken): PenaltyCandidates.RunResult? = try { c.run(cc, t) } catch (e: SearchCancelledException) {
                if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
                null
            }
            fun offer(r: PenaltyCandidates.RunResult) {
                c.record(r)
                val d = r.draft
                val outcome = when {
                    d == null -> ""
                    !pool.offer(d) -> " → doppione o non valido"
                    else -> { offered++; pool.accepted.firstOrNull { it.first === d }?.let { " → tenuta (livello ${it.second.tier}, ${it.second.label})" } ?: " → riserva (${d.verdict?.reason ?: "?"})" }
                }
                log(r.line + outcome)
            }
            while (!pool.full && executed < c.maxCandidates) {
                budget.check()
                val c1 = c.next(pool.kept, pool.rejected) ?: break
                val c2 = if (executed + 1 < c.maxCandidates) c.next(pool.kept, pool.rejected) else null
                val slot = if (c2 != null) helperSlot(rc, budget) else null
                var late = false
                if (slot != null) {
                    val (r1, r2) = onTwoThreads(rc, budget, slot, { t -> attempt(c1, t) }, { t -> attempt(c2!!, t) })
                    executed += 2
                    for (r in listOf(r1, r2)) if (r != null) offer(r) else late = true
                } else {
                    executed++
                    val r1 = attempt(c1, budget)
                    if (r1 != null) offer(r1) else late = true
                    if (c2 != null && !late && !pool.full) {
                        executed++
                        val r2 = attempt(c2, budget)
                        if (r2 != null) offer(r2) else late = true
                    }
                }
                if (late) throw SearchCancelledException(SearchCancelledException.Reason.DEADLINE)
            }
        }

        override fun summary(): String = "P $executed ($offered nuove)"
    }
}
