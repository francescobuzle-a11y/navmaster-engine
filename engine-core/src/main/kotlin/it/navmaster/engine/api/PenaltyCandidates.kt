package it.navmaster.engine.api

import com.carrotsearch.hppc.IntHashSet
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.CorridorRun
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgePenalties

/**
 * Fonte P delle varianti: il percorso migliore con le penalità su archi già usati. Esatta perché
 * penalizza solo archi esposti al core ([Router.isCoreExposed]): la ricerca col core li pesa al
 * momento, con il peso penalizzato. Un tratto del migliore senza archi penalizzati (e con lo stesso
 * verso d'arrivo) resta com'è: era il migliore e pesa uguale. Calendario dei candidati in [next]:
 * evitare un'autostrada (AR), penalizzare i tratti principali già usati (SP), combinazioni e crescite.
 */
internal class PenaltyCandidates(private val service: DirectionsService, private val solved: DirectionsService.Solved,
                                 private val rc: RequestContext, private val log: (String) -> Unit, private val ctx: VariantContext) {
    /** AR: evita (×10) un pezzo di corsa autostradale; SP: penalizza i tratti principali dei percorsi; COMBO: due AR; GROW: un AR allargato. */
    internal enum class Kind { AR, SP, COMBO, GROW }

    /** Le unità [uFrom]..[uTo] di [run], o gli archi [edgeFrom]..[edgeTo] del suo tratto (una corsa d'una unità sola: la metà centrale). */
    internal class Ar(val run: CorridorRun, val uFrom: Int, val uTo: Int, val edgeFrom: Int, val edgeTo: Int) {
        val label: String get() = "${run.ref} " + if (uFrom == 0 && uTo == run.units.size - 1 && edgeFrom == run.fromIdx && edgeTo == run.toIdx) "tutta" else "u${uFrom + 1}-${uTo + 1}"
    }

    internal class Candidate(val kind: Kind, val penalties: EdgePenalties, val label: String, val spFactor: Double = 1.0, val ar: Ar? = null)

    /** Il risultato di [run]: la variante (null se nessun percorso entro il tetto), la riga di registro, le misure dei tratti. */
    internal class RunResult(val candidate: Candidate, val draft: Draft?, val line: String, val legs: List<LegStats>)

    private val graphs = service.routingGraph
    private val opts = solved.options
    private val request = solved.request
    private val snaps = solved.snaps
    /** Un router penalizzato (senza penalità): com'è configurato quello dei candidati (niente CH), per [Router.isCoreExposed]. */
    private val expo: Router by lazy { graphs.router(opts, EdgePenalties(1)) }
    private val bestLegEdges: List<IntArray> = solved.legs.map { l -> IntArray(l.edges.size) { l.edges[it].edge } }
    /**
     * Per tratto: il migliore riparte dal punto di passaggio nel verso opposto a quello d'arrivo (non
     * c'era altra via: il ripiego con la penalità di direzione); lì anche i candidati possono farlo.
     */
    private val bestTurnsBack = BooleanArray(solved.legs.size) { j ->
        if (j == 0 || !request.passThroughAt(j)) false else {
            val a = solved.legs[j - 1].edges.lastOrNull(); val b = solved.legs[j].edges.firstOrNull()
            a != null && b != null && a.edge == snaps[j].edge && b.edge == snaps[j].edge && a.reverse != b.reverse
        }
    }
    // sul viaggio intero (gli altri pezzi di un viaggio tra Paesi: VariantContext.trip)
    private val w0 = solved.legs.sumOf { it.weight } + ctx.trip.extraWeight
    private val tripMeters = solved.legs.sumOf { it.distance } + ctx.trip.extraMeters
    private val runs: List<CorridorRun> by lazy { ctx.corridors.runs(solved.legs, tripMeters) }
    private val info get() = ctx.info

    /** Quanti candidati al più: 7 col core, 2 con la ricerca piena (lenta). */
    var maxCandidates = MAX_CANDIDATES; private set
    private var step = 0
    private var c1: Ar? = null
    private var c3: Ar? = null
    /** Gli insiemi già cercati e il loro percorso (null se nessuno): per saltare un candidato che darebbe lo stesso. */
    private val executed = ArrayList<Pair<EdgePenalties, Draft?>>()
    private val arDrafts = ArrayList<Pair<Ar, Draft>>()

    /**
     * Vero se le penalità sono esatte per questa richiesta: col core largo e la ricerca core-ALT
     * (senza la ricerca rapida); oppure con la ricerca piena (`avoid_polygons`) entro
     * [FULL_SEARCH_BEELINE] in linea d'aria, e allora al più 2 candidati.
     */
    fun supported(): Boolean {
        val r = expo
        if (r.hasQuick) return false
        if (graphs.coreWide && r.algorithm.startsWith("core_alt")) { maxCandidates = MAX_CANDIDATES; return true }
        if (r.algorithm.startsWith("dijkstra") && beeline() <= FULL_SEARCH_BEELINE) { maxCandidates = 2; return true }
        return false
    }

    private fun beeline(): Double {
        var d = 0.0
        for (i in 1 until request.coordinates.size) {
            val (aLat, aLon) = request.coordinates[i - 1]; val (bLat, bLon) = request.coordinates[i]
            d += DistanceCalc.haversine(aLat, aLon, bLat, bLon)
        }
        return d
    }

    /**
     * Il prossimo candidato del calendario, dati i percorsi tenuti [kept] (il migliore per primo) e
     * quelli calcolati e scartati [rejected]; null a calendario finito. Si saltano i candidati vuoti
     * (nessun arco esposto) e quelli che darebbero un percorso già trovato (vedi [covered]).
     */
    fun next(kept: List<Draft>, rejected: List<Draft>): Candidate? {
        while (step < 7) {
            val s = step++
            val c = when (s) {
                0 -> pickRun(kept, null)?.let { a -> c1 = a; ar(Kind.AR, a) }
                1 -> sp(kept, 1.4)
                2 -> (pickRun(kept, c1?.run) ?: c1?.let { half(it) })?.let { a -> c3 = a; ar(Kind.AR, a) }
                3 -> sp(kept + rejected, 2.0)
                4 -> grow()
                5 -> { val a = c1; val b = c3; if (a != null && b != null) Candidate(Kind.COMBO, arPenalties(a).times(arPenalties(b)), "AR ${a.label} + ${b.label}") else null }
                else -> sp(kept + rejected, 3.0)
            } ?: continue
            if (c.penalties.size == 0) continue
            if (covered(c)) { log("varianti: P ${c.label} saltato: darebbe un percorso già trovato"); continue }
            return c
        }
        return null
    }

    /**
     * Vero se un candidato già cercato con penalità P0 ⊆ P ha dato un percorso Q senza archi di P:
     * per ogni percorso X, w_P(X) ≥ w_P0(X) ≥ w_P0(Q) = w_P(Q), quindi Q è anche il migliore con P.
     */
    private fun covered(c: Candidate): Boolean = executed.any { (p0, q) -> q != null && c.penalties.covers(p0) && !c.penalties.touches(q.edgeIds) }

    /** Da chiamare con ogni risultato di [run], nell'ordine: memoria per [covered] e per la crescita degli AR. */
    fun record(r: RunResult) {
        executed += r.candidate.penalties to r.draft
        val a = r.candidate.ar
        if (a != null && r.draft != null) arDrafts += a to r.draft
        ctx.stats.legs += r.legs
    }

    /** La corsa più lunga del migliore (diversa da [other]) che nessuna variante tenuta evita già. */
    private fun pickRun(kept: List<Draft>, other: CorridorRun?): Ar? {
        for (run in runs) {
            if (run === other) continue
            if (kept.drop(1).any { k -> sharedMeters(run, k) < 0.5 * run.meters }) continue
            return central(run)
        }
        return null
    }

    private fun sharedMeters(run: CorridorRun, k: Draft): Double {
        val set = ctx.pool.judge.facts(k).edgeSet
        val es = solved.legs[run.leg].edges
        var m = 0.0
        for (i in run.fromIdx..run.toIdx) if (set.contains(es[i].edge)) m += es[i].distance
        return m
    }

    /** L'intervallo da evitare: senza la prima e l'ultima unità dove la corsa tocca un punto; una corsa d'una unità sola, la sua metà centrale. */
    private fun central(run: CorridorRun): Ar {
        val es = solved.legs[run.leg].edges
        var uf = 0; var ut = run.units.size - 1
        if (run.touchesPoint) {
            val startTouch = es.subList(0, run.fromIdx).sumOf { it.distance } < TOUCH_METERS
            val endTouch = es.subList(run.toIdx + 1, es.size).sumOf { it.distance } < TOUCH_METERS
            if (startTouch && ut > uf) uf++
            if (endTouch && ut > uf) ut--
            if (ut == uf && run.units.size == 1) return middle(run, 0.25, 0.75)
        }
        return Ar(run, uf, ut, run.units[uf].fromIdx, run.units[ut].toIdx)
    }

    /** Gli archi della corsa il cui punto medio sta fra le frazioni [a] e [b] della sua lunghezza. */
    private fun middle(run: CorridorRun, a: Double, b: Double): Ar {
        val es = solved.legs[run.leg].edges
        val total = (run.fromIdx..run.toIdx).sumOf { es[it].distance }
        var acc = 0.0; var ef = -1; var et = -1
        for (i in run.fromIdx..run.toIdx) {
            val mid = acc + es[i].distance / 2
            if (mid >= a * total && mid <= b * total) { if (ef < 0) ef = i; et = i }
            acc += es[i].distance
        }
        if (ef < 0) { ef = (run.fromIdx + run.toIdx) / 2; et = ef }
        return Ar(run, 0, run.units.size - 1, ef, et)
    }

    /** La metà più lunga di [a]: per unità se ne ha più d'una, altrimenti per distanza. */
    private fun half(a: Ar): Ar? {
        val run = a.run
        if (a.uTo > a.uFrom && a.edgeFrom == run.units[a.uFrom].fromIdx) {
            val mid = (a.uFrom + a.uTo) / 2
            val left = (a.uFrom..mid).sumOf { run.units[it].meters }; val right = (mid + 1..a.uTo).sumOf { run.units[it].meters }
            return if (left >= right) Ar(run, a.uFrom, mid, run.units[a.uFrom].fromIdx, run.units[mid].toIdx)
                else Ar(run, mid + 1, a.uTo, run.units[mid + 1].fromIdx, run.units[a.uTo].toIdx)
        }
        if (a.edgeTo <= a.edgeFrom) return null
        val es = solved.legs[run.leg].edges
        val total = (a.edgeFrom..a.edgeTo).sumOf { es[it].distance }
        var acc = 0.0; var cut = a.edgeFrom
        for (i in a.edgeFrom..a.edgeTo) { acc += es[i].distance; if (acc >= total / 2) { cut = i; break } }
        val left = Ar(run, a.uFrom, a.uTo, a.edgeFrom, cut)
        val right = Ar(run, a.uFrom, a.uTo, minOf(cut + 1, a.edgeTo), a.edgeTo)
        return if (acc >= total - acc) left else right
    }

    /** C5: un AR il cui percorso è stato tenuto solo come «non autostradale», allargato di un'unità per lato. */
    private fun grow(): Candidate? {
        for ((a, d) in arDrafts) {
            if (d.verdict?.reason != "autostrade") continue
            val run = a.run
            val uf = maxOf(0, a.uFrom - 1); val ut = minOf(run.units.size - 1, a.uTo + 1)
            val g = Ar(run, uf, ut, run.units[uf].fromIdx, run.units[ut].toIdx)
            if (g.edgeFrom >= a.edgeFrom && g.edgeTo <= a.edgeTo) continue
            return Candidate(Kind.GROW, arPenalties(g), "AR ${g.label} (allargato)", ar = g)
        }
        return null
    }

    private fun ar(kind: Kind, a: Ar) = Candidate(kind, arPenalties(a), "AR ${a.label}", ar = a)

    /** ×[AR_FACTOR] sugli archi autostradali (rampe comprese) esposti del migliore in [a]. */
    private fun arPenalties(a: Ar): EdgePenalties {
        val p = EdgePenalties(256)
        val es = solved.legs[a.run.leg].edges
        for (i in a.edgeFrom..a.edgeTo) {
            val e = es[i].edge
            if (e !in p && info.isMotorway(e) && expo.isCoreExposed(e)) p.multiply(e, AR_FACTOR)
        }
        return p
    }

    /** ×[f] sugli archi esposti dei tratti principali di [paths] (≥ 25 % del viaggio o ≥ 60 km; senza, tutto il percorso). */
    private fun sp(paths: List<Draft>, f: Double): Candidate? {
        val p = EdgePenalties(4096)
        val seen = IntHashSet()
        for (d in paths) for (range in mainStretches(d)) for (i in range) {
            val e = d.edgeIds[i]
            if (seen.add(e) && expo.isCoreExposed(e)) p.multiply(e, f)
        }
        return Candidate(Kind.SP, p, "SP×$f (${paths.size} percorsi)", spFactor = f)
    }

    /** I pezzi di [d] con lo stesso ref (gli archi senza ref non li interrompono) lunghi almeno il 25 % del viaggio o 60 km. */
    private fun mainStretches(d: Draft): List<IntRange> {
        val out = ArrayList<IntRange>()
        var cur = -1; var start = 0; var m = 0.0
        fun close(end: Int) { if (cur >= 0 && (m >= MAIN_SHARE * tripMeters || m >= MAIN_METERS)) out += start..end }
        for (i in d.edgeIds.indices) {
            val r = info.refId(d.edgeIds[i])
            if (r < 0 || r == cur) { m += d.edgeM[i]; continue }
            close(i - 1)
            cur = r; start = i; m = d.edgeM[i]
        }
        close(d.edgeIds.size - 1)
        return out.ifEmpty { listOf(d.edgeIds.indices) }
    }

    /**
     * Il percorso con le penalità di [c] (token [cancel]): i tratti del migliore senza archi
     * penalizzati e con lo stesso verso d'arrivo restano; gli altri si cercano leggeri, col tetto di
     * peso del tratto del migliore più [CAP_SLACK]·(max_factor − 1) del viaggio (più (f − 1) volte il
     * tratto per SP, che penalizza anche il migliore), senza secondo tentativo nel verso opposto
     * (salvo dove il migliore stesso è ripartito nel verso opposto, [bestTurnsBack]).
     */
    fun run(c: Candidate, cancel: CancelToken): RunResult {
        val t0 = System.nanoTime()
        val router = graphs.router(opts, c.penalties).also { it.cancel = cancel }
        val m = solved.legs.size
        val legs = ArrayList<RoutePath>(m)
        val stats = ArrayList<LegStats>()
        val nodes = ArrayList<String>()
        var reused = 0
        fun line(outcome: String) = "varianti: P ${c.label}: ${c.penalties.size} archi penalizzati, tratti $reused riusati, ${stats.size - reused} cercati" +
            (if (nodes.isNotEmpty()) " (${nodes.joinToString("; ")})" else "") + ", ${(System.nanoTime() - t0) / 1_000_000} ms$outcome"
        for (j in 0 until m) {
            cancel.check()
            val best = solved.legs[j]
            val through = j > 0 && request.passThroughAt(j)
            val arrival = if (through) legs[j - 1].edges.lastOrNull() else null
            val bestArrival = if (through) solved.legs[j - 1].edges.lastOrNull() else null
            if (!c.penalties.touches(bestLegEdges[j]) && router.arrivalKey(snaps[j], arrival) == router.arrivalKey(snaps[j], bestArrival)) {
                legs += best; reused++
                stats += LegStats(j, LegStats.Source.REUSED, 0, "riuso", "", best.edges.size, best.distance, "")
                continue
            }
            val cap = best.weight + CAP_SLACK * (ctx.opt.maxFactor - 1) * w0 + (if (c.kind == Kind.SP) (c.spFactor - 1) * best.weight else 0.0)
            val tl = System.nanoTime()
            val fromHeading = request.bearings?.getOrNull(j) ?: Double.NaN
            val toHeading = if (j == m - 1) request.bearings?.getOrNull(j + 1) ?: Double.NaN else Double.NaN
            val p = try {
                router.route(snaps[j], snaps[j + 1], arrival, fromHeading, toHeading, maxWeight = cap, light = true, relaxRetry = bestTurnsBack[j])
            } catch (e: RouteNotFoundException) {
                stats += LegStats(j, LegStats.Source.PENALIZED, (System.nanoTime() - tl) / 1_000_000, router.lastAlgorithm, router.lastSearch, 0, 0.0, Thread.currentThread().name)
                return RunResult(c, null, line(" → nessun percorso entro il tetto nel tratto $j"), stats)
            }
            nodes += router.lastSearch
            stats += LegStats(j, LegStats.Source.PENALIZED, (System.nanoTime() - tl) / 1_000_000, router.lastAlgorithm, router.lastSearch, p.edges.size, p.distance, Thread.currentThread().name)
            legs += p
        }
        return RunResult(c, Draft(legs, VariantSource.PENALTY, tag = c.label), line(""), stats)
    }

    companion object {
        const val MAX_CANDIDATES = 7
        const val AR_FACTOR = 10.0
        /** Il tetto di peso di un tratto ricercato: il migliore più 1,2 volte il margine di durata (il peso non è il tempo). */
        const val CAP_SLACK = 1.2
        const val FULL_SEARCH_BEELINE = 150_000.0
        const val MAIN_SHARE = 0.25
        const val MAIN_METERS = 60_000.0
        const val TOUCH_METERS = 1_000.0
    }
}
