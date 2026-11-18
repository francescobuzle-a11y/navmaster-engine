package it.navmaster.engine.api

import com.carrotsearch.hppc.LongHashSet
import com.carrotsearch.hppc.LongIntHashMap
import it.navmaster.engine.RouteOptions
import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.ch.CoreManyToOne
import it.navmaster.engine.index.Snap
import it.navmaster.engine.index.SnapPosition
import it.navmaster.engine.instructions.InstructionTranslator
import it.navmaster.engine.instructions.InstructionType
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.routing.CancelToken
import it.navmaster.engine.routing.CorridorAnalyzer
import it.navmaster.engine.routing.EdgeInfo
import it.navmaster.engine.routing.PointNotFoundException
import it.navmaster.engine.routing.RouteNotFoundException
import it.navmaster.engine.routing.RoutePath
import it.navmaster.engine.routing.Router
import it.navmaster.engine.routing.SearchCancelledException
import it.navmaster.engine.storage.BBox
import it.navmaster.engine.util.DistanceCalc
import java.io.File
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLongArray
import java.util.concurrent.atomic.AtomicReference
import java.util.concurrent.atomic.AtomicReferenceArray

/** Il grafo di un Paese, per i viaggi tra Paesi. */
class CountryGraph(val id: String, val service: DirectionsService) {
    val graph: RoutingGraph get() = service.routingGraph
}

/**
 * Un punto in cui si passa da un grafo all'altro: lo stesso punto della stessa strada nei due grafi
 * (le strade che attraversano il confine sono in entrambi gli estratti OSM dei due Paesi). [rank]:
 * l'importanza della strada (1 autostrada … 7 strada di servizio).
 */
class Transfer(val lat: Double, val lon: Double, val rank: Int)

/** I punti di passaggio fra due grafi, trovati una volta e salvati. */
object Transfers {
    /** Quanto è durata l'ultima ricerca dei passaggi, per il registro. */
    @Volatile var timing: String = ""

    /**
     * I punti di passaggio fra [a] e [b]: i nodi senza uscita di un grafo (dove il suo estratto
     * finisce, oltre il confine) che nell'altro sono un punto di una strada che prosegue.
     */
    fun find(a: RoutingGraph, b: RoutingGraph): List<Transfer> {
        val ba = a.graph.bounds; val bb = b.graph.bounds
        if (!ba.isValid || !bb.isValid || !ba.intersects(bb)) return emptyList()
        val overlap = BBox.empty().apply {
            include(maxOf(ba.minLat, bb.minLat), maxOf(ba.minLon, bb.minLon))
            include(minOf(ba.maxLat, bb.maxLat), minOf(ba.maxLon, bb.maxLon))
        }
        val t0 = System.currentTimeMillis()
        val degA = degrees(a); val degB = degrees(b)
        val t1 = System.currentTimeMillis()
        val out = ArrayList<Transfer>()
        val qa = scan(a, degA, b, degB, overlap, out)
        val t2 = System.currentTimeMillis()
        val qb = scan(b, degB, a, degA, overlap, out)
        timing = "gradi ${t1 - t0} ms, A→B ${t2 - t1} ms ($qa ricerche), B→A ${System.currentTimeMillis() - t2} ms ($qb ricerche)"
        // un punto per luogo: i due capi di una stessa strada di confine restano (possono distare km)
        val kept = ArrayList<Transfer>()
        for (t in out.sortedBy { it.rank }) if (kept.none { DistanceCalc.haversine(it.lat, it.lon, t.lat, t.lon) < 25.0 }) kept += t
        return kept
    }

    /** Il numero di archi di ogni nodo (fino a 3). */
    private fun degrees(g: RoutingGraph): ByteArray {
        val bg = g.graph
        val deg = ByteArray(bg.nodeCount)
        for (e in 0 until bg.edgeCount) {
            val x = bg.nodeA(e); val y = bg.nodeB(e)
            if (x == y) continue
            if (deg[x] < 3) deg[x]++
            if (deg[y] < 3) deg[y]++
        }
        return deg
    }

    private fun scan(src: RoutingGraph, degSrc: ByteArray, dst: RoutingGraph, degDst: ByteArray, overlap: BBox, out: MutableList<Transfer>): Int {
        var queries = 0
        val g = src.graph
        val it = g.createEdgeIterator()
        val rc = src.encoding.shared.roadClass
        for (n in 0 until g.nodeCount) {
            if (degSrc[n].toInt() != 1) continue
            val lat = g.lat(n); val lon = g.lon(n)
            if (!overlap.contains(lat, lon)) continue
            // solo dove l'altro grafo ha strade (la sua griglia è vuota fuori dal suo estratto)
            if (!dst.index.hasEdgesNear(lat, lon)) continue
            queries++
            val s = dst.index.findClosest(lat, lon, maxRings = 1)
            if (!s.isValid || s.distance > 1.0) continue
            // senza uscita anche nell'altro: una strada chiusa nella zona coperta da entrambi, non un passaggio
            if (s.position == SnapPosition.TOWER && degDst[s.closestNode] <= 1) continue
            it.setBaseNode(n)
            if (!it.next()) continue
            out += Transfer(lat, lon, rank(it.get(rc)))
        }
        return queries
    }

    /**
     * I valichi raggruppati per luogo (entro [PLACE] m dal punto della strada più importante), i
     * luoghi in ordine di [detour], al più [places]; in ogni luogo i punti in ordine d'importanza
     * della strada. Un luogo è un valico solo: al Brennero i capi delle due carreggiate
     * dell'autostrada e la statale, a Resia i due capi della stessa strada. Quale punto del luogo
     * serve lo decide il viaggio: un senso unico o un'autostrada vietata al mezzo da un lato non si
     * raggiungono, allora vale il punto successivo.
     */
    fun places(list: List<Transfer>, detour: (Transfer) -> Double, places: Int): List<List<Transfer>> {
        val out = ArrayList<MutableList<Transfer>>()
        for (t in list.sortedWith(compareBy<Transfer>({ it.rank }, { detour(it) }))) {
            val place = out.firstOrNull { DistanceCalc.haversine(it[0].lat, it[0].lon, t.lat, t.lon) < PLACE }
            if (place == null) out += mutableListOf(t) else place += t
        }
        return out.sortedBy { detour(it[0]) }.take(places)
    }

    /** Un luogo di valico: i passaggi entro questi metri (dal più importante) sono lo stesso valico. */
    internal const val PLACE = 5_000.0

    /**
     * Vero se un giro per il Paese di [list] (i valichi con un Paese vicino) può servire al viaggio
     * p → q in un Paese solo: due valichi t1, t2 in luoghi diversi ([PLACE]) con
     * crow(p,t1) + crow(t1,t2) + crow(t2,q) ≤ [factor]·crow(p,q). Solo geometria (nessun grafo letto):
     * lo usa anche l'app per sapere quale grafo aprire.
     */
    fun aroundOk(list: List<Transfer>, pLat: Double, pLon: Double, qLat: Double, qLon: Double, factor: Double = CrossBorderDirections.AROUND_FACTOR): Boolean {
        val crow = DistanceCalc.haversine(pLat, pLon, qLat, qLon)
        if (crow <= 0) return false
        val lim = factor * crow
        val ins = list.filter { t -> DistanceCalc.haversine(pLat, pLon, t.lat, t.lon) + DistanceCalc.haversine(t.lat, t.lon, qLat, qLon) <= lim }
        for (a in ins) {
            val da = DistanceCalc.haversine(pLat, pLon, a.lat, a.lon)
            for (b in ins) {
                val ab = DistanceCalc.haversine(a.lat, a.lon, b.lat, b.lon)
                if (ab < PLACE) continue
                if (da + ab + DistanceCalc.haversine(b.lat, b.lon, qLat, qLon) <= lim) return true
            }
        }
        return false
    }

    fun rank(c: RoadClass): Int = when (c) {
        RoadClass.MOTORWAY -> 1
        RoadClass.TRUNK -> 2
        RoadClass.PRIMARY -> 3
        RoadClass.SECONDARY -> 4
        RoadClass.TERTIARY -> 5
        RoadClass.UNCLASSIFIED, RoadClass.RESIDENTIAL, RoadClass.LIVING_STREET -> 6
        else -> 7
    }

    fun write(f: File, list: List<Transfer>) {
        f.parentFile?.mkdirs()
        val tmp = File(f.path + ".tmp")
        tmp.writeText(buildString { append("NMTRANSFERS 1\n"); for (t in list) append(t.lat).append(',').append(t.lon).append(',').append(t.rank).append('\n') })
        tmp.renameTo(f)
    }

    fun read(f: File): List<Transfer>? {
        if (!f.exists()) return null
        val lines = f.readLines()
        if (lines.firstOrNull() != "NMTRANSFERS 1") return null
        return lines.drop(1).filter { it.isNotBlank() }.mapNotNull { l ->
            val p = l.split(',')
            if (p.size != 3) null else Transfer(p[0].toDouble(), p[1].toDouble(), p[2].toInt())
        }
    }
}

/**
 * Viaggi tra Paesi con un grafo per Paese: ogni tratto che passa da un grafo all'altro è calcolato
 * nei grafi dei Paesi attraversati e cucito ai punti di passaggio del confine ([Transfers]). Fra
 * tutti i passaggi candidati di ciascun confine si sceglie la combinazione col peso totale minimo
 * (lo stesso peso di openrouteservice: tempo, distanza o "recommended" con le restrizioni del mezzo),
 * passando anche per Paesi intermedi (fino a due). La risposta è quella di openrouteservice
 * (GeoJSON) come per un percorso in un grafo solo: le istruzioni proseguono al confine senza un
 * arrivo intermedio, gli extra e i fatti sulle strade sono uniti.
 */
class CrossBorderDirections(
    private val graphs: List<CountryGraph>,
    /** I passaggi fra due grafi (calcolati una volta e conservati da chi chiama). */
    private val transfers: (CountryGraph, CountryGraph) -> List<Transfer>,
    private val log: ((String) -> Unit)? = null,
    /** Fra due Paesi il valico coi limiti dei landmark e una ricerca sola dal lato vicino ([chooseTogether]); false: uno alla volta (prove). */
    private val oneSearch: Boolean = true,
) {
    private class LL(val lat: Double, val lon: Double)

    /**
     * La finestra delle ricerche di limite oltre il passaggio più vicino ([advanceAround]): un fattore
     * e dei secondi; [startWindowFactor]/[startWindowSlack] per la ricerca dal primo capo in transito
     * (in Italia la più grande). Non toccano l'esattezza, solo il lavoro: modificabili per le prove.
     */
    internal var windowFactor = WINDOW_FACTOR
    internal var windowSlack = WINDOW_SLACK
    // dal primo capo in transito basta arrivare al passaggio più vicino: le radici del Paese di mezzo
    // prendono i limiti degli altri dalla frontiera (Soncino → Monaco di Baviera: in Italia 92 000 nodi
    // invece di 176 000, stessi valichi; per Komjatice uguale, la ricerca deve arrivare a Tarvisio)
    internal var startWindowFactor = 1.0
    internal var startWindowSlack = 0.0
    /**
     * Il percorso mirato ([Reach]): per un passaggio lontano dal capo più di [focusRatio] volte (in linea
     * d'aria) il passaggio più vicino già fissato dalla ricerca verso tutti, il peso esatto viene da un
     * percorso da punto a punto col tetto. Il limite dei landmark non serve a dirlo: per Tarvisio da
     * Canneto (312 km) è più basso della frontiera fissata al Brennero (214 km).
     * [focusBeyond] negativo: sempre mirato (prove). Non tocca l'esattezza, solo il lavoro.
     */
    internal var focusBeyond = 0.0
    /** false: "avoid" as a penalty in every country as soon as the ban finds nothing (prove, confronto). */
    internal var shutInCheck = true
    internal var focusRatio = 1.25
    /**
     * I percorsi mirati di una ricerca di limite ([Reach]) al più: oltre, la ricerca verso tutti va
     * avanti (una sola per tutti i passaggi). Senza tetto un capo coi landmark deboli (evita autostrade)
     * faceva centinaia di percorsi mirati che rifacevano ognuno lo stesso pezzo di Paese (Bolzano →
     * Stoccarda: 64 in Germania da 0,4 s, Lione → Francoforte: 913 in Francia), mentre la ricerca verso
     * tutti, portata avanti, li fissa in una volta. Non tocca l'esattezza, solo il lavoro.
     */
    internal var focusMax = FOCUS_MAX
    /** Prove: chiamato all'inizio di ogni scelta dei due valichi insieme (chooseTwo), sul suo thread. */
    internal var beforeTwo: (() -> Unit)? = null
    /** Prove: chiamato all'inizio (vero) e alla fine (falso) di ogni sequenza del passo 1 di [chain], col suo indice, sul suo thread. */
    internal var onSequence: ((Int, Boolean) -> Unit)? = null
    /**
     * Ai confini di mezzo di [dpChain] una ricerca sola all'indietro da ogni candidato verso tutto lo
     * strato ([Ctx.bestTo]); false: una ricerca per coppia (voce dello strato, candidato), come prima
     * (prove: stesso risultato). Non tocca l'esattezza, solo il lavoro.
     */
    internal var layerManyToOne = true
    /**
     * Le sequenze di Paesi di [chain] (il passo 1, il migliore, e il passo 2, le altre catene) divise
     * fra il thread della richiesta e il posto da aiutante ([helperSlot]), quando c'è; false: una dopo
     * l'altra (prove). Il migliore non cambia (vedi [chain]), cambia solo il tempo.
     */
    internal var parallelSequences = true
    /**
     * I limiti inferiori della scelta dei valichi comprendono i tratti fissi dei capi ([pieceBase]): con una
     * tappa prima del confine (Canneto → Drasenhofen «passa di qui» → Bielsko-Biała) il tratto Tarvisio →
     * Drasenhofen, che il tetto (il peso della catena intera) contiene, mancava ai limiti, e il transito
     * austria-czech-poland esaminava 169 uscite (472 ricerche, 519 tratti, 20 s in CI, 202 s sul tablet).
     * false: come prima (prove). Il migliore non cambia (sono limiti inferiori), cambia solo il lavoro.
     */
    internal var prefixBase = true
    /**
     * I grafi del passo 1 di [chain] (il migliore): quelli che l'app aprirebbe per il percorso a schermo
     * (null: tutti). Gli altri grafi aperti servono solo alle sequenze in più del passo 2 (fonte V), così
     * aprirne di più per la richiesta delle varianti non cambia il migliore.
     */
    var bestGraphs: Set<String>? = null
    /**
     * I grafi che decidono il migliore ([bestGraphs], o tutti): oltre al passo 1, i Paesi dove «evita»
     * diventa penalità ([handleIn], [shutIn]) e il grafo dei punti ([route]). Così la richiesta delle
     * varianti, coi grafi in più, ha lo stesso insieme di Paesi con la penalità e gli stessi grafi dei
     * punti della richiesta del migliore, quindi gli stessi pesi e lo stesso migliore.
     */
    private val bestPool: List<CountryGraph>
        get() = bestGraphs?.let { ids -> graphs.filter { it.id in ids } }?.takeIf { it.isNotEmpty() } ?: graphs

    /** Il peso del viaggio scelto all'ultimo confine e i suoi valichi (diagnostica, prove). */
    internal var lastWeight = Double.NaN; private set
    internal var lastCrossings: List<Transfer> = emptyList(); private set
    /** I pezzi dell'ultimo viaggio tra Paesi: il grafo e la richiesta di ciascuno (prove). */
    internal var lastPieces: List<Pair<CountryGraph, DirectionsRequest>> = emptyList(); private set
    /** Una catena esatta dell'ultima scelta ([chain]): i Paesi, i valichi, il peso (prove). */
    internal class ChainInfo(val countries: List<String>, val via: List<Transfer>, val exact: Double)
    /** Le catene dell'ultima [chain]: il migliore primo, poi le altre entro il fattore delle varianti (prove). */
    internal var lastChains: List<ChainInfo> = emptyList(); private set

    /** Un pezzo del viaggio dentro un grafo: i suoi punti (le tappe e i passaggi di confine). */
    private class Piece(val graph: CountryGraph, val points: MutableList<Pair<Double, Double>>, val radiuses: MutableList<Double>,
                        val exact: MutableList<Boolean> = MutableList(points.size) { false },
                        val bearings: MutableList<Double?> = MutableList(points.size) { null },
                        /** Per punto: si passa senza fermarsi ([DirectionsRequest.passThroughAt]); mai ai passaggi di confine. */
                        val passThrough: MutableList<Boolean> = MutableList(points.size) { false }) {
        /** Una copia indipendente (le liste sono mutabili: i pezzi delle varianti non toccano quelli del migliore). */
        fun copy() = Piece(graph, points.toMutableList(), radiuses.toMutableList(), exact.toMutableList(), bearings.toMutableList(), passThrough.toMutableList())
        /** Il pezzo senza l'ultimo punto (il passaggio di confine): il prefisso con cui [chain] l'ha scelto. */
        fun withoutLast() = Piece(graph, points.dropLast(1).toMutableList(), radiuses.dropLast(1).toMutableList(), exact.dropLast(1).toMutableList(),
            bearings.dropLast(1).toMutableList(), passThrough.dropLast(1).toMutableList())
    }

    /** La richiesta del pezzo [p]: la stessa di [exactChain] per gli stessi punti (così i tratti sono in cache). */
    private fun requestOf(request: DirectionsRequest, p: Piece): DirectionsRequest =
        request.withPoints(p.points, p.radiuses.toDoubleArray(), p.exact.toBooleanArray(), p.bearings, p.passThrough.toBooleanArray())

    /**
     * I pezzi di una catena di valichi [chain] (ogni passaggio col grafo in cui si entra) a partire
     * dal pezzo [prefix] (copiato): quelli chiusi a un valico e il nuovo pezzo corrente, che comincia
     * all'ultimo valico. Le richieste che ne escono ([requestOf]) sono quelle di [exactChain].
     */
    private fun piecesOf(prefix: Piece, chain: List<Pair<Transfer, CountryGraph>>): Pair<List<Piece>, Piece> {
        val closed = ArrayList<Piece>()
        var cur = prefix.copy()
        for ((t, next) in chain) {
            cur.points += t.lat to t.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
            closed += cur
            cur = Piece(next, mutableListOf(t.lat to t.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
        }
        return closed to cur
    }

    /** Una catena esatta di valichi: la sequenza di grafi, i passaggi (uno per confine) e il peso come i pezzi finali. */
    private class ChainChoice(val path: List<CountryGraph>, val via: List<Transfer>, val exact: Double) {
        /** Vero se [o] passa per gli stessi Paesi e ogni suo valico sta entro [Transfers.PLACE] da quello allo stesso confine. */
        fun samePlaces(o: ChainChoice): Boolean = path.map { it.id } == o.path.map { it.id } &&
            via.indices.all { i -> DistanceCalc.haversine(via[i].lat, via[i].lon, o.via[i].lat, o.via[i].lon) < Transfers.PLACE }
        fun text() = "${path.joinToString("-") { it.id }} via ${via.joinToString(" ") { "%.5f,%.5f".format(it.lat, it.lon) }}"
    }

    /** Una [chain] del viaggio: i pezzi del migliore da [first] a [last] (quello che comincia all'ultimo valico) e le sue catene (il migliore primo). */
    private class Span(val first: Int, val last: Int, val chains: List<ChainChoice>)

    /**
     * I pesi rapidi di una richiesta ([Ctx.weight], [Ctx.bestTo]), condivisi dai [Ctx] dei due thread
     * di [chain] ([Ctx.fork]): ogni peso col limite sotto cui è stato cercato, e quelli in corso su un
     * thread (l'altro aspetta invece di rifare la stessa ricerca). Un lucchetto solo.
     */
    private class Memory {
        private val weights = HashMap<String, Pair<Double, Double>>()
        private val running = HashMap<String, CountDownLatch>()
        /** Vero mentre due thread pesano sequenze insieme: [dpChain] divide fra loro il primo strato ([Ctx.warm]). */
        @Volatile var shared = false
        /**
         * Quanti thread stanno pesando una voce di [sequences] adesso: un'attesa di [belowWithin] ha senso solo
         * se un altro thread lavora (un aiutante prelazionato o mai partito, o una voce che verrà dopo sullo
         * stesso thread, non abbassano il tetto mentre si aspetta).
         */
        val working = java.util.concurrent.atomic.AtomicInteger(0)

        @Synchronized fun get(key: String): Pair<Double, Double>? = weights[key]

        /** Come [QuickWeightCache.put]: un peso finito resta; «nessun percorso sotto l» lascia il posto a uno finito o a un limite più alto. */
        @Synchronized fun put(key: String, w: Double, limit: Double) {
            val e = weights[key]
            if (e == null || (!e.first.isFinite() && (w.isFinite() || limit > e.second))) weights[key] = w to limit
        }

        /** null: [key] è di chi chiama (lo cerca e poi [release]); altrimenti l'attesa della ricerca in corso sull'altro thread. */
        @Synchronized fun claim(key: String): CountDownLatch? {
            running[key]?.let { return it }
            running[key] = CountDownLatch(1)
            return null
        }

        /** Vero se [key] è in corso su un thread. */
        @Synchronized fun isRunning(key: String): Boolean = key in running

        @Synchronized fun release(key: String) { running.remove(key)?.countDown() }
    }

    /** Lo stato di una richiesta: router e agganci per grafo, pesi già calcolati. */
    private inner class Ctx(val request: DirectionsRequest, val penalized: Set<String>? = null,
                            /** Searches without the roads to avoid at all (the yardstick of [shutIn]). */
                            val plain: Boolean = false,
                            /** L'annullamento e i posti di ricerca della richiesta. */
                            val rc: RequestContext = RequestContext.NONE,
                            /** I pesi rapidi, condivisi con i [fork] di questo contesto. */
                            private val memory: Memory = Memory()) {
        /** Per i pesi dei pezzi ([pieceWeight]): lo stesso annullamento, misure a parte (non sono tratti della risposta). */
        private val quiet = RequestContext(rc.cancel, rc.priority, rc.executor, rc.slots, rc.log, useCache = rc.useCache)
        // router, opzioni e agganci di un thread solo (un Router non è thread-safe: il [fork] ha i suoi)
        private val routers = HashMap<String, Router>()
        private val options = HashMap<String, RouteOptions>()
        private val snaps = HashMap<String, Snap?>()

        /**
         * Lo stesso contesto per un altro thread (l'aiutante di [chain]): router, agganci e contatori
         * suoi, i pesi rapidi ([Memory]) in comune; [cancel] il token del posto da aiutante,
         * [topToken] il tempo delle varianti sul suo thread. I conteggi tornano qui con [absorb].
         */
        fun fork(cancel: CancelToken, topToken: CancelToken?): Ctx =
            Ctx(request, penalized, plain, RequestContext(cancel, rc.priority, rc.executor, rc.slots, rc.log, rc.stats, rc.useCache), memory).also {
                it.topK = topK; it.topFactor = topFactor; it.topBudget = topToken; it.topDeadline = topDeadline
                it.navStart = navStart; it.navBudgetMs = navBudgetMs; it.bestKnown = bestKnown
            }

        /** I conteggi (ricerche, memoria, pesi dei pezzi) e le scelte della fonte V di un [fork], dopo che il suo thread ha finito. */
        fun absorb(f: Ctx) {
            queries += f.queries; hits += f.hits; extraSearched += f.extraSearched; extraQueries += f.extraQueries
            searchedAll += f.searchedAll; knownQueries += f.knownQueries; knownSearched += f.knownSearched
            if (f.topStopped) topStopped = true
            if (f.top.isNotEmpty()) top = f.top
            val ls = synchronized(f.quiet.stats.legs) { f.quiet.stats.legs.toList() }
            synchronized(quiet.stats.legs) { quiet.stats.legs += ls }
        }

        /** I thread che pesano una voce di [sequences] adesso ([Memory.working]). */
        val working get() = memory.working
        /** Il «passo 1 in parallelo» per [Memory.shared]. */
        var shared: Boolean
            get() = memory.shared
            set(v) { memory.shared = v }
        /** Le ricerche davvero fatte (rapide, pezzi, mirate, molti a uno). */
        var queries = 0
        /** Le ricerche rapide risparmiate dalla memoria del servizio ([DirectionsService.quickCache]). */
        var hits = 0
        /**
         * Le varianti a valichi diversi (fonte V, `navmaster.alternatives`): fra due Paesi [chooseExactTop]
         * tiene fino a [topK] luoghi entro [topFactor] volte il peso del migliore; in [top] l'ultima scelta (il migliore primo).
         */
        var topK = 1
        var topFactor = 1.0
        /**
         * Il tetto di peso delle altre catene (passo 2 di [chain], [chooseExactTop]): la regola delle
         * varianti è in tempo ([topFactor]), il peso del camion «recommended» premia le autostrade, così
         * una catena su statali ha un rapporto di peso più alto di quello di tempo. Lo stesso margine
         * della fonte P: 1 + [PenaltyCandidates.CAP_SLACK]·(topFactor − 1) (1,24 per 1,2; 1,6 per 1,5);
         * il filtro in tempo e km resta in [variantsAcross].
         */
        val topWeightFactor: Double get() = 1 + PenaltyCandidates.CAP_SLACK * (topFactor - 1)
        /** L'inizio (`System.nanoTime`) e il tempo (`budget_ms`) della richiesta delle varianti, per allungare [topBudget] dopo il passo 1. */
        var navStart = 0L
        var navBudgetMs = 0L
        var top: List<Choice> = emptyList()
        /** Le catene esatte dell'ultima [chain] (il migliore primo, poi le altre entro [topFactor] in ordine di peso). */
        var chains: List<ChainChoice> = emptyList()
        /** Per ogni [chain] del viaggio: i pezzi del migliore che copre e le sue catene (per la fonte V). */
        val spans = ArrayList<Span>()
        /**
         * Il tempo della parte della scelta che serve solo alla fonte V (oltre il migliore: il passo 2
         * di [chain], gli extra di [chooseExactTop]): [TOP_BUDGET_SHARE] di `budget_ms` dall'inizio
         * della richiesta, così alle varianti dei pezzi (fonti M e P) resta sempre il resto. Alla
         * scadenza si tiene il migliore e quel che c'è; [topStopped] se è successo.
         */
        var topBudget: CancelToken? = null
        /**
         * Il migliore della richiesta è già noto dalla memoria del servizio (la scelta dei due valichi insieme con la
         * tappa nel Paese di mezzo, [route]): le scelte per confine ([chain]) lo ritrovano e servono alla fonte V. Le
         * loro ricerche del passo 1 si mettono da parte ([knownQueries], [knownSearched]) e si contano per le varianti
         * solo se i valichi trovati per confine sono quelli della memoria.
         */
        var bestKnown = false
        /** Le ricerche del passo 1 delle scelte per confine con [bestKnown] (non già per le varianti), da contare per le varianti dopo il confronto. */
        var knownQueries = 0
        /** I tratti cercati davvero da quelle ricerche ([knownQueries]), non già per le varianti. */
        var knownSearched = 0
        /** La scadenza di [topBudget] (`System.nanoTime`), per il tempo delle varianti sul thread di un [fork]. */
        var topDeadline = Long.MAX_VALUE
        var topStopped = false

        /**
         * [r] as asked in [g]: "avoid" as a penalty only in the countries of [penalized] (null: all),
         * a ban elsewhere. The weights add up across countries as they are: off the roads to avoid
         * the penalty weighs as the ban.
         */
        fun inGraph(g: CountryGraph, r: DirectionsRequest): DirectionsRequest =
            if (r.avoidPenalty && penalized != null && g.id !in penalized) r.withAvoidPenalty(false) else r

        // the choice of the crossings without the bearings (they only touch the ends): the quick search
        fun router(g: CountryGraph): Router = routers.getOrPut(g.id) { g.graph.router(options(g)).also { it.cancel = rc.cancel } }

        /** Le opzioni di [router] in [g]. */
        fun options(g: CountryGraph): RouteOptions = options.getOrPut(g.id) {
            val o = g.service.routeOptions(inGraph(g, request)).copy(hasBearings = false).let { if (plain) it.copy(avoidFeatures = 0, avoidPenalty = 0.0) else it }
            // a trip across a border is a long one: a limited speed on the times only (the quick search)
            if (!g.graph.coreWide && o.maximumSpeed.isFinite() && o.maximumSpeed >= DirectionsRequest.MAXIMUM_SPEED_LOWER_BOUND) o.copy(maximumSpeedTimeOnly = true) else o
        }

        /**
         * Vero se al punto agganciato [snap] in [g] il tratto seguente dipende da quello che ci arriva:
         * vi si passa senza fermarsi ([through]: `continue_straight` o `pass_through`) a metà di un
         * arco percorribile nei due versi ([Router.directionFree]).
         */
        fun pt(g: CountryGraph, through: Boolean, snap: Snap): Boolean = through && !router(g).directionFree(snap)

        /**
         * L'aggancio di un punto come lo fa il pezzo finale ([exact]: un passaggio di confine, senza il
         * controllo delle reti isolate; [heading]: ORS `bearings`).
         */
        fun snap(g: CountryGraph, p: LL, radius: Double, exact: Boolean = radius == TRANSFER_RADIUS, heading: Double = Double.NaN): Snap? =
            snaps.getOrPut("${g.id}:${p.lat}:${p.lon}:$radius:$exact:$heading") {
                try { g.service.snap(router(g), options(g), 0, p.lat, p.lon, radius, !exact, heading, rc.useCache) } catch (e: PointNotFoundException) { null }
            }

        /**
         * Il peso di un pezzo come lo calcola il percorso finale ([DirectionsService.routes] con tutte le
         * impostazioni della richiesta); +∞ quando non c'è percorso. [token]: l'annullamento della
         * ricerca (un figlio di quello della richiesta, con la scadenza delle varianti).
         */
        fun pieceWeight(g: CountryGraph, piece: DirectionsRequest, token: CancelToken = rc.cancel, extra: Boolean = extraPhase): Double {
            token.check()
            val n0 = synchronized(quiet.stats.legs) { quiet.stats.legs.size }
            val t0 = System.currentTimeMillis()
            val qrc = if (token === rc.cancel) quiet else RequestContext(token, rc.priority, rc.executor, rc.slots, rc.log, quiet.stats, rc.useCache)
            val w = try { g.service.routes(inGraph(g, piece), qrc).single().sumOf { it.weight } } catch (e: ApiException) { Double.POSITIVE_INFINITY }
            // una ricerca solo se un tratto è stato cercato davvero (dalla cache: nessuna)
            val searched = synchronized(quiet.stats.legs) { quiet.stats.legs.drop(n0).count { it.source != LegStats.Source.CACHE } }
            if (searched > 0) { queries++; if (extra) extraQueries++ }
            searchedAll += searched
            if (extra) extraSearched += searched
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    pezzo ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} in $ms ms")
            return w
        }

        /**
         * Vero mentre si pesano i pezzi che servono solo alla fonte V (il passo 2 di [chain], gli extra
         * di [chooseExactTop]): i loro tratti cercati si contano a parte ([extraSearched]).
         */
        var extraPhase = false
        /** I tratti cercati davvero dai pesi dei pezzi per le varianti (compresi nei «cercati» di [pieceStats]). */
        var extraSearched = 0
        /** Tutti i tratti cercati davvero dai pesi dei pezzi (per contare a parte quelli di una sequenza, [chain]). */
        var searchedAll = 0

        /**
         * I pesi dei pezzi ([pieceWeight]) nella riga `stats`: quanti tratti, quanti cercati davvero (e
         * di questi quanti per le varianti: le catene della fonte V sono nuove per costruzione) e i loro ms.
         */
        fun pieceStats(): String {
            val ls = synchronized(quiet.stats.legs) { quiet.stats.legs.toList() }
            val forVariants = if (extraSearched > 0) " ($extraSearched per le varianti)" else ""
            return if (ls.isEmpty()) "" else " (pesi dei pezzi: ${ls.size} tratti, ${ls.count { it.source != LegStats.Source.CACHE }} cercati$forVariants, ${ls.sumOf { it.ms }} ms)"
        }

        /** Le ricerche del passo 2 di [chain] (le altre catene della fonte V), comprese in [queries]. */
        var extraQueries = 0
        /** «N ricerche (M dalla memoria, K per le varianti)». */
        fun queriesText(): String = "$queries ricerche ($hits dalla memoria${if (extraQueries > 0) ", $extraQueries per le varianti" else ""})"

        /**
         * The weight from [a] to [b] in [g], or +∞ when there is no path lighter than [limit]. Dalla
         * memoria della richiesta o, con [RequestContext.useCache], da quella del servizio
         * ([DirectionsService.quickCache]: la richiesta delle varianti rifà la scelta senza ricerche).
         */
        fun weight(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, limit: Double = Double.POSITIVE_INFINITY): Double {
            rc.cancel.check()
            val key = weightKey(g, a, b)
            while (true) {
                remembered(g, a, ra, b, rb, limit)?.let { return it }
                // la stessa ricerca in corso sull'altro thread di [chain]: si aspetta il suo peso (poi
                // dalla memoria; se l'ha cercato sotto un limite più basso del nostro, la si rifà qui)
                val other = memory.claim(key) ?: break
                while (!other.await(WAIT_MS, TimeUnit.MILLISECONDS)) rc.cancel.check()
            }
            try {
                // l'altro thread può averlo finito fra la memoria e la chiave: niente seconda ricerca
                remembered(g, a, ra, b, rb, limit)?.let { return it }
                return search(g, a, ra, b, rb, limit)
            } finally { memory.release(key) }
        }

        /** La ricerca rapida da [a] a [b] sotto [limit], ricordata (chi chiama tiene la sua chiave, [Memory.claim]). */
        private fun search(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, limit: Double): Double {
            val sa = snap(g, a, ra) ?: return Double.POSITIVE_INFINITY
            val sb = snap(g, b, rb) ?: return Double.POSITIVE_INFINITY
            queries++
            val t0 = System.currentTimeMillis()
            // for choosing: the quick search's path also when it uses a road to avoid (the pieces are then exact)
            val w = try { router(g).route(sa, sb, maxWeight = limit, exact = false).weight } catch (e: RouteNotFoundException) { Double.POSITIVE_INFINITY }
            val ms = System.currentTimeMillis() - t0
            if (ms > 300) log?.invoke("    ricerca ${g.id} ${if (w.isFinite()) "%.0f".format(w) else "∞"} (limite ${if (limit.isFinite()) "%.0f".format(limit) else "∞"}) in $ms ms: ${router(g).lastAlgorithm} ${router(g).lastSearch}")
            remember(g, a, ra, b, rb, w, limit)
            return w
        }

        /**
         * Con due thread su sequenze che cominciano dallo stesso confine ([Memory.shared]): i pesi da
         * [a] ai passaggi [bs] che nessuno sa né sta cercando, saltando quelli in corso sull'altro
         * thread (così il primo strato, comune, si divide fra i due invece di farlo due volte). Gli
         * stessi pesi di [weight] sotto lo stesso [limit]: [dpChain] poi li trova in memoria.
         */
        fun warm(g: CountryGraph, a: LL, ra: Double, bs: List<LL>, rb: Double, limit: () -> Double) {
            if (!memory.shared) return
            for (b in bs) {
                rc.cancel.check()
                val key = weightKey(g, a, b)
                if (memory.isRunning(key)) continue
                val l = limit()
                if (remembered(g, a, ra, b, rb, l) != null) continue
                if (memory.claim(key) != null) continue
                try { if (remembered(g, a, ra, b, rb, l) == null) search(g, a, ra, b, rb, l) } finally { memory.release(key) }
            }
        }

        /**
         * Le opzioni della chiave della memoria dei pesi rapidi: quelle che danno lo stesso [router]
         * ([RoutingGraph.routerKey], come gli agganci e i tratti). Col core preparato `continue_straight`
         * (che `pass_through` accende) non cambia la ricerca rapida: una via aggiunta non svuota la memoria.
         */
        private val quickOptions = HashMap<String, RouteOptions>()
        private fun quickKey(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double): QuickKey? = if (!rc.useCache) null else
            QuickKey(quickOptions.getOrPut(g.id) { g.graph.routerKey(options(g)) }, a.lat.toBits(), a.lon.toBits(), ra.toBits(), b.lat.toBits(), b.lon.toBits(), rb.toBits())
        private fun weightKey(g: CountryGraph, a: LL, b: LL) = "${g.id}:${a.lat}:${a.lon}>${b.lat}:${b.lon}"

        /** Il peso da [a] a [b] sotto [limit] se la memoria (della richiesta o del servizio) lo sa (+∞: nessun percorso sotto [limit]); null se va cercato. */
        private fun remembered(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, limit: Double): Double? {
            val key = weightKey(g, a, b)
            memory.get(key)?.let { (w, l) -> if (w.isFinite() || l >= limit) return if (w < limit) w else Double.POSITIVE_INFINITY }
            val qk = quickKey(g, a, ra, b, rb) ?: return null
            return g.service.quickCache.get(qk, limit)?.also { w -> hits++; memory.put(key, w, limit) }
        }

        /**
         * La voce della memoria (della richiesta o del servizio) da [a] a [b] così com'è: il peso e il limite
         * sotto cui è stato cercato (∞: nessun percorso più leggero del limite); null se nessuna. Per i
         * percorsi mirati di [Reach], che col peso esatto o col limite dimostrato fanno a meno della ricerca.
         */
        fun known(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double): Pair<Double, Double>? {
            memory.get(weightKey(g, a, b))?.let { return it }
            val qk = quickKey(g, a, ra, b, rb) ?: return null
            return g.service.quickCache.entry(qk)?.also { (w, l) -> memory.put(weightKey(g, a, b), w, l) }
        }

        /** Ricorda [w] cercato sotto [limit] (+∞ con [limit]: nessun percorso più leggero di [limit]). */
        fun remember(g: CountryGraph, a: LL, ra: Double, b: LL, rb: Double, w: Double, limit: Double) {
            memory.put(weightKey(g, a, b), w, limit)
            quickKey(g, a, ra, b, rb)?.let { g.service.quickCache.put(it, w, limit) }
        }

        /**
         * Il passaggio migliore di uno strato verso [b] in [g]: fra i passaggi [froms] (raggio
         * [TRANSFER_RADIUS]), ognuno col peso già percorso [offsets] (in ordine crescente), il minimo
         * di offset + peso fino a [b] sotto [limit] e il suo indice (il primo a parità); (-1, +∞) se
         * nessuno sta sotto. Lo stesso risultato di una [weight] per passaggio, in ordine, ognuna
         * sotto il minimo del momento: gli stessi pesi (dalla memoria quando ci sono), ma per quelli
         * che mancano una ricerca sola all'indietro da [b] ([Router.manyToOne], gli stessi pesi di
         * [Router.route]) invece di una per passaggio. La ricerca va avanti finché ogni passaggio ha
         * il peso definitivo o il suo offset più il limite inferiore non può più scendere sotto il
         * minimo trovato (né sotto [limit]). Senza core, una [weight] per passaggio come prima.
         */
        fun bestTo(g: CountryGraph, froms: List<LL>, offsets: List<Double>, b: LL, rb: Double, limit: Double): Pair<Int, Double> {
            rc.cancel.check()
            val n = froms.size
            val d = DoubleArray(n) { Double.NaN }
            // prima la memoria: i pesi noti (finiti), poi, sotto il minimo che danno, quelli dimostrati oltre
            // (una voce ∞ ricorda solo che non c'è percorso sotto un limite: quello del minimo di allora)
            var cut = limit
            val need = ArrayList<Int>()
            for (i in 0 until n) {
                if (offsets[i] >= limit) { d[i] = Double.POSITIVE_INFINITY; continue }
                val m = remembered(g, froms[i], TRANSFER_RADIUS, b, rb, Double.POSITIVE_INFINITY)
                if (m == null) { need += i; continue }
                d[i] = m
                if (offsets[i] + m < cut) cut = offsets[i] + m
            }
            need.retainAll { i ->
                if (offsets[i] >= cut) { d[i] = Double.POSITIVE_INFINITY; return@retainAll false }
                val m = remembered(g, froms[i], TRANSFER_RADIUS, b, rb, cut - offsets[i]) ?: return@retainAll true
                d[i] = m
                if (offsets[i] + m < cut) cut = offsets[i] + m
                false
            }
            if (need.isNotEmpty()) {
                val sb = snap(g, b, rb)
                val idx = if (sb == null) emptyList() else need.filter { i -> offsets[i] < cut && snap(g, froms[i], TRANSFER_RADIUS) != null }
                for (i in need) if (i !in idx) d[i] = Double.POSITIVE_INFINITY
                val m = if (idx.isEmpty()) null else router(g).manyToOne(sb!!, idx.map { snap(g, froms[it], TRANSFER_RADIUS)!! }, rootIsTarget = true)
                if (m == null) {
                    // senza core: una ricerca per passaggio, sotto il minimo del momento
                    for (i in idx) {
                        if (offsets[i] >= cut) { d[i] = Double.POSITIVE_INFINITY; continue }
                        d[i] = weight(g, froms[i], TRANSFER_RADIUS, b, rb, cut - offsets[i])
                        if (offsets[i] + d[i] < cut) cut = offsets[i] + d[i]
                    }
                } else {
                    queries++
                    val t0 = System.currentTimeMillis()
                    // finché ogni passaggio ha il suo peso o non può più vincere (il minimo scende coi pesi definitivi)
                    m.advance {
                        var c = cut
                        for ((k, i) in idx.withIndex()) if (m.isFinal(k)) { val x = offsets[i] + m.weight(k); if (x < c) c = x }
                        idx.indices.all { k -> m.isFinal(k) || offsets[idx[k]] + m.lowerBound(k) >= c }
                    }
                    for ((k, i) in idx.withIndex()) {
                        if (m.isFinal(k)) {
                            d[i] = m.weight(k)
                            remember(g, froms[i], TRANSFER_RADIUS, b, rb, d[i], Double.POSITIVE_INFINITY)
                        } else {
                            // nessun percorso più leggero del limite inferiore, che lo mette già oltre il minimo
                            d[i] = Double.POSITIVE_INFINITY
                            remember(g, froms[i], TRANSFER_RADIUS, b, rb, Double.POSITIVE_INFINITY, m.lowerBound(k))
                        }
                    }
                    val ms = System.currentTimeMillis() - t0
                    if (ms > 300) log?.invoke("    ricerca ${g.id} da ${idx.size} passaggi verso %.5f,%.5f in $ms ms: ${m.stats}".format(b.lat, b.lon))
                }
            }
            // il minimo sotto il limite, il primo a parità (come le ricerche una per una in ordine)
            var best = -1; var w = limit
            for (i in 0 until n) { val x = offsets[i] + d[i]; if (x < w) { w = x; best = i } }
            return if (best < 0) -1 to Double.POSITIVE_INFINITY else best to w
        }
    }

    /**
     * La risposta GeoJSON di openrouteservice per [request]; [pointGraph]: il Paese (id del grafo) di
     * ogni punto quando è noto, altrimenti il grafo che lo aggancia più vicino. [rc]: l'annullamento
     * (le ricerche lanciano [SearchCancelledException][it.navmaster.engine.routing.SearchCancelledException]),
     * un posto di ricerca per tutto il viaggio, le misure dei pezzi.
     */
    fun handle(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext = RequestContext.NONE): Map<String, Any?> {
        val started = System.nanoTime()
        try { return rc.searching { handleIn(request, pointGraph, rc) } } finally {
            rc.stats.profile = request.profile; rc.stats.points = request.coordinates.size
            rc.stats.totalMs = (System.nanoTime() - started) / 1_000_000
        }
    }

    private fun handleIn(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext): Map<String, Any?> {
        if (!request.avoidPenalty || request.avoidFeatures == 0) return route(request, pointGraph, null, rc)
        // "avoid" as a penalty. The countries where the ban hampers a point of the trip (a lorry for
        // Bratislava: from Austria the last stretch in the city is a toll road; with the ban the trip
        // came round through Hungary, Vienna → Bratislava 705 km) and those where a point is on a
        // road to avoid: there the penalty, elsewhere the ban with its landmarks (the penalty in every
        // country took 128 s on the tablet from Lombardy to Bratislava); with none, the ban everywhere
        val started = System.currentTimeMillis()
        val pool = bestPool
        val onRoad = request.coordinates.indices.flatMap { i -> pool.filter { it.service.onAvoidedRoad(request, i) }.map { it.id } }.toSet()
        val shut = onRoad + if (shutInCheck) shutIn(request, pointGraph, rc) else emptySet()
        log?.invoke("evita come penalità in: ${shut.ifEmpty { setOf("nessun Paese") }.joinToString()} (${System.currentTimeMillis() - started} ms)")
        rc.stats.addCrossings("dove la penalità ${System.currentTimeMillis() - started} ms")
        try {
            return if (shut.isEmpty()) route(request.withAvoidPenalty(false), pointGraph, null, rc) else route(request, pointGraph, shut, rc)
        } catch (e: ApiException) {
            if (!notFound(e)) throw e
            log?.invoke("evita: nessun percorso (${e.code}), come penalità in tutti i Paesi")
            rc.stats.profile = request.profile; rc.stats.points = request.coordinates.size
            rc.stats.totalMs = System.currentTimeMillis() - started
            rc.stats.setAside("divieto scartato")
        }
        return route(request, pointGraph, null, rc)
    }

    private fun notFound(e: ApiException) = e.code == RoutingErrorCodes.ROUTE_NOT_FOUND || e.code == RoutingErrorCodes.POINT_NOT_FOUND

    /**
     * The countries where the ban hampers a point of the trip: of the crossings within [LOCAL_REACH]
     * of the point without the roads to avoid (from it; to it for the arrival; both for a stop), one
     * that the ban makes unreachable or much farther ([LOCAL_DETOUR] times plus [LOCAL_SLACK]). A
     * crossing on a road to avoid does not count (another one takes its place). Only near the points:
     * a country blocked in transit is not found here (then the penalty everywhere, see [handle]).
     */
    private fun shutIn(request: DirectionsRequest, pointGraph: List<String?>, rc: RequestContext): Set<String> {
        val ban = Ctx(request.withAvoidPenalty(false), rc = rc)
        val free = Ctx(request.withAvoidPenalty(false), plain = true, rc = rc)
        val out = LinkedHashSet<String>()
        val last = request.coordinates.lastIndex
        // solo i grafi del migliore ([bestPool]): un valico verso un Paese aperto solo per le varianti non conta
        val pool = bestPool
        for ((i, c) in request.coordinates.withIndex()) {
            rc.cancel.check()
            val t0 = System.currentTimeMillis()
            val p = LL(c.first, c.second)
            val radius = request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius
            // the point's country as the trip takes it (a point on a road to avoid: already penalised)
            val hint = pointGraph.getOrNull(i)?.let { id -> pool.firstOrNull { it.id == id } }
            val (g, sb) = (if (hint != null) listOf(hint) else pool.filter { it.graph.graph.bounds.contains(p.lat, p.lon) })
                .mapNotNull { g -> ban.snap(g, p, radius)?.let { g to it } }.minByOrNull { it.second.distance } ?: continue
            if (g.id in out) continue
            val rb = ban.router(g); val r0 = free.router(g)
            // only where the searches keep to the ban (the wide core; the reduced one searches without it)
            if (!rb.searchesExact) continue
            val s0 = free.snap(g, p, radius) ?: continue
            // the crossings usable with the ban, in both countries
            val exits = pool.filter { it !== g && it.graph.graph.bounds.intersects(g.graph.graph.bounds) }
                .flatMap { n -> transfers(g, n).map { n to LL(it.lat, it.lon) } }
                .filter { (n, l) -> ban.snap(g, l, TRANSFER_RADIUS) != null && ban.snap(n, l, TRANSFER_RADIUS) != null }
                .map { it.second }
            if (exits.isEmpty()) continue
            val in0 = exits.map { free.snap(g, it, TRANSFER_RADIUS)!! }
            val inB = exits.map { ban.snap(g, it, TRANSFER_RADIUS)!! }
            for (toPoint in when (i) { 0 -> listOf(false); last -> listOf(true); else -> listOf(false, true) }) {
                val m0 = r0.manyToOne(s0, in0, rootIsTarget = toPoint) ?: break
                m0.advance { m0.bound > LOCAL_REACH }
                val near = exits.indices.filter { m0.isFinal(it) && m0.weight(it) <= LOCAL_REACH }
                if (near.isEmpty()) continue
                val mb = rb.manyToOne(sb, inB, rootIsTarget = toPoint) ?: break
                fun limit(k: Int) = m0.weight(k) * LOCAL_DETOUR + LOCAL_SLACK
                // only until every near crossing has its weight or one is proven beyond its limit
                mb.advance { near.all { mb.isFinal(it) } || near.any { !mb.isFinal(it) && mb.bound > limit(it) } }
                val hampered = near.firstOrNull { k -> if (mb.isFinal(k)) mb.weight(k) > limit(k) else mb.bound > limit(k) }
                if (hampered != null) {
                    log?.invoke("  evita: il punto $i in ${g.id} col divieto arriva male al passaggio %.5f,%.5f (%.0f invece di %.0f)".format(
                        exits[hampered].lat, exits[hampered].lon, if (mb.isFinal(hampered)) mb.weight(hampered) else Double.POSITIVE_INFINITY, m0.weight(hampered)))
                    out += g.id; break
                }
            }
            log?.invoke("  evita: punto $i in ${g.id}, ${System.currentTimeMillis() - t0} ms")
        }
        return out
    }

    /** [handle] with "avoid" as a penalty only in the countries of [penalized] (null: all). */
    /**
     * I grafi preparati con modelli di priorità diversi ([it.navmaster.engine.profile.PriorityModel]: un
     * Paese ricostruito accanto a uno di prima), null se tutti uguali. I pesi si sommano fra Paesi così
     * come sono: con i modelli misti non sono confrontabili e la scelta dei valichi pende verso i Paesi
     * coi grafi di prima (premi più forti). Non si può correggere qui (il rapporto cambia per tipo di
     * strada): si dice, e l'app fa aggiornare i Paesi insieme.
     */
    val mixedPriority: String? by lazy {
        val byModel = graphs.groupBy { it.graph.encoding.priorityModel }
        if (byModel.size < 2) null else byModel.entries.joinToString("; ") { (m, gs) -> "${m.id}: ${gs.joinToString(", ") { it.id }}" }
    }

    private fun route(request: DirectionsRequest, pointGraph: List<String?>, penalized: Set<String>?, rc: RequestContext): Map<String, Any?> {
        mixedPriority?.let { log?.invoke("ATTENZIONE grafi con priorità diverse ($it): scelta dei valichi falsata, vanno aggiornati i Paesi coi grafi di prima") }
        val started = System.currentTimeMillis()
        val ctx = Ctx(request, penalized, rc = rc)
        // varianti NavMaster: fra due Paesi anche gli altri valichi (fonte V); il loro tempo (budget_ms)
        // conta dall'inizio della richiesta, così comprende la parte della scelta dei valichi che serve solo a loro.
        // Quella parte (le altre catene, prima dei pezzi finali) ha un tetto suo, TOP_BUDGET_SHARE del tempo:
        // alle fonti M e P dei pezzi resta sempre il resto, anche quando le altre sequenze costano a freddo
        val nav = request.navAlternatives
        val navStart = System.nanoTime()
        val navBudget = nav?.let { rc.cancel.child(navStart + it.budgetMs * 1_000_000) }
        if (nav != null && VariantSource.CROSSING in nav.sources && nav.count > 1) {
            ctx.topK = nav.count; ctx.topFactor = nav.maxFactor
            // mai oltre budget_ms meno la coda dei minimi garantiti ([variantsTailMs]): la fonte V e le fonti M e P
            // del pezzo più lungo hanno il loro minimo dentro budget_ms, non dopo
            ctx.topDeadline = navStart + minOf((nav.budgetMs * TOP_BUDGET_SHARE).toLong(), nav.budgetMs - variantsTailMs(nav.budgetMs)) * 1_000_000
            ctx.topBudget = rc.cancel.child(ctx.topDeadline)
            ctx.navStart = navStart; ctx.navBudgetMs = nav.budgetMs
        }
        val pts = request.coordinates.map { LL(it.first, it.second) }
        val radius = { i: Int -> request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius }
        // il grafo di ogni punto fra quelli del migliore ([bestPool]), come nella richiesta senza i grafi delle varianti
        val pool = bestPool
        val of = pts.indices.map { i ->
            val wanted = pointGraph.getOrNull(i)?.let { id -> pool.firstOrNull { it.id == id } }
            val g = wanted?.takeIf { ctx.snap(it, pts[i], radius(i)) != null } ?: pool
                .filter { it.graph.graph.bounds.contains(pts[i].lat, pts[i].lon) }
                .mapNotNull { g -> ctx.snap(g, pts[i], radius(i))?.let { s -> g to s.distance } }
                .minByOrNull { it.second }?.first
            g ?: throw ApiException(404, RoutingErrorCodes.POINT_NOT_FOUND,
                "Could not find routable point within a radius of ${radius(i).toLong()} meters of specified coordinate $i: %.7f %.7f.".format(pts[i].lon, pts[i].lat))
        }
        log?.invoke("punti nei grafi: ${of.joinToString { it.id }}")
        if (of.all { it === of[0] }) {
            // un Paese solo: con le varianti (fonte V) anche il giro per un Paese vicino (A-B-A), come variante; il migliore resta quello del Paese
            val around = if (nav != null && ctx.topK > 1 && pts.size == 2 && request.alternatives == null) countryPaths(of[0], of[0], pts[0], pts[1], all = true) else emptyList()
            if (around.isEmpty()) return of[0].service.handle(ctx.inGraph(of[0], request), ResponseFormat.GEOJSON, rc)
            return aroundTrip(ctx, request, of[0], around, radius, rc, nav!!, navStart, navBudget!!, started)
        }
        // alternatives across borders: the via points method (each candidate is a trip like this one)
        if (request.alternatives != null && pts.size == 2) {
            return LongAlternatives(graphs.map { it.graph }, log, cancel = rc.cancel) { r, pg -> route(r, pg, penalized, rc) }.handle(request, of.map { it.id })
        }

        // i pezzi del viaggio: le tappe nello stesso grafo restano insieme, ai confini il passaggio migliore
        val pieces = ArrayList<Piece>()
        var cur = Piece(of[0], mutableListOf(request.coordinates[0]), mutableListOf(radius(0)), bearings = mutableListOf(request.bearings?.getOrNull(0)),
            passThrough = mutableListOf(request.passThroughAt(0)))
        // i blocchi di punti consecutivi nello stesso grafo; con due confini i due valichi si scelgono insieme
        val blocks = ArrayList<Pair<CountryGraph, IntRange>>()
        run { var s = 0; for (i in 1..pts.size) if (i == pts.size || of[i] !== of[s]) { blocks += of[s] to (s until i); s = i } }
        var twoDone = false
        // con le varianti (fonte V) la scelta per confine ([chain]), che registra le altre catene: con la tappa
        // nel Paese di mezzo i due valichi sono indipendenti, il migliore è lo stesso. Non con una tappa di
        // passaggio (pass_through) nel Paese di mezzo: il verso d'arrivo dal primo valico condiziona la
        // ripartenza verso il secondo, i due valichi vanno scelti insieme come nella richiesta del migliore
        val middlePass = blocks.size == 3 && blocks[1].second.any { request.passThroughAt(it) }
        if (blocks.size == 3 && oneSearch && (ctx.topK <= 1 || middlePass)) {
            val (g0, r0) = blocks[0]; val (g1, r1) = blocks[1]; val (g2, r2) = blocks[2]
            // solo coi due confini diretti e senza sequenze alternative per un terzo Paese (quelle le pesa chain)
            fun direct(x: CountryGraph, y: CountryGraph, i: Int, j: Int) = countryPaths(x, y, pts[i], pts[j]).let { it.isNotEmpty() && it.all { pa -> pa.size == 2 } }
            if (direct(g0, g1, r0.last, r1.first) && direct(g1, g2, r1.last, r2.first)) {
                for (i in r0.drop(1)) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
                val two = chooseTwo(ctx, cur, r1, g1, r2, g2, radius, { Double.POSITIVE_INFINITY })
                if (two != null) {
                    if (two.first == null) throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${r0.last} (%.7f %.7f) and ${r1.first} (%.7f %.7f).".format(
                        pts[r0.last].lon, pts[r0.last].lat, pts[r1.first].lon, pts[r1.first].lat))
                    val (x, y, w) = two
                    lastWeight = w; lastCrossings = listOf(x!!, y!!)
                    cur.points += x.lat to x.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
                    pieces += cur
                    cur = Piece(g1, mutableListOf(x.lat to x.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
                    for (i in r1) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
                    cur.points += y.lat to y.lon; cur.radiuses += TRANSFER_RADIUS; cur.exact += true; cur.bearings += null; cur.passThrough += false
                    pieces += cur
                    cur = Piece(g2, mutableListOf(y.lat to y.lon), mutableListOf(TRANSFER_RADIUS), mutableListOf(true), mutableListOf(null), mutableListOf(false))
                    for (i in r2) { cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i) }
                    twoDone = true
                }
            }
        }
        // con le varianti e la tappa nel Paese di mezzo la scelta va per confine (sopra): se la richiesta del migliore ha
        // scelto i due valichi insieme e la memoria li ha, il migliore è noto e le ricerche per confine servono alla fonte V
        // (prima contavano come «rifatte»: Milano → Innsbruck → Monaco, 3-19 ricerche). Solo se i valichi trovati per
        // confine sono quelli della memoria: se no il migliore della risposta l'hanno deciso loro e restano del migliore
        var knownTwo: TransitCache.Entry? = null
        val chosenX = ArrayList<Transfer>()
        if (!twoDone && blocks.size == 3 && oneSearch && ctx.topK > 1 && !middlePass) {
            val (g0, r0) = blocks[0]; val (g1, r1) = blocks[1]; val (g2, r2) = blocks[2]
            val pre = cur.copy()
            for (i in r0.drop(1)) { pre.points += request.coordinates[i]; pre.radiuses += radius(i); pre.exact += false; pre.bearings += request.bearings?.getOrNull(i); pre.passThrough += request.passThroughAt(i) }
            val e = transitKey(ctx, pre, r1, g1, r2, g2, radius)?.let { g0.service.transitCache.get(it) }
            if (e != null && e.w.isFinite()) {
                ctx.bestKnown = true
                knownTwo = e
                log?.invoke("varianti: i due valichi del migliore dalla memoria (%.5f,%.5f e %.5f,%.5f, peso %.0f): le scelte per confine servono alla fonte V".format(e.x!!.lat, e.x.lon, e.y!!.lat, e.y.lon, e.w))
            }
        }
        if (twoDone) { /* pezzi già fatti */ } else for (i in 1 until pts.size) {
            if (of[i] !== cur.graph) {
                // i punti dopo il confine nello stesso grafo (il pezzo che comincia al valico)
                var end = i
                while (end + 1 < pts.size && of[end + 1] === of[i]) end++
                val chain = chain(ctx, cur, i..end, radius, of[i])
                    ?: throw ApiException(404, RoutingErrorCodes.ROUTE_NOT_FOUND, "Unable to find a route between points ${i - 1} (%.7f %.7f) and $i (%.7f %.7f).".format(
                        pts[i - 1].lon, pts[i - 1].lat, pts[i].lon, pts[i].lat))
                chosenX += chain.map { it.first }
                // i pezzi del migliore che questa scelta copre (da cur al pezzo che comincia all'ultimo valico) e le sue catene
                val first = pieces.size
                val (closed, next) = piecesOf(cur, chain)
                pieces += closed
                cur = next
                ctx.spans += Span(first, pieces.size, ctx.chains)
            }
            cur.points += request.coordinates[i]; cur.radiuses += radius(i); cur.exact += false; cur.bearings += request.bearings?.getOrNull(i); cur.passThrough += request.passThroughAt(i)
        }
        pieces += cur
        knownTwo?.let { e ->
            ctx.bestKnown = false
            val same = chosenX.size == 2 && chosenX[0].lat == e.x!!.lat && chosenX[0].lon == e.x.lon && chosenX[1].lat == e.y!!.lat && chosenX[1].lon == e.y.lon
            if (same) {
                ctx.extraQueries += ctx.knownQueries; ctx.extraSearched += ctx.knownSearched
                log?.invoke("varianti: valichi per confine uguali a quelli della memoria: ${ctx.knownQueries} ricerche del passo 1 per la fonte V")
            } else log?.invoke("varianti: valichi per confine diversi da quelli della memoria (${chosenX.joinToString { "%.5f,%.5f".format(it.lat, it.lon) }}): " +
                "le ${ctx.knownQueries} ricerche del passo 1 restano del migliore")
            ctx.knownQueries = 0; ctx.knownSearched = 0
        }
        val pieceRequests = pieces.map { p -> p.graph to requestOf(request, p) }
        lastPieces = pieceRequests
        rc.stats.addCrossings("${pieces.joinToString(" → ") { it.graph.id }}: ${ctx.queriesText()} in ${System.currentTimeMillis() - started} ms${ctx.pieceStats()}")
        rc.cancel.check()
        // i pezzi finali: i tratti dalla cache (i pesi dei pezzi li hanno già cercati), su due thread con l'aiutante
        val finals = finalPieces(ctx, pieceRequests, rc)
        log?.invoke("viaggio tra Paesi: ${pieces.joinToString(" → ") { it.graph.id }}, ${ctx.queriesText()}, ${System.currentTimeMillis() - started} ms")
        val feature = merge(finals.map { featureOf(it) }, request)
        @Suppress("UNCHECKED_CAST")
        val bbox = feature["bbox"] as DoubleArray
        val metadata = LinkedHashMap<String, Any?>()
        metadata["attribution"] = pieces[0].graph.service.attribution
        metadata["service"] = "routing"
        metadata["timestamp"] = System.currentTimeMillis()
        metadata["query"] = LinkedHashMap<String, Any?>(request.echo).apply { put("profile", request.profile); put("format", "geojson") }
        metadata["engine"] = pieces[0].graph.service.engineInfo()
        metadata["countries"] = pieces.map { it.graph.id }
        mixedPriority?.let { metadata["mixed_priority"] = it }
        if (nav != null) {
            val (features, alternatives) = variantsAcross(ctx, request, pieces, pieceRequests, finals, feature, nav, rc, navStart, navBudget!!)
            metadata["navmaster"] = linkedMapOf("alternatives" to alternatives)
            val all = BBox.empty()
            for (f in features) { val b = f["bbox"] as DoubleArray; all.include(b[1], b[0]); all.include(b[3], b[2]) }
            val bboxAll = if (features.isEmpty()) doubleArrayOf() else
                doubleArrayOf(DirectionsService.r6(all.minLon), DirectionsService.r6(all.minLat), DirectionsService.r6(all.maxLon), DirectionsService.r6(all.maxLat))
            return linkedMapOf("type" to "FeatureCollection", "bbox" to bboxAll, "features" to features, "metadata" to metadata)
        }
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bbox, "features" to listOf(feature), "metadata" to metadata)
    }

    /**
     * Il tempo del passo 2 ([Ctx.topBudget]) dopo il passo 1 ([topDeadlineAfter]): un passo 1 lento, col
     * grafo appena riaperto o la memoria svuotata, non lascia il passo 2 senza tempo, ma il passo 2 non
     * prende mai più di metà del tempo che resta: l'altra metà, almeno, resta alle fonti M e P.
     */
    private fun extendTopBudget(ctx: Ctx) {
        val d = topDeadlineAfter(ctx.navStart, ctx.navBudgetMs, ctx.topDeadline, System.nanoTime())
        if (d <= ctx.topDeadline) return
        log?.invoke("varianti: tempo delle altre catene allungato di ${(d - ctx.topDeadline) / 1_000_000} ms (passo 1 lento)")
        ctx.topDeadline = d
        ctx.topBudget = ctx.rc.cancel.child(d)
    }

    /**
     * Un viaggio in un Paese solo ([g], due punti) con le varianti: il migliore è quello del Paese
     * (come [DirectionsService.handle]: lo stesso percorso, gli «evita» come prima), le varianti dei
     * pezzi (fonti M e P) e in più i giri per i Paesi vicini [around] (A-B-A: la A93/A8 tedesca da
     * Innsbruck a Salisburgo) come catene della fonte V, pesate come il passo 2 di [chain] (strato
     * corto, un passaggio per luogo, sotto il tempo delle varianti) entro il tetto di peso delle
     * varianti; la scelta è quella di [variantsAcross]. Un giro più veloce del migliore resta una variante.
     */
    private fun aroundTrip(ctx: Ctx, request: DirectionsRequest, g: CountryGraph, around: List<List<CountryGraph>>, radius: (Int) -> Double,
                           rc: RequestContext, nav: NavAlternativesOptions, navStart: Long, navBudget: CancelToken, started: Long): Map<String, Any?> {
        val c0 = request.coordinates[0]; val c1 = request.coordinates[1]
        val piece = Piece(g, mutableListOf(c0, c1), mutableListOf(radius(0), radius(1)), mutableListOf(false, false),
            mutableListOf(request.bearings?.getOrNull(0), request.bearings?.getOrNull(1)), mutableListOf(request.passThroughAt(0), request.passThroughAt(1)))
        val pieces = listOf(piece)
        val pieceRequests = listOf(g to requestOf(request, piece))
        lastPieces = pieceRequests
        // il migliore del Paese (i tratti in cache dopo la richiesta senza varianti) e il suo peso, come quello delle catene
        val finals = finalPieces(ctx, pieceRequests, rc)
        val bestW = ctx.pieceWeight(g, pieceRequests[0].second)
        val bestChoice = ChainChoice(listOf(g), emptyList(), bestW)
        val picked = ArrayList<ChainChoice>()
        if (bestW.isFinite()) {
            val t0 = System.currentTimeMillis(); val q0 = ctx.queries; val x0 = ctx.extraQueries
            val cap = bestW * ctx.topWeightFactor
            val prefix = Piece(g, mutableListOf(c0), mutableListOf(radius(0)), mutableListOf(false), mutableListOf(request.bearings?.getOrNull(0)), mutableListOf(request.passThroughAt(0)))
            val p = LL(c0.first, c0.second); val q = LL(c1.first, c1.second)
            val found = ArrayList<ChainChoice>()
            for (path in around) {
                if (ctx.topBudget?.isCancelled == true) { ctx.topStopped = true; break }
                val routers = path.distinct().map { ctx.router(it) }
                val cancels = routers.map { it.cancel }
                ctx.topBudget?.let { tk -> routers.forEach { it.cancel = tk } }
                ctx.extraPhase = true
                val before = found.size
                try {
                    dpChain(ctx, prefix, 1..1, radius, p, radius(0), q, radius(1), path, { cap }, { _, _ -> }, found,
                        layerSize = LAYER_EXTRA, onePerPlace = true, token = ctx.topBudget, maxExact = ctx.topK - 1, together = false, skip = bestChoice)
                } catch (e: SearchCancelledException) {
                    if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
                    ctx.topStopped = true
                } finally { routers.forEachIndexed { i, r -> r.cancel = cancels[i] }; ctx.extraPhase = false }
                log?.invoke("  varianti: giro ${path.joinToString("-") { it.id }}: ${found.size - before} catene esatte entro ${"%.2f".format(ctx.topWeightFactor)}× del peso")
            }
            for (c in found.filter { it.exact < cap }.sortedBy { it.exact }) if (picked.none { it.samePlaces(c) } && picked.size < ctx.topK - 1) picked += c
            ctx.extraQueries = x0 + (ctx.queries - q0)
            log?.invoke("varianti: giri per Paesi vicini: ${picked.size}" + picked.joinToString("") { " (${it.text()}: peso ${"%.0f".format(it.exact)}, ${"%+d".format(Math.round((it.exact / bestW - 1) * 100))} %)" } +
                ", ${ctx.queries - q0} ricerche in ${System.currentTimeMillis() - t0} ms" + if (ctx.topStopped) "; tempo finito" else "")
        }
        ctx.chains = listOf(bestChoice) + picked
        lastChains = ctx.chains.map { ChainInfo(it.path.map { x -> x.id }, it.via, it.exact) }
        lastWeight = bestW; lastCrossings = emptyList()
        if (picked.isNotEmpty()) ctx.spans += Span(0, 0, ctx.chains)
        rc.stats.addCrossings("${g.id} (giri per ${around.joinToString { it[1].id }}): ${ctx.queriesText()} in ${System.currentTimeMillis() - started} ms${ctx.pieceStats()}")
        val feature = merge(finals.map { featureOf(it) }, request)
        val metadata = LinkedHashMap<String, Any?>()
        metadata["attribution"] = g.service.attribution
        metadata["service"] = "routing"
        metadata["timestamp"] = System.currentTimeMillis()
        metadata["query"] = LinkedHashMap<String, Any?>(request.echo).apply { put("profile", request.profile); put("format", "geojson") }
        metadata["engine"] = g.service.engineInfo()
        metadata["countries"] = listOf(g.id)
        val (features, alternatives) = variantsAcross(ctx, request, pieces, pieceRequests, finals, feature, nav, rc, navStart, navBudget)
        metadata["navmaster"] = linkedMapOf("alternatives" to alternatives)
        val all = BBox.empty()
        for (f in features) { val b = f["bbox"] as DoubleArray; all.include(b[1], b[0]); all.include(b[3], b[2]) }
        val bboxAll = if (features.isEmpty()) doubleArrayOf() else
            doubleArrayOf(DirectionsService.r6(all.minLon), DirectionsService.r6(all.minLat), DirectionsService.r6(all.maxLon), DirectionsService.r6(all.maxLat))
        return linkedMapOf("type" to "FeatureCollection", "bbox" to bboxAll, "features" to features, "metadata" to metadata)
    }

    /** Un pezzo finale: il grafo, i tratti ([DirectionsService.solveAsHandled]) e la sua feature (null: non ancora costruita, vedi [featureOf]). */
    private class FinalPiece(val graph: CountryGraph, val solved: DirectionsService.Solved, val feature: Map<String, Any?>?) {
        val millis: Long = solved.legs.sumOf { it.millis }
        val meters: Double = solved.legs.sumOf { it.distance }
        val weight: Double = solved.legs.sumOf { it.weight }
    }

    /** La feature del pezzo [p]: quella di [finalPieces] o, se non l'ha costruita, costruita ora. */
    private fun featureOf(p: FinalPiece): Map<String, Any?> =
        p.feature ?: p.graph.service.feature(p.solved.request, p.solved.legs, p.graph.service.encoder(p.solved.request), null)

    /**
     * I pezzi finali di [pieceRequests]: i tratti come li dà la risposta di un grafo solo
     * ([DirectionsService.solveAsHandled]: dalla cache dei tratti, che i pesi dei pezzi per la scelta
     * dei valichi hanno riempito) e, con [features], la feature di ciascuno; su due thread se c'è un
     * posto da aiutante. Con un errore, quello del pezzo d'indice minimo. Le misure di ogni pezzo in
     * una parte di [RequestContext.stats] ([names], o l'id del grafo).
     */
    private fun finalPieces(ctx: Ctx, pieceRequests: List<Pair<CountryGraph, DirectionsRequest>>, rc: RequestContext, names: List<String>? = null,
                            features: Boolean = true): List<FinalPiece> {
        val n = pieceRequests.size
        val stats = pieceRequests.mapIndexed { i, (g, _) -> rc.stats.child(names?.getOrNull(i) ?: g.id) }
        val out = arrayOfNulls<Any>(n)
        fun one(i: Int, token: CancelToken) {
            val (g, r0) = pieceRequests[i]
            val st = stats[i]
            val prc = RequestContext(token, rc.priority, rc.executor, rc.slots, rc.log, st, rc.useCache)
            val t0 = System.nanoTime()
            out[i] = try {
                val s = g.service.solveAsHandled(ctx.inGraph(g, r0), prc)
                FinalPiece(g, s, if (features) g.service.feature(s.request, s.legs, g.service.encoder(s.request), null, st) else null)
            } catch (e: SearchCancelledException) { throw e } catch (e: RuntimeException) { e } finally {
                st.profile = r0.profile; st.points = r0.coordinates.size; st.totalMs = (System.nanoTime() - t0) / 1_000_000
            }
        }
        val slot = if (n >= 2) helperSlot(rc, rc.cancel) else null
        onTwoThreads(rc, rc.cancel, slot, { t -> for (i in 0 until n step 2) one(i, t) }, if (n >= 2) { t -> for (i in 1 until n step 2) one(i, t) } else null)
        return out.map { v -> if (v is RuntimeException) throw v else v as FinalPiece }
    }

    /**
     * Gli archi di un viaggio tra Paesi in fila, per la regola delle varianti sul viaggio intero
     * ([TripJudge]): id = (indice del grafo in [graphs]) << 32 | arco, con tempi e metri. L'insieme
     * degli id si costruisce alla prima domanda (serve solo per il migliore e per le tenute).
     */
    private class TripEdges(parts: List<Pair<Int, List<RoutePath>>>) {
        val ids: LongArray
        val ms: LongArray
        val m: DoubleArray

        init {
            val n = parts.sumOf { (_, legs) -> legs.sumOf { it.edges.size } }
            ids = LongArray(n); ms = LongArray(n); m = DoubleArray(n)
            var i = 0
            for ((gi, legs) in parts) for (l in legs) for (e in l.edges) {
                ids[i] = (gi.toLong() shl 32) or (e.edge.toLong() and 0xffffffffL); ms[i] = e.millis; m[i] = e.distance; i++
            }
        }

        val set: LongHashSet by lazy { LongHashSet(ids.size * 2).also { s -> for (id in ids) s.add(id) } }

        /** I tratti massimali di archi fuori da [k] (posizioni in [ids], metri, ms). */
        fun runsOff(k: TripEdges): List<Run> {
            val ks = k.set
            val out = ArrayList<Run>()
            var i = 0
            while (i < ids.size) {
                if (ks.contains(ids[i])) { i++; continue }
                var j = i; var mm = 0.0; var t = 0L
                while (j < ids.size && !ks.contains(ids[j])) { mm += m[j]; t += ms[j]; j++ }
                out += Run(i, j - 1, mm, t, 0.0)
                i = j
            }
            return out
        }
    }

    /**
     * Ref e classi degli archi per grafo ([EdgeInfo], con la sua cache dei nomi) e firma e metri per
     * ref dei pezzi già letti (per identità della lista dei tratti: i pezzi del migliore si leggono una
     * volta sola): una per richiesta, sul thread della richiesta.
     */
    private class TripLabels {
        private val infos = HashMap<CountryGraph, Pair<EdgeInfo, CorridorAnalyzer>>()
        private val memo = java.util.IdentityHashMap<List<RoutePath>, Triple<List<Pair<String, Double>>, Map<String, Double>, Map<String, Double>>>()

        private fun of(g: CountryGraph) = infos.getOrPut(g) {
            val rg = g.graph
            val info = EdgeInfo(rg.graph, rg.encoding.shared, rg.guidance)
            info to CorridorAnalyzer(rg.graph, info, rg.guidance)
        }

        fun info(g: CountryGraph): EdgeInfo = of(g).first

        /** Le strade (ref) di un viaggio: la firma unita (km), i metri per ref e la quota di autostrada o superstrada per ref (dagli archi). */
        class Refs(val sig: List<Pair<String, Double>>, val meters: Map<String, Double>, val mwShare: Map<String, Double>) {
            operator fun component1() = sig
            operator fun component2() = meters
            operator fun component3() = mwShare
        }

        /** Le strade (ref) dei pezzi [parts], per la firma e le strade nuove. */
        fun refs(parts: List<Pair<CountryGraph, List<RoutePath>>>): Refs {
            val sig = ArrayList<Pair<String, Double>>()
            val meters = HashMap<String, Double>()
            val mw = HashMap<String, Double>()
            for ((g, legs) in parts) {
                val (ps, pm, pw) = memo.getOrPut(legs) {
                    val (info, corridors) = of(g)
                    val mm = HashMap<String, Double>(); val mwm = HashMap<String, Double>()
                    for (l in legs) for (e in l.edges) info.ref(e.edge)?.let { r ->
                        mm.merge(r, e.distance, Double::plus)
                        if (info.isMotorwayOrTrunk(e.edge)) mwm.merge(r, e.distance, Double::plus)
                    }
                    Triple(corridors.signature(legs), mm, mwm)
                }
                for ((ref, km) in ps) {
                    val last = sig.lastOrNull()
                    if (last != null && last.first == ref) sig[sig.size - 1] = ref to Math.round((last.second + km) * 10) / 10.0 else sig += ref to km
                }
                for ((ref, mm) in pm) meters.merge(ref, mm, Double::plus)
                for ((ref, mm) in pw) mw.merge(ref, mm, Double::plus)
            }
            return Refs(sig, meters, meters.mapValues { (ref, m) -> if (m > 0) (mw[ref] ?: 0.0) / m else 0.0 })
        }
    }

    /**
     * La regola di accettazione delle varianti ([VariantRules], come [VariantJudge.evaluate]) sul
     * viaggio tra Paesi intero: per le varianti che cambiano più pezzi (fonte V) e per la distanza fra
     * varianti di origine diversa. Tempi e km sui totali del migliore [best]; la deviazione più lunga
     * dal tenuto più vicino ≥ dMin1 (in scala, [NavAlternativesOptions.minDetourScale]), il tratto
     * deviato, la sanità locale verso il migliore, le autostrade (stessa firma di un tenuto).
     */
    private inner class TripJudge(val best: TripEdges, private val labels: TripLabels, private val nav: NavAlternativesOptions, tripMeters: Double) {
        private val scale = nav.minDetourScale
        val d1 = VariantRules.dMin1(tripMeters) * scale
        /** id → posizione (la prima) nel migliore; i tempi cumulati del migliore (prefisso: prima dell'arco i). */
        private val pos = LongIntHashMap(best.ids.size * 2)
        private val preMs = LongArray(best.ids.size + 1)

        init {
            for (i in best.ids.indices) { if (!pos.containsKey(best.ids[i])) pos.put(best.ids[i], i); preMs[i + 1] = preMs[i] + best.ms[i] }
        }

        private fun mw(id: Long): Boolean = labels.info(graphs[(id ushr 32).toInt()]).isMotorwayOrTrunk(id.toInt())

        /** La quota di autostrada e superstrada del migliore, letta solo se serve. */
        private val bestMwShare: Double by lazy {
            var mwm = 0.0; var all = 0.0
            for (i in best.ids.indices) { all += best.m[i]; if (mw(best.ids[i])) mwm += best.m[i] }
            if (all > 0) mwm / all else 0.0
        }

        /** Il tempo del migliore fra le ancore del tratto [r] di [v] (gli archi strettamente fra le due). */
        private fun bestMsAround(v: TripEdges, r: Run): Long {
            val a = (if (r.from > 0) pos.getOrDefault(v.ids[r.from - 1], -1) else -1) + 1
            val b = if (r.to + 1 < v.ids.size) pos.getOrDefault(v.ids[r.to + 1], best.ids.size) else best.ids.size
            return if (b > a) preMs[b] - preMs[a] else 0L
        }

        private fun sane(v: TripEdges, runs: List<Run>, a: Double, bMs: Long): Boolean = runs.all { r -> r.ms <= a * bestMsAround(v, r) + bMs * scale }

        /** La deviazione minima da ogni tenuto per il livello [tier]: dMin1 a ogni livello (un'alternativa riconoscibile, come [VariantJudge.evaluate]). */
        @Suppress("UNUSED_PARAMETER")
        fun minDev(tier: Int): Double = d1

        /** Vero se [v], già giudicata dal suo pezzo al livello [tier], devia da ognuno di [others] quanto chiede il livello. */
        fun diverse(v: TripEdges, tier: Int, others: List<TripEdges>): Boolean =
            others.all { o -> (v.runsOff(o).maxOfOrNull { it.meters } ?: 0.0) >= minDev(tier) }

        /**
         * Il livello di [v] (firma [sig], tempo [r] e km [rk] sul migliore) rispetto ai tenuti [kept]
         * (il migliore primo, con le firme): 1, 2, 3 (solo con `fill`), 0 con il motivo dello scarto.
         */
        fun tier(v: TripEdges, sig: List<String>, r: Double, rk: Double, kept: List<Pair<TripEdges, List<String>>>): Pair<Int, String> {
            val runsB = v.runsOff(best)
            if (runsB.isEmpty()) return 0 to "doppione"
            // la deviazione più lunga e il suo tempo, dal tenuto più vicino
            var dMax = Double.POSITIVE_INFINITY; var majorMs = Long.MAX_VALUE; var same = false
            for ((k, ks) in kept) {
                val major = v.runsOff(k).maxByOrNull { it.meters }
                dMax = minOf(dMax, major?.meters ?: 0.0); majorMs = minOf(majorMs, major?.ms ?: 0L)
                if (ks == sig) same = true
            }
            if (dMax <= 0.0) return 0 to "doppione"
            val majorB = runsB.maxByOrNull { it.meters }!!
            fun mwOk(): Boolean {
                if (!same || bestMwShare < VariantRules.MW_BEST_SHARE) return true
                var mwm = 0.0
                for (i in majorB.from..majorB.to) if (mw(v.ids[i])) mwm += v.m[i]
                return majorB.meters > 0 && mwm / majorB.meters >= VariantRules.MW_RUN_SHARE
            }
            val tier1Time = (r <= nav.tier1Factor && rk <= nav.tier1KmFactor) || (nav.allowFewerKm && rk < 1.0 && r <= nav.maxFactor)
            val withinMax = r <= nav.maxFactor && (rk <= nav.maxKmFactor || (nav.allowFewerKm && rk < 1.0))
            if (tier1Time && dMax >= d1 && majorMs >= VariantRules.T1_RUN_MS * scale && sane(v, runsB, VariantRules.T1_A, VariantRules.T1_B_MS))
                return if (mwOk()) 1 to "" else 2 to "autostrade"
            if (withinMax && dMax >= minDev(2) && majorMs >= VariantRules.T2_RUN_MS * scale && sane(v, runsB, VariantRules.T2_A, VariantRules.T2_B_MS)) return 2 to ""
            if (nav.fill && withinMax && dMax >= minDev(3) && sane(v, runsB, VariantRules.T3_A, VariantRules.T3_B_MS)) return 3 to ""
            return 0 to when {
                r > nav.maxFactor -> "tempo"
                !withinMax -> "km"
                dMax < minDev(3) || majorMs < VariantRules.T2_RUN_MS * scale -> "diversità"
                !sane(v, runsB, VariantRules.T2_A, VariantRules.T2_B_MS) -> "locale"
                else -> "diversità"
            }
        }
    }

    /**
     * Una variante di un viaggio tra Paesi: livello (quello del suo pezzo; per la fonte V si giudica a
     * ogni scelta, [TripJudge.tier]), fonte, punteggio, tempo e km sul viaggio del migliore, archi e
     * firma (la regola sul viaggio intero), ordine d'arrivo, `variant` della risposta e come si
     * costruisce la feature (solo per le tenute).
     */
    private class CrossVariant(val tier: Int, val source: VariantSource, val score: Double, val r: Double, val rk: Double, val edges: TripEdges,
                               val sig: List<String>, val order: Int, val json: MutableMap<String, Any?>,
                               /** Solo locale nel suo pezzo ([Verdict.local]): dopo tutte le altre. */
                               val local: Boolean = false,
                               /** Fonte V per un'altra sequenza di Paesi: un posto riservato nella scelta ([variantsAcross]). */
                               val otherCountries: Boolean = false, val build: () -> Map<String, Any?>)

    /**
     * La strada (ref) al valico fra i pezzi [a] e [b] (null: nessuno dopo): la prima con un ref
     * andando indietro dal valico per al più [CROSSING_REF_M] nel pezzo prima, se no in avanti nel
     * pezzo dopo. Letta una volta per variante V (pochi archi), solo per il titolo.
     */
    private fun crossingRef(labels: TripLabels, a: FinalPiece, b: FinalPiece?): String? {
        val ia = labels.info(a.graph)
        var m = 0.0
        back@ for (l in a.solved.legs.asReversed()) for (e in l.edges.asReversed()) {
            ia.ref(e.edge)?.let { return it }
            m += e.distance; if (m > CROSSING_REF_M) break@back
        }
        if (b == null) return null
        val ib = labels.info(b.graph)
        m = 0.0
        for (l in b.solved.legs) for (e in l.edges) {
            ib.ref(e.edge)?.let { return it }
            m += e.distance; if (m > CROSSING_REF_M) return null
        }
        return null
    }

    /** "+4 min, −6 km". */
    private fun diff(extraS: Double, extraM: Double): String {
        val min = Math.round(extraS / 60).toInt(); val km = Math.round(extraM / 1000).toInt()
        fun signed(v: Int) = if (v < 0) "−${-v}" else "+$v"
        return "${signed(min)} min, ${signed(km)} km"
    }

    /** [feature] con `properties.navmaster.variant` = [variant]. */
    @Suppress("UNCHECKED_CAST")
    private fun withVariant(feature: Map<String, Any?>, variant: Map<String, Any?>): Map<String, Any?> {
        val props = feature["properties"] as MutableMap<String, Any?>
        val nm = props["navmaster"] as? MutableMap<String, Any?> ?: linkedMapOf<String, Any?>().also { props["navmaster"] = it }
        nm["variant"] = variant
        return feature
    }

    /**
     * Le varianti NavMaster di un viaggio tra Paesi (`navmaster.alternatives`, passo 7) e
     * `metadata.navmaster.alternatives`. (b) A valichi fissi: le varianti del pezzo più lungo (poi
     * degli altri pezzi ≥ [PIECE_SHARE] del viaggio, finché mancano varianti di livello 1) dal suo
     * servizio ([NavAlternativesEngine] sulla richiesta del pezzo: i valichi sono punti esatti),
     * giudicate sui totali del viaggio ([TripContext]). (c) Fonte V, se (b) non ha già tutte le
     * varianti di livello 1: gli altri luoghi di valico fra due Paesi ([chooseExactTop]) coi loro
     * tratti, dalla cache. La scelta è golosa (per livello, poi fonte M, P, V, poi punteggio): la fonte
     * V si giudica sul viaggio intero rispetto ai tenuti ([TripJudge]), le varianti dei pezzi devono
     * stare abbastanza lontane dalle tenute V. Ogni variante sono i pezzi del migliore con quelli
     * cambiati, uniti ([merge]); le feature solo per le tenute. Il tempo [budget] e i ms contano da
     * [t0] (`System.nanoTime`, l'inizio della richiesta).
     */
    private fun variantsAcross(ctx: Ctx, request: DirectionsRequest, pieces: List<Piece>, pieceRequests: List<Pair<CountryGraph, DirectionsRequest>>, finals: List<FinalPiece>, bestFeature: Map<String, Any?>,
                               nav: NavAlternativesOptions, rc: RequestContext, t0: Long, budget: CancelToken): Pair<List<Map<String, Any?>>, Map<String, Any?>> {
        val want = nav.count - 1
        // dove va il tempo delle varianti (riga «tempi tra Paesi»): la scelta dei valichi (passi 1 e 2, i pezzi finali) fin qui
        val tv0 = System.nanoTime()
        val chooseMs = (tv0 - t0) / 1_000_000
        val totalMs = finals.sumOf { it.millis }; val totalM = finals.sumOf { it.meters }; val totalW = finals.sumOf { it.weight }
        val labels = TripLabels()
        val (bestSig, bestRefM) = labels.refs(finals.map { it.graph to it.solved.legs })
        fun edgesOf(parts: List<Pair<CountryGraph, List<RoutePath>>>) = TripEdges(parts.map { (g, legs) -> graphs.indexOf(g) to legs })
        val bestEdges = edgesOf(finals.map { it.graph to it.solved.legs })
        val judge = TripJudge(bestEdges, labels, nav, totalM)
        val cands = ArrayList<CrossVariant>()
        val notes = ArrayList<String>(); val summaries = ArrayList<String>()
        if (ctx.topStopped) notes += "budget"
        fun leftMs() = nav.budgetMs - (System.nanoTime() - t0) / 1_000_000
        // le varianti solo locali di un pezzo (un'uscita dalla città) non bastano a fermare la ricerca
        fun tier1() = cands.count { it.tier == 1 && !it.local }
        // i titoli di riserva delle varianti V (la strada al valico, il km del valico), per due valichi con lo stesso titolo
        val crossingTitles = HashMap<CrossVariant, List<String>>()
        fun sigJson(sig: List<Pair<String, Double>>) = sig.map { linkedMapOf("ref" to it.first, "km" to it.second) }
        fun factor(x: Double) = Math.round(x * 1000) / 1000.0

        // (c) fonte V: le altre catene di valichi di ogni scelta (altri luoghi, altre sequenze di Paesi), coi
        // loro tratti dalla cache; il livello alla scelta. Quelle per altri Paesi sempre e prima delle varianti
        // dei pezzi (costano solo i tratti dalla cache: le M e P dei pezzi le lasciavano senza tempo); quelle
        // della stessa sequenza dopo, solo finché mancano varianti di livello 1
        var crossings = 0; var others = 0; var sameSkipped = 0
        val crossingText = HashMap<CrossVariant, String>()
        val spans = ctx.spans.filter { s -> s.chains.size > 1 && s.chains[0].via.size == s.last - s.first &&
            s.chains[0].via.indices.all { i -> pieces[s.first + i].points.last().let { (la, lo) -> la == s.chains[0].via[i].lat && lo == s.chains[0].via[i].lon } } }
        fun otherCountriesOf(span: Span, alt: ChainChoice) = alt.path.map { it.id } != (span.first..span.last).map { pieces[it].graph.id }
        val vOn = VariantSource.CROSSING in nav.sources && spans.isNotEmpty()
        var vMs = 0L
        val vStats by lazy { rc.stats.child("varianti valichi") }
        // le fonti M e P dei pezzi (dopo la fonte V per altri Paesi) e il loro minimo al pezzo più lungo
        val pieceSources = nav.sources - VariantSource.CROSSING
        val mFloorMs = if (pieceSources.isNotEmpty() && finals.isNotEmpty()) NavAlternativesEngine.MIN_SOURCE_BUDGET_MS else 0L
        /**
         * Il tempo della fonte V: per altri Paesi fino al minimo delle fonti M e P prima della fine di `budget_ms`
         * (il passo 2 finisce prima della coda, [variantsTailMs]: alle catene già pesate resta almeno [V_GRACE_MS];
         * anche più di 25 s di passo 1 sul tablet le lasciava tutte fuori: Torino → Lione senza autostrade
         * 1 percorso con tre catene a +8-12 % pronte); negli stessi Paesi fino alla fine di [budget]. Mai oltre.
         */
        val vOtherToken = if (mFloorMs > 0) rc.cancel.child(t0 + (nav.budgetMs - mFloorMs) * 1_000_000) else budget
        fun tokenOf(others: Boolean): CancelToken = if (others) vOtherToken else budget
        /**
         * Le candidate V delle catene per altri Paesi ([others] vero: prima delle varianti dei pezzi, così
         * non restano mai senza tempo: i tratti vengono dalla cache) o di quelle negli stessi Paesi (dopo,
         * solo finché mancano varianti di livello 1).
         */
        fun crossingPass(others0: Boolean) {
            val vt0 = System.nanoTime()
            try {
                for (span in spans) for (alt in span.chains.drop(1)) {
                    val otherCountries = otherCountriesOf(span, alt)
                    if (otherCountries != others0) continue
                    if (!otherCountries && tier1() >= want) { sameSkipped++; continue }
                    val token = tokenOf(others0)
                    token.check()
                    val vrc = RequestContext(token, rc.priority, rc.executor, rc.slots, rc.log, vStats, rc.useCache)
                    // i pezzi della catena: dal prefisso del primo pezzo del migliore, poi i punti del suo ultimo pezzo
                    val (closed, next) = piecesOf(pieces[span.first].withoutLast(), alt.via.zip(alt.path.drop(1)))
                    val tail = pieces[span.last]
                    for (i in 1 until tail.points.size) { next.points += tail.points[i]; next.radiuses += tail.radiuses[i]; next.exact += tail.exact[i]; next.bearings += tail.bearings[i]; next.passThrough += tail.passThrough[i] }
                    val altPieces = closed + next
                    val altRequests = pieceRequests.take(span.first) + altPieces.map { it.graph to requestOf(request, it) } + pieceRequests.drop(span.last + 1)
                    // solo i tratti (dalla cache): le feature solo per le tenute
                    val alt2 = try { finalPieces(ctx, altRequests, vrc, altRequests.map { "valico ${it.first.id}" }, features = false) } catch (e: ApiException) { continue }
                    crossings++; if (otherCountries) others++
                    val ms = alt2.sumOf { it.millis }; val m = alt2.sumOf { it.meters }
                    val r = ms.toDouble() / maxOf(1L, totalMs); val rk = m / maxOf(1.0, totalM)
                    val within = r <= nav.maxFactor && (rk <= nav.maxKmFactor || (nav.allowFewerKm && rk < 1.0))
                    val text = "V Paesi ${alt.text()}"
                    log?.invoke("varianti: $text: ${"%+d".format((ms - totalMs) / 1000)} s, ${"%+.1f".format((m - totalM) / 1000)} km → " +
                        if (within) "candidata" else "scartata (${if (r > nav.maxFactor) "tempo" else "km"})")
                    if (!within) continue
                    val (sig, refM, mwShare) = labels.refs(alt2.map { it.graph to it.solved.legs })
                    val newRefs = sig.map { it.first }.distinct().filter { ref -> (refM[ref] ?: 0.0) >= VariantRules.NEW_REF_M && (bestRefM[ref] ?: 0.0) < VariantRules.ABSENT_REF_M }
                    // i Paesi nuovi della variante (quelli che il migliore non attraversa), per il titolo
                    val newCountries = alt.path.map { it.id }.filter { id -> pieces.none { it.graph.id == id } }
                    val through = if (newCountries.isEmpty()) "" else ", per ${newCountries.joinToString(" e ") { countryName(it) }}"
                    // per altri Paesi le autostrade nuove più lunghe (non le prime strade nuove, che sono spesso
                    // statali corte vicino alla partenza); per lo stesso Paese le prime strade nuove, come prima
                    val titleRefs = if (otherCountries) otherCountriesTitleRefs(newRefs, refM, mwShare) else newRefs.take(2)
                    val label = (if (titleRefs.isEmpty()) "altro valico" else "via " + titleRefs.joinToString(" + ")) + through
                    val extraS = (ms - totalMs) / 1000.0; val extraM = m - totalM
                    // livello e «più lunga» alla scelta
                    val json = linkedMapOf<String, Any?>("rank" to 0, "tier" to 0, "source" to VariantSource.CROSSING.json, "label" to label, "diff" to diff(extraS, extraM),
                        "signature" to sig.joinToString(" · ") { it.first }, "refs" to sigJson(sig), "new_refs" to newRefs,
                        "extra_s" to Math.round(extraS * 10) / 10.0, "extra_m" to Math.round(extraM * 10) / 10.0, "factor" to factor(r), "km_factor" to factor(rk),
                        "longer" to false, "detours" to emptyList<Any>(), "crossing" to linkedMapOf("lat" to alt.via[0].lat, "lon" to alt.via[0].lon),
                        "crossings" to alt.via.map { linkedMapOf("lat" to it.lat, "lon" to it.lon) }, "countries" to altRequests.map { it.first.id })
                    // il bonus delle strade nuove come il giudice: le autostrade (dal ref o, per metà dei km, dagli archi: le D ceche)
                    val score = 0.6 * r + 0.4 * rk - VariantRules.NEW_REF_BONUS * minOf(VariantRules.NEW_REF_MAX, newRefs.count { CorridorAnalyzer.isMotorwayRef(it) || (mwShare[it] ?: 0.0) >= 0.5 })
                    val cv = CrossVariant(0, VariantSource.CROSSING, score, r, rk, edgesOf(alt2.map { it.graph to it.solved.legs }), sig.map { it.first }, cands.size, json,
                        otherCountries = otherCountries) { merge(alt2.map { featureOf(it) }, request) }
                    cands += cv
                    crossingText[cv] = text
                    // il titolo di riserva: la strada al primo valico diverso dal migliore e il suo km
                    val j = alt.via.indices.firstOrNull { i -> i >= span.chains[0].via.size || alt.via[i].let { t -> span.chains[0].via[i].let { b -> t.lat != b.lat || t.lon != b.lon } } } ?: 0
                    val at = span.first + j
                    val where = listOfNotNull(crossingRef(labels, alt2[at], alt2.getOrNull(at + 1))?.let { "sulla $it" }, "al km ${Math.round(alt2.take(at + 1).sumOf { it.meters } / 1000)}")
                    crossingTitles[cv] = where.map { w -> if (titleRefs.isEmpty()) "altro valico $w$through" else "$label, valico $w" }
                }
            } catch (e: SearchCancelledException) {
                if (e.reason != SearchCancelledException.Reason.DEADLINE || rc.cancel.isCancelled) throw e
                notes += "budget"
            }
            vMs += (System.nanoTime() - vt0) / 1_000_000
        }
        if (vOn) crossingPass(true)
        val vOtherMs = (System.nanoTime() - tv0) / 1_000_000
        val tp0 = System.nanoTime()
        val pieceMs = ArrayList<String>()

        // (b) le varianti dei pezzi lunghi, coi valichi fissi
        val sameLeft = vOn && spans.any { sp -> sp.chains.drop(1).any { !otherCountriesOf(sp, it) } }
        if (pieceSources.isNotEmpty()) for ((n, j) in finals.indices.sortedByDescending { finals[it].meters }.withIndex()) {
            if (tier1() >= want || (n > 0 && finals[j].meters < PIECE_SHARE * totalM)) break
            // con catene V negli stessi Paesi ancora da guardare (dopo), un po' di tempo resta a loro; il pezzo più
            // lungo ha il minimo delle fonti di un Paese solo (MIN_SOURCE_BUDGET_MS) prima di quella riserva, dalla
            // coda di budget_ms che il passo 2 e la fonte V gli lasciano (Lione → Francoforte senza autostrade:
            // prima 1 percorso), mai oltre la fine di budget_ms
            val left = pieceBudgetMs(n, leftMs(), if (sameLeft) V_RESERVE_MS else 0L)
            if (n == 0 && sameLeft && left >= MIN_PIECE_BUDGET_MS && left > leftMs() - V_RESERVE_MS) log?.invoke("varianti: al pezzo più lungo $left ms, prima della riserva delle catene V")
            if (left < MIN_PIECE_BUDGET_MS) { notes += "budget"; break }
            val f = finals[j]
            val trip = TripContext(totalMs - f.millis, totalM - f.meters, totalW - f.weight, finals.take(j).sumOf { it.meters })
            val opt = NavAlternativesOptions(want - tier1() + 1, nav.tier1Factor, nav.maxFactor, left, true, nav.tier1KmFactor, nav.maxKmFactor,
                nav.allowFewerKm, nav.fill, pieceSources, nav.minDetourScale).withTrace(nav.trace)
            val prc = RequestContext(rc.cancel, rc.priority, rc.executor, rc.slots, rc.log ?: log, rc.stats.child("varianti ${f.graph.id}"), rc.useCache)
            val res = NavAlternativesEngine(f.graph.service, prc, trip, hardEnd = t0 + nav.budgetMs * 1_000_000).run(f.solved.request, opt)
            pieceMs += "${f.graph.id} ${res.ms} ms su $left"
            summaries += "${f.graph.id}: ${res.summary}"
            res.note?.let { notes += "${f.graph.id}: $it" }
            val encoder = f.graph.service.encoder(f.solved.request)
            for (a in res.variants) {
                val parts = finals.mapIndexed { i, p -> p.graph to (if (i == j) a.legs else p.solved.legs) }
                val (sig, _) = labels.refs(parts)
                val json = LinkedHashMap(res.json(a))
                json["signature"] = sig.joinToString(" · ") { it.first }; json["refs"] = sigJson(sig)
                val r = (json["factor"] as Number).toDouble(); val rk = (json["km_factor"] as Number).toDouble()
                cands += CrossVariant(a.verdict.tier, a.draft.source, a.verdict.score, r, rk, edgesOf(parts), sig.map { it.first }, cands.size, json, a.verdict.local) {
                    merge(finals.mapIndexed { i, p -> if (i == j) f.graph.service.feature(f.solved.request, a.legs, encoder, null) else featureOf(p) }, request)
                }
            }
        }

        val piecesMs = (System.nanoTime() - tp0) / 1_000_000
        val ts0 = System.nanoTime()
        if (vOn) {
            crossingPass(false)
            summaries += "V $crossings ($others per altri Paesi" + (if (sameSkipped > 0) ", $sameSkipped non provate: ${tier1()} di livello 1" else "") + ") in $vMs ms"
        } else if (VariantSource.CROSSING in nav.sources) summaries += "V no (nessun'altra catena)"

        log?.invoke("varianti: tempi tra Paesi: scelta dei valichi (passi 1 e 2, pezzi finali) $chooseMs ms" + (if (ctx.topStopped) " (passo 2 fermato dal tempo)" else "") +
            ", fonte V per altri Paesi $vOtherMs ms, varianti dei pezzi $piecesMs ms" + (if (pieceMs.isEmpty()) " (nessun pezzo: ${maxOf(0L, leftMs())} ms rimasti)" else " (${pieceMs.joinToString()})") +
            ", fonte V negli stessi Paesi ${(System.nanoTime() - ts0) / 1_000_000} ms, su ${nav.budgetMs} ms")
        // le tenute, golose: per livello, poi (finché nessuna tenuta passa per altri Paesi) la fonte V per altri Paesi,
        // poi fonte (M, P, V), poi punteggio. La fonte V si giudica sul viaggio intero rispetto ai tenuti; le varianti
        // dei pezzi (giudicate dal loro pezzo) devono stare lontane dalle tenute V
        val chosen = ArrayList<CrossVariant>()
        val kept = arrayListOf(bestEdges to bestSig.map { it.first })
        val left = ArrayList(cands)
        val why = HashMap<CrossVariant, String>()
        while (chosen.size < want && left.isNotEmpty()) {
            var pick: CrossVariant? = null; var pickTier = 0
            val out = ArrayList<CrossVariant>()
            val reserved = chosen.none { it.otherCountries }
            for (c in left) {
                val t = if (c.source == VariantSource.CROSSING) judge.tier(c.edges, c.sig, c.r, c.rk, kept).let { (t, w) -> why[c] = w; t }
                    else if (judge.diverse(c.edges, c.tier, chosen.filter { it.source == VariantSource.CROSSING }.map { it.edges })) c.tier else 0.also { why[c] = "diversità" }
                // con più tenuti un livello non migliora: lo scarto è definitivo
                if (t == 0) { out += c; continue }
                val p = pick
                val better = p == null || (c.local != p.local && !c.local) || (c.local == p.local && (t < pickTier || (t == pickTier && (
                    if (reserved && c.otherCountries != p.otherCountries) c.otherCountries
                    else c.source.ordinal < p.source.ordinal || (c.source == p.source && (c.score < p.score || (c.score == p.score && c.order < p.order)))))))
                if (better) { pick = c; pickTier = t }
            }
            left.removeAll(out.toSet())
            val p = pick ?: break
            chosen += p; left.remove(p); kept += p.edges to p.sig
            if (p.source == VariantSource.CROSSING) { p.json["tier"] = pickTier; p.json["longer"] = pickTier >= 2 && p.r > nav.tier1Factor }
        }
        // il posto per un altro Paese anche fra livelli diversi: se nessuna tenuta passa per altri Paesi ma una
        // candidata V per altri Paesi vale il livello 1 o 2 (sui tenuti senza la tenuta peggiore dei pezzi), prende
        // l'ultimo posto al posto della tenuta peggiore dei pezzi (l'ultima scelta), se le altre tenute dei pezzi
        // restano abbastanza lontane da lei. Prima una V di livello 2 perdeva sempre contro tre deviazioni M di livello 1
        if (want >= 2 && chosen.size == want && chosen.none { it.otherCountries }) {
            val worst = chosen.lastOrNull { it.source != VariantSource.CROSSING }
            if (worst != null) {
                val keptW = kept.filter { it.first !== worst.edges }
                val rest = chosen.filter { it !== worst }
                val swap = cands.filter { it.otherCountries && it !in chosen }.mapNotNull { c ->
                    val (t, w) = judge.tier(c.edges, c.sig, c.r, c.rk, keptW)
                    if (t !in 1..2) { why[c] = w; return@mapNotNull null }
                    if (rest.any { o -> o.source != VariantSource.CROSSING && !judge.diverse(o.edges, o.tier, listOf(c.edges)) }) { why[c] = "diversità"; return@mapNotNull null }
                    Triple(c, t, c.score)
                }.minWithOrNull(compareBy({ it.second }, { it.third }, { it.first.order }))
                if (swap != null) {
                    val (c, t, _) = swap
                    chosen.remove(worst); chosen += c
                    kept.removeAll { it.first === worst.edges }; kept += c.edges to c.sig
                    c.json["tier"] = t; c.json["longer"] = t >= 2 && c.r > nav.tier1Factor
                    log?.invoke("varianti: posto per altri Paesi: ${crossingText[c]} (livello $t) al posto di ${worst.json["label"]} (${worst.source.json}, livello ${worst.tier})")
                }
            }
        }
        for (c in cands) if (c.source == VariantSource.CROSSING) {
            log?.invoke("varianti: ${crossingText[c]}: " + if (c in chosen) "tenuta, livello ${c.json["tier"]}" else "scartata (${why[c]?.ifEmpty { null } ?: "posti finiti"})")
        }
        val features = ArrayList<Map<String, Any?>>()
        if (!nav.excludeBest) {
            val top2 = bestSig.sortedByDescending { it.second }.take(2).map { it.first }.toSet()
            val refs = bestSig.map { it.first }.filter { it in top2 }.distinct()
            features += withVariant(bestFeature, linkedMapOf("rank" to 0, "tier" to 0, "source" to VariantSource.BEST.json,
                "label" to (if (refs.isEmpty()) "percorso migliore" else "via " + refs.joinToString(" + ")), "diff" to "",
                "signature" to bestSig.joinToString(" · ") { it.first }, "refs" to sigJson(bestSig), "new_refs" to emptyList<String>(), "extra_s" to 0.0, "extra_m" to 0.0,
                "factor" to 1.0, "km_factor" to 1.0, "longer" to false, "detours" to emptyList<Any>(), "countries" to pieces.map { it.graph.id }))
        }
        // le feature delle tenute, su due thread se c'è l'aiutante
        val built = arrayOfNulls<Map<String, Any?>>(chosen.size)
        fun build(parity: Int) { for (i in chosen.indices) if (i % 2 == parity) built[i] = chosen[i].build() }
        onTwoThreads(rc, rc.cancel, if (chosen.size >= 2) helperSlot(rc, rc.cancel) else null, { build(0) }, if (chosen.size >= 2) { _ -> build(1) } else null)
        // titoli tutti diversi (le varianti di pezzi diversi e dei valichi possono dire la stessa strada)
        val bestTitle = bestSig.sortedByDescending { it.second }.take(2).map { it.first }.toSet().let { top ->
            bestSig.map { it.first }.filter { it in top }.distinct().let { if (it.isEmpty()) "percorso migliore" else "via " + it.joinToString(" + ") } }
        val titles = VariantRules.distinct(chosen.map { it.json["label"] as? String ?: "" }, { i ->
            val c = chosen[i]; val t = c.json["label"] as? String ?: ""
            if (c.source == VariantSource.CROSSING) crossingTitles[c].orEmpty()
            else (c.json["detours"] as? List<*>).orEmpty().mapNotNull { it as? Map<*, *> }.maxByOrNull { (it["km"] as? Number)?.toDouble() ?: 0.0 }
                ?.let { d -> listOf("$t, ${VariantRules.fromKm((d["from_km"] as? Number)?.toDouble() ?: 0.0)}") }.orEmpty()
        }, listOf(bestTitle))
        for ((i, v) in chosen.withIndex()) { v.json["rank"] = i + 1; v.json["label"] = titles[i]; features += withVariant(built[i]!!, v.json) }
        val ms = (System.nanoTime() - t0) / 1_000_000
        val note = if (chosen.size >= want) null else notes.firstOrNull() ?: "nessun'altra strada entro +${Math.round((nav.maxFactor - 1) * 100)} %"
        val summary = summaries.joinToString(" | ").ifEmpty { "nessuna fonte" }
        log?.invoke("varianti: tra Paesi ${chosen.size}/$want in $ms ms ($summary)" + (note?.let { "; $it" } ?: ""))
        rc.stats.variants = "${chosen.size}/$want in $ms ms ($summary)"
        return features to linkedMapOf("asked" to nav.count, "found" to 1 + chosen.size, "variants" to chosen.size, "ms" to ms, "note" to note, "candidates" to summary)
    }

    // ------------------------------------------------------------------------------- the borders

    /**
     * Le sequenze di grafi da [a] a [b] che stanno vicino alla linea p → q ([countrySequences]): fra i
     * grafi del passo 1 ([bestGraphs], se dati) o, con [all], fra tutti; [extra] sequenze in più per il
     * passo 2. Con [a] uguale a [b] i giri per un Paese vicino (A-B-A) i cui valichi stanno entro
     * [AROUND_FACTOR] della linea ([Transfers.aroundOk]).
     */
    private fun countryPaths(a: CountryGraph, b: CountryGraph, p: LL, q: LL, extra: Int = 0, all: Boolean = false): List<List<CountryGraph>> {
        val pool = if (all) graphs else bestGraphs?.let { ids -> graphs.filter { it.id in ids || it === a || it === b } } ?: graphs
        val byId = pool.associateBy { it.id }
        fun linked(x: String, y: String) = transfers(byId.getValue(x), byId.getValue(y)).isNotEmpty()
        val seqs = countrySequences(pool.map { it.id }, { byId[it]?.graph?.graph?.bounds }, ::linked, p.lat, p.lon, q.lat, q.lon, a.id, b.id, extra)
            .map { seq -> seq.map { byId.getValue(it) } }
        return if (a !== b) seqs else aroundSequences(seqs.map { s -> s.map { it.id } }, { id -> transfers(a, byId.getValue(id)) }, p.lat, p.lon, q.lat, q.lon)
            .map { seq -> seq.map { byId.getValue(it) } }
    }

    /** I passaggi da provare su un confine: i migliori per deviazione dalla linea p → q, prima le strade principali. */
    private fun candidates(list: List<Transfer>, p: LL, q: LL, transit: Boolean): List<List<Transfer>> {
        fun detour(t: Transfer) = DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon) + DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)
        val major = Transfers.places(list.filter { it.rank <= 3 }, ::detour, if (transit) 5 else 6)
        // le strade minori solo vicino alla partenza o all'arrivo (un viaggio corto oltre il confine)
        val minor = list.filter { it.rank > 3 }.filter { t ->
            minOf(DistanceCalc.haversine(p.lat, p.lon, t.lat, t.lon), DistanceCalc.haversine(t.lat, t.lon, q.lat, q.lon)) < 40_000.0
        }.let { Transfers.places(it, ::detour, if (transit) 2 else 4) }
        return major + minor
    }

    /**
     * I passaggi di confine del percorso migliore da p (grafo [ga]) a q (grafo [gb]), con il grafo
     * in cui si entra a ciascuno; null quando nessuna sequenza di Paesi ha un percorso.
     */
    private fun chain(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, together: Boolean = oneSearch,
                      fromMemory: Boolean = true): List<Pair<Transfer, CountryGraph>>? {
        val ga = prefix.graph
        val p = prefix.points.last().let { LL(it.first, it.second) }; val rp = prefix.radiuses.last()
        val q = ctx.request.coordinates[suffix.first].let { LL(it.first, it.second) }; val rq = radius(suffix.first)
        if (together && compareOneByOne && log != null) {
            // diagnostica: la scelta di un valico alla volta, con ricerche sue, per confronto
            val t0 = System.currentTimeMillis()
            val c2 = Ctx(ctx.request, ctx.penalized, rc = ctx.rc)
            val other = chain(c2, prefix, suffix, radius, gb, together = false)
            log.invoke("  confronto, un valico alla volta: ${other?.joinToString { "%.5f,%.5f".format(it.first.lat, it.first.lon) } ?: "nessuno"} peso ${"%.0f".format(lastWeight)}, ${c2.queries} ricerche, ${System.currentTimeMillis() - t0} ms")
        }
        val paths = countryPaths(ga, gb, p, q)
        log?.invoke("sequenze ${ga.id} → ${gb.id}: ${paths.joinToString(" | ") { pa -> pa.joinToString("-") { it.id } }}")
        // passo 1: il migliore, ogni sequenza cercata sotto il peso del migliore trovato finora (un tetto
        // solo), le sequenze divise fra due thread ([sequences]). Per sequenza il suo migliore: il tetto
        // della sequenza i è il minimo dei migliori delle sequenze fino a i (si migliora solo scendendo
        // sotto) e, appena sopra, di quelle dopo (a parità vince la sequenza che viene prima, come una
        // dopo l'altra). Il tetto più basso visto prima non cambia il migliore: un passaggio, uno strato o
        // una catena scartati perché già sopra il tetto (i pesi rapidi sono limiti inferiori dei pezzi)
        // non potevano batterlo. Una dopo l'altra (un thread solo) è esattamente il calcolo di prima.
        val n = paths.size
        val seqBest = AtomicLongArray(n).also { a -> for (i in 0 until n) a.set(i, Double.POSITIVE_INFINITY.toRawBits()) }
        val seqChain = AtomicReferenceArray<List<Pair<Transfer, CountryGraph>>?>(n)
        // le catene esatte trovate (sotto il tetto del momento) di ogni sequenza: il migliore e, per la fonte V, le altre
        val seqFound = AtomicReferenceArray<List<ChainChoice>>(n)
        // la memoria del passo 1 ([chainKey]): il peso del migliore della stessa scelta e la sequenza che l'ha dato. Per la
        // memoria dei transiti ([chooseTwo]: «nessuna coppia sotto l») ogni sequenza vede subito il tetto che avrà dopo
        // quella del migliore ([memoCeiling]): la sequenza del migliore e quelle prima fino al suo peso compreso, quelle
        // dopo sotto. Prima, con due thread, un transito partito mentre la sequenza del migliore lavorava ancora vedeva il
        // tetto infinito, l'attesa finiva e si rifaceva (Lione → Francoforte senza autostrade: 18 s; Milano → Innsbruck →
        // Monaco: 3 ricerche rifatte). I tetti della programmazione dinamica restano quelli di sempre: con un tetto più
        // basso gli strati cambiano e qualche peso rapido non sta più nella memoria (una richiesta uguale rifaceva ricerche)
        val memoKey = chainKey(ctx, prefix, suffix, radius, gb, paths, together)
        val memo = if (!fromMemory || memoKey == null) null else ga.service.transitCache.get(memoKey)?.takeIf { it.w.isFinite() && it.seq in 0 until n }
        val knownW = memo?.w ?: Double.POSITIVE_INFINITY
        val knownSeq = memo?.seq ?: -1
        if (memo != null) { ctx.hits++; log?.invoke("  passo 1 dalla memoria: tetto %.0f dalla sequenza %s".format(knownW, paths[knownSeq].joinToString("-") { it.id })) }
        fun ceiling(i: Int): Double {
            var c = Double.POSITIVE_INFINITY
            for (k in 0 until n) { val w = Double.fromBits(seqBest.get(k)); val x = if (k <= i) w else Math.nextUp(w); if (x < c) c = x }
            return c
        }
        fun memoCeiling(i: Int): Double = if (knownSeq < 0) ceiling(i) else minOf(ceiling(i), if (knownSeq < i) knownW else Math.nextUp(knownW))
        // il tetto della sequenza i come una dopo l'altra (solo le sequenze fino a i): per le catene di
        // due Paesi della fonte V ([chooseExactTop]), che il passo 2 non ripesa; la sequenza 0 lo vede infinito anche
        // se una sequenza dopo, sull'altro thread, ha già un migliore
        fun ceilingBefore(i: Int): Double {
            var c = Double.POSITIVE_INFINITY
            for (k in 0..i) { val w = Double.fromBits(seqBest.get(k)); if (w < c) c = w }
            return c
        }
        // col migliore dalla memoria le ricerche del passo 1 nelle altre sequenze non lo cambiano più: servono solo alle
        // catene della fonte V (quelle trovate sotto il tetto, [found]) e si contano per le varianti, pezzi compresi
        // ([Ctx.extraPhase]). Con due thread uno strato può cambiare col momento in cui il tetto scende (la prima
        // richiesta l'ha visto scendere a metà): qualche peso rapido mancava alla memoria e risultava «rifatto» (D: 14
        // nella sequenza italy-austria-hungary-croatia, non in quella del migliore). La sequenza del migliore no: la
        // memoria ha solo il peso e la sequenza, i valichi si ritrovano con le sue ricerche (sul tablet, memoria dei
        // pesi o dei tratti svuotata in parte: ricerche vere per ricostruire il migliore, contate come tali)
        // i tratti fissi dei capi per la programmazione dinamica ([dpChain]): una volta, col contesto del thread che li chiede per primo
        val baseMemo = AtomicReference<DoubleArray?>(null)
        val bases = { c: Ctx -> baseMemo.get() ?: synchronized(baseMemo) {
            baseMemo.get() ?: doubleArrayOf(startBase(c, prefix), endBase(c, suffix, radius, gb)).also {
                baseMemo.set(it)
                if (it[0] > 0 || it[1] > 0) log?.invoke("  tratti fissi dei capi: %.0f prima del primo confine, %.0f dopo l'ultimo".format(it[0], it[1]))
            }
        } }
        val q1 = ctx.queries; val x1 = ctx.extraQueries; val xs1 = ctx.extraSearched
        // le ricerche per le varianti del passo 1 (con la memoria: tutte quelle delle altre sequenze) e quelle della sequenza del migliore
        val forVq = java.util.concurrent.atomic.AtomicInteger(); val knownSeqQ = java.util.concurrent.atomic.AtomicInteger()
        sequences(ctx, n) { c, i ->
            val path = paths[i]
            // da capo anche dopo un aiutante fermato a metà
            seqBest.set(i, Double.POSITIVE_INFINITY.toRawBits()); seqChain.set(i, null)
            val f = ArrayList<ChainChoice>()
            val sq0 = c.queries; val sh0 = c.hits; val st0 = System.currentTimeMillis()
            val sx0 = c.extraQueries; val ss0 = c.searchedAll; val sxs0 = c.extraSearched
            onSequence?.invoke(i, true)
            val wasExtra = c.extraPhase
            val vSeq = memo != null && i != knownSeq
            if (vSeq) c.extraPhase = true
            try {
                dpChain(c, prefix, suffix, radius, p, rp, q, rq, path, { ceiling(i) },
                    { w, via -> seqChain.set(i, via.mapIndexed { k, tr -> tr to path[k + 1] }); seqBest.set(i, w.toRawBits()) }, f, together = together,
                    exactTopLimit = { ceilingBefore(i) }, memoLimit = { memoCeiling(i) }, bases = bases)
            } finally { c.extraPhase = wasExtra; onSequence?.invoke(i, false) }
            val dq = c.queries - sq0; val dx = c.extraQueries - sx0
            forVq.addAndGet(if (vSeq) dq else dx)
            if (memo != null && i == knownSeq) knownSeqQ.addAndGet(dq - dx)
            // i valichi scelti insieme dalla memoria ([Ctx.bestKnown]): da parte, per le varianti solo dopo il confronto ([route])
            if (memo == null && c.bestKnown) { c.knownQueries += dq - dx; c.knownSearched += (c.searchedAll - ss0) - (c.extraSearched - sxs0) }
            seqFound.set(i, f)
            // il costo di ogni sequenza nella scelta del migliore (da misurare sul tablet), col thread che l'ha pesata
            if (n > 1) log?.invoke("  sequenza ${path.joinToString("-") { it.id }}: ${c.queries - sq0} ricerche (${c.hits - sh0} dalla memoria) in ${System.currentTimeMillis() - st0} ms @${Thread.currentThread().name}")
        }
        var best = Double.POSITIVE_INFINITY
        var bestChain: List<Pair<Transfer, CountryGraph>>? = null
        var bestSeq = -1
        val found = ArrayList<ChainChoice>()
        for (i in 0 until n) {
            seqFound.get(i)?.let { found += it }
            val w = Double.fromBits(seqBest.get(i))
            val via = seqChain.get(i)
            if (via != null && w < best) { best = w; bestChain = via; bestSeq = i }
        }
        if (memo != null && (bestSeq != knownSeq || best != knownW)) {
            // la memoria non torna (non dovrebbe mai): la scelta da capo, senza
            log?.invoke("  passo 1: la memoria dava %.0f dalla sequenza $knownSeq, trovato %.0f dalla $bestSeq: da capo senza memoria".format(knownW, best))
            // la voce sbagliata via (se no resterebbe: [TransitCache.put] non sostituisce un peso finito) e le ricerche fatte
            // con la memoria non più per le varianti: la scelta da capo la rimette giusta
            ga.service.transitCache.remove(memoKey!!)
            ctx.extraQueries = x1; ctx.extraSearched = xs1
            return chain(ctx, prefix, suffix, radius, gb, together, fromMemory = false)
        }
        // il migliore nella memoria del passo 1 (la richiesta delle varianti parte da lì)
        if (memoKey != null && memo == null && bestChain != null) ga.service.transitCache.put(memoKey, null, null, best, best, bestSeq)
        if (memo != null) {
            ctx.extraQueries = x1 + forVq.get()
            val forV = forVq.get(); val again = knownSeqQ.get()
            if (forV > 0 || again > 0) log?.invoke("  passo 1 col migliore dalla memoria: $forV ricerche delle altre sequenze per le catene della fonte V, " +
                "$again della sequenza del migliore per ritrovarne i valichi")
        }
        lastWeight = best; lastCrossings = bestChain?.map { it.first }.orEmpty()
        ctx.chains = emptyList()
        val chosen = bestChain ?: return null
        // il tempo del passo 2 dopo un passo 1 lungo (grafo appena riaperto, memoria svuotata): fino a
        // [TOP_AFTER_SHARE] del tempo dalla fine del passo 1, mai più di metà di quel che resta (il resto alle fonti M e P)
        if (ctx.topK > 1 && ctx.navBudgetMs > 0) extendTopBudget(ctx)
        val bestChoice = ChainChoice(chosen.map { it.second }.let { listOf(ga) + it }, chosen.map { it.first }, best)
        // passo 2 (solo per le varianti): le altre catene entro topFactor, su ogni sequenza di 3+ grafi
        // (quelle di 2 le ha già date chooseExactTop), strato corto, un passaggio per luogo, sotto il budget
        if (ctx.topK > 1 && best.isFinite()) {
            val cap = best * ctx.topWeightFactor
            val t0 = System.currentTimeMillis()
            val q0 = ctx.queries; val x0 = ctx.extraQueries
            val bestIds = bestChoice.path.map { it.id }
            fun ids(c: ChainChoice) = c.path.map { it.id }
            /** Le catene sotto il tetto oltre il migliore, una per luogo di valichi, in ordine di peso. */
            fun distinct(): List<ChainChoice> {
                val out = ArrayList<ChainChoice>()
                for (c in found.filter { it.exact < cap && !it.samePlaces(bestChoice) }.sortedBy { it.exact }) if (out.none { it.samePlaces(c) }) out += c
                return out
            }
            /** Le catene di [path] entro il tetto con [c] (sul suo thread), aggiunte a [into]; falso se il tempo è finito. */
            fun extra(c: Ctx, path: List<CountryGraph>, into: MutableList<ChainChoice>, maxExact: Int): Boolean {
                val before = into.size
                // le ricerche rapide si fermano anche a metà alla scadenza delle varianti (non solo fra una e l'altra)
                val routers = path.map { c.router(it) }
                val cancels = routers.map { it.cancel }
                c.topBudget?.let { tk -> routers.forEach { it.cancel = tk } }
                c.extraPhase = true
                val done = try {
                    dpChain(c, prefix, suffix, radius, p, rp, q, rq, path, { cap }, { _, _ -> }, into,
                        layerSize = LAYER_EXTRA, onePerPlace = true, token = c.topBudget, maxExact = maxExact, together = false, skip = bestChoice, bases = bases)
                } catch (e: SearchCancelledException) {
                    if (e.reason != SearchCancelledException.Reason.DEADLINE || c.rc.cancel.isCancelled) throw e
                    c.topStopped = true; false
                } finally { routers.forEachIndexed { i, r -> r.cancel = cancels[i] }; c.extraPhase = false }
                log?.invoke("  varianti: sequenza ${path.joinToString("-") { it.id }}: ${into.size - before} catene esatte entro ${"%.2f".format(ctx.topWeightFactor)}× del peso" +
                    (if (!done) " (tempo finito)" else "") + " @${Thread.currentThread().name}")
                return done
            }
            // prima le altre sequenze di Paesi, per ultima quella del migliore: le sue catene servono solo se
            // mancano varianti (variantsAcross le salta appena le fonti M e P bastano), e non devono consumare
            // il tempo prima che le altre sequenze siano state guardate. Le altre sono indipendenti (ognuna
            // guarda solo le catene sue già trovate): divise fra due thread, le loro catene in [found]
            // nell'ordine delle sequenze; a tempo finito nessuna sequenza nuova
            // oltre alle sequenze del passo 1, per il solo passo 2 qualche sequenza in più (anche coi grafi
            // fuori dal passo 1, [bestGraphs]), ordinate per lunghezza e deviazione dalla linea
            val more = countryPaths(ga, gb, p, q, extra = EXTRA_SEQUENCES, all = true).filter { pa -> paths.none { it.map { g -> g.id } == pa.map { g -> g.id } } }
            if (more.isNotEmpty()) log?.invoke("  varianti: sequenze in più: ${more.joinToString(" | ") { pa -> pa.joinToString("-") { it.id } }}")
            val longer = (paths + more).filter { it.size >= 3 }
            val otherPaths = longer.filter { it.map { g -> g.id } != bestIds }
            val added = AtomicReferenceArray<List<ChainChoice>>(otherPaths.size)
            val timeUp = java.util.concurrent.atomic.AtomicBoolean(false)
            sequences(ctx, otherPaths.size, stop = { timeUp.get() }) { c, i ->
                val path = otherPaths[i]
                val pid = path.map { it.id }
                val own = found.filter { ids(it) == pid }
                val into = ArrayList(own)
                added.set(i, null)
                val done = extra(c, path, into, ctx.topK - 1)
                added.set(i, into.subList(own.size, into.size).toList())
                if (!done) timeUp.set(true)
            }
            for (i in otherPaths.indices) added.get(i)?.let { found += it }
            val ownPath = longer.firstOrNull { it.map { g -> g.id } == bestIds }
            if (ownPath != null && !timeUp.get()) {
                // nella sequenza del migliore solo le catene che mancano ai posti (quelle già trovate li tengono)
                val maxExact = ctx.topK - 1 - distinct().size
                if (maxExact <= 0) log?.invoke("  varianti: sequenza ${ownPath.joinToString("-") { it.id }}: posti già pieni")
                else extra(ctx, ownPath, found, maxExact)
            }
            // le altre: prima la più leggera di ogni altra sequenza di Paesi (i loro posti), poi le altre in ordine di peso
            val others = distinct()
            val firsts = others.filter { ids(it) != bestIds }.distinctBy { ids(it) }
            val picked = (firsts + others.filter { c -> firsts.none { it === c } }).take(ctx.topK - 1).sortedBy { it.exact }
            ctx.chains = listOf(bestChoice) + picked
            // tutte le ricerche del passo 2 sono per le varianti (quelle già contate dai pesi dei pezzi comprese: una volta sola)
            ctx.extraQueries = x0 + (ctx.queries - q0)
            log?.invoke("varianti: catene oltre il migliore: ${others.size}" +
                picked.joinToString("") { " (${it.text()}: peso ${"%.0f".format(it.exact)}, +${Math.round((it.exact / best - 1) * 100)} %)" } +
                ", ${ctx.queries - q0} ricerche in ${System.currentTimeMillis() - t0} ms" + if (ctx.topStopped) "; tempo finito" else "")
        } else ctx.chains = listOf(bestChoice)
        lastChains = ctx.chains.map { ChainInfo(it.path.map { g -> g.id }, it.via, it.exact) }
        return chosen
    }

    /**
     * Le voci 0 until [n] (le sequenze di Paesi di [chain]) su questo thread con [ctx] e, col posto da
     * aiutante ([helperSlot]) e [parallelSequences], anche sull'aiutante con un contesto suo
     * ([Ctx.fork]: router e contatori suoi, i pesi rapidi in comune): ogni thread prende la prima voce
     * libera finché [stop] non dice basta. Una voce interrotta sull'aiutante (prelazionato: il lavoro
     * BACKGROUND lascia il posto a una richiesta MAIN) torna libera e la rifà da capo questo thread
     * ([work] deve poterla rifare da capo). Senza posto (o con un executor che rifiuta il lavoro): in
     * ordine su questo thread, con [stop] fra una voce e l'altra. Torna con l'aiutante fermo e i suoi
     * conteggi in [ctx] ([Ctx.absorb]).
     */
    private fun sequences(ctx: Ctx, n: Int, stop: () -> Boolean = { false }, work: (Ctx, Int) -> Unit) {
        val slot = if (n >= 2 && parallelSequences) helperSlot(ctx.rc, ctx.rc.cancel) else null
        if (slot == null) {
            for (i in 0 until n) { if (stop()) break; work(ctx, i) }
            return
        }
        // per voce: 0 libera, 1 in corso, 2 fatta
        val state = IntArray(n)
        fun take(): Int = synchronized(state) {
            if (stop()) return -1
            val i = state.indexOfFirst { it == 0 }
            if (i >= 0) state[i] = 1
            i
        }
        fun loop(c: Ctx) {
            while (true) {
                val i = take()
                if (i < 0) return
                c.working.incrementAndGet()
                try { work(c, i) } catch (t: Throwable) { synchronized(state) { state[i] = 0 }; throw t } finally { c.working.decrementAndGet() }
                synchronized(state) { state[i] = 2 }
            }
        }
        val helper = AtomicReference<Ctx?>(null)
        val wasShared = ctx.shared
        ctx.shared = true
        try {
            onTwoThreads(ctx.rc, ctx.rc.cancel, slot, { _ -> loop(ctx) }, { t ->
                // l'aiutante mai partito o fermato: le voci che restano su questo thread
                if (t === ctx.rc.cancel) loop(ctx)
                else loop(ctx.fork(t, ctx.topBudget?.let { t.child(ctx.topDeadline) }).also { helper.set(it) })
            })
        } finally {
            ctx.shared = wasShared
            helper.get()?.let { ctx.absorb(it) }
        }
    }

    /**
     * La programmazione dinamica sui confini della sequenza [path]: il peso migliore fino a ogni
     * passaggio con le ricerche rapide ([Ctx.weight]), ogni ricerca sotto [limit] (meno il già
     * percorso), uno strato di [layerSize] passaggi ai confini intermedi; alla fine ogni catena col
     * limite sotto [limit] si pesa come i pezzi finali ([exactChain]) e, se sta sotto [limit], va in
     * [found] e in [improve] (che può abbassare [limit]: il passo 1 di [chain]). Con [together] i
     * rami esatti di due e tre grafi ([chooseExactTop], [chooseTwo]). Per il passo 2 (le altre
     * catene della fonte V): [onePerPlace] tiene nello strato il passaggio più leggero di ogni luogo, [token] è il tempo delle
     * varianti (alla scadenza si torna con quel che c'è, [Ctx.topStopped]), al più [maxExact] catene
     * esatte, [skip] (il migliore) e le catene già in [found] nello stesso luogo non si ripesano.
     * Falso se il tempo è finito.
     */
    private fun dpChain(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, p: LL, rp: Double, q: LL, rq: Double, path: List<CountryGraph>,
                        limit: () -> Double, improve: (Double, List<Transfer>) -> Unit, found: MutableList<ChainChoice>,
                        layerSize: Int = LAYER, onePerPlace: Boolean = false, token: CancelToken? = null, maxExact: Int = Int.MAX_VALUE,
                        together: Boolean = false, skip: ChainChoice? = null, exactTopLimit: () -> Double = limit,
                        /** Il tetto per la memoria dei transiti ([chooseTwo]): con la memoria del passo 1 quello che avrà alla fine. */
                        memoLimit: () -> Double = limit,
                        /**
                         * I tratti fissi dei capi ([startBase], [endBase]) col contesto del thread, calcolati una volta per [chain]
                         * e solo se servono (non per i rami esatti di due e tre grafi, che li hanno da [ends]); null: 0 e 0.
                         */
                        bases: ((Ctx) -> DoubleArray)? = null): Boolean {
        fun stopped(): Boolean { if (token == null || !token.isCancelled) return false; ctx.rc.cancel.check(); ctx.topStopped = true; return true }
        var exacts = 0
        // un giro per un Paese vicino (A-B-A): si rientra a un altro valico (allo stesso sarebbe il percorso del Paese)
        val around = path.size == 3 && path[0] === path[2]
        fun sameAround(a: Transfer, b: Transfer) = around && DistanceCalc.haversine(a.lat, a.lon, b.lat, b.lon) < Transfers.PLACE
        /** Vero se [exact] su [via] non farebbe niente (il migliore o una catena già trovata negli stessi luoghi). */
        fun known(via: List<Transfer>): Boolean {
            if (skip == null && !onePerPlace) return false
            val c = ChainChoice(path, via, 0.0)
            return skip?.samePlaces(c) == true || found.any { it.samePlaces(c) }
        }
        /** Il peso esatto della catena [via] (null: tempo finito); registrata se sta sotto il tetto. */
        fun exact(via: List<Transfer>): Double? {
            if (stopped()) return null
            if (known(via)) return Double.POSITIVE_INFINITY
            if (exacts >= maxExact) return Double.POSITIVE_INFINITY
            exacts++
            val e = try { exactChain(ctx, prefix, suffix, radius, path, via, token) } catch (e: SearchCancelledException) {
                if (e.reason != SearchCancelledException.Reason.DEADLINE || ctx.rc.cancel.isCancelled) throw e
                ctx.topStopped = true; return null
            }
            // per le varianti ogni catena pesata resta (il tetto lo applica chain): anche una sopra il migliore
            // del momento, che il passo 1 ha già pagato, è una candidata della fonte V
            if (e < limit() || (ctx.topK > 1 && e.isFinite())) found += ChainChoice(path, via, e)
            if (e < limit()) improve(e, via)
            return e
        }
        // programmazione dinamica sui confini: il peso migliore fino a ogni passaggio
        var layer: List<Triple<Transfer, Double, List<Transfer>>> = emptyList()
        // i tratti fissi dei capi: la tappa prima del primo confine (s0, in ogni peso del primo strato: il tetto, peso
        // della catena intera, la contiene) e i tratti fra i punti dopo l'ultimo (e0, nei limiti fino all'arrivo)
        var s0 = 0.0; var e0 = 0.0
        for (k in 1 until path.size) {
            val from = path[k - 1]; val to = path[k]
            val places = candidates(transfers(from, to), p, q, path.size > 2)
                // il passaggio deve essere percorribile dal mezzo in entrambi i grafi
                .map { pl -> pl.filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } } }
                .filter { it.isNotEmpty() }
            // due Paesi: tutti i passaggi (anche uno alla volta: senza core la ricerca è la CH, rapida); in transito i candidati.
            // Anche con [onePerPlace] tutti i punti del luogo: il primo può essere il capo della carreggiata
            // nell'altro verso (peso ∞), e il luogo sparirebbe; «uno per luogo» vale per lo strato (sotto)
            val cands = if (path.size == 2) transfers(from, to).filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } }
                else places.flatten()
            val placeOf = java.util.IdentityHashMap<Transfer, Int>().also { m -> places.forEachIndexed { i, pl -> pl.forEach { m[it] = i } } }
            log?.invoke("  ${path.joinToString("-") { it.id }} confine ${from.id}/${to.id}: ${cands.size} candidati${if (path.size > 2) " in ${places.size} luoghi" else ""}")
            if (cands.isEmpty()) return true
            // transito per un Paese: i due valichi insieme, fra tutti i passaggi dei due confini
            if (together && path.size == 3 && k == 1) {
                // il tetto come funzione: riletto durante la scelta (il migliore di un'altra sequenza sull'altro thread)
                val two = chooseTwo(ctx, prefix, null, path[1], suffix, path[2], radius, limit, memoLimit)
                if (two != null) {
                    if (two.first != null && two.third < limit()) {
                        val via = listOf(two.first!!, two.second!!)
                        found += ChainChoice(path, via, two.third); improve(two.third, via)
                    }
                    return true
                }
            }
            // due Paesi: fra tutti i passaggi del confine, pesati come i pezzi finali (sotto, uno alla volta quando non si può)
            if (together && path.size == 2) {
                // con le varianti il tetto come una dopo l'altra ([exactTopLimit]): le catene dirette restano candidate V
                // anche quando una sequenza più lunga, sull'altro thread, ha già un migliore; il migliore resta sotto [limit]
                val cs = chooseExactTop(ctx, prefix, suffix, radius, to, cands, if (ctx.topK > 1) maxOf(limit(), exactTopLimit()) else limit(), ctx.topK, ctx.topWeightFactor, limit())
                if (cs != null) {
                    if (ctx.topK > 1) { ctx.top = cs; for (c in cs) if (c.transfer != null && c.weight.isFinite()) found += ChainChoice(path, listOf(c.transfer), c.weight) }
                    val c = cs.first()
                    if (c.weight < limit() && c.transfer != null) improve(c.weight, listOf(c.transfer))
                    return true
                }
            }
            if (k == 1 && bases != null) bases(ctx).let { s0 = it[0]; e0 = it[1] }
            // due thread su sequenze dallo stesso primo confine: il primo strato diviso fra loro (gli stessi pesi)
            if (k == 1 && path.size > 2 && token == null) ctx.warm(from, p, rp, cands.map { LL(it.lat, it.lon) }, TRANSFER_RADIUS) { limit() - s0 }
            val next = ArrayList<Triple<Transfer, Double, List<Transfer>>>()
            for (t in cands) {
                val tl = LL(t.lat, t.lon)
                var w = Double.POSITIVE_INFINITY; var via: List<Transfer> = emptyList()
                if (k == 1) {
                    // each search only as far as the best whole trip found so far: it cannot win beyond
                    if (stopped()) return false
                    w = s0 + ctx.weight(from, p, rp, tl, TRANSFER_RADIUS, limit() - s0); via = listOf(t)
                    // two countries: the whole trip through this crossing at once, so that the
                    // next crossings are looked for under its weight
                    if (path.size == 2 && w.isFinite()) {
                        // the weights of the search are a lower bound: the whole trip weighed as the final pieces
                        val total = w + e0 + ctx.weight(path[1], tl, TRANSFER_RADIUS, q, rq, limit() - w - e0)
                        if (total < limit()) exact(listOf(t)) ?: return false
                        continue
                    }
                } else if (layerManyToOne && !around) {
                    // da tutto lo strato a questo passaggio: una ricerca sola all'indietro (stesso risultato
                    // delle ricerche una per voce, che erano 8 per candidato)
                    if (layer.first().second >= limit()) continue
                    if (stopped()) return false
                    val (i, x) = ctx.bestTo(from, layer.map { LL(it.first.lat, it.first.lon) }, layer.map { it.second }, tl, TRANSFER_RADIUS, limit())
                    if (i >= 0) { w = x; via = layer[i].third + t }
                } else for ((pt, pw, pc) in layer) {
                    if (pw >= limit() || pw >= w) continue
                    if (sameAround(pt, t)) continue
                    if (stopped()) return false
                    val x = pw + ctx.weight(from, LL(pt.lat, pt.lon), TRANSFER_RADIUS, tl, TRANSFER_RADIUS, minOf(limit(), w) - pw)
                    if (x < w) { w = x; via = pc + t }
                }
                if (w.isFinite() && w < limit()) next += Triple(t, w, via)
            }
            if (path.size == 2) return true
            // con [onePerPlace] il punto più leggero di ogni luogo
            val sorted = next.sortedBy { it.second }.let { s -> if (onePerPlace) s.distinctBy { placeOf[it.first] } else s }
            layer = sorted.take(if (k == path.size - 1) sorted.size else layerSize)
            log?.invoke("    pesi ${from.id}/${to.id}: ${layer.joinToString { "%.0f".format(it.second) }}")
            if (layer.isEmpty()) return true
        }
        val last = path.last()
        for ((t, w, via) in layer) {
            if (w >= limit()) continue
            // la ricerca fino all'arrivo solo se poi la catena si pesa davvero (exact: non oltre maxExact, non nei luoghi già pesati)
            if (exacts >= maxExact) break
            if (known(via)) continue
            if (stopped()) return false
            val total = w + e0 + ctx.weight(last, LL(t.lat, t.lon), TRANSFER_RADIUS, q, rq, limit() - w - e0)
            if (total >= limit()) continue
            // come sopra: il viaggio pesato come i pezzi finali
            exact(via) ?: return false
        }
        return true
    }

    /**
     * Il peso del viaggio attraverso i passaggi [via] lungo la sequenza di Paesi [path], come i pezzi
     * finali: il pezzo prima del primo confine, quelli nei Paesi di transito, quello dopo l'ultimo.
     * [token]: l'annullamento dei pezzi (il tempo delle varianti nel passo 2 di [chain]).
     */
    private fun exactChain(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, path: List<CountryGraph>, via: List<Transfer>, token: CancelToken? = null): Double {
        val request = ctx.request
        val tk = token ?: ctx.rc.cancel
        var w = ctx.pieceWeight(path[0], request.withPoints(prefix.points + (via[0].lat to via[0].lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(),
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null, (prefix.passThrough + false).toBooleanArray()), tk)
        for (k in 1 until via.size) {
            if (w.isInfinite()) return w
            w += ctx.pieceWeight(path[k], request.withPoints(listOf(via[k - 1].lat to via[k - 1].lon, via[k].lat to via[k].lon),
                doubleArrayOf(TRANSFER_RADIUS, TRANSFER_RADIUS), booleanArrayOf(true, true), null), tk)
        }
        if (w.isInfinite()) return w
        val t = via.last()
        return w + ctx.pieceWeight(path.last(), request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) }, (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray()), tk)
    }

    /** Quante volte due valichi sono stati scelti insieme (diagnostica dei test). */
    internal var twoChoices = 0

    /**
     * La ricerca portata avanti fin oltre una volta e mezza il peso del passaggio più vicino (più
     * mezz'ora): i passaggi più lontani, o irraggiungibili, restano con un limite inferiore (la
     * frontiera), che basta a scartarli; portarla a termine costerebbe tutto il core (Italia: 3 s).
     */
    /**
     * Porta avanti una ricerca di limite fin oltre il passaggio più vicino di [WINDOW_FACTOR] più
     * [WINDOW_SLACK] secondi: per i passaggi più lontani restano i limiti dei landmark e della
     * frontiera. La finestra non tocca l'esattezza (sono limiti inferiori comunque), solo quanti
     * pezzi si pesano poi; con 1,5× + 30 min la ricerca dall'Italia verso l'Austria esplorava 461 000
     * nodi del core (quasi tutta l'Italia: 10 s sul tablet).
     */
    private fun CoreManyToOne.advanceAround(factor: Double = windowFactor, slack: Double = windowSlack) {
        advance {
            var near = Double.POSITIVE_INFINITY
            for (i in 0 until otherCount) if (isFinal(i) && weight(i) < near) near = weight(i)
            near.isFinite() && bound > near * factor + slack
        }
    }

    /** Un passo di ricerca che valga la pena: oltre [key] di un poco (il ciclo sopra ripesa le coppie a ogni ritorno). */
    private fun slack(key: Double) = maxOf(60.0, key * 0.02)

    /**
     * I due valichi di un transito (nessuna tappa nel Paese di mezzo), scelti insieme fra tutte le
     * coppie: con molti passaggi sul secondo confine (Austria/Germania: centinaia) una ricerca per
     * ogni X costerebbe troppo. Qui: i pesi di ricerca dal capo a ogni X ([m0], portata a termine);
     * nel Paese di mezzo una ricerca sola da tutte le X insieme (ognuna col suo peso) verso ogni Y,
     * che dà per ogni Y il limite inferiore del viaggio fin lì; dall'altro capo i limiti di [m2].
     * Gli Y si esaminano in ordine di limite: prima il limite vero del Paese di mezzo (la ricerca da
     * tutte le X portata avanti fino a quell'Y: costa poco e scarta le uscite lontane prima di ogni
     * percorso mirato), il pezzo finale, poi i pesi di ricerca da ogni X a quell'Y (una ricerca) e,
     * in ordine di limite della coppia, i pezzi esatti finché il limite della coppia successiva non
     * supera il viaggio migliore. I pesi di ricerca sono limiti inferiori dei pezzi (senza
     * avoid_features, velocità massima né direzioni).
     *
     * [limit] si rilegge a ogni passo: il migliore delle altre sequenze di Paesi, che sull'altro
     * thread può arrivare mentre questa scelta è in corso (prima il tetto era quello della partenza,
     * spesso ∞, e un transito inutile esaminava centinaia di uscite: Bolzano → Stoccarda 189 in 33 s).
     * Un tetto più basso non cambia il risultato sotto di lui: una coppia scartata perché il suo limite
     * supera il tetto non poteva batterlo. Il peso restituito, se sta sotto l'ultimo [limit], è il
     * minimo esatto fra tutte le coppie; se no nessuna coppia sta sotto quel limite.
     */
    private fun chooseTransit(ctx: Ctx, ga: CountryGraph, gMid: CountryGraph, gEnd: CountryGraph, c01: List<Transfer>, c12: List<Transfer>,
                              inM1: List<Snap>, inM2: List<Snap>, r0: Reach, r2: Reach,
                              pieceA: (Transfer) -> DirectionsRequest, pieceMid: (Transfer, Transfer) -> DirectionsRequest, pieceEnd: (Transfer) -> DirectionsRequest,
                              ends: Ends, limit: () -> Double, t0: Long, noneBelow: DoubleArray): Triple<Transfer?, Transfer?, Double>? {
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        // senza coppie e senza nessuno scarto per il tetto: nessuna coppia del tutto (la memoria lo vale per ogni tetto)
        noneBelow[0] = Double.POSITIVE_INFINITY
        val m0 = r0.base; val m2 = r2.base
        // i tratti fissi dei capi (la tappa prima del confine, [Ends.startBase]): nelle chiavi e nei limiti, non nelle
        // radici del Paese di mezzo (la finestra di [advanceAround] resta quella di sempre)
        val s0 = ends.startBase; val e0 = ends.endBase
        m0.advanceAround(startWindowFactor, startWindowSlack)
        val m0Window = m0.stats
        // le radici coi limiti inferiori (i pesi, dove definitivi): i limiti restano tali
        val roots = c01.indices.map { inM1[it] to r0.lowerBound(it) }
        if (roots.none { it.second.isFinite() }) return none
        val mm = ctx.router(gMid).manyToOne(roots, inM2, rootIsTarget = false) ?: return null
        mm.advanceAround()
        ctx.queries += 3
        val b1 = DoubleArray(c12.size) { mm.lowerBound(it) }
        val midExact = ctx.router(gMid).searchesExact
        val w0 = DoubleArray(c01.size) { Double.NaN }; val w2 = DoubleArray(c12.size) { Double.NaN }
        val done = BooleanArray(c12.size)
        var best = Double.POSITIVE_INFINITY; var bx = -1; var by = -1; var evaluated = 0; var searches = 0
        // il tetto del momento: il migliore di questa scelta o, se più basso, quello delle altre sequenze
        fun cap() = minOf(best, limit())
        // vero se qualcosa è stato scartato per un tetto finito senza un migliore proprio (allora «nessuna» vale solo sotto il tetto)
        var limited = false
        fun capped(c: Double) { if (c.isFinite() && best.isInfinite()) limited = true }
        while (true) {
            var ky = -1; var key = Double.POSITIVE_INFINITY
            for (y in c12.indices) if (!done[y]) {
                val k = s0 + b1[y] + (if (w2[y].isNaN()) e0 + r2.lowerBound(y) else w2[y])
                if (k < key) { key = k; ky = y }
            }
            val cut = cap()
            if (ky < 0) break
            if (key >= cut) { capped(cut); break }
            if (!mm.isFinal(ky)) {
                // il limite vero dalle X a questo Y (la finestra fissa lasciava ai lontani il limite della frontiera)
                val rest = if (w2[ky].isNaN()) e0 + r2.lowerBound(ky) else w2[ky]
                mm.advance { mm.isFinal(ky) || s0 + mm.lowerBound(ky) + rest >= cut }
                b1[ky] = maxOf(b1[ky], mm.lowerBound(ky))
                continue
            }
            // definitivo per un altro Y (la ricerca va avanti per tutti): il limite vero, poi di nuovo in ordine
            if (b1[ky] < mm.weight(ky)) { b1[ky] = mm.weight(ky); continue }
            if (w2[ky].isNaN()) {
                // la condizione sommata come la chiave (stesso ordine: in virgola mobile un altro ordine può dare «oltre» senza avanzare, e il ciclo non finisce)
                if (!r2.isFinal(ky)) { r2.advanceFor(ky, cut - s0 - b1[ky] - e0) { lb -> (s0 + b1[ky]) + (e0 + lb) > key }; continue }
                // il peso di ricerca è quello del pezzo quando niente lo distingue (vedi Ends); se no il pezzo
                w2[ky] = if (ends.endExact) ends.endInner + r2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                continue
            }
            // i pesi di ricerca da ogni X a questo Y, poi le coppie in ordine di limite
            val my = ctx.router(gMid).manyToOne(inM2[ky], inM1, rootIsTarget = true) ?: return null
            my.advanceAround(); searches++
            val order = c01.indices.filter { r0.lowerBound(it).isFinite() && my.lowerBound(it).isFinite() }.sortedBy { r0.lowerBound(it) + my.lowerBound(it) }
            for (x in order) {
                val c = cap()
                if (s0 + r0.lowerBound(x) + my.lowerBound(x) + w2[ky] >= c) { capped(c); continue }
                if (w0[x].isNaN()) {
                    // prima il peso vero del capo dalla sua ricerca (o da una mirata, se lontano), poi, se serve, il pezzo
                    if (!r0.isFinal(x)) r0.advanceFor(x, c - s0 - my.lowerBound(x) - w2[ky]) { lb -> s0 + lb + my.lowerBound(x) + w2[ky] >= c }
                    if (!r0.isFinal(x)) { capped(c); continue }
                    w0[x] = if (ends.startExact) ends.startInner + r0.weight(x) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[x])) }
                }
                if (w0[x] + my.lowerBound(x) + w2[ky] >= c) { capped(c); continue }
                // in transito il pezzo di mezzo va da valico a valico senza tappe né direzioni: il peso di ricerca è esatto
                if (!my.isFinal(x)) my.advance { my.isFinal(x) || w0[x] + my.lowerBound(x) + w2[ky] >= c }
                if (!my.isFinal(x)) { capped(c); continue }
                // (con la ricerca rapida del core ridotto il peso di ricerca è solo un limite: allora il pezzo)
                val w1 = if (midExact) my.weight(x) else { evaluated++; ctx.pieceWeight(gMid, pieceMid(c01[x], c12[ky])) }
                val total = w0[x] + w1 + w2[ky]
                if (total < best) { best = total; bx = x; by = ky }
            }
            done[ky] = true
        }
        val l = limit()
        if (limited) noneBelow[0] = l
        log?.invoke("    due valichi insieme, transito (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)}" +
            "${if (l.isFinite()) " (tetto %.0f)".format(l) else ""}${if (s0 > 0 || e0 > 0) " (tratti fissi dei capi %.0f + %.0f)".format(s0, e0) else ""} fra ${c01.size}×${c12.size} coppie, ${done.count { it }} uscite esaminate, $searches ricerche, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; " +
            "${ga.id} ${r0.stats} (finestra: $m0Window); ${gMid.id} ${mm.stats}; ${gEnd.id} ${r2.stats}")
        return if (bx < 0) none else Triple(c01[bx], c12[by], best)
    }

    /**
     * Una ricerca di limite da un capo verso i passaggi di un confine ([base]), più percorsi mirati per
     * i passaggi lontani. La ricerca verso tutti i passaggi ha un potenziale ALT debole (il passaggio più
     * vicino): per fissare il peso di uno lontano esplora tutto ciò che sta più vicino di lui (Canneto →
     * Tarvisio: 254 000 nodi del core, mezza Italia); anche una sola direzione verso il suo gruppo ne
     * esplora 190 000. Un percorso da punto a punto (core-ALT bidirezionale, [Router.route]) con un tetto
     * ([advanceFor]: si ferma appena è dimostrato che il passaggio non può vincere) segue un corridoio.
     * Stessi pesi, filtri e svolte: i pesi definitivi sono identici; i limiti sono il massimo dei due.
     */
    private inner class Reach(private val ctx: Ctx, private val g: CountryGraph, private val root: Snap, private val snaps: List<Snap>,
                              private val where: List<Transfer>, private val rootIsTarget: Boolean, val base: CoreManyToOne,
                              /**
                               * Il capo come lo cerca la ricerca rapida ([Ctx.weight]: senza direzione, lo stesso aggancio) e il suo
                               * raggio: allora i percorsi mirati stanno nella memoria dei pesi rapidi (della richiesta e del servizio,
                               * [DirectionsService.quickCache]) e la richiesta delle varianti non li rifà; null: niente memoria.
                               */
                              private val rootAt: LL? = null, private val rootRadius: Double = TRANSFER_RADIUS) {
        /** Il peso esatto da un percorso mirato (NaN: non calcolato) e il limite dimostrato da uno col tetto. */
        private val exact = DoubleArray(snaps.size) { Double.NaN }
        private val proven = DoubleArray(snaps.size)
        private var routes = 0
        private var remembered = 0
        private val routeStats = ArrayList<String>()

        fun isFinal(i: Int): Boolean = base.isFinal(i) || !exact[i].isNaN()
        fun weight(i: Int): Double = if (base.isFinal(i)) base.weight(i) else exact[i]
        fun lowerBound(i: Int): Double = if (!exact[i].isNaN()) exact[i] else maxOf(base.lowerBound(i), proven[i])

        /** In linea d'aria dal capo al passaggio [k], in metri. */
        private fun beeline(k: Int) = DistanceCalc.haversine(root.queryLat, root.queryLon, where[k].lat, where[k].lon)

        /** Vero se [i] è lontano: oltre [focusRatio] volte il passaggio più vicino (in linea d'aria) già fissato dalla ricerca verso tutti. */
        private fun far(i: Int): Boolean {
            if (focusBeyond < 0) return true
            var near = Double.POSITIVE_INFINITY
            for (k in where.indices) if (base.isFinal(k) && base.weight(k).isFinite()) near = minOf(near, beeline(k))
            return near.isFinite() && beeline(i) > near * focusRatio
        }

        /** I due capi del percorso mirato verso [i] nel verso della ricerca, per la memoria dei pesi rapidi. */
        private fun ends(i: Int): Pair<LL, LL>? {
            val r = rootAt ?: return null
            val t = LL(where[i].lat, where[i].lon)
            return if (rootIsTarget) t to r else r to t
        }
        private fun radii(): Pair<Double, Double> = if (rootIsTarget) TRANSFER_RADIUS to rootRadius else rootRadius to TRANSFER_RADIUS

        /**
         * Avanti finché [i] ha il peso definitivo o [enough] (sul suo limite inferiore) è vero. [cap]: il
         * peso oltre il quale [i] non può più vincere (dal viaggio migliore già trovato; +∞ se non ce n'è
         * uno), il tetto del percorso mirato. Non il confine di [enough]: quello è il limite del passaggio
         * che viene dopo, appena sopra il suo, e un percorso con quel tetto non trova niente (giri a vuoto).
         * Senza percorso trovato il peso è almeno il tetto: allora [i] non vince e si torna al chiamante,
         * che non lo sceglie più ([CAP_MARGIN] sopra il tetto, così gli arrotondamenti non lo riportano sotto).
         * Prima la memoria dei pesi rapidi (un peso esatto, o un limite già dimostrato); al più [focusMax]
         * percorsi mirati, poi la ricerca verso tutti. [extra]: un percorso che serve solo alle varianti
         * (fonte V), contato a parte ([Ctx.extraQueries]).
         */
        fun advanceFor(i: Int, cap: Double, extra: Boolean = false, enough: (Double) -> Boolean) {
            if (isFinal(i) || enough(lowerBound(i))) return
            val e = ends(i)
            if (e != null) {
                val (ra, rb) = radii()
                ctx.known(g, e.first, ra, e.second, rb)?.let { (w, l) ->
                    if (w.isFinite()) { exact[i] = w; remembered++; ctx.hits++; return }
                    if (l + CAP_MARGIN > lowerBound(i)) {
                        proven[i] = maxOf(proven[i], l + CAP_MARGIN); remembered++; ctx.hits++
                        if (enough(lowerBound(i)) || cap <= lowerBound(i)) return
                    }
                }
            }
            if (far(i) && cap > lowerBound(i) && routes < focusMax) {
                // un percorso trova solo pesi sotto il tetto (anche uguali, col nextUp)
                val top = if (cap.isFinite()) Math.nextUp(cap) else cap
                val r = ctx.router(g)
                try {
                    val p = if (rootIsTarget) r.route(snaps[i], root, maxWeight = top) else r.route(root, snaps[i], maxWeight = top)
                    exact[i] = p.weight
                    if (e != null) { val (ra, rb) = radii(); ctx.remember(g, e.first, ra, e.second, rb, p.weight, top) }
                } catch (ex: RouteNotFoundException) {
                    proven[i] = maxOf(proven[i], top + CAP_MARGIN)
                    if (e != null) { val (ra, rb) = radii(); ctx.remember(g, e.first, ra, e.second, rb, Double.POSITIVE_INFINITY, top) }
                }
                routes++; ctx.queries++
                if (extra) ctx.extraQueries++
                if (routeStats.size < 4) routeStats += r.lastStats
                return
            }
            base.advance { base.isFinal(i) || enough(lowerBound(i)) }
        }

        val stats: String get() = base.stats + (if (routes == 0) "" else " + $routes mirati (${routeStats.joinToString(" / ")})") +
            if (remembered == 0) "" else ", $remembered dalla memoria"
    }

    /** Il valico scelto e il peso del viaggio intero che ci passa (+∞ senza). */
    private class Choice(val weight: Double, val transfer: Transfer?)

    /**
     * Quando il peso di ricerca ([Router.manyToOne], stessi pesi e filtri del percorso finale) di un
     * capo è esattamente quello del pezzo: senza direzione (`bearings`) al punto del capo e, con
     * più punti nel capo, senza `continue_straight` (che lega le tappe fra loro); allora il pezzo
     * pesa la ricerca più le tappe interne del capo ([startInner]/[endInner], pesate una volta).
     * Se no la ricerca resta un limite inferiore e il pezzo si calcola.
     */
    private class Ends(val startExact: Boolean, val startInner: Double, val endExact: Boolean, val endInner: Double,
                       /**
                        * I tratti fissi dei capi ([pieceBase]): dentro ogni pezzo del primo capo i tratti fra i punti di [prefix]
                        * (la tappa prima del confine), dentro ogni pezzo dell'ultimo quelli fra i punti di suffix. Da sommare ai
                        * limiti delle ricerche dai capi (che partono dall'ultimo punto del prefisso e arrivano al primo del suffisso).
                        */
                       val startBase: Double = 0.0, val endBase: Double = 0.0)

    /** La richiesta dei soli punti di [prefix] (i tratti fra loro, come dentro ogni pezzo del primo capo). */
    private fun prefixRequest(request: DirectionsRequest, prefix: Piece): DirectionsRequest =
        request.withPoints(prefix.points, prefix.radiuses.toDoubleArray(), prefix.exact.toBooleanArray(), prefix.bearings, prefix.passThrough.toBooleanArray())

    /** La richiesta dei soli punti [suffix] (i tratti fra loro, come dentro ogni pezzo dell'ultimo capo). */
    private fun suffixRequest(request: DirectionsRequest, suffix: IntRange, radius: (Int) -> Double): DirectionsRequest =
        request.withPoints(suffix.map { request.coordinates[it] }, suffix.map { radius(it) }.toDoubleArray(), suffix.map { false }.toBooleanArray(),
            suffix.map { request.bearings?.getOrNull(it) }, suffix.map { request.passThroughAt(it) }.toBooleanArray())

    /**
     * Vero se il peso della richiesta [r] dei soli punti di un capo è un limite inferiore della loro parte in
     * ogni pezzo che li contiene: con `maximum_speed` un pezzo lungo in linea d'aria la conta sui soli tempi
     * ([DirectionsService.fastMaximumSpeed]: pesi più leggeri), allora la richiesta corta deve già farlo.
     */
    private fun baseAllowed(ctx: Ctx, g: CountryGraph, r: DirectionsRequest): Boolean {
        if (!prefixBase || r.coordinates.size < 2) return false
        val o = try { g.service.routeOptions(ctx.inGraph(g, r)) } catch (e: ApiException) { return false }
        var beeline = 0.0
        for (i in 1 until r.coordinates.size) beeline += DistanceCalc.haversine(r.coordinates[i - 1].first, r.coordinates[i - 1].second, r.coordinates[i].first, r.coordinates[i].second)
        return !g.service.fastMaximumSpeed(o, Double.MAX_VALUE) || g.service.fastMaximumSpeed(o, beeline)
    }

    /**
     * I tratti fissi di un capo: il peso della richiesta [r] dei soli suoi punti in [g] quando è un limite
     * inferiore della loro parte in ogni pezzo ([baseAllowed]); 0 se no (o senza percorso: allora lo dicono i pezzi).
     *
     * Il primo capo ([prefixRequest]): in un pezzo «punti del prefisso + valico X» i tratti fra i punti del
     * prefisso sono gli stessi per ogni X ([DirectionsService.legsOf]: un tratto dipende solo dal precedente, la
     * direzione d'arrivo vale solo per l'ultimo tratto), quindi pezzo(X) = base + tratto(ultimo punto → X) ≥
     * base + limite di ricerca. Non con la direzione (`bearings`) all'ultimo punto del prefisso: da solo sarebbe
     * la direzione d'arrivo del suo ultimo tratto.
     *
     * L'ultimo capo ([suffixRequest]): in un pezzo «valico Y + punti del suffisso» i tratti dopo il primo punto
     * sono quelli della richiesta del suffisso, salvo il primo, che col passaggio senza fermarsi (pass_through)
     * al primo punto riparte nel verso d'arrivo (più pesante o uguale); con pass_through anche a un punto di
     * mezzo del suffisso i tratti dopo cambiano con quel verso, allora niente base.
     */
    private fun pieceBase(ctx: Ctx, g: CountryGraph, r: DirectionsRequest): Double {
        if (!baseAllowed(ctx, g, r)) return 0.0
        val w = ctx.pieceWeight(g, r)
        return if (w.isFinite()) w else 0.0
    }

    /** I tratti fissi del primo capo ([pieceBase]) per un viaggio da [prefix]. */
    private fun startBase(ctx: Ctx, prefix: Piece): Double =
        if (prefix.points.size < 2 || prefix.bearings.last() != null) 0.0 else pieceBase(ctx, prefix.graph, prefixRequest(ctx.request, prefix))

    /** I tratti fissi dell'ultimo capo ([pieceBase]) per un viaggio verso i punti [suffix] in [gEnd]. */
    private fun endBase(ctx: Ctx, suffix: IntRange, radius: (Int) -> Double, gEnd: CountryGraph): Double {
        val request = ctx.request
        if (suffix.count() < 2) return 0.0
        if (request.passThroughAt(suffix.first) && (suffix.first + 1 until suffix.last).any { request.passThroughAt(it) }) return 0.0
        return pieceBase(ctx, gEnd, suffixRequest(request, suffix, radius))
    }

    /** Gli [Ends] di un viaggio da [prefix] (grafo di partenza) ai punti [suffix] in [gEnd]; null quando un capo con più tappe non ha percorso. */
    private fun ends(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gEnd: CountryGraph): Ends? {
        val request = ctx.request
        // i pesi di ricerca sono quelli dei pezzi solo se la ricerca non passa dalla rapida senza avoid_features (core ridotto)
        // con più punti nel capo, il tratto dal (o al) punto vicino al confine dipende dal resto del capo solo se lì si passa senza fermarsi (Ctx.pt)
        val startPt = prefix.points.size > 1 && prefix.passThrough.last() && prefix.points.last().let { pl ->
            ctx.snap(prefix.graph, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN)?.let { ctx.pt(prefix.graph, true, it) } ?: true }
        val startExact = ctx.router(prefix.graph).searchesExact && prefix.bearings.last() == null && !startPt
        val startInner = if (!startExact || prefix.points.size == 1) 0.0 else ctx.pieceWeight(prefix.graph, prefixRequest(request, prefix))
        val suffixPts = suffix.map { request.coordinates[it] }
        val endPt = suffixPts.size > 1 && request.passThroughAt(suffix.first) && suffixPts[0].let { q ->
            ctx.snap(gEnd, LL(q.first, q.second), radius(suffix.first), false, request.bearings?.getOrNull(suffix.first) ?: Double.NaN)?.let { ctx.pt(gEnd, true, it) } ?: true }
        val endExact = ctx.router(gEnd).searchesExact && request.bearings?.getOrNull(suffix.first) == null && !endPt
        val endInner = if (!endExact || suffixPts.size == 1) 0.0 else ctx.pieceWeight(gEnd, suffixRequest(request, suffix, radius))
        if (startInner.isInfinite() || endInner.isInfinite()) return null
        // i tratti fissi dei capi: quando il capo è esatto sono i suoi tratti interni (la stessa richiesta, già pesata)
        val sBase = if (prefix.points.size > 1 && startExact && baseAllowed(ctx, prefix.graph, prefixRequest(request, prefix))) startInner else startBase(ctx, prefix)
        val eBase = if (suffixPts.size > 1 && endExact && baseAllowed(ctx, gEnd, suffixRequest(request, suffix, radius))) endInner else endBase(ctx, suffix, radius, gEnd)
        return Ends(startExact, startInner, endExact, endInner, sBase, eBase)
    }

    /**
     * Il capo di una ricerca di limite come lo cerca la ricerca rapida ([Ctx.weight]: lo stesso aggancio,
     * nessuna direzione) quando i suoi pesi sono quelli dei percorsi ([Router.searchesExact]): allora i
     * percorsi mirati di [Reach] vanno nella memoria dei pesi rapidi; null se no.
     */
    private fun memoRoot(ctx: Ctx, g: CountryGraph, at: LL, radius: Double, exact: Boolean, heading: Double?): LL? =
        if (heading == null && exact == (radius == TRANSFER_RADIUS) && ctx.router(g).searchesExact) at else null

    /**
     * I due valichi di un viaggio che attraversa due confini, scelti insieme: da [prefix] (grafo A)
     * al grafo [gMid] (coi punti [mid], o nessuno in transito) al grafo [gEnd] (punti [suffix]).
     * Per ogni coppia (X, Y) il peso è quello dei tre pezzi finali; i limiti inferiori vengono dalla
     * ricerca sul core ([Router.manyToOne]): dai capi del viaggio ai valichi, e nel grafo di mezzo dai
     * punti del pezzo (o, in transito, da ogni X ai Y). Le coppie si valutano in ordine di limite, un
     * pezzo alla volta, finché il limite della successiva non supera il viaggio migliore.
     * Null quando i router non cercano sul core (allora un confine alla volta); (null, null, +∞) senza percorso.
     *
     * [limit] si rilegge durante la scelta (vedi [chooseTransit]). La scelta si ricorda nel servizio del
     * primo grafo ([DirectionsService.transitCache], con [RequestContext.useCache]): la richiesta delle
     * varianti, subito dopo quella del migliore, la riprende senza ricerche (stesso risultato esatto).
     */
    private fun chooseTwo(ctx: Ctx, prefix: Piece, mid: IntRange?, gMid: CountryGraph, suffix: IntRange, gEnd: CountryGraph, radius: (Int) -> Double,
                          limit: () -> Double, memoLimit: () -> Double = limit): Triple<Transfer?, Transfer?, Double>? {
        beforeTwo?.invoke()
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        val ga = prefix.graph; val request = ctx.request
        fun usable(from: CountryGraph, to: CountryGraph) = transfers(from, to).filter { t -> LL(t.lat, t.lon).let { ctx.snap(from, it, TRANSFER_RADIUS) != null && ctx.snap(to, it, TRANSFER_RADIUS) != null } }
        val c01 = usable(ga, gMid); val c12 = usable(gMid, gEnd)
        if (c01.isEmpty() || c12.isEmpty()) return none
        // la memoria del servizio: una scelta esatta, o «nessuna coppia sotto l» con un tetto non più alto
        val tKey = transitKey(ctx, prefix, mid, gMid, suffix, gEnd, radius)
        val cache = ga.service.transitCache
        if (tKey != null) cache.get(tKey)?.let { e ->
            val what = "    due valichi insieme (${ga.id}→${gMid.id}→${gEnd.id}): dalla memoria, "
            if (e.w.isFinite()) {
                ctx.hits++
                log?.invoke(what + "%.5f,%.5f e %.5f,%.5f peso %.0f".format(e.x!!.lat, e.x.lon, e.y!!.lat, e.y.lon, e.w))
                return Triple(e.x, e.y, e.w)
            }
            if (belowWithin(ctx, memoLimit, e.limit)) { ctx.hits++; log?.invoke(what + "nessuna coppia sotto %.0f".format(e.limit)); return none }
        }
        // il tetto sotto cui «nessuna coppia» è dimostrato: quello dell'ultimo momento (∞ se il transito non ha scartato nulla per lui)
        val noneBelow = doubleArrayOf(Double.NaN)
        val res = chooseTwoIn(ctx, prefix, mid, gMid, suffix, gEnd, radius, limit, c01, c12, noneBelow) ?: return null
        if (tKey != null) {
            val l = limit()
            if (res.third < l) cache.put(tKey, res.first, res.second, res.third, l)
            else cache.put(tKey, null, null, Double.POSITIVE_INFINITY, if (res.first == null && noneBelow[0].isInfinite()) Double.POSITIVE_INFINITY else l)
        }
        return res
    }

    /**
     * Vero se [limit] è (o, con due thread sulle sequenze, diventa entro [TRANSIT_WAIT_MS]) non più alto di
     * [l]: allora la memoria «nessuna coppia sotto l» basta. Due richieste uguali di fila hanno tutte le
     * sequenze in memoria: l'altro thread ha il suo migliore in pochi ms, e la scelta non si rifà.
     */
    private fun belowWithin(ctx: Ctx, limit: () -> Double, l: Double): Boolean {
        if (limit() <= l) return true
        if (!ctx.shared) return false
        val end = System.nanoTime() + TRANSIT_WAIT_MS * 1_000_000
        // solo finché un altro thread pesa una voce (questo ne pesa una: almeno 1)
        while (System.nanoTime() < end && ctx.working.get() > 1) {
            ctx.rc.cancel.check()
            try { Thread.sleep(2) } catch (e: InterruptedException) { Thread.currentThread().interrupt(); return false }
            if (limit() <= l) return true
        }
        return false
    }

    /** Un riferimento confrontato per identità (gli elenchi dei valichi di un confine, conservati da chi chiama). */
    private class Ident(val o: Any) {
        override fun equals(other: Any?) = other is Ident && other.o === o
        override fun hashCode() = System.identityHashCode(o)
    }

    /**
     * La chiave di [chooseTwo] nella memoria del servizio: i tre grafi (id, servizio, opzioni del router e
     * della richiesta, che decidono pesi e agganci), i punti del capo, del Paese di mezzo e dell'arrivo
     * (raggi, versi, direzioni, pass_through), gli elenchi dei valichi (per identità), dove «evita» è
     * penalità. Null senza [RequestContext.useCache].
     */
    private fun transitKey(ctx: Ctx, prefix: Piece, mid: IntRange?, gMid: CountryGraph, suffix: IntRange, gEnd: CountryGraph, radius: (Int) -> Double): Any? {
        if (!ctx.rc.useCache) return null
        val r = ctx.request; val ga = prefix.graph
        fun g(x: CountryGraph) = listOf(x.id, x.service.serial, x.graph.routerKey(ctx.options(x)), x.service.routeOptions(ctx.inGraph(x, r)))
        fun pts(ix: IntRange?) = ix?.map { i -> listOf(r.coordinates[i], radius(i), r.bearings?.getOrNull(i), r.passThroughAt(i)) }
        return listOf(g(ga), g(gMid), g(gEnd), prefix.points.toList(), prefix.radiuses.toList(), prefix.exact.toList(), prefix.bearings.toList(), prefix.passThrough.toList(),
            pts(mid), pts(suffix), Ident(transfers(ga, gMid)), Ident(transfers(gMid, gEnd)), ctx.penalized, ctx.plain)
    }

    /** La chiave di [chooseExactTop] nella memoria del servizio, come [transitKey] (due grafi, i passaggi [cands]); null senza cache. */
    private fun exactTopKey(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, cands: List<Transfer>): Any? {
        // i passaggi: l'elenco del confine (per identità, conservato da chi chiama) e quanti ne restano percorribili
        if (!ctx.rc.useCache) return null
        val r = ctx.request; val ga = prefix.graph
        fun g(x: CountryGraph) = listOf(x.id, x.service.serial, x.graph.routerKey(ctx.options(x)), x.service.routeOptions(ctx.inGraph(x, r)))
        return listOf("valico", g(ga), g(gb), prefix.points.toList(), prefix.radiuses.toList(), prefix.exact.toList(), prefix.bearings.toList(), prefix.passThrough.toList(),
            suffix.map { i -> listOf(r.coordinates[i], radius(i), r.bearings?.getOrNull(i), r.passThroughAt(i)) }, Ident(transfers(ga, gb)), cands.size, ctx.penalized, ctx.plain)
    }

    /**
     * La chiave della memoria del passo 1 di [chain] nel servizio del primo grafo, come [transitKey]: i grafi delle
     * sequenze [paths] (id, servizio, opzioni del router e della richiesta), le sequenze stesse (dipendono dai grafi del
     * migliore, [bestGraphs]), i punti del capo e dell'arrivo, gli elenchi dei valichi di ogni confine (per identità),
     * dove «evita» è penalità, [together]. Null senza [RequestContext.useCache].
     */
    private fun chainKey(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, paths: List<List<CountryGraph>>, together: Boolean): Any? {
        if (!ctx.rc.useCache || paths.isEmpty()) return null
        val r = ctx.request
        fun g(x: CountryGraph) = listOf(x.id, x.service.serial, x.graph.routerKey(ctx.options(x)), x.service.routeOptions(ctx.inGraph(x, r)))
        val borders = paths.flatMap { pa -> pa.zipWithNext() }.distinctBy { (x, y) -> x.id + ">" + y.id }.map { (x, y) -> Ident(transfers(x, y)) }
        return listOf("passo 1", (paths.flatten() + prefix.graph + gb).distinct().map { g(it) }, paths.map { pa -> pa.map { it.id } }, prefix.points.toList(), prefix.radiuses.toList(),
            prefix.exact.toList(), prefix.bearings.toList(), prefix.passThrough.toList(), suffix.map { i -> listOf(r.coordinates[i], radius(i), r.bearings?.getOrNull(i), r.passThroughAt(i)) },
            borders, ctx.penalized, ctx.plain, together)
    }

    /** [chooseTwo] senza la memoria, coi passaggi [c01] e [c12] già filtrati. */
    private fun chooseTwoIn(ctx: Ctx, prefix: Piece, mid: IntRange?, gMid: CountryGraph, suffix: IntRange, gEnd: CountryGraph, radius: (Int) -> Double,
                            limit: () -> Double, c01: List<Transfer>, c12: List<Transfer>, noneBelow: DoubleArray): Triple<Transfer?, Transfer?, Double>? {
        val none = Triple<Transfer?, Transfer?, Double>(null, null, Double.POSITIVE_INFINITY)
        val ga = prefix.graph; val request = ctx.request
        val pl = prefix.points.last()
        val sp = ctx.snap(ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN) ?: return none
        val qi = suffix.first
        val sq = ctx.snap(gEnd, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi) ?: Double.NaN) ?: return none
        val inA = c01.map { ctx.snap(ga, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inM1 = c01.map { ctx.snap(gMid, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inM2 = c12.map { ctx.snap(gMid, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inE = c12.map { ctx.snap(gEnd, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val t0 = System.currentTimeMillis()
        synchronized(this) { twoChoices++ }
        fun pieceA(x: Transfer) = request.withPoints(prefix.points + (x.lat to x.lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(), (prefix.exact + true).toBooleanArray(), prefix.bearings + null,
            (prefix.passThrough + false).toBooleanArray())
        fun pieceMid(x: Transfer, y: Transfer): DirectionsRequest {
            val ps = ArrayList<Pair<Double, Double>>(); val rs = ArrayList<Double>(); val ex = ArrayList<Boolean>(); val bs = ArrayList<Double?>(); val pt = ArrayList<Boolean>()
            ps += x.lat to x.lon; rs += TRANSFER_RADIUS; ex += true; bs += null; pt += false
            mid?.forEach { i -> ps += request.coordinates[i]; rs += radius(i); ex += false; bs += request.bearings?.getOrNull(i); pt += request.passThroughAt(i) }
            ps += y.lat to y.lon; rs += TRANSFER_RADIUS; ex += true; bs += null; pt += false
            return request.withPoints(ps, rs.toDoubleArray(), ex.toBooleanArray(), bs, pt.toBooleanArray())
        }
        fun pieceEnd(y: Transfer) = request.withPoints(listOf(y.lat to y.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(), listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) },
            (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray())
        // i limiti inferiori
        val m0 = ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false) ?: return null
        val m2 = ctx.router(gEnd).manyToOne(sq, inE, rootIsTarget = true) ?: return null
        val r0 = Reach(ctx, ga, sp, inA, c01, false, m0, memoRoot(ctx, ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last()), prefix.radiuses.last())
        val r2 = Reach(ctx, gEnd, sq, inE, c12, true, m2, memoRoot(ctx, gEnd, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi)), radius(qi))
        // dove i pesi di ricerca ai capi sono quelli dei pezzi (vedi Ends)
        val ends = ends(ctx, prefix, suffix, radius, gEnd) ?: return none
        if (mid == null) return chooseTransit(ctx, ga, gMid, gEnd, c01, c12, inM1, inM2, r0, r2, ::pieceA, ::pieceMid, ::pieceEnd, ends, limit, t0, noneBelow)
        val lb1: (Int, Int) -> Double
        /** Il peso vero del pezzo di mezzo dalle ricerche, quando è esatto (null se no): vedi Ends. */
        var midWeight: ((Int, Int) -> Double?)? = null
        var midStats = ""
        run {
            val f = request.coordinates[mid.first]; val l = request.coordinates[mid.last]
            val sm1 = ctx.snap(gMid, LL(f.first, f.second), radius(mid.first), false, request.bearings?.getOrNull(mid.first) ?: Double.NaN) ?: return none
            val sm2 = ctx.snap(gMid, LL(l.first, l.second), radius(mid.last), false, request.bearings?.getOrNull(mid.last) ?: Double.NaN) ?: return none
            val mA = ctx.router(gMid).manyToOne(sm1, inM1, rootIsTarget = true) ?: return null
            val mB = ctx.router(gMid).manyToOne(sm2, inM2, rootIsTarget = false) ?: return null
            // le ricerche nel Paese di mezzo e dal primo capo portate a termine: i limiti dei soli landmark
            // lasciavano pesare decine di pezzi (Soncino → Innsbruck → Monaco: 741 passaggi, 42 s)
            mA.advanceAround(); mB.advanceAround()
            midStats = "${mA.stats} / ${mB.stats}"
            // più le tappe interne del blocco di mezzo (un pezzo da sole pesa al più quanto dentro il pezzo intero)
            val inner = if (mid.first == mid.last) 0.0 else ctx.pieceWeight(gMid, request.withPoints(mid.map { request.coordinates[it] },
                mid.map { radius(it) }.toDoubleArray(), mid.map { false }.toBooleanArray(), mid.map { request.bearings?.getOrNull(it) }, mid.map { request.passThroughAt(it) }.toBooleanArray()))
            if (inner.isInfinite()) return none
            lb1 = { x, y -> mA.lowerBound(x) + mB.lowerBound(y) + inner }
            // esatto dove non si passa senza fermarsi (lega i tratti: Ctx.pt) alla prima e all'ultima tappa e senza direzione all'ultima (vale per la partenza da lì)
            if (ctx.router(gMid).searchesExact && !ctx.pt(gMid, request.passThroughAt(mid.first), sm1) && !ctx.pt(gMid, request.passThroughAt(mid.last), sm2) &&
                request.bearings?.getOrNull(mid.last) == null) midWeight = { x, y ->
                if (!mA.isFinal(x)) mA.advance { mA.isFinal(x) }
                if (!mB.isFinal(y)) mB.advance { mB.isFinal(y) }
                mA.weight(x) + inner + mB.weight(y)
            }
        }
        ctx.queries += 4
        // i tratti fissi dei capi ([Ends.startBase]): nei limiti dei pezzi ai capi non ancora pesati
        val s0 = ends.startBase; val e0 = ends.endBase
        val w0 = DoubleArray(c01.size) { Double.NaN }; val w2 = DoubleArray(c12.size) { Double.NaN }
        val w1 = Array(c01.size) { DoubleArray(c12.size) { Double.NaN } }
        var best = Double.POSITIVE_INFINITY; var bx = -1; var by = -1; var evaluated = 0
        while (true) {
            // il tetto del momento: il migliore di questa scelta o, se più basso, quello delle altre sequenze (riletto a ogni passo)
            val cut = minOf(best, limit())
            // la coppia col limite più basso del viaggio intero fra quelle non ancora pesate del tutto
            var kx = -1; var ky = -1; var key = Double.POSITIVE_INFINITY
            for (x in c01.indices) {
                val a = if (w0[x].isNaN()) s0 + r0.lowerBound(x) else w0[x]
                if (a >= cut) continue
                for (y in c12.indices) {
                    if (!w0[x].isNaN() && !w2[y].isNaN() && !w1[x][y].isNaN()) continue
                    val k = a + (if (w1[x][y].isNaN()) lb1(x, y) else w1[x][y]) + (if (w2[y].isNaN()) e0 + r2.lowerBound(y) else w2[y])
                    if (k < key) { key = k; kx = x; ky = y }
                }
            }
            if (kx < 0 || key >= cut) break
            // prima i pezzi ai capi (servono a molte coppie), poi quello di mezzo
            val rest1 = if (w1[kx][ky].isNaN()) lb1(kx, ky) else w1[kx][ky]
            when {
                // prima il limite vero del capo (la sua ricerca va avanti solo fin lì), poi il pezzo
                !r0.isFinal(kx) && w0[kx].isNaN() -> { val rest2 = if (w2[ky].isNaN()) e0 + r2.lowerBound(ky) else w2[ky]; r0.advanceFor(kx, cut - s0 - rest1 - rest2) { lb -> s0 + lb + rest1 + rest2 > key + slack(key) } }
                w0[kx].isNaN() -> w0[kx] = if (ends.startExact) ends.startInner + r0.weight(kx) else { evaluated++; ctx.pieceWeight(ga, pieceA(c01[kx])) }
                !r2.isFinal(ky) && w2[ky].isNaN() -> r2.advanceFor(ky, cut - w0[kx] - rest1 - e0) { lb -> w0[kx] + rest1 + e0 + lb > key + slack(key) }
                w2[ky].isNaN() -> w2[ky] = if (ends.endExact) ends.endInner + r2.weight(ky) else { evaluated++; ctx.pieceWeight(gEnd, pieceEnd(c12[ky])) }
                else -> {
                    w1[kx][ky] = midWeight?.invoke(kx, ky) ?: run { evaluated++; ctx.pieceWeight(gMid, pieceMid(c01[kx], c12[ky])) }
                    val total = w0[kx] + w1[kx][ky] + w2[ky]
                    if (total < best) { best = total; bx = kx; by = ky }
                }
            }
        }
        log?.invoke("    due valichi insieme (${ga.id}→${gMid.id}→${gEnd.id}): ${if (bx < 0) "nessuno" else "%.5f,%.5f e %.5f,%.5f peso %.0f".format(c01[bx].lat, c01[bx].lon, c12[by].lat, c12[by].lon, best)} fra ${c01.size}×${c12.size} coppie, $evaluated pezzi, ${System.currentTimeMillis() - t0} ms; ${ga.id} ${r0.stats}; ${gMid.id} $midStats; ${gEnd.id} ${r2.stats}")
        return if (bx < 0) none else Triple(c01[bx], c12[by], best)
    }

    /**
     * Il valico migliore fra due Paesi, esatto: fra tutti i passaggi del confine quello col peso
     * minimo del viaggio, ogni pezzo pesato come il percorso finale (avoid_features, maximum_speed,
     * bearings, tappe, continue_straight: [Ctx.pieceWeight]). Per non pesarli tutti, un limite
     * inferiore per ognuno su entrambi i lati: la ricerca sul core senza avoid_features né limite di
     * velocità e senza direzioni ([Router.manyToOne]) — togliere strade, rallentarle o vincolare una
     * direzione può solo aumentare un peso. Dal lato vicino al confine una ricerca sola dà i limiti
     * dal suo capo a tutti i passaggi, in ordine e solo fin dove serve; dal lato lontano i landmark.
     * I passaggi si pesano in ordine di limite del viaggio intero, prima il lato vicino (costa poco),
     * finché il limite del successivo non supera il viaggio migliore: nessun altro può batterlo.
     * Null quando i router non cercano sul core: allora un valico alla volta.
     *
     * Con [k] > 1 (fonte V delle varianti) la scelta va avanti oltre il migliore: anche i passaggi
     * col limite sotto [factor] volte il suo peso e sotto il peso del k-esimo luogo già trovato; un
     * passaggio che non può più essere il migliore né migliorare il suo luogo ([Transfers.PLACE]) si
     * salta. Ritorna il migliore e poi un passaggio per luogo, in ordine di peso, al più [k] in tutto.
     * Con [k] = 1 è la scelta di sempre (stessi passi, stesso valico).
     *
     * Il migliore si ricorda nel servizio del primo grafo ([DirectionsService.transitCache], come i
     * transiti di [chooseTwo]): con [k] = 1 una seconda richiesta lo riprende senza ricerche; con [k] > 1
     * (la richiesta delle varianti) si parte dal migliore già noto e tutto il resto serve solo alla fonte
     * V: contato per le varianti e sotto il loro tempo ([Ctx.topBudget]), alla scadenza il migliore e quel
     * che c'è. Prima la richiesta delle varianti rifaceva la scelta del migliore (Lione → Francoforte: 13 s
     * in CI, 39 ricerche per il migliore e 64 percorsi mirati per gli altri valichi).
     */
    private fun chooseExactTop(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, cands: List<Transfer>, limit: Double,
                               k: Int, factor: Double, bestCut: Double = limit): List<Choice>? {
        val none = listOf(Choice(Double.POSITIVE_INFINITY, null))
        if (cands.isEmpty()) return none
        val ga = prefix.graph
        val request = ctx.request
        // i capi del viaggio vicino al confine, agganciati come nei pezzi finali
        val pl = prefix.points.last()
        val sp = ctx.snap(ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last() ?: Double.NaN) ?: return none
        val qi = suffix.first
        val sq = ctx.snap(gb, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi) ?: Double.NaN) ?: return none
        // la memoria del servizio: il migliore esatto (o «nessuno sotto l» con un tetto non più alto)
        val tKey = exactTopKey(ctx, prefix, suffix, radius, gb, cands)
        val cache = ga.service.transitCache
        var known: TransitCache.Entry? = null
        // k > 1 e la memoria dice «nessuno sotto il tetto del migliore» ([bestCut]): da qui nessun migliore, tutto per la fonte V
        var forV = false
        if (tKey != null) cache.get(tKey)?.let { e ->
            if (e.w.isFinite()) {
                val j = cands.indexOfFirst { it.lat == e.x!!.lat && it.lon == e.x.lon }
                if (j >= 0) {
                    ctx.hits++
                    log?.invoke("    valico scelto: dalla memoria, %.5f,%.5f peso %.0f".format(e.x!!.lat, e.x.lon, e.w))
                    if (k == 1) return listOf(Choice(e.w, cands[j]))
                    known = e
                }
            } else if (e.limit >= limit) { ctx.hits++; log?.invoke("    valico scelto: dalla memoria, nessuno sotto %.0f".format(e.limit)); return none }
            else if (k > 1 && e.limit >= bestCut) { ctx.hits++; forV = true; log?.invoke("    valico scelto: dalla memoria, nessuno sotto %.0f: solo per le varianti".format(e.limit)) }
        }
        // k > 1 col migliore dalla memoria ([known]) o «nessuno sotto il tetto del migliore» ([forV]): tutto quel che segue
        // (le ricerche verso i passaggi, i percorsi mirati, i pezzi) serve solo alla fonte V e sta tutto nel tempo delle
        // varianti ([Ctx.topBudget]): i router dei due grafi con quel tempo, alla scadenza il migliore e quel che c'è.
        // Prima le ricerche verso i passaggi e i percorsi mirati andavano avanti senza tempo (evita autostrade, landmark
        // deboli: Lione → Francoforte 14 s su 10 di varianti, le fonti M e P dei pezzi senza tempo)
        val tb = ctx.topBudget
        val vOnly = k > 1 && (known != null || forV) && tb != null
        fun onlyKnown(why: String): List<Choice> {
            ctx.topStopped = true
            log?.invoke("varianti: valichi nessun altro ($why; tempo finito)")
            val e = known ?: return none
            return listOf(Choice(e.w, cands.first { it.lat == e.x!!.lat && it.lon == e.x.lon }))
        }
        if (vOnly && tb!!.isCancelled) { ctx.rc.cancel.check(); return onlyKnown("prima delle ricerche") }
        val routersV = if (vOnly) listOf(ga, gb).distinctBy { it.id }.map { ctx.router(it) } else emptyList()
        val cancelsV = routersV.map { it.cancel }
        routersV.forEach { it.cancel = tb!! }
        try {
            return exactTopIn(ctx, prefix, suffix, radius, gb, cands, limit, k, factor, sp, sq, tKey, known, forV, vOnly, ::onlyKnown)
        } finally { routersV.forEachIndexed { i, r -> r.cancel = cancelsV[i] } }
    }

    /** Il resto di [chooseExactTop] dopo la memoria: [vOnly] tutto per la fonte V, coi router dei due grafi nel suo tempo. */
    private fun exactTopIn(ctx: Ctx, prefix: Piece, suffix: IntRange, radius: (Int) -> Double, gb: CountryGraph, cands: List<Transfer>, limit: Double,
                           k: Int, factor: Double, sp: Snap, sq: Snap, tKey: Any?, known: TransitCache.Entry?, forV: Boolean, vOnly: Boolean,
                           onlyKnown: (String) -> List<Choice>): List<Choice>? {
        val none = listOf(Choice(Double.POSITIVE_INFINITY, null))
        val ga = prefix.graph
        val request = ctx.request
        val pl = prefix.points.last()
        val qi = suffix.first
        val cache = ga.service.transitCache
        fun deadline(e: SearchCancelledException): Boolean = vOnly && e.reason == SearchCancelledException.Reason.DEADLINE && !ctx.rc.cancel.isCancelled
        val inA = cands.map { ctx.snap(ga, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        val inB = cands.map { ctx.snap(gb, LL(it.lat, it.lon), TRANSFER_RADIUS)!! }
        // i pezzi finali con il passaggio t: prima del confine i punti del pezzo più t, dopo t più i punti nel grafo seguente
        fun pieceA(t: Transfer) = request.withPoints(prefix.points + (t.lat to t.lon), (prefix.radiuses + TRANSFER_RADIUS).toDoubleArray(),
            (prefix.exact + true).toBooleanArray(), prefix.bearings + null, (prefix.passThrough + false).toBooleanArray())
        fun pieceB(t: Transfer) = request.withPoints(listOf(t.lat to t.lon) + suffix.map { request.coordinates[it] },
            (listOf(TRANSFER_RADIUS) + suffix.map { radius(it) }).toDoubleArray(), (listOf(true) + suffix.map { false }).toBooleanArray(),
            listOf<Double?>(null) + suffix.map { request.bearings?.getOrNull(it) }, (listOf(false) + suffix.map { request.passThroughAt(it) }).toBooleanArray())
        // il lato vicino: il capo più vicino ai suoi passaggi
        val nearA = cands.minOf { DistanceCalc.haversine(sp.queryLat, sp.queryLon, it.lat, it.lon) }
        val nearB = cands.minOf { DistanceCalc.haversine(it.lat, it.lon, sq.queryLat, sq.queryLon) }
        val shortIsB = nearB <= nearA
        val near = if (shortIsB) gb else ga; val far = if (shortIsB) ga else gb
        fun nearPiece(t: Transfer) = if (shortIsB) pieceB(t) else pieceA(t)
        fun farPiece(t: Transfer) = if (shortIsB) pieceA(t) else pieceB(t)
        val t0 = System.currentTimeMillis()
        val mNear: CoreManyToOne; val mFar: CoreManyToOne
        try {
            mNear = (if (shortIsB) ctx.router(gb).manyToOne(sq, inB, rootIsTarget = true) else ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false)) ?: return null
            mFar = (if (shortIsB) ctx.router(ga).manyToOne(sp, inA, rootIsTarget = false) else ctx.router(gb).manyToOne(sq, inB, rootIsTarget = true)) ?: return null
        } catch (e: SearchCancelledException) { if (deadline(e)) return onlyKnown("ricerche verso i passaggi in ${System.currentTimeMillis() - t0} ms"); throw e }
        // il lato lontano con le ricerche mirate per i passaggi lontani dalla sua frontiera (vedi Reach)
        val rFar = if (shortIsB) Reach(ctx, ga, sp, inA, cands, false, mFar, memoRoot(ctx, ga, LL(pl.first, pl.second), prefix.radiuses.last(), prefix.exact.last(), prefix.bearings.last()), prefix.radiuses.last())
            else Reach(ctx, gb, sq, inB, cands, true, mFar, memoRoot(ctx, gb, LL(request.coordinates[qi].first, request.coordinates[qi].second), radius(qi), false, request.bearings?.getOrNull(qi)), radius(qi))
        // dove i pesi di ricerca sono quelli dei pezzi (vedi Ends): allora niente pezzo, la ricerca va avanti fino al passaggio
        val ends = try { ends(ctx, prefix, suffix, radius, gb) ?: return none } catch (e: SearchCancelledException) { if (deadline(e)) return onlyKnown("capi"); throw e }
        val nearExact = if (shortIsB) ends.endExact else ends.startExact; val nearInner = if (shortIsB) ends.endInner else ends.startInner
        val farExact = if (shortIsB) ends.startExact else ends.endExact; val farInner = if (shortIsB) ends.startInner else ends.endInner
        // i tratti fissi dei capi ([Ends.startBase]) nei limiti dei due lati
        val nearBase = if (shortIsB) ends.endBase else ends.startBase; val farBase = if (shortIsB) ends.startBase else ends.endBase
        val nearWeight = DoubleArray(cands.size) { Double.NaN }
        val farWeight = DoubleArray(cands.size) { Double.NaN }
        // i limiti dei due lati, sempre sommati nello stesso ordine (la chiave e le condizioni d'arresto delle ricerche:
        // in virgola mobile un altro ordine può dare «oltre la chiave» senza avanzare, e il ciclo non finisce)
        fun farLb(j: Int) = farBase + rFar.lowerBound(j)
        fun nearLb(j: Int) = if (nearWeight[j].isNaN()) nearBase + mNear.lowerBound(j) else nearWeight[j]
        var best = limit; var bestI = -1; var nearCount = 0; var farCount = 0; var farMs = 0L
        // il migliore dalla memoria (k > 1): il suo peso, nessun pezzo da pesare per lui; tutto il resto è per la fonte V
        known?.let { e -> val j = cands.indexOfFirst { it.lat == e.x!!.lat && it.lon == e.x.lon }; best = e.w; bestI = j; nearWeight[j] = 0.0; farWeight[j] = e.w }
        // k > 1: i passaggi saltati (il loro luogo ha già un viaggio più leggero del loro limite)
        val skip = BooleanArray(cands.size)
        fun total(j: Int) = nearWeight[j] + farWeight[j]
        fun samePlace(a: Int, b: Int) = DistanceCalc.haversine(cands[a].lat, cands[a].lon, cands[b].lat, cands[b].lon) < Transfers.PLACE
        // un passaggio per luogo fra quelli pesati del tutto, in ordine di peso
        fun places(): List<Int> {
            val out = ArrayList<Int>()
            for (j in cands.indices.filter { !farWeight[it].isNaN() && total(it).isFinite() }.sortedBy { total(it) }) if (out.none { samePlace(it, j) }) out += j
            return out
        }
        // fin dove pesare: il migliore; con k > 1 fino a factor volte e al k-esimo luogo
        fun cut(): Double = if (k == 1) best else minOf(best * factor, places().let { ps -> if (ps.size >= k) total(ps[k - 1]) else Double.POSITIVE_INFINITY })
        // k > 1, oltre il migliore (key ≥ best: con k = 1 la scelta sarebbe finita, il migliore non cambia
        // più): solo per la fonte V, entro il tempo delle varianti; alla scadenza si tiene quel che c'è
        val tb = ctx.topBudget
        var stopped = false
        var vMs = 0L
        fun piece(extra: Boolean, g: CountryGraph, r: DirectionsRequest, bounded: Boolean = extra): Double? {
            if (!bounded || tb == null) return ctx.pieceWeight(g, r, extra = extra)
            return try { ctx.pieceWeight(g, r, tb, extra = true) } catch (e: SearchCancelledException) {
                if (e.reason != SearchCancelledException.Reason.DEADLINE || ctx.rc.cancel.isCancelled) throw e
                null
            }
        }
        val loop0 = System.currentTimeMillis()
        try { while (true) {
            // il passaggio col limite più basso del viaggio intero, non ancora pesato dal lato lontano
            var i = -1; var key = Double.POSITIVE_INFINITY
            for (j in cands.indices) if (farWeight[j].isNaN() && !skip[j]) {
                val lb = farLb(j) + nearLb(j)
                if (lb < key) { key = lb; i = j }
            }
            if (i < 0 || key >= cut()) break
            val extra = k > 1 && (key >= best || known != null || forV)
            // sotto il migliore del momento si pesa a ogni tempo (decide il migliore); col migliore dalla memoria
            // ([vOnly]) anche sotto il suo peso tutto è per la fonte V: nel tempo delle varianti
            val bounded = extra && (key >= best || vOnly)
            if (bounded && tb != null && tb.isCancelled) { ctx.rc.cancel.check(); stopped = true; break }
            if (k > 1 && key >= best && cands.indices.any { j -> !farWeight[j].isNaN() && total(j) <= key && samePlace(i, j) }) { skip[i] = true; continue }
            if (nearWeight[i].isNaN()) {
                // prima il limite vero del lato vicino (la sua ricerca va avanti solo fin lì), poi il pezzo
                if (!mNear.isFinal(i)) { mNear.advance { mNear.isFinal(i) || farLb(i) + nearLb(i) > key }; continue }
                nearWeight[i] = if (nearExact) nearInner + mNear.weight(i) else { nearCount++; piece(extra, near, nearPiece(cands[i]), bounded) ?: run { stopped = true; Double.NaN } }
                if (stopped) break
                continue
            }
            // poi il limite vero del lato lontano (una ricerca sola per tutti i passaggi, fin dove serve):
            // i pezzi lontani costano (evita pedaggi: secondi) e così ne serve di solito uno
            // il tetto del percorso mirato: per il migliore quello di sempre (k = 1: la richiesta del migliore ha
            // fatto gli stessi percorsi, ora dalla memoria); oltre il migliore (solo fonte V) quello delle varianti
            if (!rFar.isFinal(i)) { rFar.advanceFor(i, (if (extra) cut() else best) - nearWeight[i] - farBase, extra) { lb -> (farBase + lb) + nearWeight[i] > key }; continue }
            val t1 = System.currentTimeMillis()
            farWeight[i] = if (farExact) farInner + rFar.weight(i) else { farCount++; piece(extra, far, farPiece(cands[i]), bounded) ?: run { stopped = true; Double.NaN } }
            farMs += System.currentTimeMillis() - t1
            if (stopped) break
            val total = nearWeight[i] + farWeight[i]
            if (total < best) { best = total; bestI = i }
        } } catch (e: SearchCancelledException) { if (deadline(e)) stopped = true else throw e }
        if (vOnly) vMs = System.currentTimeMillis() - loop0
        log?.invoke("    valico scelto: ${if (bestI < 0) "nessuno" else "%.5f,%.5f peso %.0f".format(cands[bestI].lat, cands[bestI].lon, best)} fra ${cands.size} passaggi; " +
            "lato vicino ${near.id} ${mNear.stats}, $nearCount pezzi; lato lontano ${far.id} ${rFar.stats}, $farCount pezzi in $farMs ms; in tutto ${System.currentTimeMillis() - t0} ms" +
            if (vOnly) " (il migliore dalla memoria: tutto per la fonte V, $vMs ms nel suo tempo${if (stopped) ", fermato" else ""})" else "")
        fun w(x: Double) = if (x.isFinite()) "%.0f".format(x) else "∞"
        for ((j, t) in cands.withIndex()) if (!nearWeight[j].isNaN() || !farWeight[j].isNaN() || nearBase + mNear.lowerBound(j) + farBase + rFar.lowerBound(j) < best + 600)
            log?.invoke("      %.5f,%.5f r${t.rank}: ${near.id} ${if (nearWeight[j].isNaN()) "≥" + w(nearBase + mNear.lowerBound(j)) else w(nearWeight[j])}, ${far.id} ${if (farWeight[j].isNaN()) "≥" + w(farBase + rFar.lowerBound(j)) else w(farWeight[j])}".format(t.lat, t.lon))
        // il migliore nella memoria del servizio: esatto se sta sotto il tetto, se no «nessuno sotto il tetto»
        if (tKey != null && known == null && !forV) {
            if (bestI >= 0 && best < limit) cache.put(tKey, cands[bestI], null, best, limit) else cache.put(tKey, null, null, Double.POSITIVE_INFINITY, limit)
        }
        if (bestI < 0) return none
        val out = arrayListOf(Choice(best, cands[bestI]))
        if (k > 1) {
            // gli altri luoghi entro factor volte il migliore, in ordine di peso
            val kept = arrayListOf(bestI)
            for (j in places()) if (out.size < k && total(j) <= best * factor && kept.none { samePlace(it, j) }) { kept += j; out += Choice(total(j), cands[j]) }
            if (stopped) ctx.topStopped = true
            log?.invoke("varianti: valichi ${out.drop(1).joinToString { "%.5f,%.5f peso %.0f".format(it.transfer!!.lat, it.transfer.lon, it.weight) }.ifEmpty { "nessun altro" }} " +
                "(${skip.count { it }} saltati${if (stopped) "; tempo finito" else ""})")
        }
        return out
    }

    // ------------------------------------------------------------------------------- the answer

    /** I pezzi uniti in un percorso solo, come se fosse stato calcolato in un grafo. */
    @Suppress("UNCHECKED_CAST")
    private fun merge(docs: List<Map<String, Any?>>, request: DirectionsRequest): Map<String, Any?> {
        val coords = ArrayList<DoubleArray>()
        val segments = ArrayList<MutableMap<String, Any?>>()
        val wayPoints = ArrayList<Int>().also { it += 0 }
        val extraValues = LinkedHashMap<String, MutableList<MutableList<Number>>>()
        val extraDist = LinkedHashMap<String, LinkedHashMap<Number, Double>>()
        val roads = ArrayList<Map<String, Any?>>()
        val points = ArrayList<Map<String, Any?>>()
        val bbox = BBox.empty()
        var hasRoads = false
        var cur: MutableMap<String, Any?>? = null
        val translator = InstructionTranslator.forLanguage(request.language)
        val pieceStarts = ArrayList<Int>()
        for ((pi, f) in docs.withIndex()) {
            val props = f["properties"] as Map<String, Any?>
            val geo = (f["geometry"] as Map<String, Any?>)["coordinates"] as List<DoubleArray>
            val offset = if (pi == 0) 0 else coords.size - 1
            pieceStarts += offset
            for (i in (if (pi == 0) 0 else 1) until geo.size) { coords += geo[i]; bbox.include(geo[i][1], geo[i][0]) }
            val segs = props["segments"] as List<Map<String, Any?>>
            val wps = props["way_points"] as List<Int>
            for ((si, seg0) in segs.withIndex()) {
                val seg = LinkedHashMap(seg0)
                val steps: MutableList<Map<String, Any?>>? = (seg["steps"] as? List<Map<String, Any?>>)?.map<Map<String, Any?>, Map<String, Any?>> { shiftStep(it, offset) }?.toMutableList()
                if (steps != null) seg["steps"] = steps
                val c = cur
                if (si == 0 && pi > 0 && c != null) {
                    // dopo un passaggio di confine: lo stesso tratto prosegue
                    c["distance"] = (c["distance"] as Double) + (seg["distance"] as Double)
                    c["duration"] = round1((c["duration"] as Double) + (seg["duration"] as Double))
                    val cs = c["steps"] as MutableList<Map<String, Any?>>?
                    if (cs != null && steps != null) joinSteps(cs, steps, coords, offset, translator)
                } else {
                    cur?.let { segments += it }
                    cur = seg
                }
                val endsAtBorder = si == segs.size - 1 && pi < docs.size - 1
                if (!endsAtBorder) { segments += cur!!; cur = null; wayPoints += wps[si + 1] + offset }
            }
            // gli extra: gli intervalli spostati, uniti al confine quando il valore non cambia
            (props["extras"] as? Map<String, Any?>)?.forEach { (kind, v) ->
                val m = v as Map<String, Any?>
                val vals = extraValues.getOrPut(kind) { ArrayList() }
                for (r in m["values"] as List<List<Number>>) {
                    val from = r[0].toInt() + offset; val to = r[1].toInt() + offset; val value = r[2]
                    val last = vals.lastOrNull()
                    if (last != null && last[2] == value && last[1].toInt() == from) last[1] = to else vals += mutableListOf(from, to, value)
                }
                val dist = extraDist.getOrPut(kind) { LinkedHashMap() }
                for (sm in m["summary"] as List<Map<String, Any?>>) {
                    val value = sm["value"] as Number
                    dist[value] = (dist[value] ?: 0.0) + (sm["distance"] as Number).toDouble()
                }
            }
            (props["navmaster"] as? Map<String, Any?>)?.let { nm ->
                hasRoads = true
                (nm["roads"] as? List<Map<String, Any?>>)?.forEach { r ->
                    val m = LinkedHashMap(r)
                    m["from"] = (r["from"] as Int) + offset; m["to"] = (r["to"] as Int) + offset
                    (r["nodes"] as? List<Map<String, Any?>>)?.let { ns -> m["nodes"] = ns.map { n -> LinkedHashMap(n).apply { put("index", (n["index"] as Int) + offset) } } }
                    roads += m
                }
                (nm["points"] as? List<Map<String, Any?>>)?.forEach { p ->
                    if (points.none { it["lat"] == p["lat"] && it["lon"] == p["lon"] && it["type"] == p["type"] }) {
                        points += LinkedHashMap(p).apply { put("index", (p["index"] as Int) + offset); put("piece", pi) }
                    }
                }
            }
        }
        cur?.let { segments += it }
        // le distanze lungo il percorso dei punti (autovelox…): dall'inizio del percorso unito
        if (points.isNotEmpty()) {
            val cum = DoubleArray(coords.size)
            for (i in 1 until coords.size) cum[i] = cum[i - 1] + DistanceCalc.haversine(coords[i - 1][1], coords[i - 1][0], coords[i][1], coords[i][0])
            for (p in points) {
                val m = p as MutableMap<String, Any?>
                val start = pieceStarts[m.remove("piece") as Int]
                m["offset"] = Math.round(((m["offset"] as Number).toDouble() + cum[start]) * 10) / 10.0
            }
            points.sortBy { (it["offset"] as Number).toDouble() }
        }
        val totalDistance = segments.sumOf { it["distance"] as Double }
        val totalDuration = round1(segments.sumOf { it["duration"] as Double })
        if ("percentage" in request.attributes) for (s in segments) s["percentage"] = if (totalDistance > 0) Math.round((s["distance"] as Double) * 100 / totalDistance * 100) / 100.0 else 0.0
        val properties = LinkedHashMap<String, Any?>()
        properties["segments"] = segments
        properties["way_points"] = wayPoints
        properties["summary"] = linkedMapOf("distance" to DirectionsService.roundForUnits(totalDistance, request.units), "duration" to totalDuration)
        if (extraValues.isNotEmpty()) {
            properties["extras"] = extraValues.mapValuesTo(LinkedHashMap()) { (kind, vals) ->
                val dist = extraDist[kind] ?: LinkedHashMap()
                val total = dist.values.sum()
                linkedMapOf("values" to vals, "summary" to dist.entries.sortedByDescending { it.value }.map { (v, d) ->
                    linkedMapOf("value" to v, "distance" to DirectionsService.roundForUnits(d, request.units), "amount" to (if (total > 0) Math.round(d * 100 / total * 100) / 100.0 else 0.0))
                })
            }
        }
        if (hasRoads) properties["navmaster"] = linkedMapOf("roads" to roads, "points" to points)
        val bboxArr = doubleArrayOf(DirectionsService.r6(bbox.minLon), DirectionsService.r6(bbox.minLat), DirectionsService.r6(bbox.maxLon), DirectionsService.r6(bbox.maxLat))
        return linkedMapOf("bbox" to bboxArr, "type" to "Feature", "properties" to properties,
            "geometry" to linkedMapOf("coordinates" to coords, "type" to "LineString"))
    }

    @Suppress("UNCHECKED_CAST")
    private fun shiftStep(s: Map<String, Any?>, offset: Int): MutableMap<String, Any?> {
        val m = LinkedHashMap(s)
        val wp = s["way_points"] as List<Int>
        m["way_points"] = listOf(wp[0] + offset, wp[1] + offset)
        return m
    }


    companion object {
        /**
         * Le istruzioni al passaggio di confine: via l'arrivo del pezzo prima e la partenza di quello
         * dopo; la strada prosegue (stesso nome: un'istruzione sola) o "Continua dritto su …".
         */
        internal fun joinSteps(before: MutableList<Map<String, Any?>>, after: MutableList<Map<String, Any?>>, coords: List<DoubleArray>, offset: Int,
                              t: InstructionTranslator) {
            if (before.lastOrNull()?.get("type") == InstructionType.FINISH.code) before.removeAt(before.size - 1)
            val depart = after.firstOrNull()
            if (depart == null) return
            val last = before.lastOrNull()
            val name = depart["name"] as? String ?: "-"
            val rest = after.drop(1)
            if (last != null && depart["type"] == InstructionType.DEPART.code && name != "-" && last["name"] == name) {
                // la stessa strada oltre il confine: l'istruzione di prima arriva fin dove arrivava la partenza
                val m = LinkedHashMap(last)
                m["distance"] = round1((last["distance"] as Number).toDouble() + (depart["distance"] as Number).toDouble())
                m["duration"] = round1((last["duration"] as Number).toDouble() + (depart["duration"] as Number).toDouble())
                m["way_points"] = listOf((last["way_points"] as List<*>)[0], (depart["way_points"] as List<*>)[1])
                before[before.size - 1] = m
            } else if (depart["type"] == InstructionType.DEPART.code) {
                val m = LinkedHashMap(depart)
                m["type"] = InstructionType.CONTINUE.code
                m["instruction"] = t.continueOn(InstructionType.CONTINUE, if (name == "-") "" else name)
                // la direzione d'arrivo al confine
                (depart["maneuver"] as? Map<*, *>)?.let { mv ->
                    val k = offset
                    val before0 = if (k >= 1) DistanceCalc.bearing(coords[k - 1][1], coords[k - 1][0], coords[k][1], coords[k][0]) else null
                    m["maneuver"] = LinkedHashMap<String, Any?>().apply {
                        putAll(mv.entries.associate { it.key.toString() to it.value })
                        if (before0 != null) put("bearing_before", Math.round((before0 + 360) % 360).toInt())
                    }
                }
                before += m
            } else before += depart
            before += rest
        }

        internal fun round1(v: Double) = Math.round(v * 10) / 10.0

        /** La finestra delle ricerche di limite oltre il passaggio più vicino (vedi [advanceAround]): un decimo più dieci minuti. */
        const val WINDOW_FACTOR = 1.1
        const val WINDOW_SLACK = 600.0
        /** Il raggio di aggancio di un passaggio di confine: il punto è un nodo della strada. */
        const val TRANSFER_RADIUS = 3.0
        /**
         * "Evita" ([shutIn]): i passaggi entro mezz'ora dal punto (senza le strade da evitare); col divieto
         * il punto è ostacolato se uno di questi diventa irraggiungibile o costa più del doppio e un quarto d'ora.
         */
        const val LOCAL_REACH = 1800.0
        const val LOCAL_DETOUR = 2.0
        const val LOCAL_SLACK = 900.0
        /**
         * Quanto sopra il tetto va il limite dimostrato da un percorso mirato senza esito (secondi): molto
         * più degli arrotondamenti delle somme, molto meno di qualsiasi differenza fra due viaggi.
         */
        const val CAP_MARGIN = 1e-6

        /** Varianti tra Paesi: oltre al pezzo più lungo, anche i pezzi di almeno questa quota del viaggio. */
        const val PIECE_SHARE = 0.25
        /** Fin dove si cerca la strada al valico per il titolo di una variante V ([crossingRef]). */
        const val CROSSING_REF_M = 3_000.0
        /** Sotto questo tempo rimasto non si cercano le varianti di un altro pezzo (ms). */
        const val MIN_PIECE_BUDGET_MS = 1_000L
        /**
         * La scadenza del passo 2 ([Ctx.topDeadline], `System.nanoTime`) alla fine del passo 1 ([now]):
         * mai prima di [topDeadline]; dopo, al più [TOP_AFTER_SHARE] di [budgetMs] da [now] e al più
         * metà del tempo che resta fino a [budgetMs] da [navStart] (l'altra metà, almeno, alle fonti M
         * e P dei pezzi), mai oltre [budgetMs] meno [MIN_PIECE_BUDGET_MS]. Un passo 1 che finisce entro
         * (TOP_BUDGET_SHARE − TOP_AFTER_SHARE) del tempo non allunga nulla: alle fonti M e P resta tutto
         * il loro tempo, come senza allungamento.
         */
        fun topDeadlineAfter(navStart: Long, budgetMs: Long, topDeadline: Long, now: Long): Long {
            val end = navStart + budgetMs * 1_000_000
            val want = minOf(now + (budgetMs * TOP_AFTER_SHARE).toLong() * 1_000_000, now + (end - now).coerceAtLeast(0) / 2,
                end - maxOf(MIN_PIECE_BUDGET_MS, variantsTailMs(budgetMs)) * 1_000_000)
            return maxOf(topDeadline, want)
        }

        /**
         * La coda di `budget_ms` riservata ai minimi garantiti delle varianti tra Paesi (ms): [V_GRACE_MS] alla
         * fonte V per altri Paesi (le catene già pesate dalla scelta costano solo i pezzi finali) e
         * [NavAlternativesEngine.MIN_SOURCE_BUDGET_MS] alle fonti M e P del pezzo più lungo; al più
         * [TAIL_MAX_SHARE] di `budget_ms`. Il passo 2 ([Ctx.topBudget]) finisce prima della coda; i minimi stanno
         * dentro `budget_ms`, mai dopo: le varianti automatiche entro 25 s sul tablet.
         */
        fun variantsTailMs(budgetMs: Long): Long =
            minOf(V_GRACE_MS + NavAlternativesEngine.MIN_SOURCE_BUDGET_MS, (budgetMs * TAIL_MAX_SHARE).toLong()).coerceAtLeast(0)

        /**
         * Il tempo delle fonti M e P del pezzo [n] (0: il più lungo) con [leftMs] ms fino alla fine di `budget_ms`
         * e [reserveMs] lasciati alle catene V negli stessi Paesi: il pezzo più lungo ha almeno
         * [NavAlternativesEngine.MIN_SOURCE_BUDGET_MS] (prima della riserva V), mai oltre la fine di `budget_ms`.
         */
        fun pieceBudgetMs(n: Int, leftMs: Long, reserveMs: Long): Long {
            val left = leftMs - reserveMs
            if (n == 0 && left < NavAlternativesEngine.MIN_SOURCE_BUDGET_MS) return minOf(NavAlternativesEngine.MIN_SOURCE_BUDGET_MS, leftMs).coerceAtLeast(0)
            return left.coerceAtLeast(0)
        }

        /** Il tempo lasciato alle catene V negli stessi Paesi dopo le varianti dei pezzi (ms). */
        const val V_RESERVE_MS = 3_000L
        /**
         * Il tempo minimo della fonte V per altri Paesi (ms), dentro la coda di `budget_ms` ([variantsTailMs]):
         * le catene già pesate dalla scelta dei valichi hanno i tratti nella cache, costano i pezzi finali
         * (sul tablet 0,1-0,3 s l'una). «Almeno 3 alternative sempre», ma entro il tempo.
         */
        const val V_GRACE_MS = 1_500L
        /** La quota massima di `budget_ms` per la coda dei minimi garantiti ([variantsTailMs]). */
        const val TAIL_MAX_SHARE = 0.4
        /** I percorsi mirati di una ricerca di limite al più ([focusMax]). */
        const val FOCUS_MAX = 8
        /**
         * Quanto aspetta (ms), con due thread sulle sequenze, una scelta dei due valichi che la memoria dà
         * per «nessuna coppia sotto l» mentre il tetto è ancora più alto: l'altro thread, con le sue
         * sequenze in memoria, ha il migliore in pochi ms. Poi la scelta si fa.
         */
        const val TRANSIT_WAIT_MS = 500L
        /**
         * La quota di `budget_ms` per la parte della scelta dei valichi che serve solo alla fonte V
         * ([Ctx.topBudget]: il passo 2 di [chain], gli extra di [chooseExactTop]), che viene prima
         * delle varianti dei pezzi: il resto resta alle fonti M e P (sul tablet 10 s su 25).
         */
        const val TOP_BUDGET_SHARE = 0.6
        /**
         * Dopo un passo 1 lento il passo 2 ha fino a questa quota di `budget_ms` dalla sua fine, e non
         * più di metà del tempo che resta ([topDeadlineAfter], [extendTopBudget]).
         */
        const val TOP_AFTER_SHARE = 0.2
        /** Lo strato della programmazione dinamica ai confini intermedi ([dpChain]): per il migliore, e per le altre catene della fonte V. */
        const val LAYER = 8
        const val LAYER_EXTRA = 3
        /** Ogni quanto l'attesa di un peso in corso sull'altro thread ([Ctx.weight]) controlla l'annullamento (ms). */
        const val WAIT_MS = 50L

        /**
         * Le sequenze di grafi da [from] a [to] fra gli [ids] (al più 4 grafi, nessuna ripetizione;
         * solo se nessuna di 4 c'è, 5 e poi [MAX_GRAPHS]: Italia → Svezia, Bulgaria, Lituania):
         * un grafo entra se il suo rettangolo ([bounds], null: mai) sta entro `0,35·crow + 60 km`
         * dalla linea p → q (l'arrivo sempre), ogni passo fra rettangoli che si intersecano e grafi
         * [linked] (valichi non vuoti). Al più 6: le prime 6 per lunghezza nell'ordine degli [ids]
         * (lo stesso insieme di sempre), messe prima le più corte, poi quelle coi centri dei
         * rettangoli dei grafi intermedi meno lontani dalla linea (cambia solo l'ordine di esame);
         * poi, con [extra] (solo il passo 2 delle varianti), fino a [extra] escluse, per lunghezza e
         * deviazione. Con [from] uguale a [to] i giri A-B-A per tutti i Paesi vicini, prima quelli col
         * centro del rettangolo più vicino al viaggio (p → centro → q, come l'app): quali tenere lo dice
         * [aroundSequences] coi valichi. La stessa regola nell'app (quali grafi aprire) e nel motore
         * ([countryPaths]: quali pesare).
         */
        fun countrySequences(ids: List<String>, bounds: (String) -> BBox?, linked: (String, String) -> Boolean,
                             pLat: Double, pLon: Double, qLat: Double, qLon: Double, from: String, to: String, extra: Int = 0): List<List<String>> {
            val crow = DistanceCalc.haversine(pLat, pLon, qLat, qLon)
            val reach = crow * 0.35 + 60_000.0
            val boxes = HashMap<String, BBox?>()
            fun box(id: String) = boxes.getOrPut(id) { bounds(id)?.takeIf { it.isValid } }
            fun near(id: String) = box(id)?.let { distanceToBox(pLat, pLon, qLat, qLon, it) <= reach } == true
            fun deviation(seq: List<String>) = seq.drop(1).dropLast(1).sumOf { id ->
                box(id)?.let { b -> distanceToSegment(pLat, pLon, qLat, qLon, (b.minLat + b.maxLat) / 2, (b.minLon + b.maxLon) / 2) } ?: 0.0 }
            // un Paese solo: i giri per un Paese vicino A-B-A (B vicino alla linea, rettangoli che si toccano, valichi), tutti,
            // per p → centro → q; i primi AROUND_MAX solo dopo il filtro dei valichi (aroundSequences): prima il taglio
            // veniva per primo e un vicino col centro lontano (la Germania da Innsbruck a Salisburgo, Deutsches Eck) non si guardava mai
            if (from == to) {
                if (from !in ids) return emptyList()
                val fb = box(from) ?: return emptyList()
                fun via(id: String) = box(id)!!.let { b -> val la = (b.minLat + b.maxLat) / 2; val lo = (b.minLon + b.maxLon) / 2
                    DistanceCalc.haversine(pLat, pLon, la, lo) + DistanceCalc.haversine(la, lo, qLat, qLon) }
                return ids.filter { g -> g != from && near(g) && box(g)?.let { fb.intersects(it) } == true && linked(from, g) }
                    .sortedBy { via(it) }.map { listOf(from, it, from) }
            }
            val out = ArrayList<List<String>>()
            var maxGraphs = 4
            fun walk(path: List<String>) {
                val last = path.last()
                if (last == to) { out += path; return }
                if (path.size >= maxGraphs) return
                val lb = box(last) ?: return
                for (g in ids) {
                    if (g in path) continue
                    if (g != to && !near(g)) continue
                    val gb = box(g) ?: continue
                    if (!lb.intersects(gb)) continue
                    if (!linked(last, g)) continue
                    walk(path + g)
                }
            }
            if (from in ids && to in ids) {
                walk(listOf(from))
                // nessuna sequenza di al più 4 grafi (Italia → Svezia, Bulgaria, Lituania): 5, poi 6 grafi.
                // Dove una sequenza corta c'è, l'insieme è quello di sempre (il migliore non cambia)
                while (out.isEmpty() && maxGraphs < MAX_GRAPHS) { maxGraphs++; walk(listOf(from)) }
            }
            // le prime 6 per lunghezza nell'ordine dei grafi (l'insieme di sempre: il migliore non cambia), poi
            // prima le sequenze più corte, poi quelle che deviano meno dalla linea (i centri dei grafi intermedi)
            val first = out.sortedBy { it.size }.take(6)
            val base = first.sortedWith(compareBy({ it.size }, { deviation(it) }))
            if (extra <= 0) return base
            // [extra] in più (solo il passo 2 delle varianti): le escluse, per lunghezza e deviazione
            return base + out.filter { s -> first.none { it === s } }.sortedWith(compareBy({ it.size }, { deviation(it) })).take(extra)
        }

        /**
         * I giri A-B-A di un viaggio p → q in un Paese solo fra quelli di [countrySequences] (nel suo ordine):
         * quelli il cui Paese vicino ha due valichi utili ([Transfers.aroundOk], [transfers] dell'id del
         * vicino), al più [AROUND_MAX]. Prima il filtro, poi i primi: come l'app (quali grafi aprire).
         */
        fun aroundSequences(seqs: List<List<String>>, transfers: (String) -> List<Transfer>, pLat: Double, pLon: Double, qLat: Double, qLon: Double): List<List<String>> =
            seqs.filter { seq -> Transfers.aroundOk(transfers(seq[1]), pLat, pLon, qLat, qLon) }.take(AROUND_MAX)

        /** I grafi al più di una sequenza di Paesi quando nessuna di 4 c'è ([countrySequences]). */
        const val MAX_GRAPHS = 6
        /** Le sequenze in più del solo passo 2 di [chain] (fonte V), oltre alle 6 del passo 1. */
        const val EXTRA_SEQUENCES = 3
        /** I giri per un Paese vicino (A-B-A) al più, per un viaggio in un Paese solo. */
        const val AROUND_MAX = 2
        /**
         * Un giro per un Paese vicino si guarda se due suoi valichi t1, t2 (in luoghi diversi) stanno
         * entro questo fattore della linea: crow(p,t1) + crow(t1,t2) + crow(t2,q) ≤ 1,2·crow(p,q).
         */
        const val AROUND_FACTOR = 1.2

        /** Distanza (m, approssimata) fra il segmento p → q e un rettangolo. */
        private fun distanceToBox(pLat: Double, pLon: Double, qLat: Double, qLon: Double, b: BBox): Double {
            var best = Double.MAX_VALUE
            for (i in 0..40) {
                val t = i / 40.0
                val la = pLat + (qLat - pLat) * t; val lo = pLon + (qLon - pLon) * t
                val cla = la.coerceIn(b.minLat, b.maxLat); val clo = lo.coerceIn(b.minLon, b.maxLon)
                best = minOf(best, DistanceCalc.haversine(la, lo, cla, clo))
            }
            return best
        }

        /** Distanza (m, approssimata) fra il segmento p → q e un punto. */
        private fun distanceToSegment(pLat: Double, pLon: Double, qLat: Double, qLon: Double, lat: Double, lon: Double): Double {
            var best = Double.MAX_VALUE
            for (i in 0..40) {
                val t = i / 40.0
                best = minOf(best, DistanceCalc.haversine(pLat + (qLat - pLat) * t, pLon + (qLon - pLon) * t, lat, lon))
            }
            return best
        }

        /**
         * Il nome italiano di un Paese dal suo id di grafo, con la preposizione («per la Cechia»);
         * l'id stesso se ignoto (l'app lo sostituisce col nome del catalogo).
         */
        fun countryName(id: String): String = COUNTRY_NAMES[id] ?: id
        private val COUNTRY_NAMES = mapOf(
            "italy" to "l'Italia", "austria" to "l'Austria", "germany" to "la Germania", "france" to "la Francia", "switzerland" to "la Svizzera",
            "slovenia" to "la Slovenia", "croatia" to "la Croazia", "hungary" to "l'Ungheria", "slovakia" to "la Slovacchia", "czech-republic" to "la Cechia",
            "poland" to "la Polonia", "belgium" to "il Belgio", "netherlands" to "i Paesi Bassi", "luxembourg" to "il Lussemburgo", "spain" to "la Spagna",
            "portugal" to "il Portogallo", "denmark" to "la Danimarca", "liechtenstein" to "il Liechtenstein", "monaco" to "Monaco", "andorra" to "Andorra",
            "san-marino" to "San Marino", "united-kingdom" to "il Regno Unito", "ireland" to "l'Irlanda", "sweden" to "la Svezia", "norway" to "la Norvegia",
            "finland" to "la Finlandia", "romania" to "la Romania", "bulgaria" to "la Bulgaria", "greece" to "la Grecia", "serbia" to "la Serbia",
            "bosnia-herzegovina" to "la Bosnia ed Erzegovina", "montenegro" to "il Montenegro", "albania" to "l'Albania", "north-macedonia" to "la Macedonia del Nord",
            "kosovo" to "il Kosovo", "lithuania" to "la Lituania", "latvia" to "la Lettonia", "estonia" to "l'Estonia", "belarus" to "la Bielorussia",
            "ukraine" to "l'Ucraina", "moldova" to "la Moldavia", "turkey" to "la Turchia",
        )

        /**
         * Le strade del titolo di una variante V per un'altra sequenza di Paesi: fra le strade nuove
         * [newRefs] (nell'ordine della strada) le autostrade (ref da autostrada o, per almeno metà dei
         * km, archi di autostrada o superstrada: le D ceche, [mwShare]), le due più lunghe per km
         * ([refM]), nell'ordine della strada. Vuota se nessuna strada nuova è un'autostrada: allora il
         * titolo è «altro valico» (le statali corte all'inizio non dicono la variante, che cambia Paesi).
         */
        internal fun otherCountriesTitleRefs(newRefs: List<String>, refM: Map<String, Double>, mwShare: Map<String, Double>): List<String> {
            val mw = newRefs.filter { CorridorAnalyzer.isMotorwayRef(it) || (mwShare[it] ?: 0.0) >= 0.5 }
            val top = mw.sortedByDescending { refM[it] ?: 0.0 }.take(2).toSet()
            return mw.filter { it in top }
        }

        /** Diagnostica: registra anche la scelta di un valico alla volta (con le sue ricerche) per confronto. */
        @Volatile @JvmStatic var compareOneByOne = false
    }
}
