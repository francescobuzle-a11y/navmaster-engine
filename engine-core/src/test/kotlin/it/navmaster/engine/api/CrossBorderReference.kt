package it.navmaster.engine.api

/**
 * Il riferimento esaustivo per i viaggi tra Paesi: tutti i passaggi fra ogni coppia di grafi (nessun
 * raggruppamento per luogo, nessun limite di candidati), le sequenze di Paesi diretta e con un Paese
 * intermedio, ogni pezzo pesato come il percorso finale lo calcola ([DirectionsService.routes] sulla
 * richiesta del pezzo, con tutte le impostazioni). Il viaggio è quello col peso totale minimo
 * (programmazione dinamica sui confini). Lento: solo per le prove.
 */
class CrossBorderReference(
    private val graphs: List<CountryGraph>,
    private val transfers: (CountryGraph, CountryGraph) -> List<Transfer>,
) {
    class Result(val weight: Double, val crossings: List<Transfer>, val pieceCount: Int)

    var pieceEvaluations = 0; private set

    /** Il peso di un pezzo come lo calcola il percorso finale; +∞ quando il servizio non trova il percorso. */
    fun pieceWeight(g: CountryGraph, request: DirectionsRequest): Double = try {
        pieceEvaluations++
        g.service.routes(request).single().sumOf { it.weight }
    } catch (e: ApiException) { Double.POSITIVE_INFINITY }

    /** Il peso di un viaggio già fatto a pezzi (per esempio quelli di [CrossBorderDirections]). */
    fun weightOf(pieces: List<Pair<CountryGraph, DirectionsRequest>>): Double = pieces.sumOf { (g, r) -> pieceWeight(g, r) }

    /** Un punto di un pezzo: della richiesta (indice) o un passaggio di confine. */
    private sealed class End {
        class Point(val index: Int) : End()
        class Crossing(val transfer: Transfer) : End()
    }

    /**
     * Il viaggio migliore per [request], ogni punto nel grafo [of] (come lo sceglie
     * [CrossBorderDirections]: per le prove si danno i grafi espliciti).
     */
    fun best(request: DirectionsRequest, of: List<CountryGraph>): Result {
        val n = request.coordinates.size
        // tutti i punti in un grafo: un pezzo solo, come fa CrossBorderDirections
        if (of.all { it === of[0] }) return Result(pieceWeight(of[0], request), emptyList(), 1)
        val radius = { i: Int -> request.radiuses?.getOrNull(i)?.takeIf { it > 0 } ?: graphs.first().service.maximumSnappingRadius }
        // i blocchi di punti consecutivi nello stesso grafo
        val blocks = ArrayList<IntRange>()
        var start = 0
        for (i in 1..n) if (i == n || of[i] !== of[start]) { blocks += start until i; start = i }
        // per ogni passaggio fra blocchi le sequenze di Paesi: diretta o con un Paese intermedio
        val options = (1 until blocks.size).map { k ->
            val a = of[blocks[k - 1].first]; val b = of[blocks[k].first]
            val out = ArrayList<List<CountryGraph>>()
            if (transfers(a, b).isNotEmpty()) out += listOf(a, b)
            for (c in graphs) if (c !== a && c !== b && transfers(a, c).isNotEmpty() && transfers(c, b).isNotEmpty()) out += listOf(a, c, b)
            out
        }
        if (options.any { it.isEmpty() }) return Result(Double.POSITIVE_INFINITY, emptyList(), 0)
        var best = Result(Double.POSITIVE_INFINITY, emptyList(), 0)
        // ogni combinazione di sequenze (poche: al più due passaggi nelle prove)
        fun combos(k: Int, acc: List<List<CountryGraph>>): List<List<List<CountryGraph>>> =
            if (k == options.size) listOf(acc) else options[k].flatMap { combos(k + 1, acc + listOf(it)) }
        for (combo in combos(0, emptyList())) {
            // i pezzi: (grafo, punti della richiesta nel pezzo); i confini fra un pezzo e il successivo
            val pieceGraphs = ArrayList<CountryGraph>(); val piecePoints = ArrayList<List<Int>>()
            for ((k, block) in blocks.withIndex()) {
                if (k > 0) for (g in combo[k - 1].subList(1, combo[k - 1].size - 1)) { pieceGraphs += g; piecePoints += emptyList<Int>() }
                pieceGraphs += of[block.first]; piecePoints += block.toList()
            }
            // programmazione dinamica: il peso migliore fino a ogni passaggio del confine b
            fun weight(pi: Int, from: End?, to: End?): Double {
                val ends = ArrayList<End>()
                if (from != null) ends += from
                for (i in piecePoints[pi]) ends += End.Point(i)
                if (to != null) ends += to
                val pts = ends.map { e -> when (e) { is End.Point -> request.coordinates[e.index]; is End.Crossing -> e.transfer.lat to e.transfer.lon } }
                val rad = ends.map { e -> when (e) { is End.Point -> radius(e.index); is End.Crossing -> CrossBorderDirections.TRANSFER_RADIUS } }.toDoubleArray()
                val exact = ends.map { it is End.Crossing }.toBooleanArray()
                val bearings = ends.map { e -> if (e is End.Point) request.bearings?.getOrNull(e.index) else null }
                return pieceWeight(pieceGraphs[pi], request.withPoints(pts, rad, exact, bearings))
            }
            var layer: List<Pair<Transfer, Pair<Double, List<Transfer>>>> =
                transfers(pieceGraphs[0], pieceGraphs[1]).map { t -> t to (weight(0, null, End.Crossing(t)) to listOf(t)) }
            for (pi in 1 until pieceGraphs.size - 1) {
                val ts = transfers(pieceGraphs[pi], pieceGraphs[pi + 1])
                layer = ts.map { t ->
                    var bw = Double.POSITIVE_INFINITY; var bc: List<Transfer> = emptyList()
                    for ((prev, wc) in layer) {
                        if (wc.first.isInfinite()) continue
                        val w = wc.first + weight(pi, End.Crossing(prev), End.Crossing(t))
                        if (w < bw) { bw = w; bc = wc.second + t }
                    }
                    t to (bw to bc)
                }
            }
            val last = pieceGraphs.size - 1
            for ((prev, wc) in layer) {
                if (wc.first.isInfinite()) continue
                val w = wc.first + weight(last, End.Crossing(prev), null)
                if (w < best.weight) best = Result(w, wc.second, pieceGraphs.size)
            }
        }
        return best
    }
}
