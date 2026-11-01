package it.navmaster.engine.index

import com.carrotsearch.hppc.IntArrayList
import com.carrotsearch.hppc.IntHashSet
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.FetchMode
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.weighting.EdgeFilter
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

enum class SnapPosition { TOWER, PILLAR, EDGE }

/** Risultato dell'aggancio di una coordinata alla rete. */
class Snap(val queryLat: Double, val queryLon: Double) {
    var edge = -1
    /** Nodo torre più vicino lungo l'arco (nodeA o nodeB); con [position] TOWER è il nodo esatto. */
    var closestNode = -1
    var position = SnapPosition.EDGE
    /** Indice nella geometria ALL (A→B) del punto/segmento agganciato. */
    var wayIndex = -1
    var snappedLat = Double.NaN
    var snappedLon = Double.NaN
    /** Distanza in metri fra la coordinata richiesta e il punto agganciato. */
    var distance = Double.POSITIVE_INFINITY
    val isValid get() = edge >= 0
}

/**
 * Indice spaziale a griglia regolare sulla bbox del grafo: ogni cella elenca gli archi che la
 * attraversano. Semplice e compatto (CSR); per bbox continentali il passo si adatta per non superare
 * [maxCells] celle. Un quadtree adattivo come in GraphHopper è previsto in una fase successiva.
 */
class LocationIndex private constructor(
    private val graph: BaseGraph,
    private val minLat: Double, private val minLon: Double,
    private val cellSizeLat: Double, private val cellSizeLon: Double,
    private val rows: Int, private val cols: Int,
    private val cellStart: IntArray,   // cols*rows + 1
    private val cellEdges: IntArray,
) {
    val cells get() = rows * cols
    val entries get() = cellEdges.size

    private fun row(lat: Double) = floor((lat - minLat) / cellSizeLat).toInt().coerceIn(0, rows - 1)
    private fun col(lon: Double) = floor((lon - minLon) / cellSizeLon).toInt().coerceIn(0, cols - 1)

    /**
     * Arco più vicino che passa il filtro, cercando in anelli di celle crescenti.
     * Si ferma quando l'anello successivo non può contenere nulla di più vicino, o dopo [maxRings] anelli.
     */
    fun findClosest(lat: Double, lon: Double, filter: EdgeFilter = EdgeFilter.ALL, maxRings: Int = 8): Snap {
        val snap = Snap(lat, lon)
        if (!graph.bounds.isValid || graph.edgeCount == 0) return snap
        val r0 = row(lat); val c0 = col(lon)
        val seen = IntHashSet()
        val it = graph.createEdgeIterator()
        val cellMeters = min(cellSizeLat, cellSizeLon * cos(Math.toRadians(lat))) * DistanceCalc.METERS_PER_DEGREE
        for (ring in 0..maxRings) {
            // la distanza minima a cui può stare un arco nell'anello `ring` (dal bordo della cella centrale)
            val minPossible = if (ring == 0) 0.0 else (ring - 1) * cellMeters + distanceToCellBorder(lat, lon, r0, c0)
            if (snap.distance < minPossible) break
            var any = false
            for (r in r0 - ring..r0 + ring) {
                if (r < 0 || r >= rows) continue
                for (c in c0 - ring..c0 + ring) {
                    if (c < 0 || c >= cols) continue
                    if (abs(r - r0) != ring && abs(c - c0) != ring) continue // solo il bordo dell'anello
                    any = true
                    val cell = r * cols + c
                    for (k in cellStart[cell] until cellStart[cell + 1]) {
                        val e = cellEdges[k]
                        if (!seen.add(e)) continue
                        it.setEdge(e, graph.nodeA(e))
                        if (!filter.accept(it)) continue
                        traverseEdge(e, lat, lon, snap)
                    }
                }
            }
            if (!any && ring > 0) break
        }
        return snap
    }

    private fun distanceToCellBorder(lat: Double, lon: Double, r: Int, c: Int): Double {
        val dLat = min(lat - (minLat + r * cellSizeLat), minLat + (r + 1) * cellSizeLat - lat)
        val dLon = min(lon - (minLon + c * cellSizeLon), minLon + (c + 1) * cellSizeLon - lon) * cos(Math.toRadians(lat))
        return max(0.0, min(dLat, dLon)) * DistanceCalc.METERS_PER_DEGREE
    }

    /** Esamina torri, pillar e segmenti dell'arco e aggiorna [snap] se trova qualcosa di più vicino. */
    private fun traverseEdge(edge: Int, lat: Double, lon: Double, snap: Snap) {
        val pl = graph.fetchWayGeometry(edge, reverse = false, FetchMode.ALL)
        val a = graph.nodeA(edge); val b = graph.nodeB(edge)
        for (i in 1 until pl.size) {
            val la1 = pl.lat(i - 1); val lo1 = pl.lon(i - 1); val la2 = pl.lat(i); val lo2 = pl.lon(i)
            // punto del segmento più vicino (proiezione piana)
            val cosLat = cos(Math.toRadians(lat))
            val dx = (lo2 - lo1) * cosLat; val dy = la2 - la1
            val len2 = dx * dx + dy * dy
            var t = if (len2 == 0.0) 0.0 else (((lon - lo1) * cosLat) * dx + (lat - la1) * dy) / len2
            t = t.coerceIn(0.0, 1.0)
            val pLat = la1 + t * dy; val pLon = lo1 + t * (lo2 - lo1)
            val d = DistanceCalc.planeDistance(lat, lon, pLat, pLon)
            if (d < snap.distance) {
                snap.distance = d; snap.edge = edge; snap.snappedLat = pLat; snap.snappedLon = pLon
                when {
                    t <= 1e-9 && i == 1 -> { snap.position = SnapPosition.TOWER; snap.closestNode = a; snap.wayIndex = 0 }
                    t >= 1 - 1e-9 && i == pl.size - 1 -> { snap.position = SnapPosition.TOWER; snap.closestNode = b; snap.wayIndex = pl.size - 1 }
                    t <= 1e-9 -> { snap.position = SnapPosition.PILLAR; snap.wayIndex = i - 1; snap.closestNode = closerTower(pl, i - 1, a, b) }
                    t >= 1 - 1e-9 -> { snap.position = SnapPosition.PILLAR; snap.wayIndex = i; snap.closestNode = closerTower(pl, i, a, b) }
                    else -> { snap.position = SnapPosition.EDGE; snap.wayIndex = i - 1; snap.closestNode = closerTower(pl, i - 1, a, b) }
                }
            }
        }
    }

    private fun closerTower(pl: PointList, idx: Int, a: Int, b: Int): Int = if (idx < (pl.size - 1) / 2) a else b

    /** Archi che toccano le celle della bbox data (può contenere archi che non intersecano la bbox). */
    fun query(minLat: Double, minLon: Double, maxLat: Double, maxLon: Double): IntArray {
        val set = IntHashSet()
        for (r in row(minLat)..row(maxLat)) for (c in col(minLon)..col(maxLon)) {
            val cell = r * cols + c
            for (k in cellStart[cell] until cellStart[cell + 1]) set.add(cellEdges[k])
        }
        return set.toArray()
    }

    companion object {
        /** Passo minimo delle celle in gradi (circa 300 m in latitudine). */
        const val MIN_CELL_DEG = 0.0027

        fun build(graph: BaseGraph, maxCells: Int = 4_000_000): LocationIndex {
            val b = graph.bounds
            val minLat = if (b.isValid) b.minLat else -10.0
            val minLon = if (b.isValid) b.minLon else -10.0
            val spanLat = if (b.isValid) max(b.maxLat - b.minLat, MIN_CELL_DEG) else 20.0
            val spanLon = if (b.isValid) max(b.maxLon - b.minLon, MIN_CELL_DEG) else 20.0
            var cell = MIN_CELL_DEG
            while ((spanLat / cell + 1) * (spanLon / cell + 1) > maxCells) cell *= 1.5
            val rows = (spanLat / cell).toInt() + 1
            val cols = (spanLon / cell).toInt() + 1
            val perCell = Array(rows * cols) { IntArrayList(0) }
            val cellsOfEdge = IntHashSet()
            for (e in 0 until graph.edgeCount) {
                val pl = graph.fetchWayGeometry(e, reverse = false, FetchMode.ALL)
                cellsOfEdge.clear()
                for (i in 1 until pl.size) rasterize(pl.lat(i - 1), pl.lon(i - 1), pl.lat(i), pl.lon(i), minLat, minLon, cell, rows, cols, cellsOfEdge)
                for (cur in cellsOfEdge) perCell[cur.value].add(e)
            }
            val start = IntArray(rows * cols + 1)
            for (i in perCell.indices) start[i + 1] = start[i] + perCell[i].size()
            val edges = IntArray(start[rows * cols])
            for (i in perCell.indices) { var k = start[i]; for (j in 0 until perCell[i].size()) edges[k++] = perCell[i].get(j) }
            return LocationIndex(graph, minLat, minLon, cell, cell, rows, cols, start, edges)
        }

        /** Celle attraversate da un segmento: campionamento a passo di mezza cella più gli estremi. */
        private fun rasterize(la1: Double, lo1: Double, la2: Double, lo2: Double, minLat: Double, minLon: Double, cell: Double, rows: Int, cols: Int, out: IntHashSet) {
            val steps = max(1, (max(abs(la2 - la1), abs(lo2 - lo1)) / (cell / 2)).toInt())
            for (s in 0..steps) {
                val t = s.toDouble() / steps
                val la = la1 + t * (la2 - la1); val lo = lo1 + t * (lo2 - lo1)
                val r = floor((la - minLat) / cell).toInt().coerceIn(0, rows - 1)
                val c = floor((lo - minLon) / cell).toInt().coerceIn(0, cols - 1)
                out.add(r * cols + c)
            }
        }
    }
}

/** Distanza euclidea di comodo in gradi (solo per confronti locali). */
internal fun degDistance(la1: Double, lo1: Double, la2: Double, lo2: Double) = sqrt((la1 - la2) * (la1 - la2) + (lo1 - lo2) * (lo1 - lo2))
