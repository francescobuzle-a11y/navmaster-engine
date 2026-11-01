package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable
import java.io.File
import kotlin.math.roundToLong

/** Riquadro geografico in gradi. */
data class BBox(var minLon: Double, var maxLon: Double, var minLat: Double, var maxLat: Double) {
    fun include(lat: Double, lon: Double) {
        if (lat < minLat) minLat = lat
        if (lat > maxLat) maxLat = lat
        if (lon < minLon) minLon = lon
        if (lon > maxLon) maxLon = lon
    }
    val isValid: Boolean get() = minLat <= maxLat && minLon <= maxLon
    fun contains(lat: Double, lon: Double) = lat in minLat..maxLat && lon in minLon..maxLon
    fun intersects(o: BBox) = !(o.minLon > maxLon || o.maxLon < minLon || o.minLat > maxLat || o.maxLat < minLat)

    companion object {
        fun empty() = BBox(Double.MAX_VALUE, -Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE)
    }
}

/** Lista di coordinate (lat, lon[, ele]) senza oggetti per punto. */
class PointList(initialCapacity: Int = 10, val is3D: Boolean = false) {
    private var lats = DoubleArray(initialCapacity)
    private var lons = DoubleArray(initialCapacity)
    private var eles = if (is3D) DoubleArray(initialCapacity) else DoubleArray(0)
    var size = 0; private set

    fun add(lat: Double, lon: Double, ele: Double = Double.NaN) {
        if (size == lats.size) grow()
        lats[size] = lat; lons[size] = lon
        if (is3D) eles[size] = ele
        size++
    }

    private fun grow() {
        val n = maxOf(4, lats.size * 2)
        lats = lats.copyOf(n); lons = lons.copyOf(n)
        if (is3D) eles = eles.copyOf(n)
    }

    fun lat(i: Int) = lats[i]
    fun lon(i: Int) = lons[i]
    fun ele(i: Int) = if (is3D) eles[i] else Double.NaN
    val isEmpty get() = size == 0

    fun reverse() {
        var i = 0; var j = size - 1
        while (i < j) {
            val la = lats[i]; lats[i] = lats[j]; lats[j] = la
            val lo = lons[i]; lons[i] = lons[j]; lons[j] = lo
            if (is3D) { val e = eles[i]; eles[i] = eles[j]; eles[j] = e }
            i++; j--
        }
    }

    fun copy(from: Int, toExclusive: Int): PointList {
        val p = PointList(maxOf(1, toExclusive - from), is3D)
        for (i in from until toExclusive) p.add(lats[i], lons[i], ele(i))
        return p
    }

    override fun toString() = (0 until size).joinToString(", ", "[", "]") { "(${lats[it]},${lons[it]})" }
}

/** Quali punti di un arco restituire. */
enum class FetchMode { TOWER_ONLY, PILLAR_ONLY, BASE_AND_PILLAR, PILLAR_AND_ADJ, ALL }

/**
 * Grafo stradale: nodi (torri), archi non orientati con flag per direzione, geometria intermedia (pillar).
 *
 * Layout nodo (12/16 byte): edgeRef@0 (-1 = nessun arco) | lat@4 (1e-7) | lon@8 | [ele@12 mm]
 * Layout arco (28 + 4·intsForFlags byte): nodeA@0 | nodeB@4 | linkA@8 | linkB@12 | flags@16 | dist(mm) | geoRef | nameRef
 * Geometria: a `geoRef*4`: count, poi count × (lat, lon[, ele]) in ordine nodeA→nodeB; geoRef 0 = nessun pillar.
 */
class BaseGraph(
    private val dir: GraphDirectory,
    val encodedValues: EncodedValueRegistry,
    val is3D: Boolean = false,
) : Closeable {
    private var nodes: DataAccess = dir.create("nodes", NODE_SEGMENT)
    private var edges: DataAccess = dir.create("edges", EDGE_SEGMENT)
    private var geometry: DataAccess = dir.create("geometry")
    val names = StringStore(dir.create("names"))

    val intsForFlags = encodedValues.intsForFlags
    private val nodeEntryBytes = if (is3D) 16 else 12
    private val edgeEntryBytes = 28 + 4 * intsForFlags
    private val eDist = 16 + 4 * intsForFlags
    private val eGeo = eDist + 4
    private val eName = eGeo + 4

    var nodeCount = 0; private set
    var edgeCount = 0; private set
    private var nextGeoRef = 4L // in unità int; 0 riservato a "nessuna geometria"
    val bounds = BBox.empty()
    var frozen = false; private set

    // ---------------------------------------------------------------- nodi

    fun addNode(lat: Double, lon: Double, ele: Double = Double.NaN): Int {
        check(!frozen)
        val id = nodeCount++
        nodes.ensureCapacity((nodeCount.toLong() + 1) * nodeEntryBytes)
        val p = id.toLong() * nodeEntryBytes
        nodes.setInt(p, NO_EDGE)
        nodes.setInt(p + 4, degToInt(lat))
        nodes.setInt(p + 8, degToInt(lon))
        if (is3D) nodes.setInt(p + 12, eleToInt(ele))
        bounds.include(lat, lon)
        return id
    }

    fun lat(node: Int): Double = intToDeg(nodes.getInt(node.toLong() * nodeEntryBytes + 4))
    fun lon(node: Int): Double = intToDeg(nodes.getInt(node.toLong() * nodeEntryBytes + 8))
    fun ele(node: Int): Double = if (is3D) intToEle(nodes.getInt(node.toLong() * nodeEntryBytes + 12)) else Double.NaN
    fun setNodeCoords(node: Int, lat: Double, lon: Double, ele: Double = Double.NaN) {
        val p = node.toLong() * nodeEntryBytes
        nodes.setInt(p + 4, degToInt(lat)); nodes.setInt(p + 8, degToInt(lon))
        if (is3D) nodes.setInt(p + 12, eleToInt(ele))
        bounds.include(lat, lon)
    }

    private fun edgeRef(node: Int): Int = nodes.getInt(node.toLong() * nodeEntryBytes)
    private fun setEdgeRef(node: Int, edge: Int) = nodes.setInt(node.toLong() * nodeEntryBytes, edge)

    // ---------------------------------------------------------------- archi

    /** Crea un arco tra due nodi; i flag sono zero finché non vengono impostati. */
    fun addEdge(nodeA: Int, nodeB: Int, distanceMeters: Double): Int {
        check(!frozen)
        require(nodeA in 0 until nodeCount && nodeB in 0 until nodeCount) { "nodo inesistente: $nodeA/$nodeB" }
        val id = edgeCount++
        edges.ensureCapacity((edgeCount.toLong() + 1) * edgeEntryBytes)
        val p = id.toLong() * edgeEntryBytes
        edges.setInt(p, nodeA)
        edges.setInt(p + 4, nodeB)
        edges.setInt(p + 8, edgeRef(nodeA))
        setEdgeRef(nodeA, id)
        if (nodeA != nodeB) {
            edges.setInt(p + 12, edgeRef(nodeB))
            setEdgeRef(nodeB, id)
        } else {
            edges.setInt(p + 12, NO_EDGE)
        }
        for (i in 0 until intsForFlags) edges.setInt(p + 16 + 4L * i, 0)
        setDistance(id, distanceMeters)
        edges.setInt(p + eGeo, 0)
        edges.setInt(p + eName, 0)
        return id
    }

    fun nodeA(edge: Int): Int = edges.getInt(edge.toLong() * edgeEntryBytes)
    fun nodeB(edge: Int): Int = edges.getInt(edge.toLong() * edgeEntryBytes + 4)

    fun distance(edge: Int): Double = edges.getInt(edge.toLong() * edgeEntryBytes + eDist) / 1000.0

    fun setDistance(edge: Int, meters: Double) {
        require(meters >= 0 && !meters.isNaN()) { "distanza non valida: $meters" }
        val mm = minOf((meters * 1000).roundToLong(), MAX_DIST_MM)
        edges.setInt(edge.toLong() * edgeEntryBytes + eDist, mm.toInt())
    }

    fun getFlags(edge: Int, dst: IntArray = IntArray(intsForFlags)): IntArray {
        val p = edge.toLong() * edgeEntryBytes + 16
        for (i in 0 until intsForFlags) dst[i] = edges.getInt(p + 4L * i)
        return dst
    }

    fun setFlags(edge: Int, flags: IntArray) {
        val p = edge.toLong() * edgeEntryBytes + 16
        for (i in 0 until intsForFlags) edges.setInt(p + 4L * i, flags[i])
    }

    fun nameRef(edge: Int): Int = edges.getInt(edge.toLong() * edgeEntryBytes + eName)
    fun setName(edge: Int, name: String) = edges.setInt(edge.toLong() * edgeEntryBytes + eName, names.put(name))
    fun name(edge: Int): String = names.get(nameRef(edge))

    // ---------------------------------------------------------------- geometria

    /** Memorizza i punti intermedi, dati nell'ordine nodeA→nodeB. */
    fun setWayGeometry(edge: Int, pillars: PointList) {
        val dim = if (is3D) 3 else 2
        val p = edge.toLong() * edgeEntryBytes + eGeo
        if (pillars.size == 0) { edges.setInt(p, 0); return }
        val existing = edges.getInt(p).toLong() and 0xFFFFFFFFL
        val ref: Long
        if (existing != 0L && geometry.getInt(existing * 4) >= pillars.size) {
            ref = existing // riscrittura in loco
        } else {
            ref = nextGeoRef
            nextGeoRef += 1 + pillars.size.toLong() * dim
            geometry.ensureCapacity(nextGeoRef * 4)
        }
        var pos = ref * 4
        geometry.setInt(pos, pillars.size); pos += 4
        for (i in 0 until pillars.size) {
            geometry.setInt(pos, degToInt(pillars.lat(i))); pos += 4
            geometry.setInt(pos, degToInt(pillars.lon(i))); pos += 4
            if (is3D) { geometry.setInt(pos, eleToInt(pillars.ele(i))); pos += 4 }
        }
        edges.setInt(p, ref.toInt())
    }

    /** Geometria dell'arco nella direzione base→adj data da [reverse]. */
    fun fetchWayGeometry(edge: Int, reverse: Boolean, mode: FetchMode): PointList {
        val ref = edges.getInt(edge.toLong() * edgeEntryBytes + eGeo).toLong() and 0xFFFFFFFFL
        val count = if (ref == 0L) 0 else geometry.getInt(ref * 4)
        val withBase = mode == FetchMode.ALL || mode == FetchMode.BASE_AND_PILLAR || mode == FetchMode.TOWER_ONLY
        val withAdj = mode == FetchMode.ALL || mode == FetchMode.PILLAR_AND_ADJ || mode == FetchMode.TOWER_ONLY
        val withPillars = mode != FetchMode.TOWER_ONLY
        val a = nodeA(edge); val b = nodeB(edge)
        val pl = PointList(count + 2, is3D)
        // costruiamo sempre in ordine di memorizzazione (A→B) e poi invertiamo se serve
        val first = if (reverse) withAdj else withBase
        val last = if (reverse) withBase else withAdj
        if (first) pl.add(lat(a), lon(a), ele(a))
        if (withPillars && count > 0) {
            var pos = ref * 4 + 4
            for (i in 0 until count) {
                val la = intToDeg(geometry.getInt(pos)); pos += 4
                val lo = intToDeg(geometry.getInt(pos)); pos += 4
                val el = if (is3D) intToEle(geometry.getInt(pos)).also { pos += 4 } else Double.NaN
                pl.add(la, lo, el)
            }
        }
        if (last) pl.add(lat(b), lon(b), ele(b))
        if (reverse) pl.reverse()
        return pl
    }

    // ---------------------------------------------------------------- iterazione

    /** Stato di un arco visto da un nodo base; riutilizzabile, nessuna allocazione per arco. */
    inner class EdgeIterator {
        var edge = -1; private set
        var baseNode = -1; private set
        var adjNode = -1; private set
        /** Vero se percorriamo l'arco da nodeB a nodeA (direzione opposta a quella memorizzata). */
        var reverse = false; private set
        private var next = NO_EDGE
        private val flagsBuf = IntArray(intsForFlags)

        fun setBaseNode(node: Int): EdgeIterator {
            baseNode = node; next = edgeRef(node); edge = -1; adjNode = -1
            return this
        }

        fun next(): Boolean {
            while (next != NO_EDGE) {
                edge = next
                val p = edge.toLong() * edgeEntryBytes
                val a = edges.getInt(p); val b = edges.getInt(p + 4)
                if (a == baseNode) { adjNode = b; reverse = false; next = edges.getInt(p + 8) }
                else { adjNode = a; reverse = true; next = edges.getInt(p + 12) }
                return true
            }
            return false
        }

        /** Posiziona l'iteratore su un arco preciso visto da [base]; usato per ricostruire percorsi. */
        fun setEdge(edgeId: Int, base: Int): EdgeIterator {
            edge = edgeId; baseNode = base
            val a = nodeA(edgeId); val b = nodeB(edgeId)
            require(base == a || base == b) { "il nodo $base non appartiene all'arco $edgeId" }
            if (base == a) { adjNode = b; reverse = false } else { adjNode = a; reverse = true }
            next = NO_EDGE
            return this
        }

        val edgeKey: Int get() = (edge shl 1) or (if (reverse) 1 else 0)
        val distance: Double get() = this@BaseGraph.distance(edge)
        val flags: IntArray get() = getFlags(edge, flagsBuf)
        fun setFlags(f: IntArray) = this@BaseGraph.setFlags(edge, f)
        val name: String get() = this@BaseGraph.name(edge)

        fun get(ev: BooleanEncodedValue) = ev.get(flags, reverse)
        fun getReverse(ev: BooleanEncodedValue) = ev.get(flags, !reverse)
        fun get(ev: DecimalEncodedValue) = ev.get(flags, reverse)
        fun getReverse(ev: DecimalEncodedValue) = ev.get(flags, !reverse)
        fun get(ev: IntEncodedValue) = ev.get(flags, reverse)
        fun <E : Enum<E>> get(ev: EnumEncodedValue<E>): E = ev.get(flags, reverse)

        fun fetchWayGeometry(mode: FetchMode) = this@BaseGraph.fetchWayGeometry(edge, reverse, mode)
    }

    fun createEdgeIterator() = EdgeIterator()

    // ---------------------------------------------------------------- persistenza

    fun freeze() { frozen = true }

    fun flush() {
        nodes.setHeader(0, GRAPH_FORMAT_VERSION); nodes.setHeader(1, nodeEntryBytes); nodes.setHeader(2, nodeCount)
        nodes.setHeader(3, degToInt(bounds.minLon)); nodes.setHeader(4, degToInt(bounds.maxLon))
        nodes.setHeader(5, degToInt(bounds.minLat)); nodes.setHeader(6, degToInt(bounds.maxLat))
        nodes.setHeader(7, if (frozen) 1 else 0); nodes.setHeader(8, if (is3D) 1 else 0)
        edges.setHeader(0, GRAPH_FORMAT_VERSION); edges.setHeader(1, edgeEntryBytes); edges.setHeader(2, edgeCount)
        geometry.setHeader(0, GRAPH_FORMAT_VERSION)
        geometry.setHeader(1, nextGeoRef.toInt()); geometry.setHeader(2, (nextGeoRef ushr 32).toInt())
        nodes.flush(); edges.flush(); geometry.flush(); names.flush()
        GraphProperties(File(dir.dir, "properties")).apply {
            set("graph.format_version", GRAPH_FORMAT_VERSION.toString())
            set("graph.encoded_values", encodedValues.describe())
            set("graph.3d", is3D.toString())
            save()
        }
    }

    /** Carica un grafo esistente dalla cartella; false se non c'è. */
    fun loadExisting(): Boolean {
        val n = dir.open("nodes", NODE_SEGMENT) ?: return false
        val e = dir.open("edges", EDGE_SEGMENT) ?: return false
        val g = dir.open("geometry") ?: return false
        val props = GraphProperties(File(dir.dir, "properties")).also { it.load() }
        check(props.get("graph.format_version") == GRAPH_FORMAT_VERSION.toString()) { "formato grafo incompatibile" }
        check(props.get("graph.encoded_values") == encodedValues.describe()) { "valori codificati diversi da quelli del grafo" }
        check(n.getHeader(1) == nodeEntryBytes && e.getHeader(1) == edgeEntryBytes) { "layout record diverso" }
        nodeCount = n.getHeader(2)
        bounds.minLon = intToDeg(n.getHeader(3)); bounds.maxLon = intToDeg(n.getHeader(4))
        bounds.minLat = intToDeg(n.getHeader(5)); bounds.maxLat = intToDeg(n.getHeader(6))
        frozen = n.getHeader(7) == 1
        edgeCount = e.getHeader(2)
        nextGeoRef = (g.getHeader(1).toLong() and 0xFFFFFFFFL) or (g.getHeader(2).toLong() shl 32)
        // sostituisce i DataAccess vuoti creati nel costruttore con quelli caricati
        nodes.close(); edges.close(); geometry.close()
        nodes = n; edges = e; geometry = g
        names.loadExisting(dir)
        return true
    }

    override fun close() { nodes.close(); edges.close(); geometry.close(); names.close() }

    companion object {
        const val NO_EDGE = -1
        private const val NODE_SEGMENT = 1 shl 16 // 65536 è multiplo di 12 e 16
        private const val EDGE_SEGMENT = 1 shl 20
        private const val MAX_DIST_MM = Int.MAX_VALUE.toLong() - 1
        private const val DEG_FACTOR = 1e7

        fun degToInt(deg: Double): Int = (deg * DEG_FACTOR).toInt()
        fun intToDeg(i: Int): Double = i / DEG_FACTOR
        fun eleToInt(ele: Double): Int = if (ele.isNaN()) 0 else (ele * 1000).toInt()
        fun intToEle(i: Int): Double = i / 1000.0
    }
}
