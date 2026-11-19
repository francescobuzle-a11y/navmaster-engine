package it.navmaster.engine.storage

import it.navmaster.engine.isReadableGraphFormat
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
    private var nextGeoRef = 4L // in byte (formato 5) o in int (formato 4); 0 riservato a "nessuna geometria"
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
        // la geometria degli archi è salvata a scarti dal nodo A: spostare un nodo dopo la sposterebbe
        check(nextGeoRef == 4L) { "nodo spostato dopo la scrittura della geometria" }
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
        flagWrites++
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
        flagWrites++
    }

    /** Scritture dei flag (import): invalidano i flag che gli iteratori ricordano ([EdgeIterator.flags]). */
    private var flagWrites = 0

    fun nameRef(edge: Int): Int = edges.getInt(edge.toLong() * edgeEntryBytes + eName)
    fun setName(edge: Int, name: String) = edges.setInt(edge.toLong() * edgeEntryBytes + eName, names.put(name))
    fun name(edge: Int): String = names.get(nameRef(edge))

    // ---------------------------------------------------------------- geometria
    //
    // Formato 5 ([geoVarint]): a ogni arco con punti intermedi un record al byte geoRef del file
    // `geometry`: varint del numero di punti, poi per ogni punto gli scarti zigzag-varint di lat, lon
    // (ed ele) in unità di 1e-7 gradi (mm), dal punto precedente; il primo dal nodo A. Valori identici
    // al bit al formato 4, che salvava ogni coordinata come int assoluto (8 byte per punto; in Germania
    // 366 MB contro circa 190). Il record si legge con una copia in blocco (una chiamata nativa sul
    // tablet invece di 1 + 2 per punto). Formato 4 (ancora letto): geoRef in int, count, poi gli int.

    /** Vero per la geometria a scarti varint (formato 5); falso per i grafi di formato 4. */
    private var geoVarint = true

    /** Memorizza i punti intermedi, dati nell'ordine nodeA→nodeB. */
    fun setWayGeometry(edge: Int, pillars: PointList) {
        val p = edge.toLong() * edgeEntryBytes + eGeo
        if (pillars.size == 0) { edges.setInt(p, 0); return }
        check(geoVarint) { "geometria di un grafo di formato 4: sola lettura" }
        val buf = GeoCodec.encode(pillars, nodes.getInt(nodeA(edge).toLong() * nodeEntryBytes + 4), nodes.getInt(nodeA(edge).toLong() * nodeEntryBytes + 8),
            if (is3D) nodes.getInt(nodeA(edge).toLong() * nodeEntryBytes + 12) else 0, is3D)
        // sempre in coda (l'import scrive la geometria di ogni arco una volta sola)
        val ref = nextGeoRef
        require(ref + buf.size <= 0xFFFFFFFFL) { "geometria oltre 4 GB" }
        nextGeoRef += buf.size
        geometry.ensureCapacity(nextGeoRef)
        geometry.setBytes(ref, buf, buf.size)
        edges.setInt(p, ref.toInt())
    }

    /** Geometria dell'arco nella direzione base→adj data da [reverse]. */
    fun fetchWayGeometry(edge: Int, reverse: Boolean, mode: FetchMode): PointList {
        val ref = edges.getInt(edge.toLong() * edgeEntryBytes + eGeo).toLong() and 0xFFFFFFFFL
        val withBase = mode == FetchMode.ALL || mode == FetchMode.BASE_AND_PILLAR || mode == FetchMode.TOWER_ONLY
        val withAdj = mode == FetchMode.ALL || mode == FetchMode.PILLAR_AND_ADJ || mode == FetchMode.TOWER_ONLY
        val withPillars = mode != FetchMode.TOWER_ONLY
        val a = nodeA(edge); val b = nodeB(edge)
        if (!geoVarint) return fetchLegacyGeometry(ref, a, b, reverse, withBase, withAdj, withPillars)
        // costruiamo sempre in ordine di memorizzazione (A→B) e poi invertiamo se serve
        val first = if (reverse) withAdj else withBase
        val last = if (reverse) withBase else withAdj
        if (ref == 0L || !withPillars) {
            val pl = PointList(2, is3D)
            if (first) pl.add(lat(a), lon(a), ele(a))
            if (last) pl.add(lat(b), lon(b), ele(b))
            if (reverse) pl.reverse()
            return pl
        }
        val pa = a.toLong() * nodeEntryBytes
        val aLat = nodes.getInt(pa + 4); val aLon = nodes.getInt(pa + 8); val aEle = if (is3D) nodes.getInt(pa + 12) else 0
        val avail = (nextGeoRef - ref).toInt()
        val tl = geoBuf.get()
        // prima un pezzo piccolo (il numero dei punti), poi, se serve, tutto il record
        var buf = tl
        var have = minOf(avail, GEO_FIRST_READ)
        geometry.getBytes(ref, buf, have)
        val count = GeoCodec.count(buf)
        val need = minOf(avail, GeoCodec.maxBytes(count, is3D))
        if (need > have) {
            if (buf.size < need) { buf = ByteArray(maxOf(need, buf.size * 2)); if (buf.size <= GEO_BUF_KEEP) geoBuf.set(buf) }
            geometry.getBytes(ref, buf, need)
            have = need
        }
        val pl = PointList(count + 2, is3D)
        if (first) pl.add(intToDeg(aLat), intToDeg(aLon), if (is3D) intToEle(aEle) else Double.NaN)
        GeoCodec.decodeInto(buf, have, aLat, aLon, aEle, is3D, pl)
        if (last) pl.add(lat(b), lon(b), ele(b))
        if (reverse) pl.reverse()
        return pl
    }

    /** La geometria nel formato 4: count e int assoluti a geoRef·4. */
    private fun fetchLegacyGeometry(ref: Long, a: Int, b: Int, reverse: Boolean, withBase: Boolean, withAdj: Boolean, withPillars: Boolean): PointList {
        val count = if (ref == 0L) 0 else geometry.getInt(ref * 4)
        val pl = PointList(count + 2, is3D)
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

    /** Il buffer di lettura della geometria, uno per thread (le ricerche e l'aggancio girano in parallelo). */
    private val geoBuf = ThreadLocal.withInitial { ByteArray(GEO_FIRST_READ.coerceAtLeast(1024)) }

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
        /**
         * L'arco di cui [flagsBuf] contiene i flag (e a quale scrittura del grafo): ogni valore chiesto
         * ([get]) rileggeva tutti gli interi dei flag, una chiamata nativa ciascuno sul tablet; peso e
         * filtri del camion ne chiedono una quindicina per arco (5 interi: ~75 letture). Ora una volta per arco.
         */
        private var flagsEdge = -1
        private var flagsAt = -1

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
        val flags: IntArray get() {
            if (flagsEdge != edge || flagsAt != flagWrites) { getFlags(edge, flagsBuf); flagsEdge = edge; flagsAt = flagWrites }
            return flagsBuf
        }
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
        geometry.setHeader(3, if (geoVarint) GEO_CODEC_VARINT else 0)
        nodes.flush(); edges.flush(); geometry.flush(); names.flush()
        GraphProperties(File(dir.dir, "properties")).apply {
            set("graph.format_version", GRAPH_FORMAT_VERSION.toString() + (if (formatVariant.isEmpty()) "" else ".$formatVariant"))
            set("graph.encoded_values", encodedValues.describe())
            set("graph.3d", is3D.toString())
            save()
        }
    }

    /**
     * La variante del formato scritta in `properties` dopo la versione (`4.navmaster`), vuota per i grafi
     * di sempre: un grafo che va letto in un modo che i motori di prima non conoscono (CH, core e
     * landmark preparati col peso consigliato NAVMASTER, vedi PriorityModel) la dichiara, e quei motori,
     * che vogliono la versione esatta, lo rifiutano invece di usarlo male.
     */
    var formatVariant: String = ""

    /** Carica un grafo esistente dalla cartella; false se non c'è. */
    fun loadExisting(): Boolean {
        val n = dir.open("nodes", NODE_SEGMENT) ?: return false
        val e = dir.open("edges", EDGE_SEGMENT) ?: return false
        val g = dir.open("geometry") ?: return false
        val props = GraphProperties(File(dir.dir, "properties")).also { it.load() }
        val format = props.get("graph.format_version").orEmpty()
        check(format.substringBefore('.').toIntOrNull()?.let(::isReadableGraphFormat) == true) { "formato grafo incompatibile: $format" }
        formatVariant = format.substringAfter('.', "")
        check(props.get("graph.encoded_values") == encodedValues.describe()) { "valori codificati diversi da quelli del grafo" }
        check(n.getHeader(1) == nodeEntryBytes && e.getHeader(1) == edgeEntryBytes) { "layout record diverso" }
        nodeCount = n.getHeader(2)
        bounds.minLon = intToDeg(n.getHeader(3)); bounds.maxLon = intToDeg(n.getHeader(4))
        bounds.minLat = intToDeg(n.getHeader(5)); bounds.maxLat = intToDeg(n.getHeader(6))
        frozen = n.getHeader(7) == 1
        edgeCount = e.getHeader(2)
        nextGeoRef = (g.getHeader(1).toLong() and 0xFFFFFFFFL) or (g.getHeader(2).toLong() shl 32)
        geoVarint = g.getHeader(3) == GEO_CODEC_VARINT
        // sostituisce i DataAccess vuoti creati nel costruttore con quelli caricati
        nodes.close(); edges.close(); geometry.close()
        nodes = n; edges = e; geometry = g
        names.loadExisting(dir)
        return true
    }

    /** Riscaldamento (vedi [DataAccess.touch]) dei file scelti. Byte toccati. */
    fun touch(withNodes: Boolean, withEdges: Boolean, withGeometry: Boolean, progress: ((Long) -> Unit)? = null): Long {
        var done = 0L
        fun one(da: DataAccess) { done += da.touch(progress?.let { p -> val base = done; { b: Long -> p(base + b) } }) }
        if (withNodes) one(nodes)
        if (withEdges) one(edges)
        if (withGeometry) one(geometry)
        return done
    }
    /** I byte che [touch] leggerebbe. */
    fun bytes(withNodes: Boolean, withEdges: Boolean, withGeometry: Boolean): Long =
        (if (withNodes) nodes.touchBytes else 0) + (if (withEdges) edges.touchBytes else 0) + (if (withGeometry) geometry.touchBytes else 0)

    override fun close() { nodes.close(); edges.close(); geometry.close(); names.close() }

    companion object {
        const val NO_EDGE = -1
        private const val NODE_SEGMENT = 1 shl 16 // 65536 è multiplo di 12 e 16
        private const val EDGE_SEGMENT = 1 shl 20
        private const val MAX_DIST_MM = Int.MAX_VALUE.toLong() - 1
        private const val DEG_FACTOR = 1e7
        /** L'intestazione [3] del file `geometry` con la geometria a scarti varint (formato 5); 0 nel formato 4. */
        const val GEO_CODEC_VARINT = 1
        /** I byte letti per primi di un record di geometria: il numero dei punti e, di solito, tutto il record. */
        private const val GEO_FIRST_READ = 96
        /** Il buffer di lettura più grande tenuto per thread (oltre si alloca per la chiamata). */
        private const val GEO_BUF_KEEP = 1 shl 16

        fun degToInt(deg: Double): Int = (deg * DEG_FACTOR).toInt()
        fun intToDeg(i: Int): Double = i / DEG_FACTOR
        fun eleToInt(ele: Double): Int = if (ele.isNaN()) 0 else (ele * 1000).toInt()
        fun intToEle(i: Int): Double = i / 1000.0
    }
}

/**
 * Codifica della geometria di un arco (formato 5): varint del numero di punti, poi per ogni punto gli
 * scarti zigzag-varint (a 64 bit) di lat, lon ed ele (se 3D) dal punto precedente, il primo dal nodo A;
 * coordinate in 1e-7 gradi ([BaseGraph.degToInt]) ed ele in mm, gli stessi interi del formato 4.
 */
internal object GeoCodec {
    fun encode(pillars: PointList, aLat: Int, aLon: Int, aEle: Int, is3D: Boolean): ByteArray {
        val out = java.io.ByteArrayOutputStream(5 + pillars.size * (if (is3D) 9 else 6))
        writeVar(out, pillars.size.toLong())
        var pLat = aLat.toLong(); var pLon = aLon.toLong(); var pEle = aEle.toLong()
        for (i in 0 until pillars.size) {
            val la = BaseGraph.degToInt(pillars.lat(i)).toLong()
            val lo = BaseGraph.degToInt(pillars.lon(i)).toLong()
            writeVar(out, zig(la - pLat)); writeVar(out, zig(lo - pLon))
            pLat = la; pLon = lo
            if (is3D) { val el = BaseGraph.eleToInt(pillars.ele(i)).toLong(); writeVar(out, zig(el - pEle)); pEle = el }
        }
        return out.toByteArray()
    }

    /** Il numero dei punti del record che inizia in [buf]. */
    fun count(buf: ByteArray): Int {
        var r = 0L; var s = 0; var i = 0
        while (true) { val b = buf[i++].toInt(); r = r or ((b and 0x7F).toLong() shl s); if (b and 0x80 == 0) break; s += 7 }
        return r.toInt()
    }

    /** I byte massimi di un record di [count] punti. */
    fun maxBytes(count: Int, is3D: Boolean): Int = 5 + count * (if (is3D) 3 else 2) * 10

    /** Aggiunge a [pl] i punti del record in [buf] (i primi [len] byte), dal nodo A di coordinate intere date. */
    fun decodeInto(buf: ByteArray, len: Int, aLat: Int, aLon: Int, aEle: Int, is3D: Boolean, pl: PointList) {
        var i = 0
        fun next(): Long {
            var r = 0L; var s = 0
            while (true) {
                check(i < len) { "record di geometria troncato" }
                val b = buf[i++].toInt()
                r = r or ((b and 0x7F).toLong() shl s)
                if (b and 0x80 == 0) return r
                s += 7
            }
        }
        val n = next().toInt()
        var la = aLat.toLong(); var lo = aLon.toLong(); var el = aEle.toLong()
        for (k in 0 until n) {
            la += unzig(next()); lo += unzig(next())
            if (is3D) el += unzig(next())
            pl.add(BaseGraph.intToDeg(la.toInt()), BaseGraph.intToDeg(lo.toInt()), if (is3D) BaseGraph.intToEle(el.toInt()) else Double.NaN)
        }
    }

    private fun zig(d: Long): Long = (d shl 1) xor (d shr 63)
    private fun unzig(z: Long): Long = (z ushr 1) xor -(z and 1)
    private fun writeVar(out: java.io.ByteArrayOutputStream, v: Long) {
        var x = v
        while (x and 0x7FL.inv() != 0L) { out.write(((x and 0x7F) or 0x80).toInt()); x = x ushr 7 }
        out.write(x.toInt())
    }
}
