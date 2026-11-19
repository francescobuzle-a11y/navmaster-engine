package it.navmaster.engine.storage

import java.io.Closeable
import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel

/**
 * Array di byte indirizzabile a 64 bit, diviso in segmenti di dimensione fissa (potenza di 2),
 * persistito su un file. Tutto è little-endian, intestazione compresa.
 *
 * Formato file: [0..3] magic "NME1" | [4..11] long capacità dati | [12..15] int dimensione segmento |
 * [16..95] 20 int di intestazione dell'utilizzatore | [96..99] riservato | [100..] segmenti.
 *
 * Un `int` non può stare a cavallo di due segmenti: chi scrive record li allinea a 4 byte e
 * sceglie segmenti multipli della dimensione del record.
 */
abstract class DataAccess(val name: String, segmentSize: Int) : Closeable {
    protected val segmentSizePower: Int
    val segmentSize: Int
    private val segmentMask: Int
    protected val header = IntArray(HEADER_INTS)

    init {
        require(segmentSize >= MIN_SEGMENT_SIZE) { "segmento troppo piccolo: $segmentSize" }
        segmentSizePower = 31 - Integer.numberOfLeadingZeros(segmentSize)
        this.segmentSize = 1 shl segmentSizePower
        segmentMask = this.segmentSize - 1
    }

    /** Capacità attuale in byte (multiplo della dimensione del segmento). */
    abstract val capacity: Long
    protected abstract fun segments(): Int

    /**
     * Carica il file nella cache del sistema (riscaldamento: sul tablet la prima ricerca dopo
     * l'avvio legge i file a caso e costa 6 volte quelle dopo). Byte toccati; 0 quando è in RAM.
     */
    open fun touch(progress: ((Long) -> Unit)? = null): Long = 0
    /** I byte che [touch] leggerebbe: 0 in RAM. */
    open val touchBytes: Long get() = 0
    protected abstract fun addSegments(count: Int)
    protected abstract fun segment(index: Int): ByteBuffer

    /** Garantisce almeno [bytes] byte; cresce di interi segmenti. */
    fun ensureCapacity(bytes: Long) {
        if (bytes <= capacity) return
        val needed = ((bytes + segmentMask) ushr segmentSizePower).toInt() - segments()
        if (needed > 0) addSegments(needed)
    }

    fun getHeader(index: Int): Int = header[index]
    fun setHeader(index: Int, value: Int) { header[index] = value }

    fun getInt(pos: Long): Int {
        val seg = (pos ushr segmentSizePower).toInt()
        val off = (pos and segmentMask.toLong()).toInt()
        return segment(seg).getInt(off)
    }

    fun setInt(pos: Long, value: Int) {
        val seg = (pos ushr segmentSizePower).toInt()
        val off = (pos and segmentMask.toLong()).toInt()
        segment(seg).putInt(off, value)
    }

    fun getShort(pos: Long): Short {
        val seg = (pos ushr segmentSizePower).toInt()
        val off = (pos and segmentMask.toLong()).toInt()
        return segment(seg).getShort(off)
    }

    fun setShort(pos: Long, value: Short) {
        val seg = (pos ushr segmentSizePower).toInt()
        val off = (pos and segmentMask.toLong()).toInt()
        segment(seg).putShort(off, value)
    }

    fun getByte(pos: Long): Byte = segment((pos ushr segmentSizePower).toInt()).get((pos and segmentMask.toLong()).toInt())
    fun setByte(pos: Long, value: Byte) { segment((pos ushr segmentSizePower).toInt()).put((pos and segmentMask.toLong()).toInt(), value) }

    /**
     * [count] ints from [pos] into [dst] at once (a block copy: on Android a read of a mapped
     * buffer one int at a time is a native call each).
     */
    fun getInts(pos: Long, dst: IntArray, dstOff: Int, count: Int) {
        var p = pos; var o = dstOff; var left = count
        while (left > 0) {
            val seg = (p ushr segmentSizePower).toInt()
            val off = (p and segmentMask.toLong()).toInt()
            val n = minOf(left, (segmentSize - off) / 4)
            val bb = segment(seg).duplicate().order(ByteOrder.LITTLE_ENDIAN)
            bb.position(off)
            bb.asIntBuffer().get(dst, o, n)
            p += n * 4L; o += n; left -= n
        }
    }

    fun getLong(pos: Long): Long = (getInt(pos).toLong() and 0xFFFFFFFFL) or (getInt(pos + 4).toLong() shl 32)
    fun setLong(pos: Long, value: Long) { setInt(pos, value.toInt()); setInt(pos + 4, (value ushr 32).toInt()) }

    /**
     * Copia [length] byte da [pos] in [dst] (da [dstOff]), attraversando quanti confini di segmento servono.
     * Una copia in blocco per segmento (`duplicate().position().get(dst)`, accessi relativi su una copia
     * del buffer, compatibili con Android): sul tablet ogni lettura singola di un buffer mappato è una
     * chiamata nativa, e la copia byte per byte di prima ne faceva una per byte.
     */
    fun getBytes(pos: Long, dst: ByteArray, length: Int, dstOff: Int = 0) {
        var p = pos; var o = dstOff; var left = length
        while (left > 0) {
            val seg = (p ushr segmentSizePower).toInt()
            val off = (p and segmentMask.toLong()).toInt()
            val n = minOf(left, segmentSize - off)
            val bb = segment(seg).duplicate()
            bb.position(off)
            bb.get(dst, o, n)
            p += n; o += n; left -= n
        }
    }

    /** Scrive [length] byte di [src] (da [srcOff]) a [pos], una copia in blocco per segmento attraversato. */
    fun setBytes(pos: Long, src: ByteArray, length: Int, srcOff: Int = 0) {
        var p = pos; var o = srcOff; var left = length
        while (left > 0) {
            val seg = (p ushr segmentSizePower).toInt()
            val off = (p and segmentMask.toLong()).toInt()
            val n = minOf(left, segmentSize - off)
            val bb = segment(seg).duplicate()
            bb.position(off)
            bb.put(src, o, n)
            p += n; o += n; left -= n
        }
    }

    abstract fun flush()

    companion object {
        const val HEADER_OFFSET = 100L
        const val HEADER_INTS = 20
        const val MIN_SEGMENT_SIZE = 1 shl 7
        const val DEFAULT_SEGMENT_SIZE = 1 shl 20
        private val MAGIC = byteArrayOf('N'.code.toByte(), 'M'.code.toByte(), 'E'.code.toByte(), '1'.code.toByte())

        internal fun writeFileHeader(raf: RandomAccessFile, capacity: Long, segmentSize: Int, header: IntArray) {
            val buf = ByteBuffer.allocate(HEADER_OFFSET.toInt()).order(ByteOrder.LITTLE_ENDIAN)
            buf.put(MAGIC).putLong(capacity).putInt(segmentSize)
            for (h in header) buf.putInt(h)
            raf.seek(0)
            raf.write(buf.array())
        }

        /** Legge l'intestazione; ritorna (capacità, segmentSize) o null se il file non è nostro. */
        internal fun readFileHeader(raf: RandomAccessFile, header: IntArray): Pair<Long, Int>? {
            if (raf.length() < HEADER_OFFSET) return null
            val bytes = ByteArray(HEADER_OFFSET.toInt())
            raf.seek(0)
            raf.readFully(bytes)
            val buf = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
            val magic = ByteArray(4).also { buf.get(it) }
            if (!magic.contentEquals(MAGIC)) return null
            val capacity = buf.getLong()
            val segmentSize = buf.getInt()
            for (i in header.indices) header[i] = buf.getInt()
            return capacity to segmentSize
        }
    }
}

/** DataAccess in memoria (heap) con persistenza opzionale su file. */
class RamDataAccess(name: String, private val file: File?, segmentSize: Int = DataAccess.DEFAULT_SEGMENT_SIZE) : DataAccess(name, segmentSize) {
    private var segs: Array<ByteBuffer> = emptyArray()

    override val capacity: Long get() = segs.size.toLong() * segmentSize
    override fun segments() = segs.size
    override fun segment(index: Int): ByteBuffer = segs[index]

    override fun addSegments(count: Int) {
        segs = Array(segs.size + count) { i -> if (i < segs.size) segs[i] else ByteBuffer.allocate(segmentSize).order(ByteOrder.LITTLE_ENDIAN) }
    }

    /** Carica dal file; false se non esiste. */
    fun loadExisting(): Boolean {
        val f = file ?: return false
        if (!f.exists() || f.length() == 0L) return false
        RandomAccessFile(f, "r").use { raf ->
            val (cap, segSize) = readFileHeader(raf, header) ?: error("file non riconosciuto: $f")
            require(segSize == segmentSize) { "dimensione segmento diversa in $f: $segSize vs $segmentSize" }
            val n = (cap ushr segmentSizePower).toInt()
            segs = Array(n) { ByteBuffer.allocate(segmentSize).order(ByteOrder.LITTLE_ENDIAN) }
            raf.seek(HEADER_OFFSET)
            for (s in segs) raf.readFully(s.array())
        }
        return true
    }

    override fun flush() {
        val f = file ?: return
        f.parentFile?.mkdirs()
        RandomAccessFile(f, "rw").use { raf ->
            raf.setLength(HEADER_OFFSET + capacity)
            writeFileHeader(raf, capacity, segmentSize, header)
            raf.seek(HEADER_OFFSET)
            for (s in segs) raf.write(s.array())
        }
    }

    override fun close() { segs = emptyArray() }
}

/**
 * DataAccess memory-mapped: i segmenti sono finestre del file, l'OS fa da cache.
 *
 * Un file esistente si mappa a pezzi di [CHUNK_BYTES] e i segmenti sono ritagli di quei pezzi: una
 * mappatura ogni 64 MB invece che una per segmento. Sul tablet sei Paesi facevano decine di migliaia
 * di mappature (i nodi a segmenti di 64 KB) e il processo arrivava al limite del sistema: l'app si
 * chiudeva in una raccolta della memoria (mprotect: out of memory) o in un'allocazione nativa
 * (std::bad_alloc) mentre agganciava i valichi. I segmenti aggiunti scrivendo (la costruzione del
 * grafo) restano mappati uno per uno.
 */
class MmapDataAccess(name: String, private val file: File, segmentSize: Int = DataAccess.DEFAULT_SEGMENT_SIZE, private val readOnly: Boolean = false,
                     internal val chunkBytes: Int = CHUNK_BYTES) :
    DataAccess(name, segmentSize) {
    private var raf: RandomAccessFile? = null
    private var segs: Array<ByteBuffer> = emptyArray()
    /** Le mappature vere, in ordine di file (per [touch] e [flush]); ciascuna coi suoi byte. */
    private var maps: MutableList<MappedByteBuffer> = ArrayList()

    override val capacity: Long get() = segs.size.toLong() * segmentSize
    override fun segments() = segs.size
    override fun segment(index: Int): ByteBuffer = segs[index]

    private fun openFile(): RandomAccessFile = raf ?: run {
        file.parentFile?.mkdirs()
        RandomAccessFile(file, if (readOnly) "r" else "rw").also { raf = it }
    }

    private val mode get() = if (readOnly) FileChannel.MapMode.READ_ONLY else FileChannel.MapMode.READ_WRITE

    override fun addSegments(count: Int) {
        check(!readOnly) { "$name è in sola lettura" }
        val r = openFile()
        val newTotal = segs.size + count
        val needed = HEADER_OFFSET + newTotal.toLong() * segmentSize
        if (r.length() < needed) r.setLength(needed)
        segs = Array(newTotal) { i -> if (i < segs.size) segs[i] else map(r, i) }
    }

    private fun map(r: RandomAccessFile, index: Int): ByteBuffer {
        val m = r.channel.map(mode, HEADER_OFFSET + index.toLong() * segmentSize, segmentSize.toLong()).order(ByteOrder.LITTLE_ENDIAN) as MappedByteBuffer
        maps += m
        return m
    }

    fun loadExisting(): Boolean {
        if (!file.exists() || file.length() == 0L) return false
        val r = openFile()
        val (cap, segSize) = readFileHeader(r, header) ?: error("file non riconosciuto: $file")
        require(segSize == segmentSize) { "dimensione segmento diversa in $file: $segSize vs $segmentSize" }
        val n = (cap ushr segmentSizePower).toInt()
        // whole segments per chunk (both powers of two)
        val perChunk = maxOf(1, chunkBytes / segmentSize)
        val chunks = ArrayList<MappedByteBuffer>()
        var first = 0
        while (first < n) {
            val count = minOf(perChunk, n - first)
            chunks += r.channel.map(mode, HEADER_OFFSET + first.toLong() * segmentSize, count.toLong() * segmentSize) as MappedByteBuffer
            first += count
        }
        maps = chunks.toMutableList()
        segs = Array(n) { i ->
            val c = chunks[i / perChunk].duplicate()
            val off = (i % perChunk) * segmentSize
            c.limit(off + segmentSize); c.position(off)
            c.slice().order(ByteOrder.LITTLE_ENDIAN)
        }
        return true
    }

    override fun flush() {
        if (readOnly) return
        val r = openFile()
        for (m in maps) m.force()
        writeFileHeader(r, capacity, segmentSize, header)
        r.channel.force(true)
    }

    override val touchBytes: Long get() = capacity

    override fun touch(progress: ((Long) -> Unit)?): Long {
        var done = 0L
        for (m in maps) { m.load(); done += m.capacity(); progress?.invoke(minOf(done, capacity)) }
        return capacity
    }

    override fun close() {
        segs = emptyArray()
        maps = ArrayList()
        raf?.close()
        raf = null
    }

    companion object {
        /** La dimensione di una mappatura di un file esistente (multiplo di ogni dimensione di segmento fino a 64 MB). */
        const val CHUNK_BYTES = 1 shl 26
    }
}

/** Cartella di un grafo: crea i DataAccess con il tipo scelto. */
class GraphDirectory(val dir: File, val mmap: Boolean = false, val readOnly: Boolean = false) {
    init { dir.mkdirs() }

    fun create(name: String, segmentSize: Int = DataAccess.DEFAULT_SEGMENT_SIZE): DataAccess {
        val f = File(dir, name)
        return if (mmap) MmapDataAccess(name, f, segmentSize, readOnly) else RamDataAccess(name, f, segmentSize)
    }

    /** Apre un DataAccess esistente; null se il file manca. */
    fun open(name: String, segmentSize: Int = DataAccess.DEFAULT_SEGMENT_SIZE): DataAccess? {
        val da = create(name, segmentSize)
        val ok = when (da) {
            is RamDataAccess -> da.loadExisting()
            is MmapDataAccess -> da.loadExisting()
            else -> false
        }
        return if (ok) da else null
    }
}
