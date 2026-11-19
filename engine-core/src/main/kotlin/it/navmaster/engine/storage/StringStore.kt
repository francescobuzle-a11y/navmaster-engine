package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable
import java.io.File
import java.util.Properties

/**
 * Deposito di stringhe (nomi delle strade e stringhe di guida) con de-duplicazione.
 * Record: u16 lunghezza in byte UTF-8 + byte. Puntatore 0 = stringa vuota. I puntatori partono da 1.
 *
 * Ogni stringa è scritta una volta sola: durante l'import una mappa di tutte le stringhe già scritte
 * dà il puntatore di quelle ripetute (fino al formato 4 si ricordavano solo le ultime 1000, e in
 * Germania 3,2 M record erano 0,65 M stringhe diverse: 56 MB invece di 14). Il formato del file e i
 * lettori non cambiano; i puntatori di stringhe uguali ora coincidono.
 */
class StringStore(private var da: DataAccess) : Closeable {
    private var bytePointer = 1L
    /** Le stringhe scritte e i loro puntatori (solo in scrittura: un grafo caricato non scrive). */
    private val written = HashMap<String, Int>()

    fun put(value: String): Int {
        if (value.isEmpty()) return 0
        written[value]?.let { return it }
        var bytes = value.toByteArray(Charsets.UTF_8)
        if (bytes.size > MAX_BYTES) bytes = bytes.copyOf(MAX_BYTES)
        var ptr = bytePointer
        // l'intestazione (2 byte) non deve cavalcare il confine fra due segmenti del DataAccess
        if ((ptr and (SEGMENT - 1)) == SEGMENT - 1) ptr++
        require(ptr + 2 + bytes.size < Int.MAX_VALUE) { "troppi nomi: deposito pieno" }
        da.ensureCapacity(ptr + 2 + bytes.size)
        da.setShort(ptr, bytes.size.toShort())
        da.setBytes(ptr + 2, bytes, bytes.size)
        bytePointer = ptr + 2 + bytes.size
        written[value] = ptr.toInt()
        return ptr.toInt()
    }

    fun get(ptr: Int): String {
        if (ptr == 0) return ""
        val len = da.getShort(ptr.toLong()).toInt() and 0xFFFF
        val bytes = ByteArray(len)
        da.getBytes(ptr.toLong() + 2, bytes, len)
        return String(bytes, Charsets.UTF_8)
    }

    fun flush() {
        da.setHeader(0, GRAPH_FORMAT_VERSION)
        da.setHeader(1, bytePointer.toInt()); da.setHeader(2, (bytePointer ushr 32).toInt())
        da.flush()
    }

    fun loadExisting(dir: GraphDirectory): Boolean {
        val loaded = dir.open("names") ?: return false
        da.close()
        da = loaded
        bytePointer = (loaded.getHeader(1).toLong() and 0xFFFFFFFFL) or (loaded.getHeader(2).toLong() shl 32)
        return true
    }

    override fun close() = da.close()

    companion object {
        const val MAX_BYTES = 65535
        private const val SEGMENT = DataAccess.DEFAULT_SEGMENT_SIZE.toLong()
    }
}

/** File `properties` del grafo: coppie chiave=valore in testo. */
class GraphProperties(private val file: File) {
    private val props = Properties()
    fun get(key: String): String? = props.getProperty(key)
    fun set(key: String, value: String) { props.setProperty(key, value) }
    fun load(): Boolean {
        if (!file.exists()) return false
        file.inputStream().use { props.load(it) }
        return true
    }
    fun save() {
        file.parentFile?.mkdirs()
        file.outputStream().use { props.store(it, "navmaster-engine graph") }
    }
}
