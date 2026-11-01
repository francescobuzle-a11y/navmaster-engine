package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable
import java.io.File
import java.util.Properties

/**
 * Deposito di stringhe (nomi delle strade) con de-duplicazione.
 * Record: u16 lunghezza in byte UTF-8 + byte. Puntatore 0 = stringa vuota. I puntatori partono da 1.
 */
class StringStore(private var da: DataAccess) : Closeable {
    private var bytePointer = 1L
    private val recent = object : LinkedHashMap<String, Int>(2048, 0.75f, true) {
        override fun removeEldestEntry(eldest: MutableMap.MutableEntry<String, Int>?) = size > 1000
    }

    fun put(value: String): Int {
        if (value.isEmpty()) return 0
        recent[value]?.let { return it }
        var bytes = value.toByteArray(Charsets.UTF_8)
        if (bytes.size > MAX_BYTES) bytes = bytes.copyOf(MAX_BYTES)
        val ptr = bytePointer
        require(ptr + 2 + bytes.size < Int.MAX_VALUE) { "troppi nomi: deposito pieno" }
        da.ensureCapacity(ptr + 2 + bytes.size)
        da.setShort(ptr, bytes.size.toShort())
        da.setBytes(ptr + 2, bytes, bytes.size)
        bytePointer += 2 + bytes.size
        recent[value] = ptr.toInt()
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

    companion object { const val MAX_BYTES = 65535 }
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
