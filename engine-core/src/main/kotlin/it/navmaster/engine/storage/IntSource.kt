package it.navmaster.engine.storage

/**
 * Interi letti da un array nell'heap o direttamente da un file del grafo (mappato in memoria):
 * l'indice spaziale e i landmark di un Paese grande non vengono copiati nell'heap all'apertura
 * (Italia: circa 230 MB in meno e qualche secondo di apertura risparmiato sul tablet).
 */
abstract class IntSource {
    abstract val size: Int
    abstract operator fun get(i: Int): Int
    open fun close() {}
    /** Carica nella cache del sistema i byte letti (vedi [DataAccess.touch]); 0 in RAM. */
    open fun touch(progress: ((Long) -> Unit)? = null): Long = 0
    /** I byte che [touch] leggerebbe. */
    open val bytes: Long get() = 0

    /** [count] interi da [from] in [dst] (da [dstOff]) in un colpo solo: sul file mappato una copia sola invece di [count] chiamate native. */
    abstract fun readInto(from: Int, dst: IntArray, dstOff: Int, count: Int)

    class OfArray(private val a: IntArray) : IntSource() {
        override val size get() = a.size
        override fun get(i: Int) = a[i]
        override fun readInto(from: Int, dst: IntArray, dstOff: Int, count: Int) = System.arraycopy(a, from, dst, dstOff, count)
    }

    /** [size] interi dal byte [offset] di [da]. */
    class OfData(private val da: DataAccess, private val offset: Long, override val size: Int) : IntSource() {
        override fun get(i: Int) = da.getInt(offset + i.toLong() * 4)
        override fun close() { da.close() }
        override fun touch(progress: ((Long) -> Unit)?): Long = da.touch(progress)
        override val bytes: Long get() = da.touchBytes
        /** [count] interi da [from] in [dst] in un colpo solo. */
        fun read(from: Int, dst: IntArray, count: Int) = da.getInts(offset + from.toLong() * 4, dst, 0, count)
        override fun readInto(from: Int, dst: IntArray, dstOff: Int, count: Int) = da.getInts(offset + from.toLong() * 4, dst, dstOff, count)
    }
}
