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

    class OfArray(private val a: IntArray) : IntSource() {
        override val size get() = a.size
        override fun get(i: Int) = a[i]
    }

    /** [size] interi dal byte [offset] di [da]. */
    class OfData(private val da: DataAccess, private val offset: Long, override val size: Int) : IntSource() {
        override fun get(i: Int) = da.getInt(offset + i.toLong() * 4)
        override fun close() { da.close() }
        /** [count] interi da [from] in [dst] in un colpo solo. */
        fun read(from: Int, dst: IntArray, count: Int) = da.getInts(offset + from.toLong() * 4, dst, 0, count)
    }
}
