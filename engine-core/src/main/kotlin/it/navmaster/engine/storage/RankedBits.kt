package it.navmaster.engine.storage

/**
 * Una bitmap nell'heap con il rango (quanti bit a 1 prima di una posizione) in tempo costante: un
 * conteggio ogni 8 parole da 64 bit (512 bit) più al massimo 8 `bitCount`. Per i file radi del
 * formato 5 (`guidance`, `turn_index`): in Germania 15,6 M archi danno 1,95 MB di bitmap e 0,12 MB
 * di ranghi (un rango per parola sarebbe stato 0,98 MB).
 */
internal class RankedBits(val words: LongArray) {
    private val blockRanks = IntArray((words.size + 7) ushr 3)
    /** I bit a 1 in tutto. */
    val total: Int

    init {
        var r = 0
        for (i in words.indices) {
            if (i and 7 == 0) blockRanks[i ushr 3] = r
            r += java.lang.Long.bitCount(words[i])
        }
        total = r
    }

    /** Il bit [i] (falso fuori dalla bitmap). */
    operator fun get(i: Int): Boolean {
        if (i < 0) return false
        val w = i ushr 6
        return w < words.size && words[w] and (1L shl (i and 63)) != 0L
    }

    /** I bit a 1 nelle posizioni prima di [i] (0 ≤ [i] < 64·parole). */
    fun rank(i: Int): Int {
        val w = i ushr 6
        var r = blockRanks[w ushr 3]
        var k = w and 7.inv()
        while (k < w) { r += java.lang.Long.bitCount(words[k]); k++ }
        return r + java.lang.Long.bitCount(words[w] and ((1L shl (i and 63)) - 1))
    }

    companion object {
        /** [n] parole da 64 bit lette da [da] a [pos] (ognuna come due int, prima il basso). */
        fun readWords(da: DataAccess, pos: Long, n: Int): LongArray {
            val w = LongArray(n)
            val buf = IntArray(minOf(2 * n, 1 shl 16))
            var done = 0
            while (done < n) {
                val m = minOf(n - done, buf.size / 2)
                da.getInts(pos + done * 8L, buf, 0, 2 * m)
                for (k in 0 until m) w[done + k] = (buf[2 * k].toLong() and 0xFFFFFFFFL) or (buf[2 * k + 1].toLong() shl 32)
                done += m
            }
            return w
        }

        /** Scrive [words] in [da] da [pos] (ognuna come due int, prima il basso); ritorna la posizione dopo. */
        fun writeWords(da: DataAccess, pos: Long, words: LongArray): Long {
            var p = pos
            for (x in words) { da.setInt(p, x.toInt()); da.setInt(p + 4, (x ushr 32).toInt()); p += 8 }
            return p
        }
    }
}
