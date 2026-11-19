package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable

/**
 * Costi di svolta per nodo "via": per ogni nodo una lista concatenata di voci
 * (arcoDa, arcoA, flag, successivo). I flag contengono un valore codificato per profilo
 * (1 bit: svolta vietata). Stesso schema di GraphHopper `TurnCostStorage`.
 *
 * File `turn_costs`: record da 16 byte `FROM@0 | TO@4 | FLAGS@8 | NEXT@12` (-1 = fine lista).
 *
 * Le teste delle liste (la prima voce di ogni nodo con divieti):
 * - formato 5, file `turn_index`: una bitmap dei nodi con voci e le teste dei soli nodi con voci, in
 *   ordine di nodo, caricate nell'heap all'apertura. Intestazione [1] = nodi coperti dalla bitmap
 *   ([nodeLimit]), [2] = teste, [3] = voci di `turn_costs` dello stesso giro (un `turn_index` rimasto
 *   da un altro grafo nella stessa cartella non viene usato); dati: le parole della bitmap (64 bit,
 *   ognuna come due int, prima il basso), poi le teste. In Germania 116 mila nodi su 13,5 milioni hanno voci: 2,3 MB invece di 74,5,
 *   e [hasEntries], letto per ogni nodo via delle ricerche edge-based, diventa un bit nell'heap invece
 *   di una lettura del file mappato (una chiamata nativa sul tablet);
 * - formato 4, file `turn_heads` (ancora letto): un int per nodo con l'indice della prima voce (-1 = nessuna).
 *
 * Durante l'import le teste si scrivono come nel formato 4 (file `turn_heads` di lavoro); [flush] le
 * compatta in `turn_index`, poi cancella il file di lavoro, e da lì risponde dall'heap. Un deposito
 * aperto da [loadExisting] non viene mai convertito né cancellato da [flush].
 */
class TurnCostStorage(private val dir: GraphDirectory, val encoding: TurnCostEncoding) : Closeable {
    private var costs: DataAccess = dir.create("turn_costs", 1 shl 16)
    /** Le teste per nodo (import e grafi di formato 4); null dopo la compattazione o con `turn_index`. */
    private var heads: DataAccess? = dir.create(LEGACY_HEADS, 1 shl 16)
    var count = 0; private set
    private var nodeCapacity = 0

    // le teste compatte (formato 5): bitmap dei nodi con voci e teste in ordine di nodo
    private var compact = false
    private var nodeLimit = 0
    private var bits = RankedBits(LongArray(0))
    private var headOf = IntArray(0)
    /** Aperto da [loadExisting]: [flush] non converte né cancella nulla. */
    private var loaded = false

    private fun head(node: Int): Int {
        if (compact) {
            if (node < 0 || node >= nodeLimit || !bits[node]) return NO_ENTRY
            return headOf[bits.rank(node)]
        }
        if (node >= nodeCapacity) return NO_ENTRY
        return heads!!.getInt(node.toLong() * 4)
    }

    private fun setHead(node: Int, entry: Int) {
        val h = heads ?: error("costi di svolta già compattati: nessuna scrittura dopo flush")
        if (node >= nodeCapacity) {
            val newCap = maxOf(node + 1, nodeCapacity * 2, 1024)
            h.ensureCapacity(newCap.toLong() * 4)
            for (n in nodeCapacity until newCap) h.setInt(n.toLong() * 4, NO_ENTRY)
            nodeCapacity = newCap
        }
        h.setInt(node.toLong() * 4, entry)
    }

    /** Imposta i flag della svolta (da, via, a); se la voce esiste, i flag vengono uniti con OR. */
    fun set(fromEdge: Int, viaNode: Int, toEdge: Int, flags: Int) {
        check(!compact) { "costi di svolta già compattati: nessuna scrittura dopo flush" }
        var e = head(viaNode)
        var last = NO_ENTRY
        while (e != NO_ENTRY) {
            val p = e.toLong() * ENTRY_BYTES
            if (costs.getInt(p) == fromEdge && costs.getInt(p + 4) == toEdge) {
                costs.setInt(p + 8, costs.getInt(p + 8) or flags)
                return
            }
            last = e
            e = costs.getInt(p + 12)
        }
        val idx = count++
        costs.ensureCapacity((count.toLong() + 1) * ENTRY_BYTES)
        val p = idx.toLong() * ENTRY_BYTES
        costs.setInt(p, fromEdge); costs.setInt(p + 4, toEdge); costs.setInt(p + 8, flags); costs.setInt(p + 12, NO_ENTRY)
        if (last == NO_ENTRY) setHead(viaNode, idx) else costs.setInt(last.toLong() * ENTRY_BYTES + 12, idx)
    }

    /** Flag della svolta, 0 se non registrata. */
    fun flags(fromEdge: Int, viaNode: Int, toEdge: Int): Int {
        var e = head(viaNode)
        while (e != NO_ENTRY) {
            val p = e.toLong() * ENTRY_BYTES
            if (costs.getInt(p) == fromEdge && costs.getInt(p + 4) == toEdge) return costs.getInt(p + 8)
            e = costs.getInt(p + 12)
        }
        return 0
    }

    fun hasEntries(viaNode: Int) = head(viaNode) != NO_ENTRY

    /**
     * Scrive le voci e le teste compatte (`turn_index`), poi cancella il file di lavoro `turn_heads`.
     * Su un deposito aperto da [loadExisting] scrive solo le voci (in sola lettura: nulla).
     */
    fun flush() {
        if (loaded) { costs.flush(); return }
        costs.setHeader(0, GRAPH_FORMAT_VERSION); costs.setHeader(1, count)
        costs.flush()
        if (!compact) compactHeads()
        val da = dir.create(INDEX, 1 shl 16)
        try {
            da.setHeader(0, GRAPH_FORMAT_VERSION); da.setHeader(1, nodeLimit); da.setHeader(2, headOf.size); da.setHeader(3, count)
            da.ensureCapacity(maxOf(4L, bits.words.size.toLong() * 8 + headOf.size.toLong() * 4))
            var p = RankedBits.writeWords(da, 0L, bits.words)
            for (h in headOf) { da.setInt(p, h); p += 4 }
            da.flush()
        } finally { da.close() }
        // il file di lavoro solo dopo che turn_index è scritto
        heads?.close()
        heads = null
        java.io.File(dir.dir, LEGACY_HEADS).delete()
    }

    /** Le teste per nodo diventano bitmap + teste dei soli nodi con voci (il file di lavoro resta fino a [flush]). */
    private fun compactHeads() {
        val h = heads ?: error("teste dei costi di svolta già chiuse")
        var limit = 0
        var n = 0
        for (node in 0 until nodeCapacity) if (h.getInt(node.toLong() * 4) != NO_ENTRY) { limit = node + 1; n++ }
        val w = LongArray((limit + 63) ushr 6)
        val hs = IntArray(n)
        var k = 0
        for (node in 0 until limit) {
            val e = h.getInt(node.toLong() * 4)
            if (e == NO_ENTRY) continue
            w[node ushr 6] = w[node ushr 6] or (1L shl (node and 63))
            hs[k++] = e
        }
        setCompact(limit, w, hs)
    }

    private fun setCompact(limit: Int, w: LongArray, hs: IntArray) {
        val b = RankedBits(w)
        check(b.total == hs.size) { "teste dei costi di svolta incoerenti: ${b.total} bit, ${hs.size} teste" }
        nodeLimit = limit; bits = b; headOf = hs
        compact = true
    }

    fun loadExisting(): Boolean {
        val c = dir.open("turn_costs", 1 shl 16) ?: return false
        loaded = true
        val idx = dir.open(INDEX, 1 shl 16)
        // turn_index solo se è dello stesso giro di turn_costs (formato 5 e stesso numero di voci)
        if (idx != null && c.getHeader(0) >= 5 && idx.getHeader(3) == c.getHeader(1)) {
            try {
                val limit = idx.getHeader(1); val n = idx.getHeader(2)
                val nw = (limit + 63) ushr 6
                val w = RankedBits.readWords(idx, 0L, nw)
                val hs = IntArray(n)
                if (n > 0) idx.getInts(nw * 8L, hs, 0, n)
                setCompact(limit, w, hs)
            } finally { idx.close() }
            costs.close(); heads?.close(); heads = null
            costs = c
            count = c.getHeader(1)
            return true
        }
        idx?.close()
        val h = dir.open(LEGACY_HEADS, 1 shl 16) ?: run { c.close(); return false }
        costs.close(); heads?.close()
        costs = c; heads = h
        count = c.getHeader(1); nodeCapacity = h.getHeader(1)
        return true
    }

    /** Riscaldamento (vedi [DataAccess.touch]): teste delle liste (solo nel formato 4: nel 5 sono nell'heap) e voci. Byte toccati. */
    fun touch(progress: ((Long) -> Unit)? = null): Long {
        val a = heads?.touch(progress) ?: 0L
        return a + costs.touch(progress?.let { p -> { b: Long -> p(a + b) } })
    }
    /** I byte che [touch] leggerebbe. */
    val touchBytes: Long get() = (heads?.touchBytes ?: 0L) + costs.touchBytes

    override fun close() { costs.close(); heads?.close() }

    companion object {
        const val NO_ENTRY = -1
        const val ENTRY_BYTES = 16L
        /** Le teste compatte (formato 5). */
        const val INDEX = "turn_index"
        /** Le teste per nodo (formato 4, e file di lavoro durante l'import). */
        const val LEGACY_HEADS = "turn_heads"
    }
}

/** Un bit per profilo nei flag di svolta: 1 = svolta vietata. */
class TurnCostEncoding(profileNames: List<String>) {
    val forbidden: Map<String, BooleanEncodedValue> = profileNames.associateWith { BooleanEncodedValue("${it}_turn_restriction") }
    val registry: EncodedValueRegistry = EncodedValueRegistry.Builder().also { b -> forbidden.values.forEach { b.add(it) } }.build()

    init { require(registry.intsForFlags == 1) { "troppi profili per un int di flag di svolta" } }

    fun isForbidden(flags: Int, profileName: String): Boolean {
        val ev = forbidden[profileName] ?: return false
        return ev.get(intArrayOf(flags))
    }

    fun forbid(flags: Int, profileName: String): Int {
        val f = intArrayOf(flags)
        forbidden.getValue(profileName).set(f, true)
        return f[0]
    }
}
