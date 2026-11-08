package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable

/**
 * Costi di svolta per nodo "via": per ogni nodo una lista concatenata di voci
 * (arcoDa, arcoA, flag, successivo). I flag contengono un valore codificato per profilo
 * (1 bit: svolta vietata). Stesso schema di GraphHopper `TurnCostStorage`.
 *
 * File `turn_costs`: record da 16 byte `FROM@0 | TO@4 | FLAGS@8 | NEXT@12` (-1 = fine lista).
 * File `turn_heads`: un int per nodo con l'indice della prima voce (-1 = nessuna).
 */
class TurnCostStorage(private val dir: GraphDirectory, val encoding: TurnCostEncoding) : Closeable {
    private var costs: DataAccess = dir.create("turn_costs", 1 shl 16)
    private var heads: DataAccess = dir.create("turn_heads", 1 shl 16)
    var count = 0; private set
    private var nodeCapacity = 0

    private fun head(node: Int): Int {
        if (node >= nodeCapacity) return NO_ENTRY
        return heads.getInt(node.toLong() * 4)
    }

    private fun setHead(node: Int, entry: Int) {
        if (node >= nodeCapacity) {
            val newCap = maxOf(node + 1, nodeCapacity * 2, 1024)
            heads.ensureCapacity(newCap.toLong() * 4)
            for (n in nodeCapacity until newCap) heads.setInt(n.toLong() * 4, NO_ENTRY)
            nodeCapacity = newCap
        }
        heads.setInt(node.toLong() * 4, entry)
    }

    /** Imposta i flag della svolta (da, via, a); se la voce esiste, i flag vengono uniti con OR. */
    fun set(fromEdge: Int, viaNode: Int, toEdge: Int, flags: Int) {
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

    fun flush() {
        costs.setHeader(0, GRAPH_FORMAT_VERSION); costs.setHeader(1, count)
        heads.setHeader(0, GRAPH_FORMAT_VERSION); heads.setHeader(1, nodeCapacity)
        costs.flush(); heads.flush()
    }

    fun loadExisting(): Boolean {
        val c = dir.open("turn_costs", 1 shl 16) ?: return false
        val h = dir.open("turn_heads", 1 shl 16) ?: return false
        costs.close(); heads.close()
        costs = c; heads = h
        count = c.getHeader(1); nodeCapacity = h.getHeader(1)
        return true
    }

    /** Riscaldamento (vedi [DataAccess.touch]): teste delle liste e voci. Byte toccati. */
    fun touch(progress: ((Long) -> Unit)? = null): Long {
        val a = heads.touch(progress)
        return a + costs.touch(progress?.let { p -> { b: Long -> p(a + b) } })
    }
    /** I byte che [touch] leggerebbe. */
    val touchBytes: Long get() = heads.touchBytes + costs.touchBytes

    override fun close() { costs.close(); heads.close() }

    companion object {
        const val NO_ENTRY = -1
        const val ENTRY_BYTES = 16L
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
