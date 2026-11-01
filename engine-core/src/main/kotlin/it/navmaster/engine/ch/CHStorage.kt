package it.navmaster.engine.ch

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import it.navmaster.engine.storage.DataAccess
import it.navmaster.engine.storage.GraphDirectory
import java.io.Closeable
import kotlin.math.roundToLong

/**
 * Shortcut e livelli di una Contraction Hierarchy node-based (GraphHopper `CHStorage`).
 *
 * Shortcut (20 byte): `nodeA<<1|fwd @0 | nodeB<<1|bwd @4 | peso in millesimi @8 | skip1 @12 | skip2 @16`.
 * `nodeA` è il nodo di livello più basso; gli shortcut sono scritti in ordine non decrescente di
 * livello(nodeA), così ogni nodo ha un intervallo contiguo `[firstShortcut, lastShortcut]`.
 * skip1/skip2 sono id di arco CH (arco base, oppure `baseEdges + indice shortcut`) nell'orientamento nodeA→nodeB.
 * Nodo (8 byte): `livello @0 | ultimo shortcut @4` (-1 = nessuno).
 */
class CHStorage(private val dir: GraphDirectory, val name: String, val baseEdges: Int, val baseNodes: Int) : Closeable {
    private var shortcuts: DataAccess = dir.create("shortcuts_$name", 1 shl 20)
    private var nodes: DataAccess = dir.create("levels_$name", 1 shl 16)
    var shortcutCount = 0; private set
    var coreNodeCount = 0
    private var lastNodeA = -1

    /** Da chiamare solo in preparazione: alloca i record dei nodi e azzera i livelli. */
    fun initLevels() {
        nodes.ensureCapacity(baseNodes.toLong() * NODE_BYTES)
        for (n in 0 until baseNodes) { nodes.setInt(n.toLong() * NODE_BYTES, baseNodes); nodes.setInt(n.toLong() * NODE_BYTES + 4, NO_SHORTCUT) }
    }

    fun level(node: Int): Int = nodes.getInt(node.toLong() * NODE_BYTES)
    fun setLevel(node: Int, level: Int) = nodes.setInt(node.toLong() * NODE_BYTES, level)
    fun lastShortcut(node: Int): Int = nodes.getInt(node.toLong() * NODE_BYTES + 4)

    /** Aggiunge uno shortcut registrato su [nodeA]; ritorna l'id di arco CH (`baseEdges + indice`). */
    fun addShortcut(nodeA: Int, nodeB: Int, fwd: Boolean, bwd: Boolean, weight: Double, skip1: Int, skip2: Int): Int {
        require(fwd || bwd)
        if (lastNodeA >= 0 && nodeA != lastNodeA) check(level(nodeA) >= level(lastNodeA)) { "shortcut fuori ordine di livello" }
        val idx = shortcutCount++
        shortcuts.ensureCapacity((shortcutCount.toLong() + 1) * SHORTCUT_BYTES)
        val p = idx.toLong() * SHORTCUT_BYTES
        shortcuts.setInt(p, (nodeA shl 1) or (if (fwd) 1 else 0))
        shortcuts.setInt(p + 4, (nodeB shl 1) or (if (bwd) 1 else 0))
        shortcuts.setInt(p + 8, encodeWeight(weight))
        shortcuts.setInt(p + 12, skip1); shortcuts.setInt(p + 16, skip2)
        nodes.setInt(nodeA.toLong() * NODE_BYTES + 4, idx)
        lastNodeA = nodeA
        return baseEdges + idx
    }

    fun isShortcut(chEdge: Int) = chEdge >= baseEdges
    fun shortcutIndex(chEdge: Int) = chEdge - baseEdges
    fun nodeA(sc: Int): Int = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES) ushr 1
    fun nodeB(sc: Int): Int = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES + 4) ushr 1
    fun fwd(sc: Int): Boolean = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES) and 1 == 1
    fun bwd(sc: Int): Boolean = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES + 4) and 1 == 1
    fun weight(sc: Int): Double = decodeWeight(shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES + 8))
    fun skip1(sc: Int): Int = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES + 12)
    fun skip2(sc: Int): Int = shortcuts.getInt(sc.toLong() * SHORTCUT_BYTES + 16)

    fun flush() {
        shortcuts.setHeader(0, GRAPH_FORMAT_VERSION); shortcuts.setHeader(1, shortcutCount); shortcuts.setHeader(2, baseEdges)
        nodes.setHeader(0, GRAPH_FORMAT_VERSION); nodes.setHeader(1, baseNodes); nodes.setHeader(2, coreNodeCount)
        shortcuts.flush(); nodes.flush()
    }

    fun loadExisting(): Boolean {
        val s = dir.open("shortcuts_$name", 1 shl 20) ?: return false
        val n = dir.open("levels_$name", 1 shl 16) ?: return false
        check(s.getHeader(2) == baseEdges && n.getHeader(1) == baseNodes) { "CH $name non corrisponde al grafo" }
        shortcuts.close(); nodes.close()
        shortcuts = s; nodes = n
        shortcutCount = s.getHeader(1); coreNodeCount = n.getHeader(2)
        return true
    }

    override fun close() { shortcuts.close(); nodes.close() }

    companion object {
        const val NO_SHORTCUT = -1
        const val SHORTCUT_BYTES = 20L
        const val NODE_BYTES = 8L
        private const val MIN_WEIGHT = 0.001
        private const val MAX_STORED = (Int.MAX_VALUE.toLong() shl 1)

        fun encodeWeight(w: Double): Int {
            require(w >= 0) { "peso negativo: $w" }
            if (w.isInfinite() || w >= MAX_STORED / 1000.0) return MAX_STORED.toInt() // marcatore di infinito
            val v = (maxOf(w, MIN_WEIGHT) * 1000).roundToLong()
            return v.toInt()
        }

        fun decodeWeight(i: Int): Double {
            val v = i.toLong() and 0xFFFFFFFFL
            return if (v >= MAX_STORED) Double.POSITIVE_INFINITY else v / 1000.0
        }
    }
}
