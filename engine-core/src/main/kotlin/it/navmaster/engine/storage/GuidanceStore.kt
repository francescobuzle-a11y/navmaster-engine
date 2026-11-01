package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import it.navmaster.engine.profile.GuidanceAttributes
import java.io.Closeable

/**
 * Dati di guida per arco (file `guidance`), solo informativi: non entrano nei pesi né nei filtri.
 * Record di [ENTRY_BYTES] byte per arco, nell'ordine degli archi del grafo:
 *
 * ```
 *  0 u8 maxspeed A→B      1 u8 maxspeed B→A      2 u8 maxspeed:hgv A→B   3 u8 maxspeed:hgv B→A
 *  4 u8 lanes             5 u8 lanes A→B         6 u8 lanes B→A          7 u8 flag (galleria, ponte, senso unico)
 *  8 ref                 12 turn:lanes A→B      16 turn:lanes B→A       20 destination A→B
 * 24 destination B→A     28 destination:ref A→B 32 destination:ref B→A  36 svincolo al nodo A
 * 40 svincolo al nodo B  44 nodi speciali lungo l'arco ("tipo@indice;…", indice nella geometria completa A→B)
 * 48 restrizioni a orario 52 cartelli corsie A→B 56 cartelli corsie B→A 60 cartello (relazione) partendo da A
 * 64 cartello (relazione) partendo da B           68 u8 categoria ADR galleria, 3 byte liberi
 * ```
 * Le stringhe stanno nel deposito dei nomi del grafo (puntatore 0 = vuota).
 */
class GuidanceStore(private var da: DataAccess, private val names: StringStore) : Closeable {
    private var count = 0

    fun set(edge: Int, g: GuidanceAttributes, junctionA: String?, junctionB: String?, nodes: String?, signA: String? = null, signB: String? = null) {
        ensure(edge)
        val p = edge.toLong() * ENTRY_BYTES
        da.setByte(p, g.maxSpeedFwd.toByte()); da.setByte(p + 1, g.maxSpeedBwd.toByte())
        da.setByte(p + 2, g.maxSpeedHgvFwd.toByte()); da.setByte(p + 3, g.maxSpeedHgvBwd.toByte())
        da.setByte(p + 4, g.lanes.toByte()); da.setByte(p + 5, g.lanesFwd.toByte()); da.setByte(p + 6, g.lanesBwd.toByte())
        var f = 0
        if (g.tunnel) f = f or F_TUNNEL
        if (g.bridge) f = f or F_BRIDGE
        if (g.oneway == 1) f = f or F_ONEWAY_FWD
        if (g.oneway == -1) f = f or F_ONEWAY_BWD
        da.setByte(p + 7, f.toByte())
        da.setInt(p + 8, names.put(g.ref))
        da.setInt(p + 12, names.put(g.turnLanesFwd)); da.setInt(p + 16, names.put(g.turnLanesBwd))
        da.setInt(p + 20, names.put(g.destinationFwd)); da.setInt(p + 24, names.put(g.destinationBwd))
        da.setInt(p + 28, names.put(g.destinationRefFwd)); da.setInt(p + 32, names.put(g.destinationRefBwd))
        da.setInt(p + 36, names.put(junctionA ?: "")); da.setInt(p + 40, names.put(junctionB ?: ""))
        da.setInt(p + 44, names.put(nodes ?: ""))
        da.setInt(p + 48, names.put(g.conditional))
        da.setInt(p + 52, names.put(g.laneSignsFwd)); da.setInt(p + 56, names.put(g.laneSignsBwd))
        da.setInt(p + 60, names.put(signA ?: "")); da.setInt(p + 64, names.put(signB ?: ""))
        da.setByte(p + 68, g.adrTunnel.toByte())
    }

    /** Spazio per gli archi fino a [edge] compreso (gli archi senza dati di guida restano a zero). */
    fun ensure(edge: Int) {
        if (edge >= count) {
            count = edge + 1
            da.ensureCapacity(count.toLong() * ENTRY_BYTES)
        }
    }

    private fun u8(edge: Int, off: Int): Int = da.getByte(edge.toLong() * ENTRY_BYTES + off).toInt() and 0xFF
    private fun str(edge: Int, off: Int): String = names.get(da.getInt(edge.toLong() * ENTRY_BYTES + off))
    private fun has(edge: Int) = edge < count

    /** Limite di velocità percorrendo l'arco ([reverse]: da B ad A); 0 = sconosciuto, 255 = nessun limite. */
    fun maxSpeed(edge: Int, reverse: Boolean): Int = if (!has(edge)) 0 else u8(edge, if (reverse) 1 else 0)
    fun maxSpeedHgv(edge: Int, reverse: Boolean): Int = if (!has(edge)) 0 else u8(edge, if (reverse) 3 else 2)
    fun lanesTotal(edge: Int): Int = if (!has(edge)) 0 else u8(edge, 4)
    fun lanesExplicit(edge: Int, reverse: Boolean): Int = if (!has(edge)) 0 else u8(edge, if (reverse) 6 else 5)
    fun tunnel(edge: Int): Boolean = has(edge) && u8(edge, 7) and F_TUNNEL != 0
    fun bridge(edge: Int): Boolean = has(edge) && u8(edge, 7) and F_BRIDGE != 0
    /** 1 = senso unico A→B, -1 = B→A, 0 = doppio senso (dal tag `oneway`, non dall'accesso del profilo). */
    fun oneway(edge: Int): Int = if (!has(edge)) 0 else u8(edge, 7).let { f -> if (f and F_ONEWAY_FWD != 0) 1 else if (f and F_ONEWAY_BWD != 0) -1 else 0 }
    fun ref(edge: Int): String = if (!has(edge)) "" else str(edge, 8)
    fun turnLanes(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 16 else 12)
    fun destination(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 24 else 20)
    fun destinationRef(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 32 else 28)
    /** Svincolo (`motorway_junction`, "ref|nome") al nodo da cui si entra nell'arco percorrendolo nel verso dato. */
    fun junctionAtStart(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 40 else 36)
    /** Nodi speciali lungo l'arco: "tipo@indice;…" con l'indice del punto nella geometria completa A→B. */
    fun specialNodes(edge: Int): String = if (!has(edge)) "" else str(edge, 44)
    /** Restrizioni a orario: "chiave=valore" separate da [GuidanceAttributes.SEP_ITEM]. */
    fun conditional(edge: Int): String = if (!has(edge)) "" else str(edge, 48)
    /** Cartelli sulle corsie percorrendo l'arco nel verso dato: "destinazioni|ref|colori" (campi `:lanes`). */
    fun laneSigns(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 56 else 52)
    /** Cartello di una relazione `destination_sign` all'imbocco dell'arco nel verso dato: "destinazioni|ref|colore". */
    fun relationSign(edge: Int, reverse: Boolean): String = if (!has(edge)) "" else str(edge, if (reverse) 64 else 60)
    /** Categoria ADR della galleria: 0 nessuna, 2 = B … 5 = E. */
    fun adrTunnel(edge: Int): Int = if (!has(edge)) 0 else u8(edge, 68)

    /**
     * Corsie percorrendo l'arco: quelle esplicite del verso; altrimenti tutte su un senso unico,
     * metà (per eccesso nel verso della way) su un doppio senso. 0 = sconosciute.
     */
    fun lanes(edge: Int, reverse: Boolean): Int {
        val explicit = lanesExplicit(edge, reverse)
        if (explicit > 0) return explicit
        val total = lanesTotal(edge)
        if (total == 0) return 0
        return if (oneway(edge) != 0) total else if (reverse) total / 2 else (total + 1) / 2
    }

    fun flush(edgeCount: Int) {
        if (edgeCount > count) ensure(edgeCount - 1)
        da.setHeader(0, GRAPH_FORMAT_VERSION); da.setHeader(1, ENTRY_BYTES); da.setHeader(2, count)
        da.flush()
    }

    override fun close() = da.close()

    companion object {
        const val ENTRY_BYTES = 72
        private const val F_TUNNEL = 1
        private const val F_BRIDGE = 2
        private const val F_ONEWAY_FWD = 4
        private const val F_ONEWAY_BWD = 8

        /** Apre il file `guidance` di un grafo esistente; null se il grafo non lo ha. */
        fun load(dir: GraphDirectory, names: StringStore): GuidanceStore? {
            val da = dir.open("guidance") ?: return null
            if (da.getHeader(0) != GRAPH_FORMAT_VERSION || da.getHeader(1) != ENTRY_BYTES) { da.close(); return null }
            return GuidanceStore(da, names).also { it.count = da.getHeader(2) }
        }
    }
}
