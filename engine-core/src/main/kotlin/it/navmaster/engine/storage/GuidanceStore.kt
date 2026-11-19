package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import it.navmaster.engine.isReadableGraphFormat
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
 * 64 cartello (relazione) partendo da B           68 u8 categoria ADR galleria   69 u8 origine dei limiti (bit 0: larghezza da cartello)
 * 70-71 liberi
 * ```
 * Le stringhe stanno nel deposito dei nomi del grafo (puntatore 0 = vuota).
 *
 * Su disco (file `guidance`):
 * - formato 5, rado ([SPARSE]): solo gli archi con dati. Intestazione [1] = [SPARSE], [2] = archi,
 *   [3] = archi con soli campi da un byte, [4] = archi con stringhe. Dati: la bitmap degli archi con
 *   dati e quella degli archi con stringhe (parole da 64 bit, ognuna come due int, prima il basso),
 *   poi i record corti da [SHORT_BYTES] byte degli archi senza stringhe (i byte 0-7, 68 e 69 del record
 *   intero), poi i record interi da [ENTRY_BYTES] byte degli archi con stringhe. Le bitmap e i loro
 *   ranghi ([RankedBits]) stanno nell'heap (Germania circa 4 MB); un valore costa al più una lettura del file. In Germania il 60 % delle
 *   righe è vuoto e il 26 % ha solo i campi da un byte: circa 206 MB invece di 1 126;
 * - formato 4, denso (ancora letto): un record da [ENTRY_BYTES] byte per arco.
 *
 * Durante l'import i record si scrivono densi in un file di lavoro ([forImport]); [flush] lo
 * converte nel formato rado e lo cancella. I valori letti sono identici nei due formati.
 */
class GuidanceStore private constructor(private var da: DataAccess, private val names: StringStore, private val importDir: GraphDirectory? = null) : Closeable {
    private var count = 0

    // formato rado: bitmap degli archi con dati e con stringhe, inizio delle due aree
    private var sparse = false
    private var dataBits = RankedBits(LongArray(0))
    private var strBits = RankedBits(LongArray(0))
    private var shortBase = 0L
    private var longBase = 0L

    fun set(edge: Int, g: GuidanceAttributes, junctionA: String?, junctionB: String?, nodes: String?, signA: String? = null, signB: String? = null) {
        check(!sparse) { "dati di guida già compattati: nessuna scrittura dopo flush" }
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
        da.setByte(p + 69, (if (g.widthSigned) 1 else 0).toByte())
    }

    /** Spazio per gli archi fino a [edge] compreso (gli archi senza dati di guida restano a zero). */
    private fun ensure(edge: Int) {
        if (edge >= count) {
            count = edge + 1
            da.ensureCapacity(count.toLong() * ENTRY_BYTES)
        }
    }

    /** Il byte del record intero all'offset [off]: dal record corto o da quello intero (formato rado). */
    private fun sparseU8(edge: Int, off: Int): Int {
        if (strBits[edge]) return da.getByte(longBase + strBits.rank(edge).toLong() * ENTRY_BYTES + off).toInt() and 0xFF
        val k = when (off) { in 0..7 -> off; 68 -> 8; 69 -> 9; else -> return 0 }
        val r = dataBits.rank(edge) - strBits.rank(edge)
        return da.getByte(shortBase + r.toLong() * SHORT_BYTES + k).toInt() and 0xFF
    }

    /** Il puntatore della stringa all'offset [off] del record intero; 0 per gli archi senza stringhe (formato rado). */
    private fun sparsePtr(edge: Int, off: Int): Int =
        if (!strBits[edge]) 0 else da.getInt(longBase + strBits.rank(edge).toLong() * ENTRY_BYTES + off)

    private fun u8(edge: Int, off: Int): Int =
        if (sparse) sparseU8(edge, off) else da.getByte(edge.toLong() * ENTRY_BYTES + off).toInt() and 0xFF
    private fun ptr(edge: Int, off: Int): Int = if (sparse) sparsePtr(edge, off) else da.getInt(edge.toLong() * ENTRY_BYTES + off)
    private fun str(edge: Int, off: Int): String = names.get(ptr(edge, off))
    private fun has(edge: Int) = edge in 0 until count && (!sparse || dataBits[edge])

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
    /**
     * Il puntatore della stringa `ref` dell'arco (0 = nessuno), una lettura sola: per tenere in cache
     * le stringhe già lette. Nei grafi di formato 4 i puntatori non sono canonici (lo StringStore
     * deduplicava solo le stringhe recenti): mai confrontarli, confrontare le stringhe ([string]).
     */
    fun refPointer(edge: Int): Int = if (!has(edge)) 0 else ptr(edge, 8)
    /** La stringa al puntatore [ptr] del deposito dei nomi ("" per 0). */
    fun string(ptr: Int): String = names.get(ptr)
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
    /** La larghezza massima dell'arco viene da un cartello (`maxwidth`), non dalla carreggiata (`width`). */
    fun widthSigned(edge: Int): Boolean = has(edge) && u8(edge, 69) and 1 != 0

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

    /**
     * Scrive il file `guidance`. Durante l'import ([forImport]) i record densi di lavoro diventano il
     * formato rado e il file di lavoro sparisce; da lì i valori si leggono dal file rado.
     */
    fun flush(edgeCount: Int) {
        val dir = importDir
        if (dir == null || sparse) {
            if (sparse) return
            if (edgeCount > count) ensure(edgeCount - 1)
            da.setHeader(0, GRAPH_FORMAT_VERSION); da.setHeader(1, ENTRY_BYTES); da.setHeader(2, count)
            da.flush()
            return
        }
        val n = maxOf(count, edgeCount)
        val nw = (n + 63) ushr 6
        val dw = LongArray(nw)
        val sw = LongArray(nw)
        // primo passo: quali archi hanno dati e quali stringhe
        val chunk = 4096
        val buf = ByteArray(chunk * ENTRY_BYTES)
        var shorts = 0; var longs = 0
        var e0 = 0
        while (e0 < count) {
            val m = minOf(chunk, count - e0)
            da.getBytes(e0.toLong() * ENTRY_BYTES, buf, m * ENTRY_BYTES)
            for (i in 0 until m) {
                val o = i * ENTRY_BYTES
                var any = false; var strs = false
                for (k in 0 until ENTRY_BYTES) if (buf[o + k].toInt() != 0) { any = true; if (k in 8 until 68) strs = true }
                if (!any) continue
                val e = e0 + i
                dw[e ushr 6] = dw[e ushr 6] or (1L shl (e and 63))
                if (strs) { sw[e ushr 6] = sw[e ushr 6] or (1L shl (e and 63)); longs++ } else shorts++
            }
            e0 += m
        }
        val out = dir.create(FILE)
        val wordsBytes = nw.toLong() * 8
        val sBase = 2 * wordsBytes
        val lBase = (sBase + shorts.toLong() * SHORT_BYTES + 3) and 3L.inv()
        out.ensureCapacity(maxOf(4L, lBase + longs.toLong() * ENTRY_BYTES))
        RankedBits.writeWords(out, RankedBits.writeWords(out, 0L, dw), sw)
        // secondo passo: i record corti e quelli interi, in ordine di arco
        var sp = sBase; var lp = lBase
        val short = ByteArray(SHORT_BYTES)
        e0 = 0
        while (e0 < count) {
            val m = minOf(chunk, count - e0)
            da.getBytes(e0.toLong() * ENTRY_BYTES, buf, m * ENTRY_BYTES)
            for (i in 0 until m) {
                val e = e0 + i
                if (dw[e ushr 6] and (1L shl (e and 63)) == 0L) continue
                val o = i * ENTRY_BYTES
                if (sw[e ushr 6] and (1L shl (e and 63)) != 0L) {
                    out.setBytes(lp, buf.copyOfRange(o, o + ENTRY_BYTES), ENTRY_BYTES); lp += ENTRY_BYTES
                } else {
                    for (k in 0 until 8) short[k] = buf[o + k]
                    short[8] = buf[o + 68]; short[9] = buf[o + 69]
                    out.setBytes(sp, short, SHORT_BYTES); sp += SHORT_BYTES
                }
            }
            e0 += m
        }
        out.setHeader(0, GRAPH_FORMAT_VERSION); out.setHeader(1, SPARSE); out.setHeader(2, n)
        out.setHeader(3, shorts); out.setHeader(4, longs)
        out.flush()
        da.close()
        java.io.File(dir.dir, IMPORT_FILE).delete()
        da = out
        setSparse(n, dw, sw, sBase, lBase)
    }

    private fun setSparse(n: Int, dw: LongArray, sw: LongArray, sBase: Long, lBase: Long) {
        check(dw.size == sw.size && dw.indices.all { sw[it] and dw[it].inv() == 0L }) { "dati di guida incoerenti: stringhe su un arco senza dati" }
        count = n; dataBits = RankedBits(dw); strBits = RankedBits(sw); shortBase = sBase; longBase = lBase
        sparse = true
    }

    override fun close() = da.close()

    companion object {
        const val ENTRY_BYTES = 72
        /** Il record degli archi senza stringhe nel formato rado: i byte 0-7, 68 e 69 del record intero. */
        const val SHORT_BYTES = 10
        /** L'intestazione [1] del formato rado (nel denso vale [ENTRY_BYTES]). */
        const val SPARSE = 0x53504152
        const val FILE = "guidance"
        /** Il file denso di lavoro durante l'import, cancellato da [flush]. */
        const val IMPORT_FILE = "guidance.import"
        private const val F_TUNNEL = 1
        private const val F_BRIDGE = 2
        private const val F_ONEWAY_FWD = 4
        private const val F_ONEWAY_BWD = 8

        /** Il deposito per l'import: record densi in un file di lavoro, resi radi da [flush]. */
        fun forImport(dir: GraphDirectory, names: StringStore): GuidanceStore = GuidanceStore(dir.create(IMPORT_FILE), names, dir)

        /** Apre il file `guidance` di un grafo esistente (rado o denso); null se il grafo non lo ha. */
        fun load(dir: GraphDirectory, names: StringStore): GuidanceStore? {
            val da = dir.open(FILE) ?: return null
            if (!isReadableGraphFormat(da.getHeader(0))) { da.close(); return null }
            return when (da.getHeader(1)) {
                ENTRY_BYTES -> GuidanceStore(da, names).also { it.count = da.getHeader(2) }
                SPARSE -> {
                    val n = da.getHeader(2); val shorts = da.getHeader(3)
                    val nw = (n + 63) ushr 6
                    val dw = RankedBits.readWords(da, 0L, nw)
                    val sw = RankedBits.readWords(da, nw * 8L, nw)
                    val sBase = nw.toLong() * 16
                    val lBase = (sBase + shorts.toLong() * SHORT_BYTES + 3) and 3L.inv()
                    GuidanceStore(da, names).also { it.setSparse(n, dw, sw, sBase, lBase) }
                }
                else -> { da.close(); null }
            }
        }
    }
}
