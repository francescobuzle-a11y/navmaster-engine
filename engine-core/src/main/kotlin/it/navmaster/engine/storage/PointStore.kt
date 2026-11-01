package it.navmaster.engine.storage

import it.navmaster.engine.GRAPH_FORMAT_VERSION
import java.io.Closeable

/**
 * Punti utili alla guida ma estranei al calcolo (file `points`): autovelox, inizio e fine dei
 * tratti a velocità media (Tutor), fotocamere ai semafori. Record di 16 byte: lat e lon in
 * 1e-7 gradi, tipo, valore (km/h, 0 se assente), direzione (gradi, -1 se assente).
 */
class PointStore(private var da: DataAccess) : Closeable {
    var count = 0; private set

    fun add(lat: Double, lon: Double, type: Int, valueKmh: Int, bearing: Int) {
        val p = count.toLong() * ENTRY_BYTES
        da.ensureCapacity(p + ENTRY_BYTES)
        da.setInt(p, Math.round(lat * 1e7).toInt()); da.setInt(p + 4, Math.round(lon * 1e7).toInt())
        da.setByte(p + 8, type.toByte()); da.setByte(p + 9, 0)
        da.setShort(p + 10, valueKmh.coerceIn(0, 32767).toShort())
        da.setShort(p + 12, bearing.coerceIn(-1, 360).toShort())
        da.setShort(p + 14, 0)
        count++
    }

    fun lat(i: Int): Double = da.getInt(i.toLong() * ENTRY_BYTES) / 1e7
    fun lon(i: Int): Double = da.getInt(i.toLong() * ENTRY_BYTES + 4) / 1e7
    fun type(i: Int): Int = da.getByte(i.toLong() * ENTRY_BYTES + 8).toInt() and 0xFF
    fun value(i: Int): Int = da.getShort(i.toLong() * ENTRY_BYTES + 10).toInt()
    fun bearing(i: Int): Int = da.getShort(i.toLong() * ENTRY_BYTES + 12).toInt()

    /** Gli indici dei punti dentro il rettangolo (scansione completa: i punti sono pochi). */
    fun inBBox(minLat: Double, minLon: Double, maxLat: Double, maxLon: Double): IntArray {
        val out = com.carrotsearch.hppc.IntArrayList()
        for (i in 0 until count) {
            val la = lat(i); if (la < minLat || la > maxLat) continue
            val lo = lon(i); if (lo < minLon || lo > maxLon) continue
            out.add(i)
        }
        return out.toArray()
    }

    fun flush() {
        da.setHeader(0, GRAPH_FORMAT_VERSION); da.setHeader(1, ENTRY_BYTES); da.setHeader(2, count)
        da.flush()
    }

    override fun close() = da.close()

    companion object {
        const val ENTRY_BYTES = 16
        const val SPEED_CAMERA = 1
        const val AVERAGE_SPEED_START = 2
        const val AVERAGE_SPEED_END = 3
        const val TRAFFIC_SIGNALS_CAMERA = 4

        fun typeName(t: Int): String = when (t) {
            SPEED_CAMERA -> "speed_camera"; AVERAGE_SPEED_START -> "average_speed_start"; AVERAGE_SPEED_END -> "average_speed_end"
            TRAFFIC_SIGNALS_CAMERA -> "traffic_signals_camera"; else -> "unknown"
        }

        fun load(dir: GraphDirectory): PointStore? {
            val da = dir.open("points") ?: return null
            if (da.getHeader(0) != GRAPH_FORMAT_VERSION || da.getHeader(1) != ENTRY_BYTES) { da.close(); return null }
            return PointStore(da).also { it.count = da.getHeader(2) }
        }
    }
}
