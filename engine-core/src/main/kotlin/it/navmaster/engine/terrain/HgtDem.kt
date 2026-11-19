package it.navmaster.engine.terrain

import java.io.Closeable
import java.io.File
import java.io.RandomAccessFile
import java.nio.ByteOrder
import java.nio.MappedByteBuffer
import java.nio.channels.FileChannel
import kotlin.math.floor
import kotlin.math.sqrt

/**
 * La quota del terreno (m sul livello del mare) in un punto, NaN dove non si sa (mare, cella mancante,
 * vuoto del modello). I test passano una lambda; l'import vero un [HgtDem].
 */
fun interface ElevationSource {
    fun ele(lat: Double, lon: Double): Double
}

/**
 * Un modello del terreno (DEM) in celle `.hgt` della convenzione SRTM, una per grado: `N46E011.hgt` copre
 * lat 46–47, lon 11–12; interi a 16 bit big-endian, righe da nord a sud, quadrate di 1201 (3″) o 3601
 * (1″) campioni (il lato si ricava dalla lunghezza del file); il campione (0, 0) sta esattamente
 * sull'angolo nord-ovest (lat+1, lon) e l'ultima riga e l'ultima colonna coincidono con la prima delle
 * celle vicine. -32768 è un vuoto.
 *
 * Ogni cella si apre alla prima richiesta con un [MappedByteBuffer] (fuori dallo heap: al massimo 26 MB
 * l'una); una cella assente è ricordata come tale. Interpolazione bilineare; NaN se uno dei quattro
 * campioni è un vuoto.
 *
 * Le celle le prepara la CI (tools/dem/scarica-dem.sh): Copernicus GLO-90 convertito con GDAL, o le
 * celle «skadi» di Tilezen come riserva. Attribuzione (da mettere nei crediti dell'app quando i grafi
 * sono costruiti col DEM Copernicus): «produced using Copernicus WorldDEM-90 © DLR e.V. 2010-2014 and
 * © Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA;
 * all rights reserved» (licenza COPDEM). Le celle skadi (Tilezen: EU-DEM in Europa, SRTM altrove,
 * 2016) vogliono le attribuzioni di https://github.com/tilezen/joerd/blob/master/docs/attribution.md.
 */
class HgtDem(private val dir: File) : ElevationSource, Closeable {
    private class Tile(val buf: MappedByteBuffer, val n: Int)

    private val tiles = HashMap<Int, Tile?>()

    /** Celle aperte finora (per le statistiche). */
    val openedTiles: Int get() = tiles.values.count { it != null }

    init {
        require(dir.isDirectory) { "DEM: $dir non è una cartella" }
    }

    /** Le celle `.hgt` presenti nella cartella. */
    fun tileCount(): Int = dir.listFiles { f -> f.name.lowercase().endsWith(".hgt") }?.size ?: 0

    override fun ele(lat: Double, lon: Double): Double {
        if (lat.isNaN() || lon.isNaN() || lat < -90 || lat > 90) return Double.NaN
        val la = floor(lat).toInt(); val lo = floor(lon).toInt()
        val t = tile(la, lo) ?: return Double.NaN
        val n1 = t.n - 1
        // riga dal bordo nord, colonna dal bordo ovest, in campioni
        val y = (la + 1 - lat) * n1
        val x = (lon - lo) * n1
        var r = y.toInt(); var c = x.toInt()
        if (r >= n1) r = n1 - 1
        if (c >= n1) c = n1 - 1
        val fy = y - r; val fx = x - c
        val v00 = sample(t, r, c); val v01 = sample(t, r, c + 1)
        val v10 = sample(t, r + 1, c); val v11 = sample(t, r + 1, c + 1)
        if (v00 == VOID || v01 == VOID || v10 == VOID || v11 == VOID) return Double.NaN
        return (v00 * (1 - fx) + v01 * fx) * (1 - fy) + (v10 * (1 - fx) + v11 * fx) * fy
    }

    private fun sample(t: Tile, r: Int, c: Int): Int = t.buf.getShort((r * t.n + c) * 2).toInt()

    private fun tile(lat: Int, lon: Int): Tile? {
        val key = (lat + 90) * 1000 + (lon + 180)
        if (tiles.containsKey(key)) return tiles[key]
        val t = open(lat, lon)
        tiles[key] = t
        return t
    }

    private fun open(lat: Int, lon: Int): Tile? {
        val name = name(lat, lon)
        val f = File(dir, "$name.hgt").takeIf { it.isFile } ?: File(dir, "${name.lowercase()}.hgt").takeIf { it.isFile } ?: return null
        val len = f.length()
        val n = sqrt(len / 2.0).toInt()
        require(n >= 2 && n.toLong() * n * 2 == len) { "DEM: $f non è una cella .hgt quadrata (${len} byte)" }
        RandomAccessFile(f, "r").use { raf ->
            val buf = raf.channel.map(FileChannel.MapMode.READ_ONLY, 0, len)
            buf.order(ByteOrder.BIG_ENDIAN)
            return Tile(buf, n)
        }
    }

    override fun close() { tiles.clear() }

    companion object {
        const val VOID = -32768

        /** Il nome SRTM della cella che ha l'angolo sud-ovest in ([lat], [lon]): `N46E011`, `S01W072`. */
        fun name(lat: Int, lon: Int): String =
            String.format(java.util.Locale.ROOT, "%s%02d%s%03d", if (lat >= 0) "N" else "S", kotlin.math.abs(lat), if (lon >= 0) "E" else "W", kotlin.math.abs(lon))
    }
}
