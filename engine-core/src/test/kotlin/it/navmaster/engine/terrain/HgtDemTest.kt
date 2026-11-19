package it.navmaster.engine.terrain

import java.io.File
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/** Il lettore delle celle .hgt: quote bilineari, NaN fuori dalle celle e sui vuoti, continuità fra celle vicine. */
class HgtDemTest {
    /** Scrive una cella [name].hgt di lato [n] con quota v(riga, colonna). */
    private fun writeTile(dir: File, name: String, n: Int, v: (Int, Int) -> Int) {
        val buf = ByteBuffer.allocate(n * n * 2).order(ByteOrder.BIG_ENDIAN)
        for (r in 0 until n) for (c in 0 until n) buf.putShort(v(r, c).toShort())
        File(dir, "$name.hgt").writeBytes(buf.array())
    }

    private fun tempDir() = Files.createTempDirectory("nme-dem").toFile().also { it.deleteOnExit() }

    @Test
    fun `bilinear heights, missing tiles, voids and the shared edge of two tiles`() {
        val dir = tempDir()
        val n = 1201
        // una quota lineare in riga e colonna: la bilineare la riproduce esattamente; la cella E012 la continua
        writeTile(dir, "N46E011", n) { r, c -> if (r == 100 && c == 100) HgtDem.VOID else 1000 + r - 2 * c }
        writeTile(dir, "N46E012", n) { r, c -> 1000 + r - 2 * (c + 1200) }
        HgtDem(dir).use { dem ->
            assertEquals(2, dem.tileCount())
            fun expected(lat: Double, lon: Double) = 1000 + (47 - lat) * 1200 - 2 * (lon - 11) * 1200
            assertEquals(400.0, dem.ele(46.5, 11.5), 1e-9) // campione esatto: riga 600, colonna 600
            for ((lat, lon) in listOf(46.123456 to 11.654321, 46.987 to 11.001, 46.0001 to 11.9999)) assertEquals(expected(lat, lon), dem.ele(lat, lon), 1e-6, "$lat,$lon")
            // fuori dalle celle: NaN
            assertTrue(dem.ele(47.5, 11.5).isNaN())
            assertTrue(dem.ele(45.5, 11.5).isNaN())
            // un vuoto rende NaN le quattro celle intorno, non le altre
            assertTrue(dem.ele(47 - 100.5 / 1200, 11 + 100.5 / 1200).isNaN())
            assertTrue(dem.ele(47 - 99.5 / 1200, 11 + 99.5 / 1200).isNaN())
            assertTrue(!dem.ele(47 - 98.5 / 1200, 11 + 98.5 / 1200).isNaN())
            // il bordo comune: la colonna 1200 di E011 è la colonna 0 di E012
            val west = dem.ele(46.5, 12 - 1e-9); val east = dem.ele(46.5, 12.0)
            assertEquals(east, west, 0.01)
            assertEquals(2, dem.openedTiles) // E011 ed E012 aperte, N47 e N45 ricordate come assenti
        }
    }

    @Test
    fun `the tile side comes from the file length, and the names follow SRTM`() {
        val dir = tempDir()
        writeTile(dir, "S01W072", 361) { r, c -> r + c } // un lato qualunque purché quadrato (1201, 3601, …)
        HgtDem(dir).use { dem ->
            // S01W072 copre lat -1..0, lon -72..-71
            assertEquals(180.0 + 180.0, dem.ele(-0.5, -71.5), 1e-9)
            assertEquals(0.0, dem.ele(-1e-9, -72.0), 1e-3) // lat 0 esatta è già la cella N00
            assertEquals(720.0, dem.ele(-1.0, -71.0 - 1e-9), 1e-3)
        }
        assertEquals("N46E011", HgtDem.name(46, 11))
        assertEquals("S01W072", HgtDem.name(-1, -72))
        assertEquals("N00E000", HgtDem.name(0, 0))
    }
}
