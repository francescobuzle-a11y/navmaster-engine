package it.navmaster.engine.storage

import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class DataAccessTest {
    private fun tmpDir(): File = Files.createTempDirectory("nme-da").toFile().also { it.deleteOnExit() }

    private fun exercise(create: (File) -> DataAccess, reopen: (File) -> DataAccess?) {
        val dir = tmpDir()
        val f = File(dir, "test")
        create(f).use { da ->
            da.ensureCapacity(1000)
            da.setInt(0, 42)
            da.setInt(132, -7) // nel secondo segmento (segmenti da 128 byte)
            da.setShort(300, 12345)
            da.setLong(400, 1L shl 40)
            da.setBytes(126, byteArrayOf(1, 2, 3, 4), 4) // a cavallo del confine di segmento
            da.setHeader(0, 99); da.setHeader(19, -1)
            assertEquals(42, da.getInt(0))
            assertEquals(-7, da.getInt(132))
            assertEquals(12345.toShort(), da.getShort(300))
            assertEquals(1L shl 40, da.getLong(400))
            val buf = ByteArray(4); da.getBytes(126, buf, 4)
            assertTrue(buf.contentEquals(byteArrayOf(1, 2, 3, 4)))
            assertEquals(1024, da.capacity) // 1000 arrotondato a 8 segmenti da 128
            da.flush()
        }
        val re = reopen(f) ?: error("riapertura fallita")
        re.use { da ->
            assertEquals(99, da.getHeader(0)); assertEquals(-1, da.getHeader(19))
            assertEquals(42, da.getInt(0)); assertEquals(-7, da.getInt(132))
            assertEquals(1L shl 40, da.getLong(400))
            val buf = ByteArray(4); da.getBytes(126, buf, 4)
            assertTrue(buf.contentEquals(byteArrayOf(1, 2, 3, 4)))
            assertEquals(1024, da.capacity)
        }
    }

    @Test
    fun `ram data access round trip`() = exercise(
        { RamDataAccess("test", it, 128) },
        { RamDataAccess("test", it, 128).takeIf { da -> da.loadExisting() } },
    )

    @Test
    fun `mmap data access round trip`() = exercise(
        { MmapDataAccess("test", it, 128) },
        { MmapDataAccess("test", it, 128, readOnly = true).takeIf { da -> da.loadExisting() } },
    )

    @Test
    fun `mmap data access in chunks of several segments`() {
        // 2 segments per mapping (384 bytes: 3; 128: one per segment; 1 MB: the whole file in one), the last one shorter
        for (chunk in listOf(256, 384, 128, 1 shl 20)) {
            exercise(
                { MmapDataAccess("test", it, 128) },
                { MmapDataAccess("test", it, 128, readOnly = true, chunkBytes = chunk).takeIf { da -> da.loadExisting() } },
            )
        }
        // every int of a file read back through the chunks, the warm-up over all of it, and growing a file opened in chunks
        val f = File(tmpDir(), "big")
        MmapDataAccess("big", f, 128).use { da -> da.ensureCapacity(128L * 11); for (i in 0 until 128 * 11 / 4) da.setInt(i * 4L, i * 31 + 7); da.flush() }
        MmapDataAccess("big", f, 128, chunkBytes = 512).use { da ->
            assertTrue(da.loadExisting())
            for (i in 0 until 128 * 11 / 4) assertEquals(i * 31 + 7, da.getInt(i * 4L))
            val ints = IntArray(100); da.getInts(4L * 20, ints, 0, 100)
            for (k in 0 until 100) assertEquals((20 + k) * 31 + 7, ints[k])
            var last = 0L
            assertEquals(128L * 11, da.touch { last = it })
            assertEquals(128L * 11, last)
            da.ensureCapacity(128L * 13); da.setInt(128L * 12, 5); da.setInt(0, -1); da.flush()
        }
        MmapDataAccess("big", f, 128, readOnly = true, chunkBytes = 256).use { da ->
            assertTrue(da.loadExisting())
            assertEquals(128L * 13, da.capacity); assertEquals(5, da.getInt(128L * 12)); assertEquals(-1, da.getInt(0)); assertEquals(38, da.getInt(4))
        }
    }

    @Test
    fun `ram file can be read with mmap and vice versa`() {
        val dir = tmpDir()
        val f = File(dir, "x")
        RamDataAccess("x", f, 256).use { it.ensureCapacity(10); it.setInt(8, 777); it.flush() }
        MmapDataAccess("x", f, 256).use { assertTrue(it.loadExisting()); assertEquals(777, it.getInt(8)) }
    }

    @Test
    fun `missing file is reported`() {
        val dir = tmpDir()
        assertFalse(RamDataAccess("none", File(dir, "none")).loadExisting())
        assertEquals(null, GraphDirectory(dir).open("none"))
    }
}
