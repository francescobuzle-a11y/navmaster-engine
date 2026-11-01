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
