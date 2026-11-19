package it.navmaster.cli

import it.navmaster.engine.profile.SpeedModel
import java.io.File
import java.nio.file.Files
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertNull
import kotlin.test.assertTrue

/** `--dem` della CLI: solo col modello navmaster (il confronto con ORS non deve mai avere il terreno). */
class DemArgsTest {
    private val dir: File = Files.createTempDirectory("nme-dem-args").toFile().also { it.deleteOnExit() }

    @Test
    fun `--dem with --speed-model=ors is refused`() {
        val args = listOf("import", "a.osm.pbf", "g", "--speed-model=ors", "--dem=${dir.path}")
        val profiles = profilesFromArgs(args)
        assertTrue(profiles.all { it.options.speedModel == SpeedModel.ORS })
        val e = assertFailsWith<IllegalArgumentException> { demFromArgs(args, profiles) }
        assertTrue(e.message!!.contains("navmaster"), e.message)
    }

    @Test
    fun `--dem with the navmaster model - the folder, its SOURCE file or --dem-source`() {
        val args = listOf("import", "a.osm.pbf", "g", "--dem=${dir.path}")
        val profiles = profilesFromArgs(args)
        assertEquals(dir.path to "hgt", demFromArgs(args, profiles).let { it.first?.path to it.second })
        File(dir, "SOURCE").writeText("copernicus-glo90\n")
        assertEquals("copernicus-glo90", demFromArgs(args, profiles).second)
        assertEquals("skadi", demFromArgs(args + "--dem-source=skadi", profiles).second)
        // senza --dem nessun terreno, con qualunque modello
        val ors = listOf("import", "a.osm.pbf", "g", "--speed-model=ors")
        assertNull(demFromArgs(ors, profilesFromArgs(ors)).first)
        // una cartella che non c'è
        assertFailsWith<IllegalArgumentException> { demFromArgs(listOf("--dem=${dir.path}/manca"), profiles) }
    }
}
