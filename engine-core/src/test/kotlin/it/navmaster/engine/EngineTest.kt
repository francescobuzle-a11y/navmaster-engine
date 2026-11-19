package it.navmaster.engine

import kotlin.test.Test
import kotlin.test.assertEquals

class EngineTest {
    @Test
    fun `format version is 4`() {
        assertEquals(5, GRAPH_FORMAT_VERSION)
    }
}
