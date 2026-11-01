package it.navmaster.engine

import kotlin.test.Test
import kotlin.test.assertEquals

class EngineTest {
    @Test
    fun `format version is 3`() {
        assertEquals(3, GRAPH_FORMAT_VERSION)
    }
}
