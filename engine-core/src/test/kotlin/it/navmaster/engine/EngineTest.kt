package it.navmaster.engine

import kotlin.test.Test
import kotlin.test.assertEquals

class EngineTest {
    @Test
    fun `format version is 2`() {
        assertEquals(2, GRAPH_FORMAT_VERSION)
    }
}
