package it.navmaster.engine

import kotlin.test.Test
import kotlin.test.assertEquals

class EngineTest {
    @Test
    fun `format version starts at 1`() {
        assertEquals(1, GRAPH_FORMAT_VERSION)
    }
}
