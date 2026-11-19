package it.navmaster.engine

/**
 * Version of the on-disk graph format written by this engine. Bump whenever a storage layout changes.
 *
 * 5 (November 2026): the compact files: names deduplicated, `turn_index` (bitmap of the nodes with turn
 * restrictions) instead of `turn_heads`, `guidance` sparse (only the edges with data), `geometry` as
 * zigzag varint deltas. Engines before it refuse these graphs («formato grafo incompatibile»); this
 * engine still reads the graphs of version 4 ([GRAPH_FORMAT_OLDEST]) with their old layouts.
 */
const val GRAPH_FORMAT_VERSION = 5

/** The oldest graph format this engine still reads. */
const val GRAPH_FORMAT_OLDEST = 4

/** True if a file header or `graph.format_version` with version [v] can be read by this engine. */
fun isReadableGraphFormat(v: Int): Boolean = v in GRAPH_FORMAT_OLDEST..GRAPH_FORMAT_VERSION

object Engine {
    const val NAME = "navmaster-engine"
    const val VERSION = "0.1.0"
}
