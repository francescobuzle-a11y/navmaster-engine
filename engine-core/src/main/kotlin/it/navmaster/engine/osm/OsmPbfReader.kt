package it.navmaster.engine.osm

import crosby.binary.BinaryParser
import crosby.binary.Osmformat
import crosby.binary.file.BlockInputStream
import java.io.BufferedInputStream
import java.io.File

enum class MemberType { NODE, WAY, RELATION }

class RelationMember(val type: MemberType, val ref: Long, val role: String)

/** Riceve gli elementi di un file OSM nell'ordine del file (nodi, way, relazioni). */
interface OsmSink {
    /** [tags] è null quando il nodo non ha tag utili. */
    fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) {}
    fun way(way: OsmWay) {}
    fun relation(id: Long, tags: Map<String, String>, members: List<RelationMember>) {}
}

/**
 * Lettore di file `.osm.pbf`. Un file va letto più volte (una per passata): ogni chiamata a
 * [read] riparte dall'inizio. I tag inutili per il routing vengono scartati per risparmiare memoria.
 */
class OsmPbfReader(private val file: File) {
    var nodeCount = 0L; private set
    var wayCount = 0L; private set
    var relationCount = 0L; private set

    fun read(sink: OsmSink, nodes: Boolean = true, ways: Boolean = true, relations: Boolean = true) {
        require(file.exists()) { "file OSM non trovato: $file" }
        nodeCount = 0; wayCount = 0; relationCount = 0
        BufferedInputStream(file.inputStream(), 1 shl 16).use { input ->
            BlockInputStream(input, Parser(sink, nodes, ways, relations)).process()
        }
    }

    private inner class Parser(val sink: OsmSink, val nodes: Boolean, val ways: Boolean, val relations: Boolean) : BinaryParser() {
        override fun parse(header: Osmformat.HeaderBlock) {
            for (f in header.requiredFeaturesList) {
                require(f == "OsmSchema-V0.6" || f == "DenseNodes") { "funzionalità PBF non supportata: $f" }
            }
        }

        override fun parseDense(dense: Osmformat.DenseNodes) {
            if (!nodes) { nodeCount += dense.idCount; return }
            var id = 0L; var lat = 0L; var lon = 0L
            var kv = 0
            val hasKv = dense.keysValsCount > 0
            for (i in 0 until dense.idCount) {
                id += dense.getId(i); lat += dense.getLat(i); lon += dense.getLon(i)
                var tags: HashMap<String, String>? = null
                if (hasKv) {
                    while (kv < dense.keysValsCount) {
                        val k = dense.getKeysVals(kv)
                        if (k == 0) { kv++; break }
                        val v = dense.getKeysVals(kv + 1)
                        kv += 2
                        val key = getStringById(k)
                        if (keepNodeTag(key)) (tags ?: HashMap<String, String>(4).also { tags = it })[key] = getStringById(v)
                    }
                }
                nodeCount++
                sink.node(id, parseLat(lat), parseLon(lon), tags)
            }
        }

        override fun parseNodes(list: List<Osmformat.Node>) {
            nodeCount += list.size
            if (!nodes) return
            for (n in list) {
                var tags: HashMap<String, String>? = null
                for (i in 0 until n.keysCount) {
                    val key = getStringById(n.getKeys(i))
                    if (keepNodeTag(key)) (tags ?: HashMap<String, String>(4).also { tags = it })[key] = getStringById(n.getVals(i))
                }
                sink.node(n.id, parseLat(n.lat), parseLon(n.lon), tags)
            }
        }

        override fun parseWays(list: List<Osmformat.Way>) {
            wayCount += list.size
            if (!ways) return
            for (w in list) {
                val refs = LongArray(w.refsCount)
                var ref = 0L
                for (i in 0 until w.refsCount) { ref += w.getRefs(i); refs[i] = ref }
                val tags = HashMap<String, String>(w.keysCount * 2)
                for (i in 0 until w.keysCount) {
                    val key = getStringById(w.getKeys(i))
                    if (keepWayTag(key)) tags[key] = getStringById(w.getVals(i))
                }
                sink.way(OsmWay(w.id, tags, refs))
            }
        }

        override fun parseRelations(list: List<Osmformat.Relation>) {
            relationCount += list.size
            if (!relations) return
            for (r in list) {
                val tags = HashMap<String, String>(r.keysCount * 2)
                for (i in 0 until r.keysCount) tags[getStringById(r.getKeys(i))] = getStringById(r.getVals(i))
                val members = ArrayList<RelationMember>(r.memidsCount)
                var mid = 0L
                for (i in 0 until r.memidsCount) {
                    mid += r.getMemids(i)
                    val type = when (r.getTypes(i)) {
                        Osmformat.Relation.MemberType.NODE -> MemberType.NODE
                        Osmformat.Relation.MemberType.WAY -> MemberType.WAY
                        else -> MemberType.RELATION
                    }
                    members += RelationMember(type, mid, getStringById(r.getRolesSid(i)))
                }
                sink.relation(r.id, tags, members)
            }
        }

        override fun complete() {}
    }

    companion object {
        /** Dei nodi servono i tag che influenzano la percorribilità (barriere, guadi), i limiti puntuali e gli svincoli (`ref`, `name`). */
        fun keepNodeTag(key: String): Boolean =
            key == "barrier" || key == "ford" || key == "highway" || key == "locked" || key == "access" || key == "ref" || key == "name" ||
                key == "direction" || key.startsWith("maxspeed") ||
                key == "motor_vehicle" || key == "motorcar" || key == "vehicle" || key == "hgv" || key == "goods" ||
                key == "bus" || key == "delivery" || key == "agricultural" || key == "forestry" ||
                key.startsWith("maxheight") || key.startsWith("maxwidth") || key.startsWith("maxweight") ||
                key.startsWith("maxlength") || key.startsWith("maxaxleload") || key == "traffic_signals" || key == "crossing"

        fun keepWayTag(key: String): Boolean =
            !(key == "created_by" || key.startsWith("addr:") || key.startsWith("TMC") || key.startsWith("tiger:") ||
                key.startsWith("note") || key.startsWith("source") || key.startsWith("wikipedia") || key.startsWith("wikidata") ||
                key.startsWith("description") || key.startsWith("old_name") || key.startsWith("alt_name") || key.startsWith("official_name"))
    }
}
