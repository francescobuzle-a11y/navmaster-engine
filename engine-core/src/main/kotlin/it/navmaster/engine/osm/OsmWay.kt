package it.navmaster.engine.osm

/**
 * Una way OSM con i suoi tag, così come la vede la fase di import.
 *
 * Le funzioni di ricerca sui tag riproducono la semantica di GraphHopper/ORS:
 * un valore può contenere più voci separate da `;` ("hgv;goods") e ciascuna
 * voce viene confrontata singolarmente.
 */
class OsmWay(
    val id: Long,
    val tags: Map<String, String>,
    val nodeIds: LongArray,
) {
    fun hasTag(key: String): Boolean = tags.containsKey(key)

    fun tag(key: String): String? = tags[key]

    fun tag(key: String, default: String): String = tags[key] ?: default

    /** Valori del tag, separati su `;`. Vuoto se il tag manca. */
    fun tagValues(key: String): List<String> {
        val v = tags[key] ?: return emptyList()
        return if (';' in v) v.split(';').map { it.trim() } else listOf(v)
    }

    /** Vero se il tag esiste e, se sono dati dei valori, se almeno una voce coincide. */
    fun hasTag(key: String, vararg values: String): Boolean {
        val v = tags[key] ?: return false
        if (values.isEmpty()) return true
        if (';' !in v) return values.contains(v)
        return tagValues(key).any { values.contains(it) }
    }

    fun hasTag(key: String, values: Collection<String>): Boolean {
        if (!tags.containsKey(key)) return false
        return tagValues(key).any { values.contains(it) }
    }

    /** Vero se almeno una delle chiavi ha uno dei valori. */
    fun hasTag(keys: List<String>, values: Collection<String>): Boolean = keys.any { hasTag(it, values) }

    /** Valore della prima chiave presente nell'ordine dato, altrimenti stringa vuota. */
    fun firstPriorityTag(keys: List<String>): String {
        for (k in keys) tags[k]?.let { return it }
        return ""
    }

    /** Voci (separate su `;`) della prima chiave presente nell'ordine dato. */
    fun firstPriorityTagValues(keys: List<String>): List<String> {
        for (k in keys) if (tags.containsKey(k)) return tagValues(k)
        return emptyList()
    }
}
