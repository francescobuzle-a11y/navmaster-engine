package it.navmaster.engine.routing

import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GuidanceStore
import it.navmaster.engine.storage.SharedEncoding

/**
 * I fatti di un arco che servono alle varianti: la classe (autostrada, superstrada, rampa) e il
 * `ref` normalizzato ([normalizeRef]). Letti una volta per arco e tenuti (gli archi dei percorsi si
 * ripetono fra il migliore e le varianti); le stringhe `ref` lette una volta per puntatore. Solo
 * sugli archi dei percorsi, mai dentro una ricerca. Un oggetto per richiesta.
 */
class EdgeInfo(private val graph: BaseGraph, private val sh: SharedEncoding, private val guidance: GuidanceStore?) {
    private val it = graph.createEdgeIterator()
    /** arco → (indice del ref + 1) shl [SHIFT] | classe ([MW], [TR], [LINK]); 0 = non letto ([READ] sempre presente). */
    private val codes = IntIntHashMap(4096)
    /** puntatore della stringa → indice del ref + 1 (0: nessun ref valido). */
    private val byPtr = IntIntHashMap(256)
    private val refs = ArrayList<String>()
    private val refIds = HashMap<String, Int>()

    @Synchronized private fun code(edge: Int): Int {
        val c = codes.getOrDefault(edge, 0)
        if (c != 0) return c
        it.setEdge(edge, graph.nodeA(edge))
        var bits = READ
        when (it.get(sh.roadClass)) {
            RoadClass.MOTORWAY -> bits = bits or MW
            RoadClass.TRUNK -> bits = bits or TR
            else -> {}
        }
        if (it.get(sh.roadClassLink)) bits = bits or LINK
        val ptr = guidance?.refPointer(edge) ?: 0
        val id1 = if (ptr == 0) 0 else {
            val known = byPtr.getOrDefault(ptr, -1)
            if (known >= 0) known else {
                val r = normalizeRef(guidance!!.string(ptr))
                val id = if (r == null) 0 else refIds.getOrPut(r) { refs += r; refs.size - 1 } + 1
                byPtr.put(ptr, id)
                id
            }
        }
        val v = (id1 shl SHIFT) or bits
        codes.put(edge, v)
        return v
    }

    /** `highway=motorway|trunk` (rampe comprese, `*_link`). */
    fun isMotorwayOrTrunk(edge: Int): Boolean = code(edge) and (MW or TR) != 0
    /** `highway=motorway` o `motorway_link`. */
    fun isMotorway(edge: Int): Boolean = code(edge) and MW != 0
    /** Una rampa (`*_link`). */
    fun isLink(edge: Int): Boolean = code(edge) and LINK != 0

    /** Il `ref` normalizzato dell'arco ([normalizeRef]), null se nessuno. */
    fun ref(edge: Int): String? = refId(edge).let { if (it < 0) null else refName(it) }
    /** L'indice del `ref` normalizzato (stesso ref → stesso indice in questo oggetto), -1 se nessuno. */
    fun refId(edge: Int): Int = (code(edge) ushr SHIFT) - 1
    @Synchronized fun refName(id: Int): String = refs[id]
    /** L'indice di un ref già normalizzato, -1 se nessun arco letto lo ha. */
    @Synchronized fun idOf(ref: String): Int = refIds[ref] ?: -1

    companion object {
        private const val READ = 1 shl 0
        private const val MW = 1 shl 1
        private const val TR = 1 shl 2
        private const val LINK = 1 shl 3
        private const val SHIFT = 4
        private val TOKEN = Regex("^[A-Z]{1,3}\\d+[A-Z]?$")

        /**
         * Il ref principale di una strada: "A1, E35"→"A1"; "A 5"→"A5"; "A4;A22"→"A4"; solo europei →
         * il primo ("E35"); nessun ref valido → null. Un token vale se, tolti gli spazi interni e in
         * maiuscolo, è `^[A-Z]{1,3}\d+[A-Z]?$`; gli europei (E + cifre) solo in mancanza d'altro.
         */
        fun normalizeRef(raw: String): String? {
            if (raw.isBlank()) return null
            var european: String? = null
            for (part in raw.split(';', ',', '/')) {
                val t = part.replace(" ", "").replace(" ", "").uppercase()
                if (!TOKEN.matches(t)) continue
                if (t[0] == 'E' && t[1].isDigit()) { if (european == null) european = t; continue }
                return t
            }
            return european
        }
    }
}
