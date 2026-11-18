package it.navmaster.engine.osm

import com.carrotsearch.hppc.IntIntHashMap
import it.navmaster.engine.profile.NodeDelays
import it.navmaster.engine.profile.NodeDelays.Control
import it.navmaster.engine.profile.RoadClass
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import java.util.BitSet
import kotlin.math.min

/**
 * Le perdite di tempo agli incroci (modello NAVMASTER): per ogni nodo del grafo e ogni arco che vi
 * arriva, il controllo che lo rallenta — semaforo, casello, passaggio a livello, stop o precedenza
 * mappati sulla torre, mini-rotatoria, ingresso in rotatoria, o la precedenza implicita di chi entra in
 * una strada più importante che prosegue — e i secondi persi ([NodeDelays.seconds]) alla velocità
 * dell'arco in quel senso, sommati al ritardo già scritto (nodi interni) con tetto a 63 s.
 *
 * Un nodo di grado 2 senza controllo (ponte, cambio di corsie, way spezzata) non costa mai: è la
 * differenza dal modello di accelerazione di ORS, che pagava una partenza e un arrivo per ogni way.
 *
 * Si esegue dopo la lettura delle way e prima dei divieti di svolta e del `freeze` del grafo.
 *
 * @param towerControls nodo del grafo → codice di [NodeDelays.classify]
 * @param nearEnd sensi d'arco (arco·2 + 0 avanti, + 1 indietro) con un controllo esplicito entro 30 m dal capo d'arrivo
 */
class JunctionDelayPass(
    private val graph: BaseGraph,
    private val encoding: GraphEncoding,
    private val towerControls: IntIntHashMap,
    private val nearEnd: BitSet,
) {
    /** Arrivi rallentati per tipo ([Control.ordinal]). */
    val events = IntArray(Control.entries.size)
    /** Archi con un ritardo > 0 in almeno un senso e un profilo (contati a fine passo, anche coi nodi interni). */
    var delayedEdges = 0; private set

    private var cap = 16
    private var edges = IntArray(cap)
    private var ranks = IntArray(cap)
    private var roundabout = BooleanArray(cap)
    private var link = BooleanArray(cap)
    /** Il senso memorizzato che arriva al nodo: false = avanti (il nodo è nodeB), true = indietro. */
    private var arrivesReverse = BooleanArray(cap)
    private var adj = IntArray(cap)
    private var dist = DoubleArray(cap)

    fun run() {
        val shared = encoding.shared
        val profiles = encoding.profiles.filter { it.delay != null }
        if (profiles.isEmpty()) return
        val it = graph.createEdgeIterator()
        val it2 = graph.createEdgeIterator()
        val flags = IntArray(graph.intsForFlags)
        for (node in 0 until graph.nodeCount) {
            var g = 0
            it.setBaseNode(node)
            while (it.next()) {
                if (it.distance <= 0.0) continue // archi delle barriere (lunghezza zero)
                if (g == cap) grow()
                val f = it.flags
                edges[g] = it.edge
                link[g] = shared.roadClassLink.get(f)
                ranks[g] = rank(shared.roadClass.get(f), link[g])
                roundabout[g] = shared.roundabout.get(f)
                arrivesReverse[g] = !it.reverse
                adj[g] = it.adjNode
                dist[g] = it.distance
                g++
            }
            val code = towerControls.getOrDefault(node, 0)
            if (g == 0 || (g < 3 && code == 0)) continue
            var rankMax = 0; var nMax = 0; var anyRoundabout = false; var allSame = true
            for (j in 0 until g) {
                if (ranks[j] > rankMax) { rankMax = ranks[j]; nMax = 1 } else if (ranks[j] == rankMax) nMax++
                if (roundabout[j]) anyRoundabout = true
                if (ranks[j] != ranks[0]) allSame = false
            }
            val c = NodeDelays.control(code)
            val allWay = NodeDelays.allWay(code)
            // un semaforo senza direzione su un nodo di grado 2 (la way spezzata) a pochi metri da un incrocio
            // è quello di un accesso: ferma solo chi arriva dall'altra parte, verso l'incrocio
            var onlyArrival = -1
            if (g == 2 && c == Control.SIGNALS && NodeDelays.direction(code) == NodeDelays.DIR_BOTH) {
                val near0 = dist[0] <= NodeDelays.SIGNAL_APPROACH_M && degree(it2, adj[0]) >= 3
                val near1 = dist[1] <= NodeDelays.SIGNAL_APPROACH_M && degree(it2, adj[1]) >= 3
                if (near0 != near1) onlyArrival = if (near0) 1 else 0
            }
            for (j in 0 until g) {
                if (onlyArrival >= 0 && j != onlyArrival) continue
                val type = when {
                    c == Control.SIGNALS || c == Control.TOLL || c == Control.RAIL || c == Control.CROSSING_SIGNALS -> c
                    (c == Control.STOP || c == Control.GIVE_WAY) && (ranks[j] < rankMax || allSame || allWay) -> c
                    c == Control.MINI_ROUNDABOUT -> Control.MINI_ROUNDABOUT // costa come una precedenza, contata a parte
                    !roundabout[j] && anyRoundabout -> Control.ROUNDABOUT_ENTRY
                    g >= 3 && !roundabout[j] && ranks[j] < rankMax && nMax >= 2 &&
                        !nearEnd.get(edges[j] * 2 + if (arrivesReverse[j]) 1 else 0) &&
                        !(link[j] && rankMax >= TRUNK_RANK) -> Control.IMPLIED_GIVE_WAY
                    else -> Control.NONE
                }
                if (type == Control.NONE) continue
                val e = edges[j]; val rev = arrivesReverse[j]
                graph.getFlags(e, flags)
                var changed = false
                for (pe in profiles) {
                    if (!pe.access.get(flags, rev)) continue
                    val v = pe.speed.get(flags, rev)
                    val sec = NodeDelays.seconds(type, v, pe.profile.acceleration, pe.profile.deceleration)
                    if (sec <= 0) continue
                    val d = pe.delay!!
                    d.set(flags, rev, min(NodeDelays.MAX_SECONDS.toDouble(), Math.round(d.get(flags, rev) + sec).toDouble()))
                    changed = true
                }
                if (changed) { graph.setFlags(e, flags); events[type.ordinal]++ }
            }
        }
        for (e in 0 until graph.edgeCount) {
            graph.getFlags(e, flags)
            if (profiles.any { pe -> pe.delay!!.get(flags, false) > 0 || pe.delay!!.get(flags, true) > 0 }) delayedEdges++
        }
    }

    /** Gli archi veri (non quelli di lunghezza zero delle barriere) di [node]. */
    private fun degree(it: BaseGraph.EdgeIterator, node: Int): Int {
        var d = 0
        it.setBaseNode(node)
        while (it.next()) if (it.distance > 0.0) d++
        return d
    }

    private fun grow() {
        cap *= 2
        edges = edges.copyOf(cap); ranks = ranks.copyOf(cap); roundabout = roundabout.copyOf(cap)
        link = link.copyOf(cap); arrivesReverse = arrivesReverse.copyOf(cap); adj = adj.copyOf(cap); dist = dist.copyOf(cap)
    }

    companion object {
        const val TRUNK_RANK = 12

        /** Rango di una strada per la precedenza implicita: chi arriva da un rango più basso cede il passo. */
        fun rank(rc: RoadClass, isLink: Boolean): Int {
            if (isLink) return 5
            return when (rc) {
                RoadClass.MOTORWAY -> 14
                RoadClass.TRUNK -> TRUNK_RANK
                RoadClass.PRIMARY -> 10
                RoadClass.SECONDARY -> 8
                RoadClass.TERTIARY -> 6
                RoadClass.UNCLASSIFIED, RoadClass.RESIDENTIAL -> 4
                else -> 2
            }
        }
    }
}
