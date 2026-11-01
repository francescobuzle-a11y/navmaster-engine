package it.navmaster.engine.osm

import com.carrotsearch.hppc.LongIntHashMap
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.TurnCostStorage

/**
 * Traduce le relazioni `type=restriction` in costi di svolta (GraphHopper `OSMTurnRelationParser`):
 * - divieti (`no_*`): vietata la svolta (from, via, to);
 * - obblighi (`only_*`): vietate tutte le altre uscite dal nodo via, tranne il ritorno sull'arco di provenienza;
 * - `restriction:<veicolo>` vale solo per i profili che riconoscono quel veicolo, `except=` li esonera.
 * Il nodo via deve essere una torre; le restrizioni con via-way non sono supportate (come in GraphHopper).
 */
class TurnRestrictionApplier(private val graph: BaseGraph, private val encoding: GraphEncoding, private val turnCosts: TurnCostStorage) {
    /** Valori di `restriction:<veicolo>` / `except` riconosciuti da ciascun profilo (ordine di `OSMRoadAccessParser.toOSMRestrictions`). */
    private val vehicleKeys: Map<String, List<String>> = mapOf(
        "car-ors" to listOf("motorcar", "motor_vehicle", "vehicle"),
        "heavyvehicle" to listOf("hgv", "motor_vehicle", "vehicle"),
    )

    var applied = 0; private set
    var skipped = 0; private set

    fun apply(relations: List<TurnRestrictionRelation>, osmNodeToTower: LongIntHashMap) {
        val it = graph.createEdgeIterator()
        val osmWayId = encoding.shared.osmWayId
        for (r in relations) {
            val via = osmNodeToTower.getOrDefault(r.viaNode, -1)
            if (via < 0) { skipped++; continue }
            val type = restrictionType(r.restriction) ?: run { skipped++; null } ?: continue
            // archi al nodo via che appartengono alle way from/to (una way può avere più archi al nodo solo in casi degeneri)
            var fromEdge = -1; var toEdge = -1
            it.setBaseNode(via)
            while (it.next()) {
                val wid = it.get(osmWayId).toLong()
                if (wid == r.fromWay && fromEdge < 0) fromEdge = it.edge
                else if (wid == r.toWay && toEdge < 0) toEdge = it.edge
            }
            if (fromEdge < 0 || (type == Type.NOT && toEdge < 0)) { skipped++; continue }
            var any = false
            for (pe in encoding.profiles) {
                val name = pe.profile.name
                val keys = vehicleKeys[name] ?: continue
                if (r.vehicle != null && r.vehicle !in keys) continue       // restrizione per un altro veicolo
                if (r.except.any { it in keys }) continue                      // profilo esonerato
                when (type) {
                    Type.NOT -> turnCosts.set(fromEdge, via, toEdge, turnCosts.encoding.forbid(0, name))
                    Type.ONLY -> {
                        it.setBaseNode(via)
                        while (it.next()) {
                            if (it.edge == fromEdge || it.get(osmWayId).toLong() == r.toWay) continue
                            turnCosts.set(fromEdge, via, it.edge, turnCosts.encoding.forbid(0, name))
                        }
                    }
                }
                any = true
            }
            if (any) applied++ else skipped++
        }
    }

    private enum class Type { NOT, ONLY }

    private fun restrictionType(value: String): Type? = when (value) {
        "no_left_turn", "no_right_turn", "no_straight_on", "no_u_turn", "no_entry", "no_exit" -> Type.NOT
        "only_right_turn", "only_left_turn", "only_straight_on" -> Type.ONLY
        else -> null
    }
}
