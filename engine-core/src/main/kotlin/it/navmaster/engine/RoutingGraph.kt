package it.navmaster.engine

import it.navmaster.engine.ch.CHGraph
import it.navmaster.engine.ch.CHPreparation
import it.navmaster.engine.ch.CHStorage
import it.navmaster.engine.ch.CoreEdgeSelector
import it.navmaster.engine.ch.CoreLandmarks
import it.navmaster.engine.index.LocationIndex
import it.navmaster.engine.weighting.Weighting
import it.navmaster.engine.osm.ImportOptions
import it.navmaster.engine.osm.ImportStats
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.VehicleProfile
import it.navmaster.engine.routing.Router
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GuidanceStore
import it.navmaster.engine.storage.PointStore
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.GraphProperties
import it.navmaster.engine.storage.TurnCostEncoding
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.weighting.EdgeFilter
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.weighting.EdgeFilterFactory
import it.navmaster.engine.weighting.TurnCostProvider
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.Closeable
import java.io.File

/** Opzioni di una richiesta di percorso, nel vocabolario di ORS. */
data class RouteOptions(
    val profile: String = "driving-hgv",
    val weighting: WeightingMethod = WeightingMethod.RECOMMENDED,
    val avoidFeatures: Int = 0,
    val vehicleType: Int = HeavyVehicleAttributes.HGV,
    val vehicleParameters: VehicleParameters? = null,
    /** Poligoni da evitare (lon/lat); come in ORS disattivano le preparazioni e usano la ricerca piena. */
    val avoidPolygons: List<org.locationtech.jts.geom.Polygon> = emptyList(),
    /** Velocità massima in km/h (≥ `maximum_speed_lower_bound`, 80): gli archi più veloci vengono rallentati. */
    val maximumSpeed: Double = Double.POSITIVE_INFINITY,
    /** Vero per usare divieti di svolta e inversioni (ORS: profili con `turn_costs: true`). */
    val turnCosts: Boolean = true,
    /**
     * Consente la Contraction Hierarchy quando la richiesta non ha restrizioni né avoid_features.
     * Come la modalità "static" di ORS, la CH node-based ignora i divieti di svolta.
     */
    val allowCH: Boolean = true,
    val maximumSnappingRadius: Double = 400.0,
    /** ORS `continue_straight`: come in ORS forza la modalità flessibile (niente CH node-based). */
    val continueStraight: Boolean = false,
) {
    /** Richiesta "statica" nel senso di ORS: nessun filtro dipendente dalla richiesta. */
    val isStatic: Boolean get() = avoidFeatures == 0 && (vehicleParameters == null || !vehicleParameters.hasAttributes) &&
        vehicleType == HeavyVehicleAttributes.HGV && avoidPolygons.isEmpty() && maximumSpeed.isInfinite() && !continueStraight

    /**
     * Niente CH né Core: i poligoni possono toccare qualunque arco (ORS "non decreasing weights");
     * `avoid_features` e `maximum_speed` toccano archi fuori dal nostro core (vedi [CoreEdgeSelector]).
     */
    val needsFullSearch: Boolean get() = avoidPolygons.isNotEmpty() || avoidFeatures != 0 || maximumSpeed.isFinite()
}

/**
 * Un grafo pronto al calcolo: storage, costi di svolta, codifica dei profili e indice spaziale,
 * caricati o costruiti insieme da una cartella.
 */
class RoutingGraph private constructor(
    val dir: File,
    val encoding: GraphEncoding,
    val graph: BaseGraph,
    val turnCosts: TurnCostStorage,
    /** Dati di guida per arco (file `guidance`), assenti nei grafi costruiti senza. */
    val guidance: GuidanceStore?,
    /** Autovelox, Tutor, fotocamere ai semafori (file `points`). */
    val points: PointStore? = null,
) : Closeable {
    lateinit var index: LocationIndex; private set

    /** CH disponibili, per chiave `<encoder>_<weighting>` (es. `heavyvehicle_recommended`). */
    val chStorages = LinkedHashMap<String, CHStorage>()
    /** Landmark del core, per chiave della preparazione core. */
    val landmarks = LinkedHashMap<String, CoreLandmarks>()

    val profileNames: List<String> get() = encoding.profiles.map { it.profile.name }

    /** Pesi su cui preparare la CH per ciascun profilo (ORS: auto `fastest`, camion `recommended`). */
    private fun defaultCHWeighting(encoder: String): WeightingMethod = if (encoder == "car-ors") WeightingMethod.FASTEST else WeightingMethod.RECOMMENDED

    private fun chKey(encoder: String, weighting: Weighting) = "${encoder}_${weighting.name}"

    /**
     * Prepara per ogni profilo la CH completa (richieste "statiche") e il Core (richieste con restrizioni,
     * avoid_features o divieti di svolta), come le preparazioni `ch` e `core` di ORS, e le salva.
     */
    fun prepareCH(log: ((String) -> Unit)? = null, withCore: Boolean = true, landmarkCount: Int = 16) {
        for (pe in encoding.profiles) for (method in listOf(defaultCHWeighting(pe.profile.name), WeightingMethod.SHORTEST)) {
            val name = pe.profile.name
            val w = WeightingFactory.create(encoding, pe, method)
            val key = chKey(name, w)
            val storage = CHStorage(GraphDirectory(dir, mmap = false), key, graph.edgeCount, graph.nodeCount)
            var t0 = System.currentTimeMillis()
            CHPreparation(graph, w, storage, params = CHPreparation.Params(log = log)).prepare()
            storage.flush()
            log?.invoke("CH $key: ${storage.shortcutCount} shortcut in ${System.currentTimeMillis() - t0} ms")
            chStorages[key] = storage
            // il core solo per il peso principale del profilo (come ORS): "shortest" con restrizioni usa la ricerca piena
            if (withCore && method != WeightingMethod.SHORTEST) {
                val selector = CoreEdgeSelector(encoding, name, turnCosts)
                val coreNodes = selector.coreNodes(graph)
                val coreKey = "${key}_core"
                val coreStorage = CHStorage(GraphDirectory(dir, mmap = false), coreKey, graph.edgeCount, graph.nodeCount)
                t0 = System.currentTimeMillis()
                val prep = CHPreparation(graph, w, coreStorage, filter = EdgeFilter { !selector.isCoreEdge(it) }, params = CHPreparation.Params(log = log))
                    .apply { doNotContract = { coreNodes[it] } }
                prep.prepare()
                if (prep.keptInCore > 0) log?.invoke("Core $coreKey: ${prep.keptInCore} nodi lasciati nel core per troppi shortcut")
                coreStorage.flush()
                log?.invoke("Core $coreKey: ${coreStorage.coreNodeCount} nodi nel core (${100 * coreStorage.coreNodeCount / maxOf(1, graph.nodeCount)} %), ${coreStorage.shortcutCount} shortcut in ${System.currentTimeMillis() - t0} ms")
                chStorages[coreKey] = coreStorage
                t0 = System.currentTimeMillis()
                val lm = CoreLandmarks.prepare(CHGraph(graph, coreStorage, w), w, landmarkCount)
                lm.save(GraphDirectory(dir, mmap = false), coreKey)
                landmarks[coreKey] = lm
                log?.invoke("Landmark $coreKey: ${lm.landmarkCount} in ${System.currentTimeMillis() - t0} ms")
            }
        }
        GraphProperties(File(dir, "profiles")).apply { load(); set("profiles", profileNames.joinToString(",")); set("ch", chStorages.keys.joinToString(",")); save() }
    }

    /** Nome del profilo ORS (`driving-hgv`) → nome interno dell'encoder (`heavyvehicle`). */
    fun encoderName(profile: String): String = when (profile) {
        "driving-car", "car", "car-ors" -> "car-ors"
        "driving-hgv", "hgv", "truck", "heavyvehicle" -> "heavyvehicle"
        else -> throw IllegalArgumentException("profilo sconosciuto: $profile")
    }

    fun router(options: RouteOptions): Router {
        val name = encoderName(options.profile)
        val pe = encoding.profile(name)
        val weighting = WeightingFactory.create(encoding, pe, options.weighting, options.maximumSpeed)
        val filter: EdgeFilter = EdgeFilterFactory.create(encoding, name, options.avoidFeatures, options.vehicleType, options.vehicleParameters, options.avoidPolygons)
        val tcp = if (options.turnCosts) TurnCostProvider(turnCosts, name) else null
        val key = chKey(name, weighting)
        // come `RoutingRequest.setSpeedups` di ORS: CH per le richieste statiche, altrimenti Core se preparato, altrimenti Dijkstra
        val ch = if (options.allowCH && options.isStatic) chStorages[key]?.let { CHGraph(graph, it, weighting) } else null
        val core = if (ch == null && options.allowCH && !options.needsFullSearch) chStorages["${key}_core"]?.let { CHGraph(graph, it, weighting) } else null
        val lm = if (core != null) landmarks["${key}_core"] else null
        return Router(graph, index, weighting, filter, tcp, ch, core, lm)
    }

    fun flush() { graph.flush(); turnCosts.flush(); guidance?.flush(graph.edgeCount); points?.flush(); GraphProperties(File(dir, "profiles")).apply { load(); set("profiles", profileNames.joinToString(",")); save() } }

    override fun close() { graph.close(); turnCosts.close(); guidance?.close(); points?.close(); chStorages.values.forEach { it.close() } }

    companion object {
        fun profileByName(name: String): VehicleProfile = when (name) {
            "car", "driving-car", "car-ors" -> CarProfile()
            "hgv", "driving-hgv", "truck", "heavyvehicle" -> HgvProfile()
            else -> throw IllegalArgumentException("profilo sconosciuto: $name")
        }

        /** Importa un PBF in [dir] con i profili dati e lascia il grafo pronto. */
        fun build(pbf: File, dir: File, profiles: List<VehicleProfile>, mmap: Boolean = false, importOptions: ImportOptions = ImportOptions()): Pair<RoutingGraph, ImportStats> {
            val encoding = GraphEncoding(profiles)
            val gdir = GraphDirectory(dir, mmap)
            val graph = BaseGraph(gdir, encoding.registry)
            val tc = TurnCostStorage(gdir, TurnCostEncoding(profiles.map { it.name }))
            val guidance = if (importOptions.guidance) GuidanceStore(gdir.create("guidance"), graph.names) else null
            val points = if (importOptions.guidance) PointStore(gdir.create("points")) else null
            val rg = RoutingGraph(dir, encoding, graph, tc, guidance, points)
            val stats = OsmGraphImporter(graph, encoding, tc, importOptions, guidance, points).import(pbf)
            rg.flush()
            rg.index = LocationIndex.build(graph)
            rg.index.save(gdir)
            return rg to stats
        }

        /** Apre un grafo esistente; i profili sono letti dal file `profiles` della cartella. */
        fun load(dir: File, mmap: Boolean = true): RoutingGraph {
            val props = GraphProperties(File(dir, "profiles"))
            require(props.load()) { "nessun grafo in $dir (manca il file profiles)" }
            val profiles = props.get("profiles")!!.split(',').map { profileByName(it) }
            val encoding = GraphEncoding(profiles)
            val gdir = GraphDirectory(dir, mmap, readOnly = mmap)
            val graph = BaseGraph(gdir, encoding.registry)
            check(graph.loadExisting()) { "grafo non caricabile da $dir" }
            val tc = TurnCostStorage(gdir, TurnCostEncoding(profiles.map { it.name }))
            check(tc.loadExisting()) { "costi di svolta non caricabili da $dir" }
            val rg = RoutingGraph(dir, encoding, graph, tc, GuidanceStore.load(gdir, graph.names), PointStore.load(gdir))
            for (key in (props.get("ch") ?: "").split(',').filter { it.isNotBlank() }) {
                val st = CHStorage(gdir, key, graph.edgeCount, graph.nodeCount)
                if (st.loadExisting()) rg.chStorages[key] = st else st.close()
                if (key.endsWith("_core")) CoreLandmarks.load(gdir, key, graph.nodeCount)?.let { rg.landmarks[key] = it }
            }
            rg.index = LocationIndex.load(graph, gdir) ?: LocationIndex.build(graph)
            return rg
        }
    }
}
