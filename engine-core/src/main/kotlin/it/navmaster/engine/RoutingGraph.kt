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
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.weighting.EdgeFilterFactory
import it.navmaster.engine.weighting.AvoidFeaturesEdgeFilter
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
    /** ORS `bearings`: come in ORS (`requiresNonDecreasingWeights`) niente CH node-based. */
    val hasBearings: Boolean = false,
    /**
     * [maximumSpeed] (≥ 80 km/h, `maximum_speed_lower_bound`) sui tempi soltanto: il percorso con
     * le preparazioni, le durate con la velocità limitata. ORS la applica dentro il suo core, che
     * contiene gli archi più veloci di 80 km/h; il nostro core ridotto no, e la ricerca piena su un
     * viaggio lungo non sta nella memoria del tablet. Usato oltre i 60 km in linea d'aria.
     */
    val maximumSpeedTimeOnly: Boolean = false,
    /** ORS `maximum_visited_nodes` (1 000 000): oltre, la ricerca piena si ferma (non esaurisce la memoria). */
    val maxVisitedNodes: Int = 1_000_000,
) {
    /** Richiesta "statica" nel senso di ORS: nessun filtro dipendente dalla richiesta. */
    val isStatic: Boolean get() = avoidFeatures == 0 && (vehicleParameters == null || !vehicleParameters.hasAttributes) &&
        vehicleType == HeavyVehicleAttributes.HGV && avoidPolygons.isEmpty() && (maximumSpeed.isInfinite() || maximumSpeedTimeOnly) && !continueStraight && !hasBearings

    /**
     * Niente CH né Core: i poligoni possono toccare qualunque arco (ORS "non decreasing weights");
     * `avoid_features` e `maximum_speed` toccano archi fuori dal nostro core (vedi [CoreEdgeSelector]).
     */
    val needsFullSearch: Boolean get() = avoidPolygons.isNotEmpty() || avoidFeatures != 0 || (maximumSpeed.isFinite() && !maximumSpeedTimeOnly)
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
    /** Landmark del core (l'insieme completo, `allow_all`), per chiave della preparazione core. */
    val landmarks = LinkedHashMap<String, CoreLandmarks>()
    /**
     * Gli altri insiemi di landmark di ogni core, preparati con delle strade escluse (ORS `lmsets`,
     * per esempio `tollways`): per chiave della preparazione core, nell'ordine di preparazione.
     */
    val avoidLandmarks = LinkedHashMap<String, MutableList<CoreLandmarks>>()

    /**
     * L'insieme di landmark per una richiesta che evita [avoidFeatures] (già ridotti a quelli del
     * profilo): fra quelli ammissibili (le strade escluse nella preparazione sono tutte evitate dalla
     * richiesta, così i limiti restano inferiori) quello con più strade escluse, cioè coi limiti più
     * stretti; ORS prende il primo ammissibile nell'ordine della configurazione (`highways;allow_all`).
     */
    fun landmarksFor(coreKey: String, avoidFeatures: Int): CoreLandmarks? {
        var best = landmarks[coreKey]
        for (lm in avoidLandmarks[coreKey] ?: return best) {
            if (!lm.admissibleFor(avoidFeatures)) continue
            if (best == null || Integer.bitCount(lm.avoidFeatures) > Integer.bitCount(best.avoidFeatures)) best = lm
        }
        return best
    }

    /** Gli insiemi di landmark con strade escluse preparati (nomi come `tollways`), letti dal file `profiles`. */
    var landmarkSetNames: List<String> = emptyList(); private set

    val profileNames: List<String> get() = encoding.profiles.map { it.profile.name }

    /** Pesi su cui preparare la CH per ciascun profilo (ORS: auto `fastest`, camion `recommended`). */
    private fun defaultCHWeighting(encoder: String): WeightingMethod = if (encoder == "car-ors") WeightingMethod.FASTEST else WeightingMethod.RECOMMENDED

    private fun chKey(encoder: String, weighting: Weighting) = "${encoder}_${weighting.name}"

    /**
     * Prepara per ogni profilo la CH completa (richieste "statiche") e il Core (richieste con restrizioni,
     * avoid_features o divieti di svolta), come le preparazioni `ch` e `core` di ORS, e le salva.
     */
    fun prepareCH(log: ((String) -> Unit)? = null, withCore: Boolean = true, landmarkCount: Int = 16, coreWide: Boolean = true,
                  landmarkSets: List<Int> = DEFAULT_LANDMARK_SETS) {
        val setNames = LinkedHashSet<String>()
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
            // il core per ogni peso, come ORS (`core.weightings: recommended,shortest`)
            if (withCore) {
                val selector = CoreEdgeSelector(encoding, name, turnCosts, wide = coreWide)
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
                val chg = CHGraph(graph, coreStorage, w)
                val lm = CoreLandmarks.prepare(chg, w, landmarkCount, log)
                lm.save(GraphDirectory(dir, mmap = false), coreKey)
                landmarks[coreKey] = lm
                log?.invoke("Landmark $coreKey: ${lm.landmarkCount} in ${System.currentTimeMillis() - t0} ms")
                // gli insiemi con delle strade escluse (ORS `lmsets`): solo col core largo, dove quelle
                // strade sono archi del core e non stanno in nessuno shortcut (vedi CHGraph.Iterator.accepts)
                if (coreWide) for (set in landmarkSets) {
                    val avoid = set and AvoidFeatureFlags.forCategory(pe.profile.category)
                    if (avoid == 0 || avoidLandmarks[coreKey]?.any { it.avoidFeatures == avoid } == true) continue
                    t0 = System.currentTimeMillis()
                    val filter = AvoidFeaturesEdgeFilter(encoding.shared, avoid, pe.profile.category, name == "heavyvehicle")
                    val set2 = CoreLandmarks.prepare(chg, w, landmarkCount, log, filter = filter, avoidFeatures = avoid)
                    set2.save(GraphDirectory(dir, mmap = false), CoreLandmarks.fileName(coreKey, avoid), withIndex = false)
                    avoidLandmarks.getOrPut(coreKey) { ArrayList() }.add(set2)
                    setNames += set2.setName
                    log?.invoke("Landmark $coreKey ${set2.setName}: ${set2.landmarkCount} in ${System.currentTimeMillis() - t0} ms")
                }
            }
        }
        coreVersion = CoreEdgeSelector.CORE_VERSION
        this.coreWide = coreWide && withCore
        landmarkSetNames = setNames.toList()
        GraphProperties(File(dir, "profiles")).apply {
            load(); set("profiles", profileNames.joinToString(",")); set("ch", chStorages.keys.joinToString(","))
            set("core_version", coreVersion.toString()); set("core_kind", if (this@RoutingGraph.coreWide) "wide" else "narrow")
            set("landmark_sets", landmarkSetNames.joinToString(",")); save()
        }
    }

    /**
     * Vero se il core contiene anche autostrade, pedaggi, traghetti e strade veloci (come quello di
     * ORS): allora `avoid_features` e `maximum_speed` usano il core, esatti; altrimenti la ricerca piena.
     */
    var coreWide = false; private set

    /** L'edizione della regola del core con cui il grafo è stato preparato ([CoreEdgeSelector.CORE_VERSION]); 1 per i grafi senza l'indicazione. */
    var coreVersion = 1; private set

    /** Nome del profilo ORS (`driving-hgv`) → nome interno dell'encoder (`heavyvehicle`). */
    fun encoderName(profile: String): String = when (profile) {
        "driving-car", "car", "car-ors" -> "car-ors"
        "driving-hgv", "hgv", "truck", "heavyvehicle" -> "heavyvehicle"
        else -> throw IllegalArgumentException("profilo sconosciuto: $profile")
    }

    /**
     * Niente CH né Core per questa richiesta: i poligoni possono toccare qualunque arco (ORS "non
     * decreasing weights"); `avoid_features` e `maximum_speed` toccano archi fuori dal core ridotto,
     * dentro quello largo ([coreWide]).
     */
    fun needsFullSearch(options: RouteOptions): Boolean =
        options.avoidPolygons.isNotEmpty() || (!coreWide && (options.avoidFeatures != 0 || (options.maximumSpeed.isFinite() && !options.maximumSpeedTimeOnly)))

    fun router(options: RouteOptions): Router {
        val name = encoderName(options.profile)
        val pe = encoding.profile(name)
        val weighting = WeightingFactory.create(encoding, pe, options.weighting, if (options.maximumSpeedTimeOnly) Double.POSITIVE_INFINITY else options.maximumSpeed)
        // the times with the limited speed, also when the search does not use it (see maximumSpeedTimeOnly)
        val timeWeighting = if (options.maximumSpeedTimeOnly) WeightingFactory.create(encoding, pe, options.weighting, options.maximumSpeed) else weighting
        val filter: EdgeFilter = EdgeFilterFactory.create(encoding, name, options.avoidFeatures, options.vehicleType, options.vehicleParameters, options.avoidPolygons)
        val tcp = if (options.turnCosts) TurnCostProvider(turnCosts, name) else null
        val key = chKey(name, weighting)
        // come `RoutingRequest.setSpeedups` di ORS: CH per le richieste statiche, altrimenti Core se preparato, altrimenti Dijkstra.
        // La nostra CH è node-based e ignora i divieti di svolta (ORS per i profili con costi di svolta prepara una CH
        // edge-based): con i costi di svolta una richiesta statica usa il core, che li rispetta, quando c'è
        val coreStorage = chStorages["${key}_core"]
        val ch = if (options.allowCH && options.isStatic && (tcp == null || coreStorage == null)) chStorages[key]?.let { CHGraph(graph, it, weighting) } else null
        // le misure dell'auto (camper, furgoni): nel core solo dall'edizione 2 della regola; prima, ricerca piena
        val coreFits = name == "heavyvehicle" || options.vehicleParameters?.hasDimensions != true || coreVersion >= 2
        val core = if (ch == null && options.allowCH && !needsFullSearch(options) && coreFits) coreStorage?.let { CHGraph(graph, it, weighting) } else null
        // l'insieme di landmark della richiesta: con «evita pedaggi» quello preparato senza i pedaggi, se c'è
        val lm = if (core != null) landmarksFor("${key}_core", options.avoidFeatures and AvoidFeatureFlags.forCategory(pe.profile.category)) else null
        // avoid_features alone: first the quick search without them; its path is the best one with
        // them too when it uses none of the roads to avoid (the filter only takes roads away), else
        // the full search (ORS keeps those roads in its core; our reduced core does not, the wide one does)
        val quick = if (ch == null && core == null && options.avoidFeatures != 0 && options.avoidPolygons.isEmpty() && options.allowCH &&
            (options.maximumSpeed.isInfinite() || options.maximumSpeedTimeOnly)) router(options.copy(avoidFeatures = 0)) else null
        val avoid = if (quick != null) AvoidFeaturesEdgeFilter(encoding.shared, options.avoidFeatures, pe.profile.category, name == "heavyvehicle") else null
        return Router(graph, index, weighting, filter, tcp, ch, core, lm, maxVisitedNodes = options.maxVisitedNodes, timeWeighting = timeWeighting,
            quick = quick, quickAccepts = avoid)
    }

    /**
     * Riscaldamento: carica nella cache del sistema i file che una ricerca col core per [profile]
     * tocca a caso (sul tablet la prima ricerca dopo l'avvio costava 6 volte quelle dopo: 28 s contro
     * 16 s fino a Komjatice). In ordine di resa per byte: i livelli del core, i landmark (l'insieme
     * completo e quello per [avoidFeatures]), gli shortcut del core, i nodi, gli archi; si ferma
     * oltre [budget] byte. Da chiamare in sottofondo, a bassa priorità; in RAM non fa nulla.
     * Ritorna una riga di diagnostica (file e MB toccati, tempo).
     */
    fun warm(profile: String, weighting: WeightingMethod? = null, avoidFeatures: Int = AvoidFeatureFlags.TOLLWAYS, budget: Long = 1L shl 30): String {
        val t0 = System.currentTimeMillis()
        val name = encoderName(profile)
        val pe = encoding.profile(name)
        val w = WeightingFactory.create(encoding, pe, weighting ?: defaultCHWeighting(name))
        val coreKey = "${chKey(name, w)}_core"
        var bytes = 0L
        val done = ArrayList<String>()
        fun step(label: String, run: () -> Long): Boolean {
            if (bytes >= budget) return false
            val b = run(); bytes += b
            if (b > 0) done += "$label ${b shr 20} MB"
            return true
        }
        val core = chStorages[coreKey]
        step("livelli") { core?.touch(withShortcuts = false) ?: 0 } &&
            step("landmark") { landmarks[coreKey]?.touch() ?: 0 } &&
            step("landmark ${CoreLandmarks.setName(avoidFeatures and AvoidFeatureFlags.forCategory(pe.profile.category))}") {
                landmarksFor(coreKey, avoidFeatures and AvoidFeatureFlags.forCategory(pe.profile.category))?.takeIf { it.avoidFeatures != 0 }?.touch() ?: 0 } &&
            step("shortcut") { core?.touch(withShortcuts = true)?.let { it - (core.touch(withShortcuts = false)) } ?: 0 } &&
            step("nodi") { graph.touch(withNodes = true, withEdges = false, withGeometry = false) } &&
            step("archi") { graph.touch(withNodes = false, withEdges = true, withGeometry = false) }
        return "riscaldamento $coreKey: ${bytes shr 20} MB in ${System.currentTimeMillis() - t0} ms (${done.joinToString()})"
    }

    fun flush() { graph.flush(); turnCosts.flush(); guidance?.flush(graph.edgeCount); points?.flush(); GraphProperties(File(dir, "profiles")).apply { load(); set("profiles", profileNames.joinToString(",")); save() } }

    override fun close() {
        graph.close(); turnCosts.close(); guidance?.close(); points?.close(); chStorages.values.forEach { it.close() }
        avoidLandmarks.values.forEach { l -> l.forEach { it.close() } }
        landmarks.values.forEach { it.close() }
        if (this::index.isInitialized) index.close()
    }

    companion object {
        /**
         * Gli insiemi di landmark con strade escluse preparati per ogni core oltre a quello completo
         * (ORS: `lmsets: highways;allow_all`): «evita pedaggi», il filtro dell'app che senza il suo
         * insieme esplorava un terzo del core italiano (9 s sul tablet). Ognuno costa un file di
         * `2·16` float per nodo del core (~90 MB per core in Italia) e ~20 s di preparazione.
         */
        val DEFAULT_LANDMARK_SETS: List<Int> = listOf(AvoidFeatureFlags.TOLLWAYS)

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
            rg.coreVersion = props.get("core_version")?.toIntOrNull() ?: 1
            rg.coreWide = props.get("core_kind") == "wide"
            rg.landmarkSetNames = (props.get("landmark_sets") ?: "").split(',').filter { it.isNotBlank() }
            for (key in (props.get("ch") ?: "").split(',').filter { it.isNotBlank() }) {
                val st = CHStorage(gdir, key, graph.edgeCount, graph.nodeCount)
                if (st.loadExisting()) rg.chStorages[key] = st else st.close()
                if (key.endsWith("_core")) CoreLandmarks.load(gdir, key, graph.nodeCount)?.let { base ->
                    rg.landmarks[key] = base
                    for (set in rg.landmarkSetNames) {
                        val avoid = CoreLandmarks.parseSetName(set)
                        if (avoid == 0) continue
                        CoreLandmarks.load(gdir, CoreLandmarks.fileName(key, avoid), graph.nodeCount, base)?.let { rg.avoidLandmarks.getOrPut(key) { ArrayList() }.add(it) }
                    }
                }
            }
            rg.index = LocationIndex.load(graph, gdir) ?: LocationIndex.build(graph)
            return rg
        }
    }
}
