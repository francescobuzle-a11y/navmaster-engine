package it.navmaster.engine.osm

import com.carrotsearch.hppc.LongIntHashMap
import com.carrotsearch.hppc.LongArrayList
import com.carrotsearch.hppc.LongObjectHashMap
import it.navmaster.engine.profile.GuidanceParser
import it.navmaster.engine.storage.GuidanceStore
import it.navmaster.engine.storage.PointStore
import it.navmaster.engine.profile.Access
import it.navmaster.engine.profile.ClimbModel
import it.navmaster.engine.terrain.ElevationSource
import it.navmaster.engine.terrain.TerrainSpeeds
import it.navmaster.engine.terrain.WayTerrain
import it.navmaster.engine.profile.NodeDelays
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.WaySpeedAccess
import it.navmaster.engine.profile.ZoneInfo
import it.navmaster.engine.profile.WayAttributeParser
import it.navmaster.engine.profile.Toll
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.storage.PointList
import it.navmaster.engine.storage.TurnCostStorage
import it.navmaster.engine.util.DistanceCalc
import java.io.File

/** Divieto di svolta letto da una relazione `type=restriction` (applicato in una fase successiva). */
class TurnRestrictionRelation(val id: Long, val fromWay: Long, val viaNode: Long, val toWay: Long, val restriction: String, val vehicle: String?, val except: Set<String>)

data class ImportStats(
    val osmNodes: Long, val osmWays: Long, val acceptedWays: Long,
    val towers: Int, val pillars: Int, val edges: Int, val barrierEdges: Int,
    /** Divieti di svolta applicati al grafo (0 se non è stato passato un TurnCostStorage). */
    val turnRestrictions: Int,
    /** Relazioni di divieto lette dal file. */
    val turnRestrictionRelations: Int,
    val millis: Long,
    /** Archi messi a pedaggio da un casello o da un portale ([ImportOptions.tollNodes]). */
    val tollNodeEdges: Int = 0,
    /** Il modello delle velocità dei profili ([SpeedModel]). */
    val speedModel: SpeedModel = SpeedModel.ORS,
    /** Eventi di ritardo ai nodi per tipo (nome di [NodeDelays.Control]); vuoto col modello ORS. */
    val delayEvents: Map<String, Int> = emptyMap(),
    /** Archi con un ritardo > 0 in almeno un senso e un profilo. */
    val delayedEdges: Int = 0,
    /** Velocità media del camion (km/h, pesata per lunghezza) per classe di strada. */
    val hgvSpeedByClass: Map<String, Double> = emptyMap(),
    /** Quota dei km di primary e secondary con cartello > 50 dove il camion va sotto 30 km/h (NaN se nessuno), senza il terreno. */
    val hgvSlowSignedShare: Double = Double.NaN,
    /** Salite, discese e curve del camion (modello NAVMASTER); null se il terreno non si è applicato. */
    val terrain: TerrainStats? = null,
)

/**
 * Il terreno del camion all'import (modello NAVMASTER, [it.navmaster.engine.terrain.WayTerrain]),
 * contato per arco e per senso percorribile: i km coperti dal DEM, i km frenati dalla pendenza e i
 * minuti persi per la pendenza sola, per le curve sole e per le due insieme (non sono additivi).
 */
data class TerrainStats(
    /** Vero se è stato usato un DEM (altrimenti solo curve). */
    val dem: Boolean,
    /** km di archi del camion (per senso percorribile). */
    val km: Double,
    /** Quota di quei km con una quota dal DEM (NaN senza DEM). */
    val demCoverage: Double,
    /** km in cui la pendenza frena il camion (salita o discesa, per senso). */
    val slopeKm: Double,
    val slopeLossMin: Double,
    /** Curve contate sulle way del camion. */
    val curves: Int,
    val curveLossMin: Double,
    /** Minuti persi per pendenza e curve insieme. */
    val lossMin: Double,
    /** Sensi d'arco con la velocità cambiata dal terreno. */
    val changedEdgeDirections: Int,
)

/** Le strade su cui un casello o un portale fa pedaggio (non i parcheggi né le strade di servizio). */
private val TOLL_NODE_ROADS = setOf("motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link")

/**
 * Costruisce un [BaseGraph] da un file PBF in due passate, come GraphHopper:
 *
 * 1. legge le way accettate da almeno un profilo e conta quante volte ogni nodo è usato;
 *    raccoglie le relazioni di divieto di svolta;
 * 2. legge i nodi (torre = usato da ≥2 way, estremo di way, o barriera; pillar = intermedio),
 *    poi le way: le spezza alle torri e crea un arco per segmento con geometria, distanza e flag.
 *
 * I nodi assenti dall'estratto tagliano il segmento. Un nodo barriera all'interno di una way
 * viene sdoppiato: un arco di lunghezza zero fra le due copie porta il blocco per i profili interessati.
 */
/** Opzioni di costruzione del grafo. */
data class ImportOptions(
    /** Vedi [WayAttributeParser.parse]: con `false` il tag `width` non diventa un limite di larghezza. */
    val widthFromCarriageway: Boolean = true,
    /** Scrive il file `guidance` (limiti di velocità, corsie, destinazioni, svincoli, caselli): solo dati, nessun effetto sul calcolo. */
    val guidance: Boolean = true,
    /**
     * Estensione NavMaster: l'arco di una strada principale che passa da un casello (`barrier=toll_booth`)
     * è a pedaggio anche senza `toll=yes` sulla way (in OSM le barriere del sistema aperto e le rampe dei
     * caselli spesso non l'hanno: «evita pedaggi» ci passava); da un portale (`highway=toll_gantry`), a
     * pedaggio per i mezzi pesanti se la way non dice già di più.
     */
    val tollNodes: Boolean = true,
    /**
     * Estensione NavMaster, solo modello NAVMASTER e mezzi pesanti ([it.navmaster.engine.profile.VehicleProfile.climb]):
     * il modello del terreno (DEM) per le pendenze ([it.navmaster.engine.terrain.HgtDem]); null = senza
     * pendenze. Mai coi profili ORS (la CLI rifiuta `--dem` con `--speed-model=ors`).
     */
    val dem: ElevationSource? = null,
    /** Da dove viene il DEM (`copernicus-glo90`, `skadi`), scritto nel file `profiles` del grafo. */
    val demSource: String? = null,
    /** Le curve dalla geometria (modello NAVMASTER, mezzi pesanti, anche senza DEM); false solo per le prove. */
    val curves: Boolean = true,
)

class OsmGraphImporter(
    private val graph: BaseGraph, private val encoding: GraphEncoding, private val turnCosts: TurnCostStorage? = null,
    private val options: ImportOptions = ImportOptions(),
    /** Se presente, riceve i dati di guida di ogni arco (file `guidance`). */
    private val guidance: GuidanceStore? = null,
    /** Se presente, riceve autovelox, Tutor e fotocamere ai semafori (file `points`). */
    private val points: PointStore? = null,
) {
    private val profiles = encoding.profiles
    private val junctions = LongObjectHashMap<String>()   // id OSM → svincolo "ref|nome"
    private val specialNodes = LongObjectHashMap<String>() // id OSM → casello / portale / dogana
    private val tollBooths = com.carrotsearch.hppc.LongHashSet()   // nodi barrier=toll_booth delle strade (ImportOptions.tollNodes)
    private val tollGantries = com.carrotsearch.hppc.LongHashSet() // nodi highway=toll_gantry
    private class Enforcement(val type: Int, val maxspeed: Int, val device: Long, val from: Long, val to: Long)
    private val enforcements = ArrayList<Enforcement>()
    private val neededNodes = LongObjectHashMap<DoubleArray>() // nodi delle relazioni enforcement → [lat, lon]
    private val cameraNodes = com.carrotsearch.hppc.LongHashSet()  // nodi highway=speed_camera già scritti
    private val relationSigns = HashMap<Long, MutableList<Pair<Long, String>>>() // way "to" → (nodo intersection, cartello)
    private val nodeUse = LongIntHashMap()       // id OSM → bit: 1 = visto una volta, 2 = più volte, 4 = estremo di way
    private val towers = LongIntHashMap()        // id OSM → id nodo del grafo
    private val pillars = LongIntHashMap()       // id OSM → indice pillar
    private var pillarLat = DoubleArray(1024)
    private var pillarLon = DoubleArray(1024)
    private var pillarCount = 0
    private val barrierTags = LongObjectHashMap<Map<String, String>>()
    val turnRestrictions = ArrayList<TurnRestrictionRelation>()

    private var acceptedWays = 0L
    private var barrierEdges = 0
    private var tollNodeEdges = 0
    private val wayAccess = Array(profiles.size) { Access.CAN_SKIP }
    private val wayResults = arrayOfNulls<WaySpeedAccess>(profiles.size)

    /** Le perdite ai nodi del modello NAVMASTER ([NodeDelays], [JunctionDelayPass]): attive se un profilo usa quel modello. */
    private val nodeDelays = profiles.any { it.profile.options.speedModel == SpeedModel.NAVMASTER }
    /** Modello NAVMASTER: le corsie `private` delle strade principali non bloccano i mezzi pesanti ([WayAttributeParser.parse]). */
    private val privateMainRoads = nodeDelays
    private val controlNodes = LongIntHashMap()  // id OSM → codice di NodeDelays.classify
    /** Sensi d'arco (arco·2 + 0 avanti, + 1 indietro) con uno stop, una precedenza o un semaforo entro 30 m dal capo d'arrivo. */
    private val nearEnd = java.util.BitSet()
    private val delayEvents = IntArray(NodeDelays.Control.entries.size)
    private val hgvIndex = profiles.indexOfFirst { it.profile.name == "heavyvehicle" }
    private val classLength = DoubleArray(it.navmaster.engine.profile.RoadClass.entries.size)
    private val classSpeedLength = DoubleArray(it.navmaster.engine.profile.RoadClass.entries.size)
    private var signedKm = 0.0
    private var slowSignedKm = 0.0

    /**
     * I profili a cui si applica il terreno ([it.navmaster.engine.terrain.WayTerrain]): modello NAVMASTER
     * e un [it.navmaster.engine.profile.ClimbModel] (oggi solo il camion). Nessuno nei grafi ORS.
     */
    private val terrainProfile = BooleanArray(profiles.size) { i ->
        val p = profiles[i].profile
        p.options.speedModel == SpeedModel.NAVMASTER && p.climb != null && (options.curves || options.dem != null)
    }
    private val terrainOn = terrainProfile.any { it }
    private var tKm = 0.0; private var tDemKm = 0.0; private var tSlopeKm = 0.0
    private var tSlopeLoss = 0.0; private var tCurveLoss = 0.0; private var tLoss = 0.0
    private var tCurves = 0; private var tChanged = 0

    fun import(pbf: File): ImportStats = import(OsmPbfReader(pbf))

    fun import(reader: OsmSource): ImportStats {
        val start = System.currentTimeMillis()
        reader.read(Pass1(), nodes = false)
        reader.read(Pass2Nodes(), ways = false, relations = false)
        reader.read(Pass2Ways(), nodes = false, relations = false)
        var delayedEdges = 0
        if (nodeDelays) {
            // i controlli sulle torri (capi d'arco): semafori e stop agli incroci, precedenze implicite, rotatorie
            val towerControls = com.carrotsearch.hppc.IntIntHashMap()
            for (c in controlNodes) { val t = towers.getOrDefault(c.key, -1); if (t >= 0) towerControls.put(t, c.value) }
            val pass = JunctionDelayPass(graph, encoding, towerControls, nearEnd)
            pass.run()
            for (i in delayEvents.indices) delayEvents[i] += pass.events[i]
            delayedEdges = pass.delayedEdges
        }
        // autovelox e Tutor delle relazioni enforcement (il nodo device con highway=speed_camera è già scritto)
        if (points != null) for (e in enforcements) {
            fun at(id: Long): DoubleArray? = if (id < 0) null else neededNodes.get(id)?.takeIf { it.size == 2 }
            when (e.type) {
                PointStore.AVERAGE_SPEED_START -> {
                    at(e.from)?.let { points.add(it[0], it[1], PointStore.AVERAGE_SPEED_START, e.maxspeed, -1) }
                    at(e.to)?.let { points.add(it[0], it[1], PointStore.AVERAGE_SPEED_END, e.maxspeed, -1) }
                }
                else -> if (e.device < 0 || !cameraNodes.contains(e.device)) (at(e.device) ?: at(e.from))?.let { points.add(it[0], it[1], e.type, e.maxspeed, -1) }
            }
        }
        var appliedRestrictions = 0
        if (turnCosts != null) {
            val applier = TurnRestrictionApplier(graph, encoding, turnCosts)
            applier.apply(turnRestrictions, towers)
            appliedRestrictions = applier.applied
        }
        graph.freeze()
        val byClass = LinkedHashMap<String, Double>()
        for (rc in it.navmaster.engine.profile.RoadClass.entries) if (classLength[rc.ordinal] > 0) byClass[rc.osm] = classSpeedLength[rc.ordinal] / classLength[rc.ordinal]
        return ImportStats(reader.nodeCount, reader.wayCount, acceptedWays, graph.nodeCount, pillarCount, graph.edgeCount, barrierEdges, appliedRestrictions, turnRestrictions.size, System.currentTimeMillis() - start, tollNodeEdges,
            speedModel = if (nodeDelays) SpeedModel.NAVMASTER else SpeedModel.ORS,
            delayEvents = if (nodeDelays) NodeDelays.Control.entries.filter { it != NodeDelays.Control.NONE }.associate { it.name to delayEvents[it.ordinal] } else emptyMap(),
            delayedEdges = delayedEdges, hgvSpeedByClass = byClass,
            hgvSlowSignedShare = if (signedKm > 0) slowSignedKm / signedKm else Double.NaN,
            terrain = if (!terrainOn || hgvIndex < 0 || !terrainProfile[hgvIndex]) null else TerrainStats(
                dem = options.dem != null, km = tKm / 1000, demCoverage = if (options.dem != null && tKm > 0) tDemKm / tKm else Double.NaN,
                slopeKm = tSlopeKm, slopeLossMin = tSlopeLoss / 60, curves = tCurves, curveLossMin = tCurveLoss / 60,
                lossMin = tLoss / 60, changedEdgeDirections = tChanged))
    }

    /** Vero se almeno un profilo può percorrere la way; riempie [wayAccess]. */
    private fun acceptWay(way: OsmWay): Boolean {
        if (way.nodeIds.size < 2 || way.tags.isEmpty()) return false
        var any = false
        for (i in profiles.indices) {
            wayAccess[i] = profiles[i].profile.access(way)
            if (!wayAccess[i].canSkip) any = true
        }
        return any
    }

    private inner class Pass1 : OsmSink {
        override fun way(way: OsmWay) {
            if (!acceptWay(way)) return
            acceptedWays++
            val ids = way.nodeIds
            for (i in ids.indices) {
                val id = ids[i]
                var v = nodeUse.getOrDefault(id, 0)
                v = if (v and 3 == 0) v or 1 else v or 2
                if (i == 0 || i == ids.size - 1) v = v or 4
                nodeUse.put(id, v)
            }
        }

        override fun relation(id: Long, tags: Map<String, String>, members: List<RelationMember>) {
            if (guidance != null) when (tags["type"]) {
                "enforcement" -> {
                    val t = when (tags["enforcement"]) {
                        "maxspeed" -> PointStore.SPEED_CAMERA; "average_speed" -> PointStore.AVERAGE_SPEED_START
                        "traffic_signals" -> PointStore.TRAFFIC_SIGNALS_CAMERA; else -> 0
                    }
                    if (t == 0) return
                    fun node(role: String) = members.firstOrNull { it.type == MemberType.NODE && it.role == role }?.ref ?: -1L
                    val e = Enforcement(t, GuidanceParser.speed(tags["maxspeed"]), node("device"), node("from"), node("to"))
                    enforcements += e
                    for (n in longArrayOf(e.device, e.from, e.to)) if (n >= 0) neededNodes.put(n, DoubleArray(0))
                    return
                }
                "destination_sign" -> {
                    val sign = GuidanceParser.relationSign(tags) ?: return
                    val to = members.firstOrNull { it.type == MemberType.WAY && it.role == "to" }?.ref ?: return
                    val inter = members.firstOrNull { it.type == MemberType.NODE && it.role == "intersection" }?.ref ?: -1L
                    relationSigns.getOrPut(to) { ArrayList(2) } += inter to sign
                    return
                }
            }
            if (tags["type"] != "restriction") return
            var restriction: String? = null; var vehicle: String? = null
            for ((k, v) in tags) {
                if (k == "restriction") { restriction = v; vehicle = null; break }
                if (k.startsWith("restriction:")) { restriction = v; vehicle = k.substring(12) }
            }
            val r = restriction ?: return
            val from = members.firstOrNull { it.type == MemberType.WAY && it.role == "from" } ?: return
            val to = members.firstOrNull { it.type == MemberType.WAY && it.role == "to" } ?: return
            val via = members.firstOrNull { it.type == MemberType.NODE && it.role == "via" } ?: return // via-way non supportata
            val except = tags["except"]?.split(';')?.map { it.trim() }?.toSet() ?: emptySet()
            turnRestrictions += TurnRestrictionRelation(id, from.ref, via.ref, to.ref, r, vehicle, except)
        }
    }

    private inner class Pass2Nodes : OsmSink {
        override fun node(id: Long, lat: Double, lon: Double, tags: Map<String, String>?) {
            if (points != null) {
                if (tags != null && tags["highway"] == "speed_camera") {
                    points.add(lat, lon, PointStore.SPEED_CAMERA, GuidanceParser.speed(tags["maxspeed"]), bearingOf(tags["direction"]))
                    cameraNodes.add(id)
                }
                if (!neededNodes.isEmpty && neededNodes.containsKey(id)) neededNodes.put(id, doubleArrayOf(lat, lon))
            }
            val use = nodeUse.getOrDefault(id, 0)
            if (use == 0) return
            if (nodeDelays && tags != null) NodeDelays.classify(tags).let { if (it != 0) controlNodes.put(id, it) }
            var isBarrier = false
            if (tags != null && options.tollNodes) {
                if (tags["barrier"] == "toll_booth") tollBooths.add(id)
                if (tags["highway"] == "toll_gantry") tollGantries.add(id)
            }
            if (tags != null && guidance != null) {
                GuidanceParser.junction(tags)?.let { junctions.put(id, it) }
                GuidanceParser.specialNode(tags)?.let { specialNodes.put(id, it) }
            }
            if (tags != null && (tags.containsKey("barrier") || tags["ford"] == "yes" || tags["highway"] == "ford")) {
                isBarrier = profiles.any { it.profile.isBarrier(tags) }
                if (isBarrier) barrierTags.put(id, tags)
            }
            if (use and 2 != 0 || use and 4 != 0 || isBarrier) {
                towers.put(id, graph.addNode(lat, lon))
            } else {
                if (pillarCount == pillarLat.size) {
                    pillarLat = pillarLat.copyOf(pillarCount * 2); pillarLon = pillarLon.copyOf(pillarCount * 2)
                }
                pillarLat[pillarCount] = lat; pillarLon[pillarCount] = lon
                pillars.put(id, pillarCount++)
            }
        }
    }

    private inner class Pass2Ways : OsmSink {
        private val lats = DoubleArrayList(); private val lons = DoubleArrayList()
        private val segIds = LongArrayList() // id OSM dei punti del segmento corrente, in parallelo a lats/lons
        private var segmentStart = -1 // nodo del grafo in cui inizia il segmento corrente
        private var segmentStartIdx = -1 // indice nella way del primo punto del segmento corrente (vedi WayTerrain.nodeCum)
        /**
         * ORS `estimated_way_distance`: la distanza in linea d'aria fra il primo e l'ultimo nodo della
         * way intera (non la lunghezza dell'arco), usata dal modello di accelerazione e dalla penalità
         * delle strade residenziali; NaN quando un capo manca dall'estratto (ORS non mette il tag).
         */
        private var wayBeeline = Double.NaN

        private fun nodeLat(id: Long): Double { val t = towers.getOrDefault(id, -1); if (t >= 0) return graph.lat(t); val p = pillars.getOrDefault(id, -1); return if (p >= 0) pillarLat[p] else Double.NaN }
        private fun nodeLon(id: Long): Double { val t = towers.getOrDefault(id, -1); if (t >= 0) return graph.lon(t); val p = pillars.getOrDefault(id, -1); return if (p >= 0) pillarLon[p] else Double.NaN }

        override fun way(way: OsmWay) {
            if (!acceptWay(way)) return
            val ids = way.nodeIds
            wayBeeline = if (ids.isEmpty()) Double.NaN else {
                val la1 = nodeLat(ids[0]); val lo1 = nodeLon(ids[0]); val la2 = nodeLat(ids[ids.size - 1]); val lo2 = nodeLon(ids[ids.size - 1])
                if (la1.isNaN() || lo1.isNaN() || la2.isNaN() || lo2.isNaN()) Double.NaN else DistanceCalc.haversine(la1, lo1, la2, lo2)
            }
            segmentStart = -1; segmentStartIdx = -1
            lats.clear(); lons.clear(); segIds.clear()
            beginTerrain(way)
            for (i in ids.indices) {
                val id = ids[i]
                var tower = towers.getOrDefault(id, -1)
                if (tower < 0) {
                    val p = pillars.getOrDefault(id, -1)
                    if (p < 0) { // nodo assente dall'estratto: taglia il segmento
                        closeSegment(way, promoteLast = true)
                        continue
                    }
                    if (segmentStart < 0) { // il segmento deve iniziare da una torre: promuovi il pillar
                        tower = promote(id, p)
                    } else {
                        lats.add(pillarLat[p]); lons.add(pillarLon[p]); segIds.add(id); continue
                    }
                }
                // nodo barriera (anche al primo o all'ultimo nodo della way, come l'OSMReader di GraphHopper):
                // la way si ferma su una copia del nodo, unita al nodo vero da un arco di lunghezza zero
                // che porta l'accesso della barriera; una barriera vale una volta sola (il primo passaggio)
                val bt = barrierTags.get(id)
                if (bt != null) barrierTags.remove(id)
                val next: Int
                if (segmentStart >= 0) {
                    lats.add(graph.lat(tower)); lons.add(graph.lon(tower)); segIds.add(id)
                    addEdge(way, segmentStart, if (bt != null) splitBarrier(way, tower, bt) else tower, segmentStartIdx)
                    next = tower
                } else {
                    next = if (bt != null) splitBarrier(way, tower, bt) else tower
                }
                segmentStart = next; segmentStartIdx = i
                lats.clear(); lons.clear(); segIds.clear()
                lats.add(graph.lat(next)); lons.add(graph.lon(next)); segIds.add(id)
            }
        }

        /** Chiude il segmento aperto su un nodo mancante: l'ultimo pillar diventa torre. */
        private fun closeSegment(way: OsmWay, promoteLast: Boolean) {
            if (segmentStart >= 0 && lats.size > 1 && promoteLast) {
                val end = graph.addNode(lats[lats.size - 1], lons[lons.size - 1])
                addEdge(way, segmentStart, end, segmentStartIdx)
            }
            segmentStart = -1; segmentStartIdx = -1
            lats.clear(); lons.clear(); segIds.clear()
        }

        private fun promote(id: Long, pillarIndex: Int): Int {
            val t = graph.addNode(pillarLat[pillarIndex], pillarLon[pillarIndex])
            towers.put(id, t)
            pillars.remove(id)
            return t
        }

        private fun splitBarrier(way: OsmWay, tower: Int, tags: Map<String, String>): Int {
            val copy = graph.addNode(graph.lat(tower), graph.lon(tower))
            val e = graph.addEdge(tower, copy, 0.0)
            val flags = graph.encodedValues.newFlags()
            val attrs = WayAttributeParser.parse(way, isFerry = false, widthFromCarriageway = options.widthFromCarriageway, privateMainRoads = privateMainRoads)
            encoding.shared.write(flags, attrs)
            for (i in profiles.indices) {
                val pe = profiles[i]
                val r = pe.profile.analyse(way, wayAccess[i], 0.0)
                val blocked = pe.profile.isBarrier(tags)
                pe.write(flags, if (r == null || blocked) null else r)
            }
            graph.setFlags(e, flags)
            barrierEdges++
            return copy
        }

        /** Il pedaggio dei caselli e dei portali sui punti dell'arco corrente (vedi [ImportOptions.tollNodes]). */
        private fun tollFromNodes(way: OsmWay, a: it.navmaster.engine.profile.WayAttributes, n: Int): it.navmaster.engine.profile.WayAttributes {
            if (!options.tollNodes || a.toll == Toll.ALL || (tollBooths.isEmpty && tollGantries.isEmpty) || segIds.size() != n) return a
            if (way.tag("highway") !in TOLL_NODE_ROADS) return a
            var booth = false; var gantry = false
            for (k in 0 until n) { val id = segIds.get(k); if (tollBooths.contains(id)) booth = true; if (tollGantries.contains(id)) gantry = true }
            return when {
                booth -> { tollNodeEdges++; a.copy(toll = Toll.ALL) }
                gantry && (a.toll == Toll.MISSING || a.toll == Toll.NO) -> { tollNodeEdges++; a.copy(toll = Toll.HGV) }
                else -> a
            }
        }

        // --- terreno (modello NAVMASTER, camion): pendenze e curve per way intera, lette a pezzi dagli archi
        private var terrainWay: OsmWay? = null
        private var terrain: WayTerrain? = null
        private val terrainSpeeds = arrayOfNulls<TerrainSpeeds>(profiles.size)
        private var wayLat = DoubleArray(64); private var wayLon = DoubleArray(64)

        /** Nuova way: il terreno si calcola al primo arco che lo chiede (le coordinate però si leggono ora, prima delle promozioni). */
        private fun beginTerrain(way: OsmWay) {
            terrainWay = null; terrain = null; terrainSpeeds.fill(null)
            if (!terrainOn) return
            var needed = false
            for (i in profiles.indices) if (terrainProfile[i] && !wayAccess[i].canSkip && !wayAccess[i].isFerry) needed = true
            if (!needed) return
            val ids = way.nodeIds
            if (wayLat.size < ids.size) { wayLat = DoubleArray(ids.size * 2); wayLon = DoubleArray(ids.size * 2) }
            for (k in ids.indices) {
                val id = ids[k]
                val t = towers.getOrDefault(id, -1)
                if (t >= 0) { wayLat[k] = graph.lat(t); wayLon[k] = graph.lon(t); continue }
                val p = pillars.getOrDefault(id, -1)
                if (p >= 0) { wayLat[k] = pillarLat[p]; wayLon[k] = pillarLon[p] } else { wayLat[k] = Double.NaN; wayLon[k] = Double.NaN }
            }
            terrainWay = way
        }

        /**
         * La velocità dell'arco (punti [startIdx]..[startIdx] + n - 1 della way) per senso, dal profilo di
         * velocità della way ([TerrainSpeeds]); mai sopra la base, mai sotto il minimo del profilo. Senza
         * pendenza né curve, o se il terreno non cambia il passo memorizzato, [r] resta com'è (stessi flag).
         *
         * Il terreno si applica come rapporto di tempi alla base già arrotondata al passo dei flag (5 km/h):
         * una base di 63 si salva 65, e una perdita dell'1 % (una curva lontana) darebbe 62,4 → 60, un salto
         * dell'8 % rispetto all'arco accanto; così invece 65 × 0,99 → 65, e al 6 % 65 × 38/63 → 40.
         */
        private fun withTerrain(i: Int, way: OsmWay, r: WaySpeedAccess, startIdx: Int, n: Int): WaySpeedAccess {
            if (terrainWay !== way || startIdx < 0) return r
            val t = terrain ?: WayTerrain.analyse(way, wayLat, wayLon, way.nodeIds.size, options.dem, options.curves).also {
                terrain = it
                if (hgvIndex >= 0 && !wayAccess[hgvIndex].canSkip) tCurves += it.curves
            }
            val endIdx = startIdx + n - 1
            if (endIdx >= t.nodeCum.size) return r
            val a = t.nodeCum[startIdx]; val b = t.nodeCum[endIdx]
            val stats = i == hgvIndex
            if (stats) {
                val dirs = (if (r.forward) 1 else 0) + (if (r.backward) 1 else 0)
                tKm += (b - a) * dirs
                if (!t.demCoverage.isNaN()) tDemKm += (b - a) * dirs * t.demCoverage
            }
            if (t.neutral || b - a < 1.0) return r
            val profile = profiles[i].profile
            val climb = profile.climb ?: return r
            val sp = terrainSpeeds[i]?.takeIf { it.baseKmh == r.speedKmh }
                ?: TerrainSpeeds(t, climb, r.speedKmh).also { terrainSpeeds[i] = it }
            val step = profile.options.speedFactor
            fun steps(v: Double) = Math.round(v / step)
            val baseQ = maxOf(step, steps(r.speedKmh) * step)
            fun scaled(v: Double) = maxOf(profile.minSpeed, baseQ * minOf(1.0, v / r.speedKmh))
            val fwd = scaled(sp.speedKmh(a, b, true)); val bwd = scaled(sp.speedKmh(a, b, false))
            if (stats) terrainStats(t, climb, r, sp, a, b)
            val fwdChanged = steps(fwd) != steps(r.speedKmh); val bwdChanged = steps(bwd) != steps(r.speedKmh)
            if (!fwdChanged && !bwdChanged) return r
            if (stats) { if (r.forward && fwdChanged) tChanged++; if (r.backward && bwdChanged) tChanged++ }
            return r.copy(speedKmh = if (fwdChanged) fwd else r.speedKmh, speedBackwardKmh = if (bwdChanged) bwd else r.speedKmh)
        }

        /** Le perdite di tempo del camion su un arco, per senso percorribile: pendenza sola, curve sole, insieme. */
        private fun terrainStats(t: WayTerrain, climb: ClimbModel, r: WaySpeedAccess,
                                 sp: TerrainSpeeds, a: Double, b: Double) {
            val base = (b - a) / (r.speedKmh / 3.6)
            val both = t.grade != null && t.curves > 0
            val slopeOnly = if (both) TerrainSpeeds(t, climb, r.speedKmh, slope = true, curves = false) else null
            val curvesOnly = if (both) TerrainSpeeds(t, climb, r.speedKmh, slope = false, curves = true) else null
            for (fwd in booleanArrayOf(true, false)) {
                if (fwd && !r.forward || !fwd && !r.backward) continue
                val all = sp.seconds(a, b, fwd) - base
                tLoss += all
                tSlopeKm += sp.slowedBySlope(a, b, fwd) / 1000
                when {
                    both -> { tSlopeLoss += slopeOnly!!.seconds(a, b, fwd) - base; tCurveLoss += curvesOnly!!.seconds(a, b, fwd) - base }
                    t.grade != null -> tSlopeLoss += all
                    else -> tCurveLoss += all
                }
            }
        }

        private val delayFwd = DoubleArray(profiles.size)
        private val delayBwd = DoubleArray(profiles.size)
        private var cum = DoubleArray(64)

        /**
         * I controlli sui nodi interni dell'arco (pillar: semafori, stop, precedenze, caselli, passaggi a
         * livello, semafori pedonali): i secondi persi per senso e profilo, sommati e scritti nei flag. Le
         * torri (capi d'arco) le tratta [JunctionDelayPass]. L'arco segue l'ordine della way: «avanti» è
         * il senso della way. Senza direzione, uno stop o una precedenza vale verso il capo più vicino, e
         * così un semaforo entro [NodeDelays.SIGNAL_APPROACH_M] da un capo (il semaforo di un accesso);
         * due semafori a meno di 40 m nello stesso senso contano una volta.
         */
        private fun interiorDelays(edge: Int, n: Int, flags: IntArray) {
            if (controlNodes.isEmpty || n < 3 || segIds.size() != n) return
            var any = false
            for (k in 1 until n - 1) if (controlNodes.containsKey(segIds.get(k))) { any = true; break }
            if (!any) return
            if (cum.size < n) cum = DoubleArray(n * 2)
            cum[0] = 0.0
            for (k in 1 until n) cum[k] = cum[k - 1] + DistanceCalc.haversine(lats[k - 1], lons[k - 1], lats[k], lons[k])
            val total = cum[n - 1]
            delayFwd.fill(0.0); delayBwd.fill(0.0)
            var lastSignalF = Double.NEGATIVE_INFINITY; var lastSignalB = Double.NEGATIVE_INFINITY
            for (k in 1 until n - 1) {
                val code = controlNodes.getOrDefault(segIds.get(k), 0)
                if (code == 0) continue
                val c = NodeDelays.control(code)
                var dir = NodeDelays.direction(code)
                if (dir == NodeDelays.DIR_BOTH && (c == NodeDelays.Control.STOP || c == NodeDelays.Control.GIVE_WAY))
                    dir = if (total - cum[k] < cum[k]) NodeDelays.DIR_FORWARD else NodeDelays.DIR_BACKWARD
                // il semaforo di un accesso (senza direzione, appena prima di un capo): vale solo verso quel capo,
                // altrimenti chi attraversa l'incrocio pagherebbe anche il semaforo dell'accesso opposto, in uscita
                else if (dir == NodeDelays.DIR_BOTH && c == NodeDelays.Control.SIGNALS && minOf(cum[k], total - cum[k]) <= NodeDelays.SIGNAL_APPROACH_M)
                    dir = if (total - cum[k] < cum[k]) NodeDelays.DIR_FORWARD else NodeDelays.DIR_BACKWARD
                var fwd = dir != NodeDelays.DIR_BACKWARD
                var bwd = dir != NodeDelays.DIR_FORWARD
                if (c == NodeDelays.Control.SIGNALS) {
                    if (fwd) { if (cum[k] - lastSignalF < 40) fwd = false else lastSignalF = cum[k] }
                    if (bwd) { if (cum[k] - lastSignalB < 40) bwd = false else lastSignalB = cum[k] }
                }
                if (!fwd && !bwd) continue
                // uno stop, una precedenza o un semaforo appena prima del capo: lì la precedenza implicita non serve
                if (c == NodeDelays.Control.STOP || c == NodeDelays.Control.GIVE_WAY || c == NodeDelays.Control.SIGNALS) {
                    if (fwd && total - cum[k] <= 30) nearEnd.set(edge * 2)
                    if (bwd && cum[k] <= 30) nearEnd.set(edge * 2 + 1)
                }
                delayEvents[c.ordinal]++
                for (i in profiles.indices) {
                    val r = wayResults[i] ?: continue
                    val p = profiles[i].profile
                    if (fwd && r.forward) delayFwd[i] += NodeDelays.seconds(c, r.speedKmh, p.acceleration, p.deceleration)
                    if (bwd && r.backward) delayBwd[i] += NodeDelays.seconds(c, r.speedBackwardKmh, p.acceleration, p.deceleration)
                }
            }
            for (i in profiles.indices) if (wayResults[i] != null) profiles[i].writeDelay(flags, delayFwd[i], delayBwd[i])
        }

        /** [startIdx]: l'indice nella way del primo punto dell'arco (l'arco va da lì a startIdx + n - 1). */
        private fun addEdge(way: OsmWay, from: Int, to: Int, startIdx: Int) {
            val n = lats.size
            if (from == to) {
                if (n <= 2) return // anello di due nodi: scartato
                // anello chiuso: l'ultimo pillar diventa torre e si creano due archi
                val mid = graph.addNode(lats[n - 2], lons[n - 2])
                val savedLat = lats[n - 1]; val savedLon = lons[n - 1]
                val midId = segIds.get(n - 2); val savedId = segIds.get(n - 1)
                lats.size = n - 1; lons.size = n - 1; segIds.elementsCount = n - 1
                addEdge(way, from, mid, startIdx)
                lats.clear(); lons.clear(); segIds.clear()
                lats.add(graph.lat(mid)); lons.add(graph.lon(mid)); lats.add(savedLat); lons.add(savedLon)
                segIds.add(midId); segIds.add(savedId)
                addEdge(way, mid, to, startIdx + n - 2)
                return
            }
            var dist = DistanceCalc.length(lats.array, lons.array, n)
            if (dist < 0.001) dist = 0.001
            val edge = graph.addEdge(from, to, dist)
            if (n > 2) {
                val pl = PointList(n - 2)
                for (i in 1 until n - 1) pl.add(lats[i], lons[i])
                graph.setWayGeometry(edge, pl)
            }
            val isFerry = way.tag("highway") == null && way.hasTag("route", "ferry", "shuttle_train")
            val flags = graph.encodedValues.newFlags()
            encoding.shared.write(flags, tollFromNodes(way, WayAttributeParser.parse(way, isFerry, widthFromCarriageway = options.widthFromCarriageway, privateMainRoads = privateMainRoads), n))
            val ferryDuration = if (isFerry) OsmValues.parseDurationSeconds(way.tag("duration")) else -1L
            var baseHgvSpeed = Double.NaN
            for (i in profiles.indices) {
                // i traghetti: la velocità dalla durata e dalla lunghezza del tratto; le strade: la linea d'aria della way (ORS)
                var r = profiles[i].profile.analyse(way, wayAccess[i], if (isFerry) dist else wayBeeline, ferryDuration)
                if (i == hgvIndex && r != null) baseHgvSpeed = r.speedKmh
                // modello NAVMASTER, camion: salite, discese e curve (le velocità diventano diverse per senso)
                if (r != null && terrainProfile[i] && !isFerry) r = withTerrain(i, way, r, startIdx, n)
                wayResults[i] = r
                profiles[i].write(flags, r)
            }
            if (nodeDelays) interiorDelays(edge, n, flags)
            speedStats(way, dist, baseHgvSpeed)
            graph.setFlags(edge, flags)
            val name = wayName(way)
            if (name.isNotEmpty()) graph.setName(edge, name)
            guidance?.let { g ->
                var special: StringBuilder? = null
                if (!specialNodes.isEmpty && segIds.size() == n) {
                    for (k in 0 until n) {
                        val t = specialNodes.get(segIds.get(k)) ?: continue
                        if (special == null) special = StringBuilder() else special.append(';')
                        special.append(t).append('@').append(k)
                    }
                }
                val jA = if (segIds.size() == n) junctions.get(segIds.get(0)) else null
                val jB = if (segIds.size() == n) junctions.get(segIds.get(n - 1)) else null
                // cartelli delle relazioni destination_sign verso questa way: all'imbocco dal nodo dell'incrocio
                val rs = relationSigns[way.id]
                val sA = if (rs != null && segIds.size() == n) rs.firstOrNull { it.first == segIds.get(0) }?.second else null
                val sB = if (rs != null && segIds.size() == n) rs.firstOrNull { it.first == segIds.get(n - 1) }?.second else null
                g.set(edge, GuidanceParser.parse(way), jA, jB, special?.toString(), sA, sB)
            }
        }
    }

    /**
     * Le statistiche delle velocità del camion (ImportStats): la media per classe coi due sensi (in salita
     * e in discesa differiscono); la quota lenta delle strade col cartello alto sulla velocità di base
     * ([baseKmh], prima del terreno: misura gli artefatti del modello, non le montagne).
     */
    private fun speedStats(way: OsmWay, dist: Double, baseKmh: Double) {
        if (hgvIndex < 0) return
        val r = wayResults[hgvIndex] ?: return
        if (!r.forward && !r.backward) return
        val rc = WayAttributeParser.roadClass(way)
        val mean = if (r.forward && r.backward) (r.speedKmh + r.speedBackwardKmh) / 2 else if (r.forward) r.speedKmh else r.speedBackwardKmh
        classLength[rc.ordinal] += dist; classSpeedLength[rc.ordinal] += dist * mean
        val h = way.tag("highway")
        if (h == "primary" || h == "secondary") {
            var signed = Double.NaN
            for (k in arrayOf("maxspeed", "maxspeed:forward", "maxspeed:backward")) {
                val v = ZoneInfo.parseSigned(way.tag(k)); if (!v.isNaN() && (signed.isNaN() || v < signed)) signed = v
            }
            if (!signed.isNaN() && signed > 50) {
                signedKm += dist / 1000
                if ((if (baseKmh.isNaN()) r.speedKmh else baseKmh) < 30) slowSignedKm += dist / 1000
            }
        }
    }

    /** Il tag `direction` di un autovelox in gradi (numero o punto cardinale), -1 se assente. */
    private fun bearingOf(raw: String?): Int {
        val v = raw?.trim()?.substringBefore(';')?.lowercase() ?: return -1
        v.toDoubleOrNull()?.let { return ((it.toInt() % 360) + 360) % 360 }
        return when (v) {
            "n", "north" -> 0; "ne" -> 45; "e", "east" -> 90; "se" -> 135; "s", "south" -> 180; "sw" -> 225; "w", "west" -> 270; "nw" -> 315
            else -> -1
        }
    }

    /** Nome come GraphHopper: `name` (o `name:<lingua>`), più `, ref`; `;` diventa `, `. */
    private fun wayName(way: OsmWay): String {
        val name = way.tag("name") ?: ""
        val ref = way.tag("ref") ?: ""
        val s = when {
            name.isEmpty() -> ref
            ref.isEmpty() -> name
            else -> "$name, $ref"
        }
        return s.replace(";", ", ")
    }

    /** Lista di double senza boxing, con accesso diretto all'array per i calcoli di distanza. */
    private class DoubleArrayList {
        var array = DoubleArray(64); private set
        var size = 0
        fun add(v: Double) { if (size == array.size) array = array.copyOf(size * 2); array[size++] = v }
        fun clear() { size = 0 }
        operator fun get(i: Int) = array[i]
    }
}
