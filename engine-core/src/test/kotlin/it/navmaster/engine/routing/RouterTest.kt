package it.navmaster.engine.routing

import it.navmaster.engine.index.LocationIndex
import it.navmaster.engine.osm.OsmGraphImporter
import it.navmaster.engine.profile.AvoidFeatureFlags
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HeavyVehicleAttributes
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.storage.BaseGraph
import it.navmaster.engine.util.DistanceCalc
import it.navmaster.engine.storage.GraphDirectory
import it.navmaster.engine.storage.GraphEncoding
import it.navmaster.engine.weighting.EdgeFilterFactory
import it.navmaster.engine.weighting.VehicleParameters
import it.navmaster.engine.weighting.WeightingFactory
import it.navmaster.engine.weighting.WeightingMethod
import java.io.File
import java.nio.file.Files
import kotlin.test.AfterTest
import kotlin.test.BeforeTest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFailsWith
import kotlin.test.assertTrue

class RouterTest {
    private lateinit var graph: BaseGraph
    private lateinit var encoding: GraphEncoding
    private lateinit var index: LocationIndex

    // Heidelberg: Bismarckplatz e Hauptbahnhof (circa 1,6 km in linea d'aria)
    private val bismarckplatz = 49.4094 to 8.6937
    private val hauptbahnhof = 49.4036 to 8.6757
    private val ziegelhausen = 49.4186 to 8.7606

    @BeforeTest
    fun setUp() {
        encoding = GraphEncoding(listOf(CarProfile(), HgvProfile()))
        graph = BaseGraph(GraphDirectory(Files.createTempDirectory("nme-route").toFile().also { it.deleteOnExit() }), encoding.registry)
        OsmGraphImporter(graph, encoding).import(File("../data/heidelberg.test.pbf"))
        index = LocationIndex.build(graph)
    }

    @AfterTest
    fun tearDown() = graph.close()

    private fun router(profile: String, method: WeightingMethod = WeightingMethod.FASTEST, avoid: Int = 0, params: VehicleParameters? = null): Router {
        val pe = encoding.profile(profile)
        val filter = EdgeFilterFactory.create(encoding, profile, avoid, params = params)
        return Router(graph, index, WeightingFactory.create(encoding, pe, method), filter)
    }

    private fun Router.go(from: Pair<Double, Double>, to: Pair<Double, Double>): RoutePath =
        route(snap(from.first, from.second, 400.0, 0), snap(to.first, to.second, 400.0, 1))

    @Test
    fun `location index snaps close to the query`() {
        val s = index.findClosest(bismarckplatz.first, bismarckplatz.second)
        assertTrue(s.isValid)
        assertTrue(s.distance < 50, "snap a ${s.distance} m")
        assertTrue(index.cells > 100 && index.entries >= graph.edgeCount)
    }

    @Test
    fun `car route across the city centre`() {
        val r = router("car-ors")
        val s1 = r.snap(bismarckplatz.first, bismarckplatz.second, 400.0, 0)
        val s2 = r.snap(hauptbahnhof.first, hauptbahnhof.second, 400.0, 1)
        println("snap1 ${s1.position} d=${s1.distance} edge=${s1.edge} wayIndex=${s1.wayIndex} (${s1.snappedLat},${s1.snappedLon})")
        println("snap2 ${s2.position} d=${s2.distance} edge=${s2.edge} wayIndex=${s2.wayIndex} (${s2.snappedLat},${s2.snappedLon})")
        val path = r.route(s1, s2)
        assertTrue(path.found, "percorso vuoto")
        assertTrue(path.distance in 1500.0..4000.0, "distanza ${path.distance}")
        assertTrue(path.millis in 60_000..900_000, "tempo ${path.millis} ms")
        val pts = path.points()
        println("archi=${path.edges.size} punti=${pts.size} primo=(${pts.lat(0)},${pts.lon(0)}) ultimo=(${pts.lat(pts.size - 1)},${pts.lon(pts.size - 1)})")
        assertTrue(pts.size > 10, "pochi punti: ${pts.size}")
        // la geometria parte e arriva vicino ai punti richiesti
        val dStart = DistanceCalc.haversine(pts.lat(0), pts.lon(0), bismarckplatz.first, bismarckplatz.second)
        val dEnd = DistanceCalc.haversine(pts.lat(pts.size - 1), pts.lon(pts.size - 1), hauptbahnhof.first, hauptbahnhof.second)
        // la geometria deve iniziare e finire esattamente sui punti agganciati (la distanza di aggancio dipende dalla mappa)
        assertTrue(dStart <= s1.distance + 1, "inizio a $dStart m dal punto richiesto (snap a ${s1.distance} m)")
        assertTrue(dEnd <= s2.distance + 1, "fine a $dEnd m dal punto richiesto (snap a ${s2.distance} m)")
        // gli archi sono contigui
        for (i in 1 until path.edges.size) assertEquals(path.edges[i - 1].adjNode, path.edges[i].baseNode, "arco $i non contiguo")
        // la lunghezza della geometria coincide con la distanza dichiarata
        var geomLen = 0.0
        for (i in 1 until pts.size) geomLen += DistanceCalc.haversine(pts.lat(i - 1), pts.lon(i - 1), pts.lat(i), pts.lon(i))
        assertEquals(path.distance, geomLen, path.distance * 0.02)
    }

    @Test
    fun `shortest is not longer than fastest`() {
        val fastest = router("car-ors", WeightingMethod.FASTEST).go(bismarckplatz, ziegelhausen)
        val shortest = router("car-ors", WeightingMethod.SHORTEST).go(bismarckplatz, ziegelhausen)
        assertTrue(shortest.distance <= fastest.distance + 1, "shortest ${shortest.distance} > fastest ${fastest.distance}")
        assertTrue(fastest.millis <= shortest.millis + 1000, "fastest ${fastest.millis} > shortest ${shortest.millis}")
    }

    @Test
    fun `truck is slower than car on the same trip`() {
        val car = router("car-ors").go(bismarckplatz, ziegelhausen)
        val truck = router("heavyvehicle").go(bismarckplatz, ziegelhausen)
        assertTrue(truck.found && car.found)
        assertTrue(truck.millis > car.millis, "camion ${truck.millis} ms, auto ${car.millis} ms")
    }

    @Test
    fun `truck restrictions change or block the route`() {
        val free = router("heavyvehicle").go(bismarckplatz, ziegelhausen)
        // quanti archi bloccherebbe ogni limite (diagnostica)
        val sh = encoding.shared
        val it = graph.createEdgeIterator()
        var h = 0; var w = 0; var wd = 0; var l = 0; var hgvNo = 0
        for (e in 0 until graph.edgeCount) {
            it.setEdge(e, graph.nodeA(e))
            if (it.get(sh.maxHeight) < 4.5) h++
            if (it.get(sh.maxWeight) < 40.0) w++
            if (it.get(sh.maxWidth) < 2.55) wd++
            if (it.get(sh.maxLength) < 18.75) l++
            if (it.get(sh.hgvBlocked) and HeavyVehicleAttributes.HGV != 0) hgvNo++
        }
        println("archi bloccati: altezza<4.5: $h, peso<40: $w, larghezza<2.55: $wd, lunghezza<18.75: $l, hgv=no: $hgvNo su ${graph.edgeCount}")
        assertTrue(h > 0 && w > 0, "nessun limite di altezza/peso nell'estratto")
        // un camion medio (12 t, 3,8 m) deve trovare strada e il costo non può diminuire
        val medium = router("heavyvehicle", params = VehicleParameters(weight = 12.0, height = 3.8)).go(bismarckplatz, ziegelhausen)
        assertTrue(medium.found)
        assertTrue(medium.weight >= free.weight - 1e-6, "con restrizioni il costo non può diminuire")
        // un bilico completo può non avere percorso in centro: accettiamo entrambi gli esiti, ma senza errori diversi
        val heavy = runCatching { router("heavyvehicle", params = VehicleParameters(weight = 40.0, height = 4.5, width = 2.55, length = 18.75)).go(bismarckplatz, ziegelhausen) }
        heavy.exceptionOrNull()?.let { ex -> assertTrue(ex is RouteNotFoundException, "errore inatteso: $ex") }
        heavy.getOrNull()?.let { p -> assertTrue(p.weight >= medium.weight - 1e-6) }
    }

    @Test
    fun `avoid highways removes motorway edges`() {
        val path = router("car-ors", avoid = AvoidFeatureFlags.HIGHWAYS).go(bismarckplatz, ziegelhausen)
        for (e in path.edges) assertTrue(!graph.createEdgeIterator().setEdge(e.edge, e.baseNode).get(encoding.shared.motorway), "autostrada nel percorso")
    }

    @Test
    fun `point far from roads is rejected`() {
        val ex = assertFailsWith<PointNotFoundException> { router("car-ors").snap(49.6, 9.5, 400.0, 0) }
        assertEquals(0, ex.pointIndex)
    }

    @Test
    fun `same point gives empty route`() {
        val r = router("car-ors")
        val s = r.snap(bismarckplatz.first, bismarckplatz.second, 400.0, 0)
        val path = r.route(s, s)
        assertEquals(0.0, path.distance)
    }
}
