package it.navmaster.engine.storage

import it.navmaster.engine.RoutingGraph
import it.navmaster.engine.api.DirectionsRequest
import it.navmaster.engine.api.DirectionsService
import it.navmaster.engine.api.Json
import it.navmaster.engine.api.ResponseFormat
import it.navmaster.engine.profile.CarProfile
import it.navmaster.engine.profile.HgvProfile
import it.navmaster.engine.profile.SpeedModel
import it.navmaster.engine.profile.VehicleProfileOptions
import java.io.File
import java.nio.file.Files
import java.security.MessageDigest
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

/**
 * Impronta completa dei dati salvati nel grafo e delle risposte ORS su Heidelberg, per provare che un
 * cambio di formato dei file non cambia nulla: per ogni arco nodi, distanza, flag, nome, tutti i
 * campi di guida e la geometria; per ogni nodo con divieti le svolte vietate; poi le risposte complete
 * (percorso, pesi, istruzioni, extra_info, fatti sulle strade) per molte coppie di punti, con le opzioni
 * dell'app. Usa solo API che c'erano già col formato 4, così lo stesso test gira anche sul motore di
 * prima: scrive le impronte e i byte di dati veri in `build/fingerprint`, da confrontare fra i due motori.
 *
 * Qui controlla che il grafo appena costruito, quello riaperto dal disco, quello riaperto senza le CH
 * semplici (che le richieste dell'app non leggono mai) e quello costruito coi file mappati diano le
 * stesse impronte.
 */
class GraphFormatFingerprintTest {
    private val pbf = File("../data/heidelberg.test.pbf")

    @Test
    fun `ors and navmaster graphs - same data and answers after reload and without the plain CH`() {
        val out = File("build/fingerprint").apply { mkdirs() }
        val kinds = listOf(
            "ors" to listOf(CarProfile(), HgvProfile()),
            "navmaster" to listOf(
                CarProfile(VehicleProfileOptions(useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")),
                HgvProfile(VehicleProfileOptions(maximumGradeLevel = 1, useAcceleration = false, blockFords = false, speedModel = SpeedModel.NAVMASTER, country = "DE")),
            ),
        )
        for ((kind, profiles) in kinds) {
            val dir = Files.createTempDirectory("nme-fp-$kind").toFile()
            try {
                val (built, _) = RoutingGraph.build(pbf, dir, profiles)
                val dataBuilt: String
                val answersBuilt: String
                built.use { rg ->
                    rg.prepareCH()
                    dataBuilt = dataDump(rg)
                    answersBuilt = answersDump(rg)
                }
                println("byte di dati $kind:\n" + dataBytes(dir))
                val sizes = dir.listFiles()!!.filter { it.isFile }.sortedBy { it.name }.joinToString("\n") { "${it.name}\t${it.length()}" }
                val (dataLoaded, answersLoaded) = RoutingGraph.load(dir).use { dataDump(it) to answersDump(it) }
                assertEquals(sha(dataBuilt), sha(dataLoaded), "$kind: dati del grafo diversi dopo la riapertura")
                assertEquals(sha(answersBuilt), sha(answersLoaded), "$kind: risposte diverse dopo la riapertura")
                // senza i file delle CH semplici (grafo-nm.yml non li pubblica): stesse risposte
                val plain = dir.listFiles()!!.filter { f -> (f.name.startsWith("shortcuts_") || f.name.startsWith("levels_")) && !f.name.endsWith("_core") }
                assertTrue(plain.size == 8, "$kind: attesi 8 file di CH semplici, trovati ${plain.map { it.name }}")
                plain.forEach { it.delete() }
                val answersNoPlain = RoutingGraph.load(dir).use { rg ->
                    assertTrue(rg.chStorages.keys.all { it.endsWith("_core") }, "$kind: CH semplici caricate senza file: ${rg.chStorages.keys}")
                    answersDump(rg)
                }
                assertEquals(sha(answersBuilt), sha(answersNoPlain), "$kind: risposte diverse senza le CH semplici")
                // costruito con i file mappati in scrittura, come grafo-nm.yml per i Paesi grandi (--mmap)
                val mdir = Files.createTempDirectory("nme-fp-$kind-mmap").toFile()
                try {
                    RoutingGraph.build(pbf, mdir, profiles, mmap = true).first.use { it.prepareCH() }
                    val (dataMmap, answersMmap) = RoutingGraph.load(mdir).use { dataDump(it) to answersDump(it) }
                    assertEquals(sha(dataBuilt), sha(dataMmap), "$kind: dati diversi costruendo coi file mappati")
                    assertEquals(sha(answersBuilt), sha(answersMmap), "$kind: risposte diverse costruendo coi file mappati")
                    assertTrue(!File(mdir, "turn_heads").exists() && !File(mdir, "guidance.import").exists(), "$kind: file di lavoro rimasti")
                } finally { mdir.deleteRecursively() }
                File(out, "$kind-data.txt").writeText(dataBuilt)
                File(out, "$kind-answers.txt").writeText(answersBuilt)
                File(out, "$kind-sizes.txt").writeText(sizes + "\n" + dataBytes(dir))
                println("impronta $kind: dati ${sha(dataBuilt).take(16)} risposte ${sha(answersBuilt).take(16)}\n$sizes")
            } finally {
                dir.deleteRecursively()
            }
        }
    }

    /** Tutti i dati per arco e le svolte vietate, in testo. */
    private fun dataDump(rg: RoutingGraph): String {
        val g = rg.graph
        val gd = rg.guidance
        val sb = StringBuilder()
        val flags = IntArray(g.intsForFlags)
        for (e in 0 until g.edgeCount) {
            g.getFlags(e, flags)
            sb.append(e).append(' ').append(g.nodeA(e)).append(' ').append(g.nodeB(e)).append(' ').append(g.distance(e))
                .append(' ').append(flags.joinToString(",")).append(" n=").append(g.name(e))
            if (gd != null) {
                for (r in listOf(false, true)) {
                    sb.append(" | ").append(gd.maxSpeed(e, r)).append(',').append(gd.maxSpeedHgv(e, r)).append(',').append(gd.lanesExplicit(e, r))
                        .append(',').append(gd.lanes(e, r)).append(',').append(gd.turnLanes(e, r)).append(',').append(gd.destination(e, r))
                        .append(',').append(gd.destinationRef(e, r)).append(',').append(gd.junctionAtStart(e, r)).append(',').append(gd.laneSigns(e, r))
                        .append(',').append(gd.relationSign(e, r))
                }
                sb.append(" | ").append(gd.lanesTotal(e)).append(',').append(gd.tunnel(e)).append(',').append(gd.bridge(e)).append(',').append(gd.oneway(e))
                    .append(',').append(gd.ref(e)).append(',').append(gd.string(gd.refPointer(e))).append(',').append(gd.specialNodes(e))
                    .append(',').append(gd.conditional(e)).append(',').append(gd.adrTunnel(e)).append(',').append(gd.widthSigned(e))
            }
            for (mode in listOf(FetchMode.ALL, FetchMode.PILLAR_ONLY)) {
                val pl = g.fetchWayGeometry(e, false, mode)
                sb.append(" g")
                for (i in 0 until pl.size) sb.append(' ').append(pl.lat(i)).append(',').append(pl.lon(i))
            }
            val rev = g.fetchWayGeometry(e, true, FetchMode.BASE_AND_PILLAR)
            sb.append(" r")
            for (i in 0 until rev.size) sb.append(' ').append(rev.lat(i)).append(',').append(rev.lon(i))
            sb.append('\n')
        }
        // le svolte vietate: per ogni nodo con voci, ogni coppia di archi incidenti
        val it1 = g.createEdgeIterator(); val it2 = g.createEdgeIterator()
        for (n in 0 until g.nodeCount) {
            if (!rg.turnCosts.hasEntries(n)) continue
            sb.append("t ").append(n)
            it1.setBaseNode(n)
            while (it1.next()) {
                it2.setBaseNode(n)
                while (it2.next()) {
                    val f = rg.turnCosts.flags(it1.edge, n, it2.edge)
                    if (f != 0) sb.append(' ').append(it1.edge).append('>').append(it2.edge).append('=').append(f)
                }
            }
            sb.append('\n')
        }
        sb.append("nodes=").append(g.nodeCount).append(" edges=").append(g.edgeCount).append(" turnEntries=").append(rg.turnCosts.count).append('\n')
        return sb.toString()
    }

    /** Le risposte ORS complete (senza i metadati con l'ora) per coppie di punti fisse e le opzioni dell'app. */
    private fun answersDump(rg: RoutingGraph): String {
        val service = DirectionsService(rg)
        val g = rg.graph
        val rnd = java.util.Random(20261119)
        val sb = StringBuilder()
        val truck = """"profile_params":{"restrictions":{"height":4.0,"width":2.55,"length":16.5,"weight":27.0,"axleload":11.5}}"""
        val common = """"units":"m","instructions":true,"language":"it","maneuvers":true,"extra_info":["tollways","waycategory","waytype","surface","roadaccessrestrictions","osmid","avgspeed"],"geometry":true,"radiuses":[-1,-1]"""
        val bodies = listOf(
            "driving-hgv" to """{COORDS,$common,"preference":"recommended","navmaster":{"roads":true},"options":{"avoid_features":["ferries"],"vehicle_type":"hgv",$truck},"maximum_speed":80.0}""",
            "driving-hgv" to """{COORDS,$common,"preference":"shortest","navmaster":{"roads":true},"options":{"vehicle_type":"hgv",$truck}}""",
            "driving-hgv" to """{COORDS,$common,"preference":"recommended","navmaster":{"roads":true,"avoid_penalty":true},"options":{"avoid_features":["tollways"],"vehicle_type":"hgv",$truck}}""",
            "driving-hgv" to """{COORDS,$common,"preference":"recommended"}""",
            "driving-hgv" to """{COORDS,$common,"preference":"recommended","options":{"avoid_features":["highways"]}}""",
            "driving-car" to """{COORDS,$common,"preference":"fastest","navmaster":{"roads":true}}""",
            "driving-car" to """{COORDS,$common,"preference":"shortest"}""",
            "driving-car" to """{COORDS,$common,"preference":"fastest","options":{"avoid_features":["tollways","ferries"],"profile_params":{"restrictions":{"height":3.2,"weight":3.5}}}}""",
        )
        val alternatives = """{COORDS,$common,"preference":"recommended","navmaster":{"roads":true,"alternatives":{"count":3,"tier1_factor":1.25,"tier1_km_factor":1.25,"max_factor":1.5,"max_km_factor":1.6}},"options":{"vehicle_type":"hgv",$truck}}"""
        for (k in 0 until 60) {
            // punti vicini a nodi a caso del grafo (nel riquadro dell'estratto le strade sono rade)
            val na = rnd.nextInt(g.nodeCount); val nc = rnd.nextInt(g.nodeCount)
            val a = doubleArrayOf(g.lon(na) + (rnd.nextDouble() - 0.5) * 0.002, g.lat(na) + (rnd.nextDouble() - 0.5) * 0.002)
            val c = doubleArrayOf(g.lon(nc) + (rnd.nextDouble() - 0.5) * 0.002, g.lat(nc) + (rnd.nextDouble() - 0.5) * 0.002)
            val coords = "\"coordinates\":[[${a[0]},${a[1]}],[${c[0]},${c[1]}]]"
            val list = if (k % 10 == 0) bodies + ("driving-hgv" to alternatives) else bodies
            for ((profile, body) in list) {
                sb.append(k).append(' ').append(profile).append(' ')
                try {
                    @Suppress("UNCHECKED_CAST")
                    val req = DirectionsRequest.fromJson(profile, Json.parse(body.replace("COORDS", coords)) as Map<String, Any?>)
                    val res = service.handle(req, ResponseFormat.GEOJSON)
                    val m = res.toMutableMap()
                    m.remove("metadata")
                    sb.append(Json.stringify(m))
                } catch (e: Exception) {
                    sb.append("ERR ").append(e.javaClass.simpleName).append(' ').append(e.message)
                }
                sb.append('\n')
            }
        }
        return sb.toString()
    }

    /**
     * I byte di dati veri dei file che cambiano formato (i file sono arrotondati al segmento da 1 MiB:
     * su Heidelberg la loro lunghezza non mostra nulla), dalle intestazioni, per il formato 4 e il 5.
     */
    private fun dataBytes(dir: File): String {
        val gd = GraphDirectory(dir, mmap = true, readOnly = true)
        val sb = StringBuilder()
        gd.open("geometry")?.let { g ->
            val next = (g.getHeader(1).toLong() and 0xFFFFFFFFL) or (g.getHeader(2).toLong() shl 32)
            sb.append("geometry\t").append(if (g.getHeader(3) == 1) next else next * 4).append('\n'); g.close()
        }
        gd.open("names")?.let { n -> sb.append("names\t").append((n.getHeader(1).toLong() and 0xFFFFFFFFL) or (n.getHeader(2).toLong() shl 32)).append('\n'); n.close() }
        gd.open("guidance")?.let { g ->
            val n = g.getHeader(2).toLong()
            sb.append("guidance\t").append(if (g.getHeader(1) == 72) n * 72 else ((n + 63) / 64) * 16 + g.getHeader(3) * 10L + g.getHeader(4) * 72L).append('\n'); g.close()
        }
        val ti = gd.open("turn_index", 1 shl 16)
        if (ti != null) { sb.append("turn_index\t").append(((ti.getHeader(1) + 63L) / 64) * 8 + ti.getHeader(2) * 4L).append('\n'); ti.close() }
        else gd.open("turn_heads", 1 shl 16)?.let { h -> sb.append("turn_heads\t").append(h.getHeader(1) * 4L).append('\n'); h.close() }
        return sb.toString()
    }

    private fun sha(s: String): String = MessageDigest.getInstance("SHA-256").digest(s.toByteArray()).joinToString("") { "%02x".format(it) }
}
