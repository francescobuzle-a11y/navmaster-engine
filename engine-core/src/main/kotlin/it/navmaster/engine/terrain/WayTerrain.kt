package it.navmaster.engine.terrain

import it.navmaster.engine.osm.OsmWay
import it.navmaster.engine.profile.ClimbModel
import it.navmaster.engine.util.DistanceCalc
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.ceil
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Il terreno di una way OSM intera (modello NAVMASTER, solo mezzi pesanti): la pendenza lungo la way dal
 * DEM e le curve dalla geometria. Si calcola una volta per way, non per arco (gli archi fra due incroci
 * sono spesso di 20-100 m, troppo corti per una pendenza onesta); ogni arco ne legge il pezzo fra i suoi
 * due nodi ([nodeCum] per indice del nodo nella way). La pendenza è nel senso della way (positiva in
 * salita); la way è divisa in celle di circa [CELL_M] m per il profilo di velocità ([TerrainSpeeds]).
 *
 * Pendenza. Un DEM di superficie (Copernicus) ha gradini di 5-20 m ai margini dei boschi e sui paesi: una
 * regressione su 450 m ne fa una pendenza finta del 5 % (gradino di 15 m a metà finestra). Per questo il
 * profilo delle quote passa prima da un filtro del minimo su ~90 m (±[MIN_FILTER_HALF_M] m: via file
 * d'alberi, case, capannoni più stretti) e la pendenza è la mediana delle pendenze delle sottocorde di
 * [LAG_M] m dentro la finestra ([windowSlope]): un gradino ne tocca 4 su 16, la mediana resta zero; una
 * salita vera le alza tutte.
 * - way lunghe (≥ [LONG_WAY_M] m, non ponti né gallerie): quote del DEM ogni ~[DEM_STEP_M] m lungo la way,
 *   pendenza di ogni tratto dalla finestra di ~450 m centrata sul tratto e spostata dentro la way vicino
 *   ai capi (mai tagliata: una finestra di 200 m non distingue un gradino da una rampa);
 * - way corte (non ponti né gallerie): una pendenza sola, la più piccola di due stime che devono dare lo
 *   stesso segno (altrimenti zero): (a) la finestra di 450 m centrata sulla way, prolungata in linea
 *   retta oltre i capi (un gradino del bosco a cavallo di una way di 150 m resta un gradino in mezzo a
 *   450 m); (b) i capi della way, ciascuno la mediana del DEM entro [END_RADIUS_M] m (sotto
 *   [SHORT_BASELINE_M] m di corda la base si allunga lungo la corda, perché 1-2 m di rumore su 20 m sono
 *   già il 5-10 %). (a) da sola sbaglierebbe sui tornanti (il prolungamento entra nel versante), (b) da
 *   sola sui gradini: la più piccola delle due non inventa salite;
 * - ponti, gallerie, coperture (`bridge`, `tunnel`, `covered`, `man_made=bridge`): una pendenza sola dai
 *   capi, ciascuno la mediana dei campioni FUORI dalla struttura, da [APPROACH_FROM_M] a [APPROACH_TO_M] m
 *   oltre il capo sul prolungamento della way (per un ponte il DEM in mezzo e intorno ai capi è la valle,
 *   per una galleria il monte; una spalla sull'orlo di una gola non porta la gola nella media); con la
 *   stessa regola delle way corte (la più piccola, stesso segno) contro la tendenza dei due accessi fino
 *   a 255 m oltre i capi (filtro del minimo e mediana delle sottocorde), perché un bosco o un paese
 *   contro una sola spalla non diventi una salita;
 * - `incline` numerico (`8%`, `-8 %`, `4°`; `up`/`down` non dicono quanto) vale al posto del DEM sulle
 *   way corte e dove il DEM manca;
 * - sotto il [MIN_GRADE] la pendenza è zero (il camion non rallenta e il DEM lì è rumore); poi il tetto
 *   per classe ([maxGrade]): autostrade e superstrade 7 %, statali e rampe 10 %, le altre 16 %.
 *
 * Curve: la geometria è diradata a punti distanti almeno [THIN_M] m; i vertici consecutivi che girano
 * nello stesso verso (più di [TURN_MIN_DEG]°) fanno una curva sola, col raggio più piccolo dei suoi
 * vertici ((l1 + l2)/2 / Δθ); un gomito solo ha il raggio del cerchio tangente ai due lati entro metà del
 * lato più corto (min(l1, l2)/2 / tan(Δθ/2)): con (l1 + l2)/2 un incrocio fra un lato di 15 m e uno di
 * 200 m dava un raggio di 70 m, cioè quasi nessuna frenata, solo perché il lato lungo è diritto. Conta solo se gira più di [CURVE_MIN_DEG]° entro [CURVE_WINDOW_M] m (il
 * rumore delle geometrie OSM e le curve larghe, che un camion fa comunque alla sua velocità, no). Un
 * gomito isolato subito annullato dal gomito opposto è uno spostamento laterale (corsie, disegno), non
 * una curva. Le rotatorie hanno già il loro tetto ([it.navmaster.engine.profile.VehicleProfile]).
 */
class WayTerrain private constructor(
    /** Lunghezza della way (m, somma degli haversine come gli archi). */
    val length: Double,
    /** Distanza progressiva di ogni nodo della way, per indice (i nodi mancanti dall'estratto ripetono il precedente). */
    val nodeCum: DoubleArray,
    val cells: Int,
    val cellLen: Double,
    /** Pendenza di ogni cella nel senso della way; null se la way è in piano (o senza DEM). */
    val grade: DoubleArray?,
    /** Le curve: inizio e fine (m lungo la way) e raggio (m). */
    val curveFrom: DoubleArray,
    val curveTo: DoubleArray,
    val curveRadius: DoubleArray,
    /** Rampa di svincolo (`*_link`): curve con [ClimbModel.lateralAccelLink]. */
    val isLink: Boolean,
    /** Quota della way coperta dal DEM (0..1), NaN senza DEM. */
    val demCoverage: Double,
) {
    val curves: Int get() = curveRadius.size

    /** Né pendenza né curve: le velocità della way restano quelle di base. */
    val neutral: Boolean get() = grade == null && curveRadius.isEmpty()

    companion object {
        /** Il passo del profilo di velocità: un tornante di 12 m è lungo ~38 m, e la curva vale per celle intere. */
        const val CELL_M = 5.0
        const val DEM_STEP_M = 25.0
        /** Da qui in su la pendenza varia lungo la way: ogni finestra di 450 m sta tutta dentro la way. */
        const val LONG_WAY_M = 450.0
        const val FIT_HALF_WINDOW_M = 225.0
        /** Il filtro del minimo sul profilo delle quote: ±45 m (~90 m, due campioni per parte a 25 m). */
        const val MIN_FILTER_HALF_M = 45.0
        /** La lunghezza delle sottocorde di cui si prende la pendenza mediana. */
        const val LAG_M = 100.0
        /** Le sottocorde valide che servono per una pendenza (meno: il DEM manca). */
        const val MIN_LAGS = 5
        const val END_RADIUS_M = 50.0
        const val SHORT_BASELINE_M = 60.0
        /** I campioni dei capi di ponti e gallerie: da 30 a 80 m oltre il capo, ogni 10 m, sul prolungamento. */
        const val APPROACH_FROM_M = 30.0
        const val APPROACH_TO_M = 80.0
        const val APPROACH_STEP_M = 10.0
        /** La direzione del prolungamento oltre un capo: dagli ultimi ~30 m della way. */
        const val APPROACH_DIR_M = 30.0
        const val MIN_GRADE = 0.02
        const val THIN_M = 15.0
        const val TURN_MIN_DEG = 4.0
        const val CURVE_WINDOW_M = 60.0
        const val CURVE_MIN_DEG = 30.0
        /** Di quanto una curva si allunga oltre il primo e l'ultimo vertice (al più metà della corda vicina). */
        const val CURVE_EDGE_M = 10.0

        private val TURN_MIN = Math.toRadians(TURN_MIN_DEG)
        private val CURVE_MIN = Math.toRadians(CURVE_MIN_DEG)

        /** Il tetto della pendenza per classe di strada (frazione). */
        fun maxGrade(highway: String?): Double = when (highway) {
            "motorway", "trunk", "motorroad" -> 0.07
            "primary", "motorway_link", "trunk_link", "primary_link" -> 0.10
            else -> 0.16
        }

        /** Ponte, galleria o tratto coperto: la pendenza dai capi, non dal DEM in mezzo. */
        fun isStructure(way: OsmWay): Boolean {
            fun yes(k: String) = way.tag(k)?.let { it != "no" } ?: false
            return yes("bridge") || yes("tunnel") || way.hasTag("covered", "yes") || way.hasTag("man_made", "bridge")
        }

        /** `incline` numerico come frazione (positiva in salita nel senso della way); NaN se assente o senza numero. */
        fun parseIncline(raw: String?): Double {
            var v = raw?.trim()?.lowercase()?.replace(',', '.') ?: return Double.NaN
            if (v.isEmpty()) return Double.NaN
            val deg = v.endsWith("°") || v.endsWith("deg")
            v = v.removeSuffix("°").removeSuffix("deg").removeSuffix("%").trim()
            val x = v.toDoubleOrNull() ?: return Double.NaN
            val g = if (deg) kotlin.math.tan(Math.toRadians(x)) else x / 100
            return if (abs(g) > 0.5) Double.NaN else g
        }

        /**
         * Analizza una way: [lats]/[lons] i punti in ordine (NaN per i nodi mancanti dall'estratto), [dem]
         * null per le sole curve. [curves] false per la sola pendenza.
         */
        fun analyse(way: OsmWay, lats: DoubleArray, lons: DoubleArray, count: Int, dem: ElevationSource?, curves: Boolean = true): WayTerrain {
            val nodeCum = DoubleArray(count)
            // i punti validi, in metri su un piano locale (x est, y nord) e alla loro distanza lungo la way
            val vx = DoubleArray(count); val vy = DoubleArray(count); val vs = DoubleArray(count)
            val vlat = DoubleArray(count); val vlon = DoubleArray(count)
            var nv = 0
            var lat0 = Double.NaN; var lon0 = Double.NaN; var kx = 0.0
            var s = 0.0
            for (i in 0 until count) {
                val la = lats[i]; val lo = lons[i]
                if (la.isNaN() || lo.isNaN()) { nodeCum[i] = s; continue }
                if (nv == 0) { lat0 = la; lon0 = lo; kx = cos(Math.toRadians(la)) * DistanceCalc.METERS_PER_DEGREE }
                else s += DistanceCalc.haversine(vlat[nv - 1], vlon[nv - 1], la, lo)
                nodeCum[i] = s
                vx[nv] = (lo - lon0) * kx; vy[nv] = (la - lat0) * DistanceCalc.METERS_PER_DEGREE; vs[nv] = s
                vlat[nv] = la; vlon[nv] = lo
                nv++
            }
            val length = s
            val cells = max(1, ceil(length / CELL_M).toInt())
            val cellLen = if (length > 0) length / cells else CELL_M
            val highway = way.tag("highway")
            val isLink = highway?.endsWith("_link") == true
            if (nv < 2 || length < 1.0) return WayTerrain(length, nodeCum, cells, cellLen, null, EMPTY, EMPTY, EMPTY, isLink, Double.NaN)

            // --- pendenza
            var grade: DoubleArray? = null
            var coverage = Double.NaN
            if (dem != null) {
                val pts = Polyline(vlat, vlon, vs, nv)
                val incline = parseIncline(way.tag("incline"))
                val limit = maxGrade(highway)
                fun finish(g: Double, mayUseIncline: Boolean): Double {
                    var x = if (!incline.isNaN() && (mayUseIncline || g.isNaN())) incline else g
                    if (x.isNaN() || abs(x) < MIN_GRADE) return 0.0
                    if (x > limit) x = limit else if (x < -limit) x = -limit
                    return x
                }
                val structure = isStructure(way)
                val ends = Ends(pts, vlat, vlon, nv, length)
                if (structure) {
                    // i capi fuori dalla struttura, sul prolungamento della way (mediana, non media): la quota
                    // mediana sta a ~55 m oltre ogni capo, e la pendenza è su quella distanza
                    val mid = (APPROACH_FROM_M + APPROACH_TO_M) / 2
                    val gEnds = (ends.approach(dem, false) - ends.approach(dem, true)) / (length + 2 * mid)
                    // la tendenza dei due accessi, da 30 m a 255 m oltre ogni capo (la struttura e i 30 m vicini
                    // esclusi): un bosco o un paese contro una sola spalla sposta gEnds, non la tendenza
                    val side = FIT_HALF_WINDOW_M
                    val m = ceil((length + 2 * (APPROACH_FROM_M + side)) / DEM_STEP_M).toInt()
                    val step = (length + 2 * (APPROACH_FROM_M + side)) / m
                    val h = DoubleArray(m + 1) { k ->
                        val t = -(APPROACH_FROM_M + side) + k * step
                        when {
                            t <= -APPROACH_FROM_M -> if (ends.at(true, -t)) dem.ele(ends.lat, ends.lon) else Double.NaN
                            t >= length + APPROACH_FROM_M -> if (ends.at(false, t - length)) dem.ele(ends.lat, ends.lon) else Double.NaN
                            else -> Double.NaN
                        }
                    }
                    val gTrend = windowSlope(minFilter(h, Math.round(MIN_FILTER_HALF_M / step).toInt()), 0, m, max(1, Math.round(LAG_M / step).toInt()), step, DoubleArray(m + 1))
                    val g = agree(gEnds, gTrend)
                    coverage = if (g.isNaN()) 0.0 else 1.0
                    val gg = finish(g, mayUseIncline = length < LONG_WAY_M)
                    if (gg != 0.0) grade = DoubleArray(cells) { gg }
                } else if (length < LONG_WAY_M) {
                    // (b) i capi: la corda; sotto SHORT_BASELINE_M allungata intorno al suo centro, poi riportata lungo la way
                    val dx = vx[nv - 1] - vx[0]; val dy = vy[nv - 1] - vy[0]
                    val chord = hypot(dx, dy)
                    val gEnds = if (chord < 1.0) Double.NaN else if (chord >= SHORT_BASELINE_M) {
                        (ground(dem, vlat[nv - 1], vlon[nv - 1]) - ground(dem, vlat[0], vlon[0])) / length
                    } else {
                        val mx = (vx[0] + vx[nv - 1]) / 2; val my = (vy[0] + vy[nv - 1]) / 2
                        val ux = dx / chord * SHORT_BASELINE_M / 2; val uy = dy / chord * SHORT_BASELINE_M / 2
                        fun latOf(y: Double) = lat0 + y / DistanceCalc.METERS_PER_DEGREE
                        fun lonOf(x: Double) = lon0 + x / kx
                        val ha = ground(dem, latOf(my - uy), lonOf(mx - ux)); val hb = ground(dem, latOf(my + uy), lonOf(mx + ux))
                        (hb - ha) / SHORT_BASELINE_M * (chord / length)
                    }
                    // (a) la finestra di 450 m centrata sulla way, prolungata in linea retta oltre i capi
                    val window = 2 * FIT_HALF_WINDOW_M
                    val e = (window - length) / 2
                    val m = ceil(window / DEM_STEP_M).toInt()
                    val step = window / m
                    val h = DoubleArray(m + 1)
                    pts.reset()
                    for (k in 0..m) {
                        val t = -e + k * step
                        h[k] = when {
                            t < 0 -> if (ends.at(true, -t)) dem.ele(ends.lat, ends.lon) else Double.NaN
                            t > length -> if (ends.at(false, t - length)) dem.ele(ends.lat, ends.lon) else Double.NaN
                            else -> { pts.at(t); dem.ele(pts.lat, pts.lon) }
                        }
                    }
                    val hm = minFilter(h, Math.round(MIN_FILTER_HALF_M / step).toInt())
                    val gWindow = windowSlope(hm, 0, m, max(1, Math.round(LAG_M / step).toInt()), step, DoubleArray(m + 1))
                    val g = agree(gEnds, gWindow)
                    coverage = if (g.isNaN()) 0.0 else 1.0
                    val gg = finish(g, mayUseIncline = true)
                    if (gg != 0.0) grade = DoubleArray(cells) { gg }
                } else {
                    val m = max(1, ceil(length / DEM_STEP_M).toInt())
                    val step = length / m
                    val h = DoubleArray(m + 1)
                    var valid = 0
                    pts.reset()
                    for (j in 0..m) {
                        pts.at(j * step)
                        h[j] = dem.ele(pts.lat, pts.lon)
                        if (!h[j].isNaN()) valid++
                    }
                    coverage = valid.toDouble() / (m + 1)
                    val hm = minFilter(h, Math.round(MIN_FILTER_HALF_M / step).toInt())
                    val lag = max(1, Math.round(LAG_M / step).toInt())
                    val half = Math.round(FIT_HALF_WINDOW_M / step).toInt()
                    val buf = DoubleArray(m + 1)
                    val g = DoubleArray(m)
                    var any = false
                    for (j in 0 until m) {
                        // la finestra centrata sul tratto j (campioni j - half .. j + 1 + half), spostata dentro la way ai capi
                        var lo = j - half; var hi = j + 1 + half
                        if (lo < 0) { hi -= lo; lo = 0 }
                        if (hi > m) { lo = max(0, lo - (hi - m)); hi = m }
                        g[j] = finish(windowSlope(hm, lo, hi, lag, step, buf), mayUseIncline = false)
                        if (g[j] != 0.0) any = true
                    }
                    if (any) grade = DoubleArray(cells) { c -> g[min(m - 1, floor((c + 0.5) * cellLen / step).toInt())] }
                }
            }

            // --- curve
            var cf = EMPTY; var ct = EMPTY; var cr = EMPTY
            val roundabout = way.hasTag("junction", "roundabout", "circular") || way.hasTag("highway", "mini_roundabout")
            if (curves && !roundabout && nv >= 3) {
                val found = findCurves(vx, vy, vs, nv)
                if (found != null) { cf = found[0]; ct = found[1]; cr = found[2] }
            }
            return WayTerrain(length, nodeCum, cells, cellLen, grade, cf, ct, cr, isLink, coverage)
        }

        private val EMPTY = DoubleArray(0)

        /**
         * La quota del terreno vicino a un punto: la mediana del DEM entro [END_RADIUS_M] m (centro, 8 punti a
         * metà raggio, 8 sul bordo); NaN se nessuno. La mediana, non la media: un bosco o un argine su metà
         * del cerchio non sposta la quota.
         */
        fun ground(dem: ElevationSource, lat: Double, lon: Double): Double {
            val kLat = 1 / DistanceCalc.METERS_PER_DEGREE
            val kLon = 1 / (DistanceCalc.METERS_PER_DEGREE * cos(Math.toRadians(lat)))
            val v = DoubleArray(17); var n = 0
            fun add(x: Double) { if (!x.isNaN()) v[n++] = x }
            add(dem.ele(lat, lon))
            for (r in doubleArrayOf(END_RADIUS_M / 2, END_RADIUS_M)) for (k in 0 until 8) {
                val a = k * PI / 4
                add(dem.ele(lat + r * cos(a) * kLat, lon + r * sin(a) * kLon))
            }
            return median(v, n)
        }

        /** Due stime della stessa pendenza: la più piccola se hanno lo stesso segno, zero se no (una sola se l'altra manca). */
        private fun agree(a: Double, b: Double): Double = when {
            a.isNaN() -> b
            b.isNaN() -> a
            a * b <= 0 -> 0.0
            abs(a) < abs(b) -> a
            else -> b
        }

        /** La mediana dei primi [n] valori di [v] (riordinati); NaN se n = 0. */
        private fun median(v: DoubleArray, n: Int): Double {
            if (n == 0) return Double.NaN
            java.util.Arrays.sort(v, 0, n)
            return if (n % 2 == 1) v[n / 2] else (v[n / 2 - 1] + v[n / 2]) / 2
        }

        /** Il minimo delle quote entro ±[r] campioni (i NaN non contano; NaN se tutti NaN). */
        internal fun minFilter(h: DoubleArray, r: Int): DoubleArray {
            if (r <= 0) return h
            val n = h.size
            return DoubleArray(n) { k ->
                var best = Double.NaN
                for (j in max(0, k - r)..min(n - 1, k + r)) { val x = h[j]; if (!x.isNaN() && (best.isNaN() || x < best)) best = x }
                best
            }
        }

        /**
         * La pendenza della finestra di campioni [lo]..[hi] (passo [step] m): la mediana delle pendenze delle
         * sottocorde di [lag] campioni con i due capi nella finestra e validi; NaN se meno di [MIN_LAGS].
         * [buf]: almeno hi - lo + 1 posti.
         */
        internal fun windowSlope(hm: DoubleArray, lo: Int, hi: Int, lag: Int, step: Double, buf: DoubleArray): Double {
            var n = 0
            for (k in lo..hi - lag) {
                val a = hm[k]; val b = hm[k + lag]
                if (a.isNaN() || b.isNaN()) continue
                buf[n++] = (b - a) / (lag * step)
            }
            return if (n < MIN_LAGS) Double.NaN else median(buf, n)
        }

        /** Le curve della geometria (vedi la classe): [inizi, fini, raggi], o null se nessuna. */
        private fun findCurves(vx: DoubleArray, vy: DoubleArray, vs: DoubleArray, nv: Int): Array<DoubleArray>? {
            // diradamento: punti ad almeno THIN_M m; l'ultimo punto sostituisce un ultimo tenuto troppo vicino
            val keep = IntArray(nv); var nk = 0
            keep[nk++] = 0
            for (i in 1 until nv - 1) if (hypot(vx[i] - vx[keep[nk - 1]], vy[i] - vy[keep[nk - 1]]) >= THIN_M) keep[nk++] = i
            if (nk >= 2 && hypot(vx[nv - 1] - vx[keep[nk - 1]], vy[nv - 1] - vy[keep[nk - 1]]) < THIN_M) keep[nk - 1] = nv - 1 else keep[nk++] = nv - 1
            if (nk < 3) return null
            // angolo di svolta (con segno) e raggio a ogni vertice interno
            val turn = DoubleArray(nk); val radius = DoubleArray(nk); val lIn = DoubleArray(nk); val lOut = DoubleArray(nk)
            for (k in 1 until nk - 1) {
                val a = keep[k - 1]; val b = keep[k]; val c = keep[k + 1]
                val l1 = hypot(vx[b] - vx[a], vy[b] - vy[a]); val l2 = hypot(vx[c] - vx[b], vy[c] - vy[b])
                if (l1 < 1e-3 || l2 < 1e-3) continue
                var d = atan2(vy[c] - vy[b], vx[c] - vx[b]) - atan2(vy[b] - vy[a], vx[b] - vx[a])
                while (d > PI) d -= 2 * PI
                while (d <= -PI) d += 2 * PI
                turn[k] = d; lIn[k] = l1; lOut[k] = l2
                radius[k] = if (abs(d) < 1e-9) Double.POSITIVE_INFINITY else (l1 + l2) / 2 / abs(d)
            }
            // gruppi di vertici consecutivi che girano nello stesso verso
            val gFirst = ArrayList<Int>(); val gLast = ArrayList<Int>()
            var k = 1
            while (k < nk - 1) {
                if (abs(turn[k]) < TURN_MIN) { k++; continue }
                val sign = Math.signum(turn[k])
                var e = k
                while (e + 1 < nk - 1 && abs(turn[e + 1]) >= TURN_MIN && Math.signum(turn[e + 1]) == sign) e++
                gFirst += k; gLast += e
                k = e + 1
            }
            if (gFirst.isEmpty()) return null
            val drop = BooleanArray(gFirst.size)
            // gomito subito annullato dal gomito opposto: spostamento laterale, non curva
            for (g in 0 until gFirst.size - 1) {
                val a = gFirst[g]; val b = gFirst[g + 1]
                if (gLast[g] != a || gLast[g + 1] != b || b != a + 1) continue
                if (abs(turn[a] + turn[b]) < 0.35 * min(abs(turn[a]), abs(turn[b]))) { drop[g] = true; drop[g + 1] = true }
            }
            val from = ArrayList<Double>(); val to = ArrayList<Double>(); val rad = ArrayList<Double>()
            for (g in gFirst.indices) {
                if (drop[g]) continue
                val a = gFirst[g]; val b = gLast[g]
                // la svolta più grande entro CURVE_WINDOW_M m
                var best = 0.0
                for (i in a..b) {
                    var sum = 0.0
                    for (j in i..b) {
                        if (vs[keep[j]] - vs[keep[i]] > CURVE_WINDOW_M) break
                        sum += turn[j]
                        if (abs(sum) > best) best = abs(sum)
                    }
                }
                if (best <= CURVE_MIN) continue
                var r = Double.POSITIVE_INFINITY
                var theta = 0.0
                for (i in a..b) { if (radius[i] < r) r = radius[i]; theta += turn[i] }
                // un gomito solo: il cerchio tangente ai due lati entro metà del lato più corto
                if (a == b) r = min(lIn[a], lOut[a]) / 2 / kotlin.math.tan(abs(turn[a]) / 2)
                // l'arco: dal primo all'ultimo vertice più mezza corda per parte (al più CURVE_EDGE_M), ma non
                // più lungo di un arco di quel raggio che gira quanto la curva (un tornante di 12 m è lungo ~38 m)
                val f0 = vs[keep[a]] - min(CURVE_EDGE_M, lIn[a] / 2); val t0 = vs[keep[b]] + min(CURVE_EDGE_M, lOut[b] / 2)
                val len = min(t0 - f0, abs(theta) * r)
                val mid = (f0 + t0) / 2
                from += mid - len / 2
                to += mid + len / 2
                rad += r
            }
            if (rad.isEmpty()) return null
            return arrayOf(from.toDoubleArray(), to.toDoubleArray(), rad.toDoubleArray())
        }
    }

    /**
     * Il prolungamento della way in linea retta oltre i suoi capi, nella direzione degli ultimi
     * [APPROACH_DIR_M] m (o di tutta la way, se più corta): per i capi di ponti e gallerie e per la
     * finestra delle way corte. [at] mette il punto in [lat]/[lon] (falso se la direzione non si sa).
     */
    private class Ends(pts: Polyline, vlat: DoubleArray, vlon: DoubleArray, nv: Int, length: Double) {
        private val sLat = vlat[0]; private val sLon = vlon[0]
        private val eLat = vlat[nv - 1]; private val eLon = vlon[nv - 1]
        /** Gradi per metro verso l'esterno, all'inizio e alla fine; NaN se la direzione non si sa. */
        private val sdLat: Double; private val sdLon: Double; private val edLat: Double; private val edLon: Double
        var lat = 0.0; var lon = 0.0

        init {
            val d = min(APPROACH_DIR_M, length)
            pts.reset(); pts.at(d)
            val s = outward(sLat, sLon, pts.lat, pts.lon)
            pts.at(length - d)
            val e = outward(eLat, eLon, pts.lat, pts.lon)
            sdLat = s[0]; sdLon = s[1]; edLat = e[0]; edLon = e[1]
        }

        /** La direzione da (lat, lon) verso l'esterno, via da (iLat, iLon), in gradi per metro. */
        private fun outward(lat: Double, lon: Double, iLat: Double, iLon: Double): DoubleArray {
            val k = cos(Math.toRadians(lat)) * DistanceCalc.METERS_PER_DEGREE
            val dy = (lat - iLat) * DistanceCalc.METERS_PER_DEGREE; val dx = (lon - iLon) * k
            val norm = hypot(dx, dy)
            if (norm < 0.5) return doubleArrayOf(Double.NaN, Double.NaN)
            return doubleArrayOf(dy / norm / DistanceCalc.METERS_PER_DEGREE, dx / norm / k)
        }

        /** Il punto a [d] m oltre il capo d'inizio ([start]) o di fine. */
        fun at(start: Boolean, d: Double): Boolean {
            val dl = if (start) sdLat else edLat
            if (dl.isNaN()) return false
            lat = (if (start) sLat else eLat) + dl * d
            lon = (if (start) sLon else eLon) + (if (start) sdLon else edLon) * d
            return true
        }

        /** La quota di un capo dai campioni fuori dalla struttura ([APPROACH_FROM_M]..[APPROACH_TO_M] m oltre), mediana; NaN se nessuno. */
        fun approach(dem: ElevationSource, start: Boolean): Double {
            val count = ((APPROACH_TO_M - APPROACH_FROM_M) / APPROACH_STEP_M).toInt() + 1
            val v = DoubleArray(count); var n = 0
            for (k in 0 until count) {
                if (!at(start, APPROACH_FROM_M + k * APPROACH_STEP_M)) break
                val x = dem.ele(lat, lon)
                if (!x.isNaN()) v[n++] = x
            }
            return median(v, n)
        }
    }

    /** Un punto che scorre lungo la polilinea dei punti validi (a distanze crescenti; tornando indietro riparte dal primo tratto). */
    private class Polyline(val lats: DoubleArray, val lons: DoubleArray, val cum: DoubleArray, val n: Int) {
        var lat = 0.0; var lon = 0.0
        private var i = 0
        fun reset() { i = 0 }
        fun at(s: Double) {
            if (s < cum[i]) i = 0
            while (i < n - 2 && cum[i + 1] < s) i++
            val seg = cum[i + 1] - cum[i]
            val f = if (seg <= 0) 0.0 else ((s - cum[i]) / seg).coerceIn(0.0, 1.0)
            lat = lats[i] + (lats[i + 1] - lats[i]) * f
            lon = lons[i] + (lons[i + 1] - lons[i]) * f
        }
    }
}

/**
 * Il profilo di velocità di una way per un mezzo ([ClimbModel]) e una velocità di base, nei due sensi:
 * i tetti «duri» di ogni cella sono la base, il tetto della discesa (i freni) e quello della curva, e
 * una passata all'indietro ci arriva frenando a [ClimbModel.decel]; la salita invece non è un tetto da
 * raggiungere frenando: nella passata in avanti la velocità cambia secondo la potenza (ripartenze a
 * 0,5 m/s² al più, meno in salita; sopra la velocità sostenibile della salita il camion rallenta da sé,
 * col solo deficit di potenza, finché non ci arriva). Il tempo si integra cella per cella (moto
 * uniformemente accelerato). Ai capi della way il mezzo entra ed esce al tetto locale: le
 * fermate agli incroci le paga [it.navmaster.engine.profile.NodeDelays].
 *
 * [slope] e [curves] false escludono la pendenza o le curve (le statistiche dell'import separano i due effetti).
 */
class TerrainSpeeds(private val t: WayTerrain, private val climb: ClimbModel, val baseKmh: Double, slope: Boolean = true, curves: Boolean = true) {
    private val n = t.cells
    private val h = t.cellLen
    /** Tempo (s) dall'inizio della way a ogni nodo della griglia, in avanti. */
    private val tf = DoubleArray(n + 1)
    /** Tempo (s) dalla fine della way a ogni nodo della griglia, all'indietro. */
    private val tb = DoubleArray(n + 1)
    /** Metri di celle in cui la pendenza frena il mezzo, dall'inizio della way (in avanti e all'indietro). */
    private val slowF = DoubleArray(n + 1)
    private val slowB = DoubleArray(n + 1)
    /** Vero se in quel senso qualche cella ha un tetto sotto la base. */
    var slowedForward = false; private set
    var slowedBackward = false; private set

    init {
        val base = baseKmh / 3.6
        val curveCap = DoubleArray(n) { Double.POSITIVE_INFINITY }
        if (curves) for (c in 0 until t.curves) {
            val v = climb.curveKmh(t.curveRadius[c], t.isLink) / 3.6
            val k0 = floor(t.curveFrom[c] / h).toInt().coerceIn(0, n - 1)
            val k1 = floor(t.curveTo[c] / h).toInt().coerceIn(0, n - 1)
            for (k in k0..k1) if (v < curveCap[k]) curveCap[k] = v
        }
        val g = if (slope) t.grade else null
        slowedForward = run(true, base, g, curveCap, tf, slowF)
        slowedBackward = run(false, base, g, curveCap, tb, slowB)
    }

    private fun run(forward: Boolean, base: Double, grade: DoubleArray?, curveCap: DoubleArray, out: DoubleArray, slow: DoubleArray): Boolean {
        // hard: base, discesa, curva (si frena per arrivarci); up: la velocità sostenibile in salita (infinita altrove)
        val hard = DoubleArray(n); val up = DoubleArray(n); val gr = DoubleArray(n)
        var any = false
        for (i in 0 until n) {
            val c = if (forward) i else n - 1 - i
            val gg = if (grade == null) 0.0 else if (forward) grade[c] else -grade[c]
            gr[i] = gg
            val sc = if (gg == 0.0) Double.POSITIVE_INFINITY else climb.slopeKmh(gg) / 3.6
            var v = base
            if (gg < 0 && sc < v) v = sc
            if (curveCap[c] < v) v = curveCap[c]
            hard[i] = v
            up[i] = if (gg > 0) sc else Double.POSITIVE_INFINITY
            if (min(v, up[i]) < base) any = true
            // celle frenate dalla pendenza, contate per posizione lungo la way
            slow[c + 1] = if (sc < base) h else 0.0
        }
        for (k in 1..n) slow[k] += slow[k - 1]
        if (!any) {
            for (k in 0..n) out[k] = (if (forward) k else n - k) * h / base
            return false
        }
        // tetti duri ai nodi della griglia (il più basso delle due celle vicine), poi la frenata all'indietro
        val v = DoubleArray(n + 1)
        v[0] = hard[0]; v[n] = hard[n - 1]
        for (k in 1 until n) v[k] = min(hard[k - 1], hard[k])
        for (k in n - 1 downTo 0) {
            val reach = sqrt(v[k + 1] * v[k + 1] + 2 * climb.decel * h)
            if (reach < v[k]) v[k] = reach
        }
        // in avanti: si entra alla velocità sostenibile della prima cella; poi la potenza decide
        if (up[0] < v[0]) v[0] = up[0]
        for (k in 0 until n) {
            val vk = v[k]
            val next = if (vk > up[k]) {
                // sopra la velocità sostenibile della salita: rallenta col deficit di potenza, non sotto quella velocità
                val a = climb.powerAcceleration(vk, gr[k])
                max(up[k], sqrt(max(0.0, vk * vk + 2 * a * h)))
            } else {
                min(up[k], sqrt(vk * vk + 2 * climb.acceleration(vk, gr[k]) * h))
            }
            if (next < v[k + 1]) v[k + 1] = next
        }
        val tt = DoubleArray(n + 1)
        for (k in 0 until n) tt[k + 1] = tt[k] + 2 * h / (v[k] + v[k + 1])
        for (k in 0..n) out[k] = if (forward) tt[k] else tt[n - k]
        return true
    }

    private fun interp(a: DoubleArray, s: Double): Double {
        val x = (s / h).coerceIn(0.0, n.toDouble())
        val k = min(n - 1, x.toInt())
        val f = x - k
        return a[k] + (a[k + 1] - a[k]) * f
    }

    /** Secondi da [from] a [to] (m lungo la way, from < to) nel senso dato. */
    fun seconds(from: Double, to: Double, forward: Boolean): Double {
        if (forward) return if (!slowedForward) (to - from) / (baseKmh / 3.6) else interp(tf, to) - interp(tf, from)
        return if (!slowedBackward) (to - from) / (baseKmh / 3.6) else interp(tb, from) - interp(tb, to)
    }

    /** La velocità media (km/h) del pezzo [from, to] nel senso dato: esattamente la base se lì nulla frena. */
    fun speedKmh(from: Double, to: Double, forward: Boolean): Double {
        if (forward && !slowedForward || !forward && !slowedBackward) return baseKmh
        val sec = seconds(from, to, forward)
        if (sec <= 0) return baseKmh
        return min(baseKmh, (to - from) / sec * 3.6)
    }

    /** Metri del pezzo [from, to] in cui la pendenza frena il mezzo nel senso dato. */
    fun slowedBySlope(from: Double, to: Double, forward: Boolean): Double = interp(if (forward) slowF else slowB, to) - interp(if (forward) slowF else slowB, from)
}
