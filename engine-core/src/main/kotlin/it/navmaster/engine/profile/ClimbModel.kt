package it.navmaster.engine.profile

import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Il mezzo di riferimento per le salite, le discese e le curve del modello NAVMASTER (vedi
 * [it.navmaster.engine.terrain.WayTerrain]): massa, potenza alle ruote, resistenze, accelerazione
 * laterale in curva, accelerazione e frenata in rettilineo. Il mezzo è fissato all'import (il grafo
 * non sa del camion della richiesta): un camion più leggero sale più svelto di così.
 *
 * In salita la velocità è quella a cui la potenza basta appena: P = (m·g·(G + Crr) + ½·ρ·CdA·v²)·v,
 * risolta per bisezione una volta sola, in una tabella a passi di 0,5 %. In discesa un tetto da
 * tabella (il freno motore e la marcia bassa di un 40 t). In curva v = √(aLat·R), almeno [MIN_CURVE_KMH].
 */
class ClimbModel(
    val massKg: Double,
    val wheelPowerW: Double,
    val crr: Double,
    val cdA: Double,
    val rho: Double = 1.2,
    /** Accelerazione laterale sopportata in curva (m/s²), e quella sulle rampe degli svincoli (anse strette, raggio noto a chi guida). */
    val lateralAccel: Double,
    val lateralAccelLink: Double,
    /** Accelerazione massima in piano (m/s²) e frenata (m/s²) del profilo di velocità. */
    val accel: Double,
    val decel: Double,
    /** Tetti in discesa: (pendenza, km/h), pendenze crescenti; sotto la prima nessun tetto, sopra l'ultima l'ultimo. */
    private val downhill: List<Pair<Double, Double>>,
) {
    private val uphillTable = DoubleArray(TABLE_STEPS + 1) { i -> if (i == 0) Double.POSITIVE_INFINITY else solveUphill(i * TABLE_STEP) * 3.6 }

    /** La velocità sostenibile (km/h) in salita su una pendenza [grade] (frazione, 0,06 = 6 %); infinita in piano. */
    fun uphillKmh(grade: Double): Double {
        if (grade <= 0 || grade.isNaN()) return Double.POSITIVE_INFINITY
        val x = grade / TABLE_STEP
        if (x >= TABLE_STEPS) return solveUphill(grade) * 3.6
        val i = x.toInt()
        if (i == 0) {
            // fra il piano (nessun tetto) e il primo passo: si usa il valore del primo passo, che non frena mai
            return uphillTable[1]
        }
        val f = x - i
        return uphillTable[i] * (1 - f) + uphillTable[i + 1] * f
    }

    /** Il tetto (km/h) in discesa su una pendenza [grade] (frazione positiva); infinito sotto la prima voce della tabella. */
    fun downhillKmh(grade: Double): Double {
        if (grade.isNaN() || grade < downhill.first().first) return Double.POSITIVE_INFINITY
        for (k in 1 until downhill.size) {
            val (g1, v1) = downhill[k]
            if (grade <= g1) {
                val (g0, v0) = downhill[k - 1]
                return v0 + (v1 - v0) * (grade - g0) / (g1 - g0)
            }
        }
        return downhill.last().second
    }

    /** Il tetto della pendenza [grade] nel senso di marcia (positiva in salita, negativa in discesa). */
    fun slopeKmh(grade: Double): Double = if (grade > 0) uphillKmh(grade) else if (grade < 0) downhillKmh(-grade) else Double.POSITIVE_INFINITY

    /** La velocità (km/h) in una curva di raggio [radiusM]; sulle rampe ([link]) l'accelerazione laterale è più alta. */
    fun curveKmh(radiusM: Double, link: Boolean = false): Double {
        val a = if (link) lateralAccelLink else lateralAccel
        return max(MIN_CURVE_KMH, 3.6 * sqrt(a * max(0.0, radiusM)))
    }

    /** L'accelerazione possibile (m/s²) a [vMs] su una pendenza [grade]: [accel], o meno se la potenza non basta; mai negativa. */
    fun acceleration(vMs: Double, grade: Double): Double {
        val v = max(vMs, 0.5)
        val spare = wheelPowerW / v - resistance(v, grade)
        return max(0.0, min(accel, spare / massKg))
    }

    /** La forza resistente (N) a [vMs] su [grade]: pendenza, rotolamento, aria. */
    private fun resistance(vMs: Double, grade: Double): Double = massKg * G * (grade + crr) + 0.5 * rho * cdA * vMs * vMs

    /** La velocità (m/s) a cui la potenza alle ruote equilibra la resistenza sulla pendenza [grade], per bisezione. */
    private fun solveUphill(grade: Double): Double {
        var lo = 0.1; var hi = 80.0
        repeat(80) {
            val mid = (lo + hi) / 2
            if (resistance(mid, grade) * mid > wheelPowerW) hi = mid else lo = mid
        }
        return (lo + hi) / 2
    }

    companion object {
        const val G = 9.81
        const val TABLE_STEP = 0.005
        /** Fino al 16 % (il tetto delle strade minori); oltre si risolve ogni volta. */
        const val TABLE_STEPS = 32
        const val MIN_CURVE_KMH = 10.0

        /**
         * Autoarticolato da 40 t con 280 kW alle ruote (un motore da ~330 kW meno le perdite), Crr 0,006,
         * CdA 6 m²: 3 % 66, 4 % 53, 5 % 45, 6 % 38, 7 % 34, 8 % 30, 10 % 24, 12 % 20, 14 % 18 km/h. In
         * discesa 4 % 70, 5 % 60, 6 % 50, 7 % 45, 8 % 40, 10 % 35, 12 % 30, dal 14 % 25 km/h. Curve a
         * 1,5 m/s² (2,0 sulle rampe), accelerazione 0,5 m/s², frenata 1,0 m/s² come [HgvProfile].
         * Valori fisici ragionati, da tarare sui tempi veri (TomTom, cronotachigrafo) di 3-4 valichi.
         */
        val HGV = ClimbModel(
            massKg = 40_000.0, wheelPowerW = 280_000.0, crr = 0.006, cdA = 6.0, rho = 1.2,
            lateralAccel = 1.5, lateralAccelLink = 2.0, accel = 0.5, decel = 1.0,
            downhill = listOf(0.04 to 70.0, 0.05 to 60.0, 0.06 to 50.0, 0.07 to 45.0, 0.08 to 40.0, 0.10 to 35.0, 0.12 to 30.0, 0.14 to 25.0),
        )
    }
}
