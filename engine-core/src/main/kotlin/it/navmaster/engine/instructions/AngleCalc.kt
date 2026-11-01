package it.navmaster.engine.instructions

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.roundToLong

/** Calcoli angolari di GraphHopper (`AngleCalc`), riprodotti esattamente, approssimazione veloce compresa. */
object AngleCalc {
    private const val PI_4 = PI / 4
    private const val PI_2 = PI / 2

    /**
     * Orientamento (radianti, −π..π) del vettore p1→p2 su una proiezione piana locale.
     * Con [exact] = false usa la stessa approssimazione di `atan2` di GraphHopper.
     */
    fun calcOrientation(lat1: Double, lon1: Double, lat2: Double, lon2: Double, exact: Boolean = true): Double {
        val shrink = cos(Math.toRadians((lat1 + lat2) / 2))
        return if (exact) atan2(lat2 - lat1, shrink * (lon2 - lon1)) else atan2Quick(lat2 - lat1, shrink * (lon2 - lon1))
    }

    private fun atan2Quick(y: Double, x: Double): Double {
        val absY = abs(y) + 1e-10 // evita 0/0
        val r: Double; var angle: Double
        if (x < 0.0) { r = (x + absY) / (absY - x); angle = 3.0 * PI_4 } else { r = (x - absY) / (x + absY); angle = PI_4 }
        angle += (0.1963 * r * r - 0.9817) * r
        return if (y < 0.0) -angle else angle
    }

    /** Riporta [orientation] nell'intervallo di ±π attorno a [base], così la differenza è l'angolo di svolta. */
    fun alignOrientation(base: Double, orientation: Double): Double {
        var o = orientation
        if (base >= 0) { if (o < -PI + base) o += 2 * PI } else { if (o > PI + base) o -= 2 * PI }
        return o
    }

    /** Azimut in gradi [0, 360) da p1 a p2 (0 = nord, 90 = est), arrotondato a 4 decimali in radianti come GraphHopper. */
    fun calcAzimuth(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        var orientation = PI_2 - calcOrientation(lat1, lon1, lat2, lon2)
        if (orientation < 0) orientation += 2 * PI
        return Math.toDegrees(round4(orientation)) % 360
    }

    fun round4(v: Double): Double = (v * 10000).roundToLong() / 10000.0
}
