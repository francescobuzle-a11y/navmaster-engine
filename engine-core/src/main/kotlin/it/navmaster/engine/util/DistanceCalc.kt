package it.navmaster.engine.util

import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/** Distanze sulla sfera terrestre (stesso raggio di GraphHopper: 6371000 m). */
object DistanceCalc {
    const val EARTH_RADIUS_M = 6371000.0
    const val METERS_PER_DEGREE = EARTH_RADIUS_M * Math.PI / 180.0

    /** Distanza haversine in metri. */
    fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1)
        val sinLat = sin(dLat / 2); val sinLon = sin(dLon / 2)
        val a = sinLat * sinLat + sinLon * sinLon * cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2))
        return 2 * EARTH_RADIUS_M * asin(sqrt(a))
    }

    /**
     * Distanza approssimata con proiezione piana (equirettangolare), in metri.
     * Molto più veloce dell'haversine; adatta a distanze brevi (snapping, indice spaziale).
     */
    fun planeDistance(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val dLat = Math.toRadians(lat2 - lat1)
        val dLon = Math.toRadians(lon2 - lon1) * cos(Math.toRadians((lat1 + lat2) / 2))
        return EARTH_RADIUS_M * sqrt(dLat * dLat + dLon * dLon)
    }

    /** Azimut iniziale da (lat1,lon1) a (lat2,lon2) in gradi [0, 360). */
    fun bearing(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val φ1 = Math.toRadians(lat1); val φ2 = Math.toRadians(lat2)
        val dλ = Math.toRadians(lon2 - lon1)
        val y = sin(dλ) * cos(φ2)
        val x = cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(dλ)
        return (Math.toDegrees(atan2(y, x)) + 360) % 360
    }

    /** Lunghezza in metri di una polilinea di coordinate (lat, lon). */
    fun length(lats: DoubleArray, lons: DoubleArray, size: Int): Double {
        var d = 0.0
        for (i in 1 until size) d += haversine(lats[i - 1], lons[i - 1], lats[i], lons[i])
        return d
    }
}
