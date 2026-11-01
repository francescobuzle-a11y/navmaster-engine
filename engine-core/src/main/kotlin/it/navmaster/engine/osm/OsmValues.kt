package it.navmaster.engine.osm

/**
 * Conversione dei valori testuali dei tag OSM in numeri.
 * Stessa semantica di `OSMValueExtractor` / `AbstractFlagEncoder.parseSpeed` di GraphHopper:
 * un valore non interpretabile vale [Double.NaN].
 */
object OsmValues {
    /** Velocità usata per `maxspeed=none` (autostrade tedesche). */
    const val UNLIMITED_SIGN_SPEED = 150.0

    private const val KM_PER_MILE = 1.609344
    private const val KM_PER_KNOT = 1.852

    private val tonPattern = Regex("tons?")
    private val mgwPattern = Regex("mgw")
    private val whitespacePattern = Regex("\\s")
    private val meterPattern = Regex("meters?|mtrs?|mt|m\\.")
    private val inchPattern = Regex("\"|''")
    private val feetPattern = Regex("'|feet")
    private val approxPattern = Regex("~|approx")

    /** `maxspeed` e simili → km/h. */
    fun parseKmh(raw: String?): Double {
        if (raw.isNullOrEmpty()) return Double.NaN
        var str = raw
        if (str == "none") return UNLIMITED_SIGN_SPEED
        if (str.endsWith(":rural") || str.endsWith(":trunk")) return 80.0
        if (str.endsWith(":urban")) return 50.0
        if (str == "walk" || str.endsWith(":living_street")) return 6.0

        val mp = str.indexOf("mp")
        val knots = str.indexOf("knots")
        val km = str.indexOf("km")
        val kph = str.indexOf("kph")
        val factor: Double
        when {
            mp > 0 -> { str = str.substring(0, mp).trim(); factor = KM_PER_MILE }
            knots > 0 -> { str = str.substring(0, knots).trim(); factor = KM_PER_KNOT }
            else -> {
                if (km > 0) str = str.substring(0, km).trim()
                else if (kph > 0) str = str.substring(0, kph).trim()
                factor = 1.0
            }
        }
        val value = str.toIntOrNull()?.times(factor) ?: return Double.NaN
        return if (value <= 0) Double.NaN else value
    }

    /** `maxweight`, `maxaxleload` → tonnellate. */
    fun parseTons(raw: String?): Double {
        var value = tonPattern.replace((raw ?: "").lowercase(), "t")
        value = mgwPattern.replace(value, "").trim()
        if (isInvalidValue(value)) return Double.NaN
        var factor = 1.0
        when {
            value.endsWith("st") -> { value = value.dropLast(2); factor = 0.907194048807 }
            value.endsWith("t") -> value = value.dropLast(1)
            value.endsWith("lbs") -> { value = value.dropLast(3); factor = 0.00045359237 }
            value.endsWith("kg") -> { value = value.dropLast(2); factor = 0.001 }
        }
        return value.toDoubleOrNull()?.times(factor) ?: Double.NaN
    }

    /** `maxheight`, `maxwidth`, `maxlength` → metri. Gestisce piedi/pollici e "~". */
    fun parseMeter(raw: String?): Double {
        var value = whitespacePattern.replace((raw ?: "").lowercase(), "")
        value = meterPattern.replace(value, "m")
        value = inchPattern.replace(value, "in")
        value = feetPattern.replace(value, "ft")
        if (isInvalidValue(value)) return Double.NaN
        var factor = 1.0
        var offset = 0.0
        if (value.startsWith("~") || value.contains("approx")) {
            value = approxPattern.replace(value, "").trim()
            factor = 0.8
        }
        if (value.endsWith("in")) {
            var start = value.indexOf("ft")
            start = if (start < 0) 0 else start + 2
            val inches = value.substring(start, value.length - 2)
            value = value.substring(0, start)
            offset = inches.toDoubleOrNull()?.times(0.0254) ?: return Double.NaN
        }
        when {
            value.endsWith("ft") -> { value = value.dropLast(2); factor *= 0.3048 }
            value.endsWith("cm") -> { value = value.dropLast(2); factor *= 0.01 }
            value.endsWith("m") -> value = value.dropLast(1)
        }
        if (value.isEmpty()) return offset
        return value.toDoubleOrNull()?.let { it * factor + offset } ?: Double.NaN
    }

    fun isInvalidValue(raw: String): Boolean {
        val value = raw.lowercase()
        return value.isEmpty() || value.startsWith("default") || value == "none" || value == "unknown" ||
            value.contains("unrestricted") || value.startsWith("〜") ||
            value.contains("narrow") || value == "unsigned" || value == "fixme" || value == "small" ||
            value.contains(";") || value.contains(":") || value.contains("(") ||
            value.contains(">") || value.contains("<") || value.contains("-") ||
            // solo il punto come separatore decimale
            value.contains(",")
    }

    /** `duration=hh:mm[:ss]` oppure minuti o ISO 8601 (`PT1H30M`) → secondi; -1 se non valido. */
    fun parseDurationSeconds(raw: String?): Long {
        if (raw.isNullOrBlank()) return -1
        val s = raw.trim()
        if (s.startsWith("P", ignoreCase = true)) {
            return try { java.time.Duration.parse(s.uppercase()).seconds } catch (e: Exception) { -1 }
        }
        val parts = s.split(':')
        return try {
            when (parts.size) {
                1 -> parts[0].toLong() * 60
                2 -> parts[0].toLong() * 3600 + parts[1].toLong() * 60
                3 -> parts[0].toLong() * 3600 + parts[1].toLong() * 60 + parts[2].toLong()
                else -> -1
            }
        } catch (e: NumberFormatException) { -1 }
    }
}
