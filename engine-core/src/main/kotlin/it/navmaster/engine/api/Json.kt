package it.navmaster.engine.api

import java.math.BigDecimal

/**
 * JSON minimale senza dipendenze: valori = `Map<String, Any?>` (ordinata), `List<Any?>`, `String`,
 * `Number`, `Boolean`, `null`. Sufficiente per le richieste e le risposte ORS e usabile su Android.
 */
object Json {
    fun stringify(value: Any?): String = StringBuilder(1024).also { write(value, it) }.toString()

    fun write(value: Any?, sb: StringBuilder) {
        when (value) {
            null -> sb.append("null")
            is String -> writeString(value, sb)
            is Boolean -> sb.append(value)
            is Int, is Long, is Short, is Byte -> sb.append(value.toString())
            is Double -> writeDouble(value, sb)
            is Float -> writeDouble(value.toDouble(), sb)
            is Number -> sb.append(value.toString())
            is Map<*, *> -> {
                sb.append('{'); var first = true
                for ((k, v) in value) { if (!first) sb.append(','); first = false; writeString(k.toString(), sb); sb.append(':'); write(v, sb) }
                sb.append('}')
            }
            is Iterable<*> -> {
                sb.append('['); var first = true
                for (v in value) { if (!first) sb.append(','); first = false; write(v, sb) }
                sb.append(']')
            }
            is DoubleArray -> { sb.append('['); value.forEachIndexed { i, d -> if (i > 0) sb.append(','); writeDouble(d, sb) }; sb.append(']') }
            is IntArray -> { sb.append('['); value.forEachIndexed { i, d -> if (i > 0) sb.append(','); sb.append(d) }; sb.append(']') }
            is Array<*> -> write(value.asList(), sb)
            else -> writeString(value.toString(), sb)
        }
    }

    private fun writeDouble(d: Double, sb: StringBuilder) {
        if (d.isNaN() || d.isInfinite()) { sb.append("null"); return }
        sb.append(BigDecimal.valueOf(d).toPlainString())
    }

    private fun writeString(s: String, sb: StringBuilder) {
        sb.append('"')
        for (c in s) when (c) {
            '"' -> sb.append("\\\""); '\\' -> sb.append("\\\\"); '\n' -> sb.append("\\n"); '\r' -> sb.append("\\r"); '\t' -> sb.append("\\t")
            else -> if (c < ' ') sb.append("\\u%04x".format(c.code)) else sb.append(c)
        }
        sb.append('"')
    }

    /** Analizza un documento JSON; numeri come `Long` quando interi, altrimenti `Double`. */
    fun parse(text: String): Any? = Parser(text).parseDocument()

    class ParseException(message: String) : RuntimeException(message)

    private class Parser(val s: String) {
        var i = 0
        fun parseDocument(): Any? { val v = parseValue(); skipWs(); if (i != s.length) fail("caratteri inattesi dopo il documento"); return v }
        private fun fail(msg: String): Nothing = throw ParseException("JSON non valido alla posizione $i: $msg")
        private fun skipWs() { while (i < s.length && s[i].isWhitespace()) i++ }
        private fun parseValue(): Any? {
            skipWs()
            if (i >= s.length) fail("fine inattesa")
            return when (val c = s[i]) {
                '{' -> parseObject(); '[' -> parseArray(); '"' -> parseString()
                't' -> literal("true", true); 'f' -> literal("false", false); 'n' -> literal("null", null)
                else -> if (c == '-' || c.isDigit()) parseNumber() else fail("carattere '$c'")
            }
        }
        private fun literal(word: String, value: Any?): Any? { if (!s.startsWith(word, i)) fail("atteso $word"); i += word.length; return value }
        private fun parseObject(): Map<String, Any?> {
            val m = LinkedHashMap<String, Any?>(); i++; skipWs()
            if (i < s.length && s[i] == '}') { i++; return m }
            while (true) {
                skipWs(); if (i >= s.length || s[i] != '"') fail("attesa chiave")
                val k = parseString(); skipWs(); if (i >= s.length || s[i] != ':') fail("atteso ':'"); i++
                m[k] = parseValue(); skipWs()
                if (i >= s.length) fail("oggetto non chiuso")
                if (s[i] == ',') { i++; continue }
                if (s[i] == '}') { i++; return m }
                fail("atteso ',' o '}'")
            }
        }
        private fun parseArray(): List<Any?> {
            val l = ArrayList<Any?>(); i++; skipWs()
            if (i < s.length && s[i] == ']') { i++; return l }
            while (true) {
                l.add(parseValue()); skipWs()
                if (i >= s.length) fail("array non chiuso")
                if (s[i] == ',') { i++; continue }
                if (s[i] == ']') { i++; return l }
                fail("atteso ',' o ']'")
            }
        }
        private fun parseString(): String {
            i++; val sb = StringBuilder()
            while (i < s.length) {
                val c = s[i++]
                when (c) {
                    '"' -> return sb.toString()
                    '\\' -> {
                        if (i >= s.length) fail("escape troncato")
                        when (val e = s[i++]) {
                            '"' -> sb.append('"'); '\\' -> sb.append('\\'); '/' -> sb.append('/'); 'b' -> sb.append('\b'); 'f' -> sb.append('\u000C')
                            'n' -> sb.append('\n'); 'r' -> sb.append('\r'); 't' -> sb.append('\t')
                            'u' -> { if (i + 4 > s.length) fail("\\u troncato"); sb.append(s.substring(i, i + 4).toInt(16).toChar()); i += 4 }
                            else -> fail("escape sconosciuto \\$e")
                        }
                    }
                    else -> sb.append(c)
                }
            }
            fail("stringa non chiusa")
        }
        private fun parseNumber(): Number {
            val start = i
            if (s[i] == '-') i++
            while (i < s.length && (s[i].isDigit() || s[i] in ".eE+-")) i++
            val txt = s.substring(start, i)
            return if (txt.any { it == '.' || it == 'e' || it == 'E' }) txt.toDoubleOrNull() ?: fail("numero non valido '$txt'")
            else txt.toLongOrNull() ?: (txt.toDoubleOrNull() ?: fail("numero non valido '$txt'"))
        }
    }
}

/** Codifica Google Polyline a precisione 1e5 (ORS `PolylineEncoder`), con quota opzionale ×100 (non standard). */
object PolylineEncoder {
    fun encode(lats: DoubleArray, lons: DoubleArray, size: Int, eles: DoubleArray? = null): String {
        val sb = StringBuilder(size * 6)
        var prevLat = 0L; var prevLon = 0L; var prevEle = 0L
        for (i in 0 until size) {
            val lat = Math.round(lats[i] * 1e5); val lon = Math.round(lons[i] * 1e5)
            encodeNumber(lat - prevLat, sb); encodeNumber(lon - prevLon, sb)
            prevLat = lat; prevLon = lon
            if (eles != null) { val e = Math.floor(eles[i] * 100).toLong(); encodeNumber(e - prevEle, sb); prevEle = e }
        }
        return sb.toString()
    }

    private fun encodeNumber(num: Long, sb: StringBuilder) {
        var v = if (num < 0) (num shl 1).inv() else num shl 1
        while (v >= 0x20) { sb.append(((0x20 or (v and 0x1f).toInt()) + 63).toChar()); v = v shr 5 }
        sb.append((v.toInt() + 63).toChar())
    }

    fun decode(encoded: String): List<DoubleArray> {
        val out = ArrayList<DoubleArray>()
        var i = 0; var lat = 0L; var lon = 0L
        while (i < encoded.length) {
            var result = 0L; var shift = 0; var b: Int
            do { b = encoded[i++].code - 63; result = result or ((b and 0x1f).toLong() shl shift); shift += 5 } while (b >= 0x20)
            lat += if (result and 1L != 0L) (result shr 1).inv() else result shr 1
            result = 0; shift = 0
            do { b = encoded[i++].code - 63; result = result or ((b and 0x1f).toLong() shl shift); shift += 5 } while (b >= 0x20)
            lon += if (result and 1L != 0L) (result shr 1).inv() else result shr 1
            out.add(doubleArrayOf(lat / 1e5, lon / 1e5))
        }
        return out
    }
}
