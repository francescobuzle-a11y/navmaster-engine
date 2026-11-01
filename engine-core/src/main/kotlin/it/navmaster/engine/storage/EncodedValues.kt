package it.navmaster.engine.storage

import kotlin.math.roundToLong

/**
 * Allocatore di bit nei flag di un arco (GraphHopper `InitializerConfig`): i valori sono
 * impacchettati in sequenza e nessuno attraversa il confine di un int da 32 bit.
 */
class BitAllocator {
    var dataIndex = -1; private set
    var shift = 32; private set
    private var nextShift = 32
    var bitMask = 0; private set

    fun next(bits: Int) {
        shift = nextShift
        if ((shift - 1 + bits) / 32 > (shift - 1) / 32) {
            dataIndex++
            shift = 0
        }
        bitMask = ((1L shl bits) - 1).toInt() shl shift
        nextShift = shift + bits
    }

    val requiredBits: Int get() = dataIndex * 32 + nextShift
    val requiredInts: Int get() = (requiredBits + 31) / 32
}

/** Valore memorizzato nei bit dei flag di ogni arco. */
sealed class EncodedValue(val name: String, val bits: Int, val storeTwoDirections: Boolean) {
    protected var fwdIndex = 0; private set
    protected var fwdShift = 0; private set
    protected var fwdMask = 0; private set
    protected var bwdIndex = 0; private set
    protected var bwdShift = 0; private set
    protected var bwdMask = 0; private set
    var initialized = false; private set

    val maxRaw: Int get() = ((1L shl bits) - 1).toInt()

    fun init(alloc: BitAllocator) {
        require(bits in 1..31) { "$name: bit fuori intervallo: $bits" }
        check(!initialized) { "$name già inizializzato" }
        alloc.next(bits)
        fwdIndex = alloc.dataIndex; fwdShift = alloc.shift; fwdMask = alloc.bitMask
        if (storeTwoDirections) {
            alloc.next(bits)
            bwdIndex = alloc.dataIndex; bwdShift = alloc.shift; bwdMask = alloc.bitMask
        }
        initialized = true
    }

    fun rawGet(flags: IntArray, reverse: Boolean): Int =
        if (reverse && storeTwoDirections) (flags[bwdIndex] and bwdMask) ushr bwdShift
        else (flags[fwdIndex] and fwdMask) ushr fwdShift

    fun rawSet(flags: IntArray, reverse: Boolean, value: Int) {
        require(value in 0..maxRaw) { "$name: valore $value fuori da [0, $maxRaw]" }
        if (reverse) {
            require(storeTwoDirections) { "$name non memorizza la direzione inversa" }
            flags[bwdIndex] = (flags[bwdIndex] and bwdMask.inv()) or (value shl bwdShift)
        } else {
            flags[fwdIndex] = (flags[fwdIndex] and fwdMask.inv()) or (value shl fwdShift)
        }
    }

    /** Descrizione stabile per il file properties: cambiarla invalida i grafi esistenti. */
    open fun describe(): String = "$name|bits=$bits|two_dir=$storeTwoDirections"
}

class IntEncodedValue(name: String, bits: Int, twoDirections: Boolean = false) : EncodedValue(name, bits, twoDirections) {
    fun get(flags: IntArray, reverse: Boolean = false): Int = rawGet(flags, reverse)
    fun set(flags: IntArray, reverse: Boolean, value: Int) = rawSet(flags, reverse, value)
    fun set(flags: IntArray, value: Int) = rawSet(flags, false, value)
}

class BooleanEncodedValue(name: String, twoDirections: Boolean = false) : EncodedValue(name, 1, twoDirections) {
    fun get(flags: IntArray, reverse: Boolean = false): Boolean = rawGet(flags, reverse) == 1
    fun set(flags: IntArray, reverse: Boolean, value: Boolean) = rawSet(flags, reverse, if (value) 1 else 0)
    fun set(flags: IntArray, value: Boolean) = rawSet(flags, false, if (value) 1 else 0)
}

/**
 * Numero decimale memorizzato come intero `round(value / factor)`.
 * Il valore [defaultValue] si memorizza come 0 (e 0 si legge come default);
 * con [useMaximumAsInfinity] il massimo raw rappresenta +∞.
 */
class DecimalEncodedValue(
    name: String, bits: Int, val factor: Double,
    val defaultValue: Double = 0.0, twoDirections: Boolean = false, val useMaximumAsInfinity: Boolean = false,
) : EncodedValue(name, bits, twoDirections) {
    val maxDecimal: Double get() = if (useMaximumAsInfinity) (maxRaw - 1) * factor else maxRaw * factor

    fun get(flags: IntArray, reverse: Boolean = false): Double {
        val raw = rawGet(flags, reverse)
        if (useMaximumAsInfinity && raw == maxRaw) return Double.POSITIVE_INFINITY
        if (raw == 0) return defaultValue
        return raw * factor
    }

    fun set(flags: IntArray, reverse: Boolean, value: Double) {
        if (useMaximumAsInfinity && value.isInfinite()) { rawSet(flags, reverse, maxRaw); return }
        if (value == defaultValue) { rawSet(flags, reverse, 0); return }
        val raw = (value / factor).roundToLong()
        require(raw >= 0) { "$name: valore negativo $value" }
        val limit = if (useMaximumAsInfinity) maxRaw - 1 else maxRaw
        rawSet(flags, reverse, minOf(raw, limit.toLong()).toInt())
    }

    fun set(flags: IntArray, value: Double) = set(flags, false, value)

    override fun describe() = super.describe() + "|factor=$factor|default=$defaultValue|max_inf=$useMaximumAsInfinity"
}

class EnumEncodedValue<E : Enum<E>>(name: String, private val values: Array<E>, twoDirections: Boolean = false) :
    EncodedValue(name, bitsFor(values.size), twoDirections) {
    fun get(flags: IntArray, reverse: Boolean = false): E = values[rawGet(flags, reverse)]
    fun set(flags: IntArray, reverse: Boolean, value: E) = rawSet(flags, reverse, value.ordinal)
    fun set(flags: IntArray, value: E) = rawSet(flags, false, value.ordinal)

    override fun describe() = super.describe() + "|values=${values.joinToString(",") { it.name }}"

    companion object {
        fun bitsFor(count: Int): Int = if (count <= 1) 1 else 32 - Integer.numberOfLeadingZeros(count - 1)
    }
}

/**
 * Insieme ordinato dei valori codificati di un grafo. L'ordine di registrazione fissa
 * la posizione dei bit, quindi fa parte del formato del grafo.
 */
class EncodedValueRegistry private constructor(private val byName: LinkedHashMap<String, EncodedValue>, val intsForFlags: Int) {
    fun <T : EncodedValue> get(name: String): T {
        @Suppress("UNCHECKED_CAST")
        return byName[name] as? T ?: error("valore codificato sconosciuto: $name")
    }

    fun has(name: String) = byName.containsKey(name)
    fun int(name: String): IntEncodedValue = get(name)
    fun bool(name: String): BooleanEncodedValue = get(name)
    fun decimal(name: String): DecimalEncodedValue = get(name)
    @Suppress("UNCHECKED_CAST")
    fun <E : Enum<E>> enum(name: String): EnumEncodedValue<E> = get(name)

    val all: Collection<EncodedValue> get() = byName.values

    /** Stringa stabile salvata nelle properties del grafo per verificare la compatibilità al caricamento. */
    fun describe(): String = byName.values.joinToString(";") { it.describe() }

    fun newFlags(): IntArray = IntArray(intsForFlags)

    class Builder {
        private val values = LinkedHashMap<String, EncodedValue>()

        fun add(ev: EncodedValue): Builder {
            require(!values.containsKey(ev.name)) { "valore codificato duplicato: ${ev.name}" }
            values[ev.name] = ev
            return this
        }

        fun build(): EncodedValueRegistry {
            val alloc = BitAllocator()
            for (ev in values.values) ev.init(alloc)
            return EncodedValueRegistry(values, maxOf(1, alloc.requiredInts))
        }
    }
}
