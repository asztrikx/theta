package hu.bme.mit.theta.analysis.algorithm.cegar.astar

class Distance(var type: Type, value: Int) : Comparable<Distance> {
	var value = value
		get(): Int {
			check(type == Type.EXACT)
			return field
		}

	/**
	 * This constructor is only for types which don't have a value.
	 * Exact types should explicitly state the value even if its zero, which could be default, for clarity
	 */
	constructor(type: Type) : this(type, 0) {
		require(type != Type.EXACT)
	}

	val isKnown: Boolean
		get() = type == Type.EXACT || type == Type.INFINITE

	val isUnknown: Boolean
		get() = !isKnown

	// Both distances should be known
	override fun compareTo(other: Distance): Int {
		require(isKnown)
		require(other.isKnown)
		return when {
			type == Type.INFINITE && other.type == Type.INFINITE -> 0
			type == Type.INFINITE -> 1
			other.type == Type.INFINITE -> -1
			else -> value - other.value
		}
	}

	override fun equals(other: Any?) = other is Distance && compareTo(other) == 0

	override fun hashCode(): Int = 31 * type.hashCode() + value

	override fun toString() = when (type) {
		Type.EXACT -> "(E$value)"
		Type.UNKNOWN -> "(U)"
		Type.INFINITE -> "(I)"
	}

	enum class Type {
		INFINITE,
		EXACT,
		UNKNOWN,
	}
}