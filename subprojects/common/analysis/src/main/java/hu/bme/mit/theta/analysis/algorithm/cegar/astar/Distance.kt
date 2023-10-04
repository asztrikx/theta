package hu.bme.mit.theta.analysis.algorithm.cegar.astar

class Distance private constructor(
	private val type: Type,
	value: Int,
) : Comparable<Distance> {
	val value = value
		get(): Int {
			// TODO if we auto set infinite for inf heuristic then propagateUpDistanceFromInfiniteDistance's 5) logic have to be rechecked
			check(type === Type.BOUNDED)
			return field
		}

	val isBounded: Boolean
		get() = type === Type.BOUNDED

	val isInfinite: Boolean
		get() = type === Type.INFINITE

	val isKnown: Boolean
		get() = isBounded || isInfinite

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

	override fun hashCode() = 31 * type.hashCode() + value

	override fun toString() = when (type) {
		Type.BOUNDED -> "(B$value)"
		Type.INFINITE -> "(I)"
		Type.UNKNOWN -> "(U)"
	}

	enum class Type {
		BOUNDED,
		INFINITE,
		UNKNOWN,
	}

	companion object {
		// static factory with caching: Distance object with same immutable content are often created
		private val cache = HashMap<Int, Distance>(100) // random number, not measured
		fun boundedOf(value: Int) = cache.computeIfAbsent(value) { Distance(Type.BOUNDED, it) }

		val ZERO = boundedOf(0)
		val ONE = boundedOf(1)
		val INFINITE = Distance(Type.INFINITE, 0)
		val UNKNOWN = Distance(Type.UNKNOWN, 0)
	}
}
