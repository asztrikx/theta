package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType

class Distance private constructor(
	private val type: Type,
	value: Int,
) : Comparable<Distance> {
	val value = value
		get(): Int {
			// TODO if we auto set infinite for inf heuristic then propagateUpDistanceFromInfiniteDistance's 5) logic have to be rechecked
			check(hasValue)
			return field
		}

	val isFinite: Boolean
		get() = type == Type.FINITE

	val isInfinite: Boolean
		get() = type == Type.INFINITE

	val isKnown: Boolean
		get() = isFinite || isInfinite

	val isUnknown: Boolean
		get() = !isKnown

	val hasValue: Boolean
		get() = isFinite || isUnknown

	// Both distances should be known
	override fun compareTo(other: Distance): Int {
		return when {
			type == Type.INFINITE && other.type == Type.INFINITE -> 0
			type == Type.INFINITE -> 1
			other.type == Type.INFINITE -> -1
			else -> value.compareTo(other.value)
		}
	}

	override fun equals(other: Any?) = other is Distance && compareTo(other) == 0

	override fun hashCode() = 31 * type.hashCode() + value

	override fun toString() = when (type) {
		Type.FINITE -> "F$value"
		Type.INFINITE -> "I"
		Type.LOWERBOUND -> if (DI.heuristicSearchType == HeuristicSearchType.FULLY_ONDEMAND) "L$value" else "U"
	}

	enum class Type {
		FINITE,
		INFINITE,
		LOWERBOUND,
	}

	companion object {
		// There is no need to clear cache as distances to targets will keep get longer

		// static factory with caching: Distance object with same immutable content are often created
		private val finiteCache = HashMap<Int, Distance>(100) // random number, not measured
		fun finiteOf(value: Int) = finiteCache.computeIfAbsent(value) { Distance(Type.FINITE, it) }

		private val lowerBoundCache = HashMap<Int, Distance>(100)
		fun lowerBoundOf(value: Int) = lowerBoundCache.computeIfAbsent(value) { Distance(Type.LOWERBOUND, it) }
		fun lowerBoundOf(distance: Distance): Distance {
			// Lower-bound of infinite should be infinite
			check(distance.hasValue)
			return lowerBoundOf(distance.value)
		}

		fun sameTypeOfNoCache(distance: Distance, value: Int) = Distance(distance.type, value)

		val F0 = finiteOf(0)
		val INFINITE = Distance(Type.INFINITE, 0)
		val UNKNOWN = lowerBoundOf(0)
	}
}
