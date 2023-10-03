package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType

class AstarNode<S: State, A: Action>(
	val argNode: ArgNode<S, A>,
	// Can be null if it is the first arg.
	// Can change to its then coverer as it may get covered later.
	// Its distance may change later.
	var providerAstarNode: AstarNode<S, A>?, // TODO rename to provider?
	val astarArg: AstarArg<S, A>,
) {
	private var _distance = Distance.UNKNOWN
	var distance: Distance
		get() {
			if (argNode.isTarget) {
				check(_distance.isUnknown || _distance.value == 0)
			}
			return _distance
		}
		// Do not call with unknown distance, use reset().
		set(value) {
			checkAdmissibility(value)
			if (argNode.isTarget) {
				require(value == Distance.ZERO)
			}
			_distance = value
		}

	private var _heuristic = Distance.UNKNOWN
	var heuristic: Distance
		// It is guaranteed that once it returns a known value it won't change unless reset is called (e.g. AstarIterator).
		get() {
			return _heuristic
		}
		// Do not call with unknown distance, use reset().
		set(value) {
			require(value.isKnown)

			providerAstarNode?.let {
				if (it.distance.isKnown) {
					// Provider can change if original provider is covered => use equals
					check(value == it.distance)
					// Once provider's distance is known it can't change in value => no need to recheck
				} else {
					check(DI.heuristicSearchType == HeuristicSearchType.DECREASING)
				}
			}

			// Requirement for heuristic consistency
			if (argNode.isTarget) {
				check(value == Distance.ZERO)
			}
			_heuristic = value
		}

	// Get g(n) = h(n) + depth
	// Depth is dependent on the search (can start from any node) therefore it is not stored here
	fun getWeight(depth: Int) = if (heuristic.isInfinite) {
		heuristic
	} else {
		require(heuristic.isBounded)
		heuristic + depth
	}

	// Checks property: Heuristic <= Distance
	// Should be called with known distance.
	private fun checkAdmissibility(distance: Distance) {
		check(!(heuristic.isInfinite && !distance.isInfinite))
		check(distance.isKnown)
		check(heuristic.isKnown)
		if (!heuristic.isInfinite && !distance.isInfinite) {
			check(heuristic.value <= distance.value)
		}
	}

	fun reset() {
		_heuristic = Distance.UNKNOWN
		_distance = Distance.UNKNOWN
	}

	override fun toString() = "$argNode D$distance H$heuristic"
}
