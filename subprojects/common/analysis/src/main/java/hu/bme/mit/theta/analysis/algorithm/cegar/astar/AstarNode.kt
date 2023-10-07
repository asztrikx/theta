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
			if (argNode.isTarget) {
				require(value === Distance.ZERO)
			}
			_distance = value
			// Distance needs to be already set for this
			checkAdmissibility()
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
					check(value === it.distance)
				} else {
					check(DI.heuristicSearchType == HeuristicSearchType.DECREASING)
				}
			}

			_heuristic = value

			// Requirement for heuristic consistency
			if (argNode.isTarget) {
				check(value === Distance.ZERO)
			}
			val checkConsistency: ArgNode<S, A>.() -> Unit = {
				// Null when copying AstarArg // TODO check this whether this can be avoided by changing copying, same for below
				astarArg.astarNodes[this]?.let {
					// When starting leftovers from init nodes we can either reach a node first through normal or cover edge
					if (it.heuristic.isKnown) {
						it.checkConsistency(this@AstarNode)
					}
				}
			}
			// Heuristic needs to be already set for these
			argNode.parent()?.let { it.checkConsistency() }
			argNode.coveredNodes().forEach { it.checkConsistency() }
		}

	// Get g(n) = h(n) + depth
	// Depth is dependent on the search (can start from any node) therefore it is not stored here
	fun getWeight(depth: Int) = if (heuristic.isInfinite) {
		heuristic.value
	} else {
		require(heuristic.isFinite)
		heuristic.value + depth
	}

	fun reset() {
		_heuristic = Distance.UNKNOWN
		_distance = Distance.UNKNOWN
	}

	override fun toString() = "$argNode D$distance H$heuristic"
}
