package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance.Companion.lowerBoundOf
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
		get() = _distance
		// Do not call with unknown distance, use reset().
		set(value) {
			_distance = value
			if (argNode.isTarget) {
				// distance property
				require(value === Distance.F0)
			}

			check(!(heuristic.isInfinite && value.isKnown && !value.isInfinite))

			// Leftover node may not have heuristic // TODO check this
			if (heuristic.hasValue && distance.isKnown) { // TODO when does this fail
				// admissibility
				check(heuristic <= value)
			}
			// Can't set known distance as heuristic as it can break consistency
		}

	private var _heuristic = Distance.UNKNOWN
	var heuristic: Distance
		// It is guaranteed that once it returns a known value it won't change unless reset is called (e.g. AstarIterator).
		get() = _heuristic
		// Do not call with unknown distance, use reset().
		set(value) {
			check(value >= _heuristic)
			_heuristic = value

			providerAstarNode?.let {
				check(value >= it.distance)
				if (it.distance.isUnknown) {
					// TODO By updating provider to covering node, this can break; also wait until this is compared to old code
					//check(DI.heuristicSearchType == HeuristicSearchType.DECREASING)
				}
			}

			if (argNode.isTarget) {
				// consistency
				check(value === Distance.F0)
			}
			// TODO decreasing when copied can fail here
			val checkConsistency: ArgNode<S, A>.() -> Unit = {
				// Null when copying AstarArg // TODO check this whether this can be avoided by changing copying, same for below
				astarArg.astarNodes[this]?.let {
					// When starting leftovers from init nodes we can either reach a node first through normal or cover edge
					if (it.heuristic.isKnown && !it.heuristic.isInfinite) { // TODO can parent heuristic be infinite?
						// Both heuristic is known
						// Parent's heuristic is not infinite
						it.checkConsistency(this@AstarNode)
					}
				}
			}
			if (value.isKnown) {
				// Heuristic needs to be already set for these
				argNode.parent()?.let { it.checkConsistency() }
				argNode.coveredNodes().forEach { it.checkConsistency() }
			}

			// Infinite distance will be set by [DistanceSetter]
			if (!argNode.isTarget && value.hasValue && !distance.isKnown && DI.heuristicSearchType == HeuristicSearchType.FULLY_ONDEMAND) {
				distance = lowerBoundOf(value)
			}
		}

	// Get g(n) = h(n) + depth
	// Depth is dependent on the search (can start from any node) therefore it is not stored here
	fun getWeight(depth: Int) =
		if (distance.isKnown) {
			safeAdd(distance, depth)
		} else {
			safeAdd(heuristic, depth)
		}

	fun reset() {
		_heuristic = Distance.UNKNOWN
		_distance = Distance.UNKNOWN
	}

	override fun toString() = "$argNode D($distance) H($heuristic)"
}
