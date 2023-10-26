package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*

// Must be class: there is no such thing as protected fun in interface
abstract class HeuristicFinder<S: State, A: Action, P: Prec> {
	/**
	 * Calculates the heuristic for [astarNode].
	 *
	 * This may be recursive.
	 *
	 * @param astarNode should already have providerNode if not in the first arg
	 */
	operator fun invoke(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
		weightStopAfter: Long,
	) {
		common(astarNode)
		if (astarNode.heuristic.isKnown) {
			return
		}

		findHeuristicFromPrevious(astarNode, astarAbstractor, search, weightStopAfter)
	}

	fun common(astarNode: AstarNode<S, A>) {
//		if (astarNode.argNode.isTarget && DI.enableOptimization) {
//			astarNode.heuristic = Distance.F0
//			return
//		}
		if (astarNode.heuristic.isKnown) {
			return
		}

		if (astarNode.astarArg.provider == null) {
			// 0 is always a known lower-bound, so it is more precise to use than negative numbers (which are wrong for targets)
			astarNode.heuristic = Distance.F0
			return
		}

		astarNode.providerAstarNode?.let {
			if (it.distance.isKnown) {
				astarNode.heuristic = it.distance
				return
			}
		}
	}

	// TODO naming
	protected abstract fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
		weightStopAfter: Long,
	)
}
