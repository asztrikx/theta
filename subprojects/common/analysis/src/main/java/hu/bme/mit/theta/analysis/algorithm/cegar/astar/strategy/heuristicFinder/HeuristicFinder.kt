package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarAbstractor
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance

abstract class HeuristicFinder<S: State, A: Action, P: Prec> {
	// Must be class: there is no such thing as protected fun in interface

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
	) {
		if (astarNode.heuristic.isKnown) {
			return
		}
		if (astarNode.astarArg.provider == null) {
			// Lowest lower bound that satisfies a* requirements
			astarNode.heuristic = Distance.ZERO
			return
		}
		astarNode.providerAstarNode?.let {
			if (it.distance.isKnown) {
				astarNode.heuristic = it.distance
				return
			}
		}

		findHeuristicFromPrevious(astarNode, astarAbstractor)
		check(astarNode.heuristic.isKnown)
	}

	protected abstract fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
	)
}