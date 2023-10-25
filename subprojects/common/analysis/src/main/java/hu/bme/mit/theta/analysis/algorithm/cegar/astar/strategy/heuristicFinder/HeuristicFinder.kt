package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarAbstractor
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance

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
	) {
		if (astarNode.heuristic.isKnown) {
			return
		}
		// TODO if target -> 0, but then they won't be expanded, which can be a problem when doing getProvider()?
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

		findHeuristicFromPrevious(astarNode, astarAbstractor, search)

		check(astarNode.heuristic.isKnown)
	}

	// TODO naming
	protected abstract fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
	)
}
