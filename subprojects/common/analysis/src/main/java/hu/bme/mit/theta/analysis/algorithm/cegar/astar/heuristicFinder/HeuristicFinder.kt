package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance

interface HeuristicFinder<S: State, A: Action> {
	/**
	 * Calculates the heuristic for [astarNode].
	 *
	 * This may be recursive.
	 *
	 * @param astarNode should already have providerNode if not in the first arg
	 */
	fun findHeuristic(astarNode: AstarNode<S, A>) {
		if (astarNode.astarArg.provider == null) {
			// Lowest lower bound that satisfies a* requirements
			astarNode.heuristic = Distance.ZERO
			return
		}
	}
}