package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.setDistanceFromAllTargets

class FullHeuristicFinder<S: State, A: Action>: HeuristicFinder<S, A> {
	override fun findHeuristic(astarNode: AstarNode<S, A>) {
		super.findHeuristic(astarNode)
		if (astarNode.heuristic.isKnown) {
			return
		}

		check(false)
	}
}