package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarAbstractor
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch

class FullHeuristicFinder<S: State, A: Action, P: Prec>: HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
		weightStopAfter: Long,
	)
		= check(false)
}