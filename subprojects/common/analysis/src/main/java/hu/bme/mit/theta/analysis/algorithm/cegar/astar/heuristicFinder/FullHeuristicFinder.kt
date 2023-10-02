package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarAbstractor
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode

class FullHeuristicFinder<S: State, A: Action, P: Prec>: HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(astarNode: AstarNode<S, A>, astarAbstractor: AstarAbstractor<S, A, P>)
		= check(false)
}