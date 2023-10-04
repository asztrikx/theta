package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.setDistanceFromAllTargets

class FullDistanceSetter<S: State, A: Action>: DistanceSetter<S, A> {
	override operator fun invoke(search: AstarSearch<S, A>) {
		check(search.reachedBoundeds.all { it.argNode.isTarget })
		search.astarArg.setDistanceFromAllTargets(search.reachedBoundeds.map { it.argNode })
	}
}
