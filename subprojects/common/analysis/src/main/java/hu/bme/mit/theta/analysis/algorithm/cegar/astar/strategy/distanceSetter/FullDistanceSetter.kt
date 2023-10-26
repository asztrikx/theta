package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.setDistanceFromAllTargets

class FullDistanceSetter<S: State, A: Action, P: Prec>: DistanceSetter<S, A, P> {
	override operator fun invoke(search: AstarSearch<S, A, P>) = search.run {
		check(reachedFinites.all { it.argNode.isTarget })
		astarArg.setDistanceFromAllTargets(reachedFinites)
		check(startAstarNodes.any { it.distance.isFinite } || startAstarNodes.all { it.distance.isInfinite })
	}
}
