package hu.bme.mit.theta.analysis.algorithm.cegar.astar.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*

class NonFullDistanceSetter<S: State, A: Action>: DistanceSetter<S, A> {
	override fun setInnerNodesDistances(search: AstarSearch<S, A>) {
		val startNodes = search.startAstarNodes.map { it.argNode }
		search.reachedBoundeds.apply {
			// TODO: [reachedBoundeds] should be ordered
			forEach { search.astarArg.propagateUpDistanceFromKnownDistance(it, startNodes.toSet(), search.parents) }
			clear()
		}

		if (search.startAstarNodes.none { it.distance.isBounded }) {
			search.astarArg.propagateDownDistanceFromInfiniteDistance(startNodes)
		} else {
			search.astarArg.propagateUpDistanceFromInfiniteDistance()
		}
		search.astarArg.checkShortestDistance()
	}
}
