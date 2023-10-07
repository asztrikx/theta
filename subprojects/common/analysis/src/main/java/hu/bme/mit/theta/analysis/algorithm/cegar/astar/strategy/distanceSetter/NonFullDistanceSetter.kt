package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*

class NonFullDistanceSetter<S: State, A: Action, P: Prec>: DistanceSetter<S, A, P> {
	override operator fun invoke(search: AstarSearch<S, A, P>) {
		val startNodes = search.startAstarNodes.map { it.argNode }
		search.reachedFinites.apply {
			// [search.reachedFinites] are expected to be ordered by depth
			check(zipWithNext { a, b -> search.minDepths[a]!! <= search.minDepths[b]!! }.all { it })
			filter { it.argNode.isTarget }.forEach { it.distance = Distance.ZERO }
			forEach { search.astarArg.propagateUpDistanceFromFiniteDistance(it, startNodes.toSet(), search.parents) }
		}

		search.astarArg.apply {
			if (search.startAstarNodes.none { it.distance.isFinite }) {
				propagateDownDistanceFromInfiniteDistance(startNodes)
			} else {
				propagateUpDistanceFromInfiniteDistance()
			}
			//checkShortestDistance()
		}

		search.reachedFinites.clear()
	}
}
