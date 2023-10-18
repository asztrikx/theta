package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*

class NonFullDistanceSetter<S: State, A: Action, P: Prec>: DistanceSetter<S, A, P> {
	override operator fun invoke(search: AstarSearch<S, A, P>) {
		search.apply {
			val startNodes = startAstarNodes.map { it.argNode }

			// [search.reachedFinites] are expected to be ordered by distance
			check(reachedFinites.map { distanceUntilTarget(it) }.zipWithNext { a, b -> a <= b }.all { it })
			reachedFinites.filter { it.argNode.isTarget }.forEach { it.distance = Distance.F0 }
			reachedFinites.forEach { astarArg.propagateUpDistanceFromFiniteDistance(it, startNodes.toSet(), parents) }

			if (startAstarNodes.none { it.distance.isFinite }) {
				astarArg.propagateDownDistanceFromInfiniteDistance(startNodes)
			} else {
				astarArg.propagateUpDistanceFromInfiniteDistance()
			}
			//checkShortestDistance()

			check(startAstarNodes.any { it.distance.isFinite } || startAstarNodes.all { it.distance.isInfinite })
		}
	}
}
