package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance.Companion.lowerBoundOf

// TODO merge with semi ondemand
class FullyOndemandDistanceSetter<S: State, A: Action, P: Prec>: DistanceSetter<S, A, P> {
	override operator fun invoke(search: AstarSearch<S, A, P>) = search.run {
		val startNodes = startAstarNodes.map { it.argNode }

		if (reachedFinites.isEmpty()) {
			// [search] stopped because there isn't any node to process
			if (isUnreachable) {
				astarArg.propagateDownDistanceFromInfiniteDistance(startNodes)
			} else {
				lowerBoundForStartNode(search)
			}
		} else {
			// [reachedFinites] are expected to be ordered by distance
			check(reachedFinites.map { distanceUntilTarget(it) }.zipWithNext { a, b -> a <= b }.all { it })
			reachedFinites.filter { it.argNode.isTarget }.forEach { it.distance = Distance.F0 }
			reachedFinites.forEach { astarArg.propagateUpDistanceFromFiniteDistance(it, startNodes.toSet(), parents) }

			astarArg.propagateUpDistanceFromInfiniteDistance()
			// TODO LB distance can be used as a better heuristic so it would not be useless to set it, also check how would AstarNode handle that
		}
		//checkShortestDistance()
	}

	private fun lowerBoundForStartNode(search: AstarSearch<S, A, P>) = search.run {
		check(startAstarNodes.size == 1)
		startAstarNodes.forEach {
			it.distance = lowerBoundOf(lowerBoundUntilTarget())
		}
	}
}