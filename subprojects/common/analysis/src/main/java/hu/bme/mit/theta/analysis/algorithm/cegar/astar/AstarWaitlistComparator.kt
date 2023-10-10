package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge
import kotlin.math.abs

class AstarWaitlistComparator<S: State, A: Action> : Comparator<Edge<S, A>> {
	override fun compare(edge1: Edge<S, A>, edge2: Edge<S, A>): Int {
		/*
		TODO https://photos.app.goo.gl/wguQ7K9opyLqTUPa7
		Monotonicity
		Indirect proof
		- target selection
		this handles the case where we visit a node with a distance
		*/
		val weight1 = edge1.weight
		val weight2 = edge2.weight
		val comparator =
			if (weight1 == weight2) {
				predicateOrderedComparator(
					{ it.end.reachesTarget },
					{ it.end.heuristic.value in 0..1 },
				)
			} else if (abs((weight1 - weight2).value) == 1) {
				val smaller = minOf(weight1, weight2)
				val larger = smaller + 1
				predicateOrderedComparator(
					{ it.weight == smaller && it.end.reachesTarget },
					{ it.weight == smaller && it.end.heuristic.value in 0..1 },
					{ it.weight == larger && it.end.reachesTarget },
					{ it.weight == smaller },
					{ it.weight == larger },
				)
			} else {
				compareBy { it: Edge<S, A> ->
					it.weight
				}
			}
		return comparator.compare(edge1, edge2)
	}
}
