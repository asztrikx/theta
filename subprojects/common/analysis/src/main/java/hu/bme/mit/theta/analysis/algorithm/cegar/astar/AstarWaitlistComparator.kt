package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge

class AstarWaitlistComparator<S: State, A: Action> : Comparator<Edge<S, A>> {
	override fun compare(edge1: Edge<S, A>, edge2: Edge<S, A>): Int {
		val astarNode1 = edge1.end
		val astarNode2 = edge2.end
		val argNode1 = astarNode1.argNode
		val argNode2 = astarNode2.argNode

		val weight1 = astarNode1.getWeight(edge1.depthFromAStartNode).value
		val weight2 = astarNode2.getWeight(edge2.depthFromAStartNode).value

		// optimization
		return when {
			weight1 == weight2 -> when {
				argNode1.isTarget && argNode2.isTarget -> 0
				argNode1.isTarget -> -1
				argNode2.isTarget -> 1
				else -> 0
			}
			else -> weight1 - weight2
		}
	}
}
