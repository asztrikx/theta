package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import kotlin.math.max

class DecreasingHeuristicFinder<S: State, A: Action>: HeuristicFinder<S, A> {
	override fun findHeuristic(astarNode: AstarNode<S, A>,) {
		super.findHeuristic(astarNode)
		if (astarNode.heuristic.isKnown) {
			return
		}

		// We don't have heuristic from provider therefore we decrease parent's
		// astarArg.provider == null case could also be handled by this
		val astarArg = astarNode.astarArg
		val parentAstarNode = astarNode.argNode.parent()?.let { astarArg[it] }

		// init node as we are always starting from startNodes
		if (parentAstarNode == null) {
			astarNode.heuristic = Distance.ZERO
			return
		}
		check(parentAstarNode.argNode.coveringNode() == null)
		check(parentAstarNode.heuristic.isKnown)
		var parentHeuristicValue = parentAstarNode.heuristic.value
		parentHeuristicValue = max(parentHeuristicValue - 1, 0)
		astarNode.heuristic = Distance.boundedOf(parentHeuristicValue)
	}
}