package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import kotlin.math.max

class DecreasingHeuristicFinder<S: State, A: Action, P: Prec>: HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
		weightStopAfter: Long,
	) {
		val astarArg = astarNode.astarArg
		val argNode = astarNode.argNode

		// tree parent still can be null if we get to an init node by a cover edge
		val treeParentAstarNode = argNode.parent()?.let { astarArg[it] }?.apply {
			check(heuristic.isKnown)
		}

		// TODO proof that we can't use covered nodes provider as they can be more specific not always providing a lowerbound
		astarNode.heuristic = if (treeParentAstarNode == null) {
			Distance.F0
		} else {
			Distance.finiteOf(
				// 0 is always a known lower-bound, so it is more precise to use than negative numbers (which are wrong for targets)
				max(treeParentAstarNode.heuristic.value - 1, 0)
			)
		}
	}
}