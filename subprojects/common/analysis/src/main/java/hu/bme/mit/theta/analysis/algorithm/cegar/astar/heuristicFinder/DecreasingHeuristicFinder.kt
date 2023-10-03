package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import kotlin.math.max

class DecreasingHeuristicFinder<S: State, A: Action, P: Prec>: HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
	) {
		val astarArg = astarNode.astarArg
		val argNode = astarNode.argNode

		// tree parent still can be null if we get to an init node by a cover edge
		val treeParentAstarNode = argNode.parent()?.let { astarArg[it] }

		// This can probably be simplified by some proof but this is more robust
		val candidates = mutableListOf<AstarNode<S, A>>()
		candidates.addAll(argNode.coveredNodes().map{ astarArg[it] }.filter { it.heuristic.isKnown })
		if (treeParentAstarNode != null && treeParentAstarNode.heuristic.isKnown) {
			candidates += treeParentAstarNode
		}

		// Take larger (= more precise) heuristic
		val heuristicProvider = candidates.maxByOrNull { it.heuristic }

		astarNode.heuristic = if (heuristicProvider == null) {
			Distance.ZERO
		} else {
			Distance.boundedOf(
				// 0 is always a known lowerbound, so it is more precise to use than negative numbers
				max(heuristicProvider.heuristic.value - 1, 0)
			)
		}
	}
}