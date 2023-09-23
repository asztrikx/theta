package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import kotlin.math.max

class DecreasingHeuristicFinder<S: State, A: Action, P: Prec>: HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(astarNode: AstarNode<S, A>, abstractor: AstarAbstractor<S, A, P>) {
		// We don't have heuristic from provider therefore we decrease parent's
		// astarArg.provider == null case could also be handled by this
		val astarArg = astarNode.astarArg
		// TODO why take graph parent? take max of covered node's heuristics (if they exists) and graph parent - 1 (-,,-)
		// TODO ^-- this is a bug introduced in refactoring (see theta-tdk for correct)
		/*
		https://github.com/asztrikx/theta/blob/9f50b52f9fd098b32e09f7f65e0aab0ad19cbe48/subprojects/common/analysis/src/main/java/hu/bme/mit/theta/analysis/algorithm/cegar/astar/AstarNode.java#L48
			fedőélen át lehet olyan h a szülő csökkentett heurisztikás de a rendes szülője nem az: https://github.com/asztrikx/theta/blob/9f50b52f9fd098b32e09f7f65e0aab0ad19cbe48/subprojects/common/analysis/src/main/java/hu/bme/mit/theta/analysis/algorithm/cegar/astar/AstarAbstractor.java#L479
			biztosítani kell ebben az esetben is h a rendes szülőnek legyen már meglegyen a csökkentett heurisztikája (copy miatt lehet nincs még meg)
			illetve hogy azt csökkentse
		*/
		val treeParentAstarNode = astarNode.argNode.parent()?.let { astarArg[it] }

		// init node as we are always starting from startNodes
		if (treeParentAstarNode == null) {
			astarNode.heuristic = Distance.ZERO
			return
		}
		check(treeParentAstarNode.heuristic.isKnown) // TODO why should it always exists? give proof
		astarNode.heuristic = Distance.boundedOf(
			// 0 is always a known lowerbound, so it is more precise to use than negative numbers
			max(treeParentAstarNode.heuristic.value - 1, 0)
		)
	}
}