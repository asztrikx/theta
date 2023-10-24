package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer

class SemiOndemandHeuristicFinder<S: State, A: Action, P: Prec>(
	private val astarFileVisualizer: AstarFileVisualizer<S, A, P>,
	private val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
): HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
		search: AstarSearch<S, A, P>?,
	) {
		val astarArg = astarNode.astarArg
		val providerAstarNode = astarNode.providerAstarNode!!

		// Visualize current
		astarFileVisualizer.visualize("paused ${astarNode.argNode}", astarArg, search)
		DI.logger.substepLine("|  |  Paused AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")

		// get the heuristic with findDistance in parent arg
		val (_, prec) = cegarHistoryStorage.find(astarArg.provider!!)
		astarAbstractor.findDistanceForAny(listOf(providerAstarNode), AstarDistanceKnowable(), "${providerAstarNode.argNode}", prec)
		astarNode.heuristic = providerAstarNode.distance

		// Visualize current (redundant)
		astarFileVisualizer.visualize("resumed ${astarNode.argNode}", astarArg, search)
		DI.logger.substepLine("|  |  Resumed AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
	}
}