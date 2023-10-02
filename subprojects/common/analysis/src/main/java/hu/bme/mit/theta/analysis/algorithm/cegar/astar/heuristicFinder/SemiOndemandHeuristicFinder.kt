package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.common.logging.Logger

class SemiOndemandHeuristicFinder<S: State, A: Action, P: Prec>(
	private val logger: Logger,
	private val astarFileVisualizer: AstarFileVisualizer<S, A, P>,
	private val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
): HeuristicFinder<S, A, P>() {
	override fun findHeuristicFromPrevious(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
	) {
		val astarArg = astarNode.astarArg

		check(astarNode.providerAstarNode != null)
		val providerAstarNode = astarNode.providerAstarNode!!
		if (providerAstarNode.heuristic.isInfinite) {
			check(providerAstarNode.distance.isInfinite)
		}

		val (_, prec) = cegarHistoryStorage.find(astarArg.provider!!)

		// Visualize current
		astarFileVisualizer.visualize("paused ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Paused AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")

		// get the heuristic with findDistance in parent arg
		astarAbstractor.findDistanceForAny(listOf(providerAstarNode), AstarDistanceKnowable(), "${providerAstarNode.argNode}", prec)

		// Visualize current (redundant)
		astarFileVisualizer.visualize("resumed ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Resumed AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
	}
}