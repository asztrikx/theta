package hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.common.logging.Logger

// TODO these could be just ..functions...
class SemiOndemandHeuristicFinder<S: State, A: Action, P: Prec>(
	val logger: Logger,
	val astarFileVisualizer: AstarFileVisualizer<S, A, P>,
	val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
): HeuristicFinder<S, A> {
	override fun findHeuristic(astarNode: AstarNode<S, A>, findDistanceForAny: Collection<AstarNode<S, A>>.(stopCriterion: StopCriterion<S, A>, visualizerState: String, prec: P) -> Unit) {
		super.findHeuristic(astarNode)
		if (astarNode.heuristic.isKnown) {
			return
		}

		val astarArg = astarNode.astarArg
		val providerAstarArg = astarArg.provider!!

		// Provider AstarNode can be null
		check(astarNode.providerAstarNode != null)
		val providerAstarNode = astarNode.providerAstarNode!!
		if (providerAstarNode.heuristic.isInfinite) {
			check(providerAstarNode.distance.isInfinite)
		}

		val (_, prec) = cegarHistoryStorage.find(providerAstarArg)

		// Visualize current
		astarFileVisualizer.visualize("paused ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Paused AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")

		// get the heuristic with findDistance in parent arg
		listOf(providerAstarNode).findDistanceForAny(AstarDistanceKnown(providerAstarNode), "${providerAstarNode.argNode}", prec)
		check(astarNode.heuristic.isKnown)

		// Visualize current (redundant)
		astarFileVisualizer.visualize("resumed ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Resumed AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
	}
}