package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.initNodes
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer
import hu.bme.mit.theta.common.visualization.Graph

class AstarFileVisualizer<S: State, A: Action, P: Prec>(
	internal val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
): FileVisualizer() {
	override fun visualize(state: String, index: Int) = visualize(state, cegarHistoryStorage[index].first, null)

	fun visualize(state: String, astarArg: AstarArg<S, A>, search: AstarSearch<S, A, P>?) {
		// TODO code repetition: do not create graph; old code: lambda was passed to base
		if (!enabled) return
		val startNodes = astarArg.arg.initNodes()
		visualize(state, astarArg, startNodes, search)
	}

	fun visualize(state: String, astarArg: AstarArg<S, A>, startNodes: Collection<ArgNode<S, A>>, search: AstarSearch<S, A, P>?) {
		// TODO code repetition: do not create graph; old code: lambda was passed to base
		if (!enabled) return
		super.visualizeBase(getTitle(state, cegarHistoryStorage.indexOf(astarArg)), getGraph(astarArg, startNodes, search))
	}

	fun getTitle(state: String, index: Int): String {
		// To be consistent with Logger outputs iteration should start from 1
		// but for avoiding confusion during debugging this will start from 0
		val maxIndex = cegarHistoryStorage.size - 1
		// '⁄' != '/' (for every OS)
		return "$index ⁄ $maxIndex $state"
	}

	private fun getGraph(astarArg: AstarArg<S, A>, startNodes: Collection<ArgNode<S, A>>, search: AstarSearch<S, A, P>?): Graph {
		return AstarArgVisualizer.getDefault().visualize(astarArg, startNodes, search)
	}
}