package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.initNodes
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer
import hu.bme.mit.theta.common.visualization.Graph

class AstarFileVisualizer<S: State, A: Action, P: Prec>(
	enabled: Boolean,
	internal val cegarHistoryStorage: CegarHistoryStorage<S, A, P>
) : FileVisualizer(enabled) {
	override fun visualize(state: String, index: Int) {
		val astarArg = cegarHistoryStorage[index].first
		val startNodes = astarArg.arg.initNodes()
		visualize(state, index, startNodes)
	}

	fun visualize(state: String, index: Int, startNodes: Collection<ArgNode<S, A>>) {
		super.visualizeBase(getTitle(state, index), getGraph(index, startNodes))
	}

	fun getTitle(state: String, index: Int): String {
		// To be consistent with Logger outputs iteration should start from 1
		// but for avoiding confusion during debugging this will start from 0
		val maxIndex = cegarHistoryStorage.size - 1
		// '⁄' != '/' (for every OS)
		return "$index ⁄ $maxIndex $state"
	}

	fun getGraph(index: Int, startNodes: Collection<ArgNode<S, A>>): Graph {
		return AstarArgVisualizer.getDefault().visualize(cegarHistoryStorage[index].first, startNodes)
	}
}