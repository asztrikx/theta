package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.utils.ArgVisualizer
import hu.bme.mit.theta.common.logging.Logger
import hu.bme.mit.theta.common.visualization.Graph

class ArgFileVisualizer<S: State, A: Action>(
	private val arg: ARG<S, A>,
) : FileVisualizer() {
	override fun visualize(state: String, index: Int) {
		super.visualizeBase(getTitle(state, index), getGraph(index))
	}

	fun getTitle(state: String, index: Int) = "$index $state"

	fun getGraph(index: Int): Graph = ArgVisualizer.getDefault().visualize(arg)
}