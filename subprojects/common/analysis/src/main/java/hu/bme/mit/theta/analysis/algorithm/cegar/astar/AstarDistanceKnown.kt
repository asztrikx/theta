package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion

class AstarDistanceKnown<S: State, A: Action>(private val astarNode: AstarNode<S, A>) : StopCriterion<S, A> {
	override fun canStop(arg: ARG<S, A>): Boolean {
		return true
	}

	override fun canStop(arg: ARG<S, A>, newNodes: Collection<ArgNode<S, A>>): Boolean {
		return canStop(arg)
	}
}
