package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion

class AstarDistanceKnown<S: State, A: Action>(private val astarNode: AstarNode<S, A>) : StopCriterion<S, A> {
	/**
	 * Currently distance is always set after canStop is called so it always returns true.
	 * **Only use when closest target is known.**
	 */
	override fun canStop(arg: ARG<S, A>) = true

	/**
	 * See [canStop]
	 */
	override fun canStop(arg: ARG<S, A>, newNodes: Collection<ArgNode<S, A>>) = canStop(arg)
}
