package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion

/**
 * StopCriterion used for finding distance for an AstarNode
 */
class AstarDistanceKnowable<S: State, A: Action> : StopCriterion<S, A> {
	/**
	 * Can't be decided based on an Arg. When searching in previous args we already have target nodes.
	 */
	override fun canStop(arg: ARG<S, A>) = throw NotImplementedError()

	/**
	 * Stops when current search reaches a target either directly or indirectly (and no other found closer) so the distance for the start node can be set.
 	 */
	override fun canStop(arg: ARG<S, A>, newNodes: Collection<ArgNode<S, A>>) = newNodes.isNotEmpty()
}
