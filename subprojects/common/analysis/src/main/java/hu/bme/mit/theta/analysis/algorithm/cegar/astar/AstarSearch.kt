package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist

/**
 * TODO document early exit
 */
class AstarSearch<S: State, A: Action, P: Prec>(
	val startAstarNodes: Collection<AstarNode<S, A>>,
	private val stopCriterion: StopCriterion<S, A>,
	private val heuristicFinder: HeuristicFinder<S, A, P>,
	private val astarAbstractor: AstarAbstractor<S, A, P>,
) {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	private val doneSet = hashSetOf<AstarNode<S, A>>()

	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	val minDepths = hashMapOf<AstarNode<S, A>, Int>()

	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	// TODO map always gives null if key is not found, value type is somewhat misleading
	var parents = hashMapOf<AstarNode<S, A>, AstarNode<S, A>?>()

	val astarArg: AstarArg<S, A> = startAstarNodes.first().astarArg

	private val waitlist = PriorityWaitlist.create<Edge<S, A>>(AstarWaitlistComparator())
	init {
		startAstarNodes.forEach { addToWaitlist(it, null, 0) }
	}

	// We might reach a node with known distance making an upper limit for the closest target and also for the search.
	// Possible cases:
	//   - target in a different subgraph reached by covering edge
	//   - target in the same subgraph which was reached earlier from a different subgraph (by a covering edge)
	private var weightSupremumValue: Int? = null // TODO document: only when not refining
	private var weightSupremumAstarNode: AstarNode<S, A>? = null

	// Contains reached targets ordered by closest to furthest. Not unique and should not be as it is used for setting distances.
	// Non-target nodes the finite values must be from a previous findDistanceForAny call as we set finite distances at the end of iteration.
	// TODO target doesn't have a distance so name can be confusing
	var reachedFinites = mutableListOf<AstarNode<S, A>>()

	fun addToWaitlist(astarNode: AstarNode<S, A>, parentAstarNode: AstarNode<S, A>?, depth: Int) {
		heuristicFinder(astarNode, astarAbstractor)

		if (astarNode.heuristic.isInfinite) {
			return
		}

		// TODO rephrase: We can reach into infinite subgraph (cover or even with normal edge as some parts are marked as infinite in optimization)
		if (astarNode.distance.isInfinite) {
			return
		}

		if (astarNode in doneSet) {
			check(minDepths[astarNode]!! <= depth)
			return
		}

		if (astarNode !in minDepths || minDepths[astarNode]!! > depth) {
			parents[astarNode] = parentAstarNode
			minDepths[astarNode] = depth
			waitlist.add(Edge(astarNode, depth))
		}
	}

	fun removeFromWaitlist(): Edge<S, A>? {
		while (!waitlist.isEmpty) {
			val peek = waitlist.peek()
			if (peek.end.getWeight(peek.depthFromAStartNode) >= (weightSupremumValue ?: Int.MAX_VALUE)) {
				reachedFinites += weightSupremumAstarNode!!
				weightSupremumValue = null
				weightSupremumAstarNode = null
			}

			if (stopCriterion.canStop(astarArg.arg, reachedFinites.map { it.argNode })) {
				return null
			}

			val edge = waitlist.remove()
			val (astarNode, depth) = edge

			if (astarNode in doneSet) {
				continue
			}
			doneSet += astarNode

			if (astarNode.argNode.isTarget) {
				reachedFinites += astarNode
				// TODO rephrase: do not return Target as it is a useless target under a target
				continue
			}

			if (!astarNode.distance.isFinite) {
				return edge
			}

			// A node with better weightSupremumValue will be processed before current [weightSupremumValue] is reached.
			// proof: current [weightSupremumValue] > weightSupremumValue' = depth + distance > depth + heuristic = priority of a node in queue
			if ((weightSupremumValue ?: Int.MAX_VALUE) > depth + astarNode.distance.value) {
				// TODO Note for future: if target gets distance immediately then this won't be true <== is not target !!!!!!!!!!!!!!!!!!!!!
				check(DI.heuristicSearchType != HeuristicSearchType.FULL)

				weightSupremumValue = depth + astarNode.distance.value
				weightSupremumAstarNode = astarNode
				continue
			}
		}

		// If we can't reach a depth greater than [weightSupremumValue] then other target is not reachable.
		if (weightSupremumValue != null) {
			check(DI.heuristicSearchType !== HeuristicSearchType.FULL)
			reachedFinites += weightSupremumAstarNode!!
		}

		return null
	}

	// Pair would create .first and .second properties which would be hard to read
	data class Edge<S: State, A: Action>(
		val end: AstarNode<S, A>,
		val depthFromAStartNode: Int
	) {
		val weight
			get() = end.getWeight(depthFromAStartNode)
	}
}
