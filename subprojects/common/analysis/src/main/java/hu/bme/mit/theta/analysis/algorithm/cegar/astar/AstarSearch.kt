package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist

// TODO projection!
/**
 * TODO document early exit
 */
class AstarSearch<S: State, A: Action>(
	val startAstarNodes: Collection<AstarNode<S, A>>,
	val stopCriterion: StopCriterion<S, A>,
) {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	private val doneSet = hashSetOf<AstarNode<S, A>>()

	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	val minDepths = mutableMapOf<AstarNode<S, A>, Int>()

	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	// TODO map always gives null if not found, this is somewhat missleading
	var parents = mutableMapOf<ArgNode<S, A>, ArgNode<S, A>?>() // TODO why argNodes?

	val astarArg: AstarArg<S, A> = startAstarNodes.first().astarArg

	private val waitlist = PriorityWaitlist.create<Edge<S, A>>(AstarWaitlistComparator())
	init {
		startAstarNodes.forEach { addToWaitlist(it, null, 0) }
	}

	// We might reach a node with known distance making an upper limit for the closest target and also for the search.
	// Possible cases:
	//   - target in a different subgraph reached by covering edge
	//   - target in the same subgraph which was reached earlier from a different subgraph (by a covering edge)
	private var weightSupremumValue: Int? = null
	private var weightSupremumAstarNode: AstarNode<S, A>? = null

	// Contains reached targets ordered by closest to furthest. Not unique and should not be as it is used for setting distances.
	// Non-target nodes the exact values must be from a previous findDistanceForAny call as we set exact distances at the end of iteration.
	// TODO target doesn't have a distance so name can be confusing
	var reachedBoundeds = mutableListOf<AstarNode<S, A>>()

	fun addToWaitlist(astarNode: AstarNode<S, A>, parentAstarNode: AstarNode<S, A>?, depth: Int) {
		val argNode = astarNode.argNode
		check(astarNode.heuristic.isKnown)

		if (argNode.isTarget) {
			reachedBoundeds += astarNode
		}

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

		if (!minDepths.containsKey(astarNode) || minDepths[astarNode]!! > depth) {
			parents[argNode] = parentAstarNode?.argNode
			minDepths[astarNode] = depth
			// TODO document early exit, only need to set [parents]
			if (!argNode.isTarget) {
				// !expanded && isLeaf: handle leftover nodes
				if (astarNode.heuristic == Distance.ZERO && !argNode.isCovered && (!argNode.isExpanded && argNode.isLeaf)) {
					// [AstarWaitlistComparator] depends on knowing whether a node is coverable // TODO move to comparator?
					astarNode.close(astarArg.reachedSet[astarNode], this)?.let {}
				}
				waitlist.add(Edge(astarNode, depth))
			}
		}
	}

	fun removeFromWaitlist(): Edge<S, A>? {
		if (stopCriterion.canStop(astarArg.arg, reachedBoundeds.map { it.argNode })) {
			return null
		}

		while (!waitlist.isEmpty) {
			val edge = waitlist.remove()
			val (astarNode, depth) = edge

			if (astarNode in doneSet) {
				continue
			}
			doneSet += astarNode

			if (astarNode.getWeight(depth).value >= (weightSupremumValue ?: Int.MAX_VALUE)) {
				reachedBoundeds += weightSupremumAstarNode!!
				weightSupremumValue = null
				weightSupremumAstarNode = null
				return edge
			}

			if (!astarNode.distance.isBounded) {
				return edge
			}

			// A node with better weightSupremumValue will be processed before current [weightSupremumValue] is reached.
			// proof: current [weightSupremumValue] > weightSupremumValue' = depth + distance > depth + heuristic = priority of a node in queue
			if ((weightSupremumValue ?: Int.MAX_VALUE) > depth + astarNode.distance.value) {
				// TODO Note for future: if target gets distance immediately then this won't be true <== is not target !!!!!!!!!!!!!!!!!!!!!
				check(AstarAbstractor.heuristicSearchType != AstarAbstractor.HeuristicSearchType.FULL)

				weightSupremumValue = depth + astarNode.distance.value
				weightSupremumAstarNode = astarNode
				continue
			}
		}

		// If we can't reach a depth greater than [weightSupremumValue] then other target is not reachable.
		if (weightSupremumValue != null) {
			check(AstarAbstractor.heuristicSearchType !== AstarAbstractor.HeuristicSearchType.FULL)
			reachedBoundeds += weightSupremumAstarNode!!
		}

		return null
	}

	// Pair would create .first and .second properties which would be hard to read
	class Edge<S: State, A: Action>(
		val end: AstarNode<S, A>,
		val depthFromAStartNode: Int
	) {
		operator fun component1() = end
		operator fun component2() = depthFromAStartNode
	}
}
