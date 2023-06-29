package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist
import kotlin.collections.ArrayDeque

class AstarSearch<S: State, A: Action> {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	private val doneSet = hashSetOf<AstarNode<S, A>>()

	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	private val minDepths = mutableMapOf<AstarNode<S, A>, Int>()

	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	var parents = mutableMapOf<ArgNode<S, A>, ArgNode<S, A>?>() // TODO why argNodes? // TODO map always gives null if not found, this is somewhat missleading

	private val waitlist = PriorityWaitlist.create<Edge<S, A>>(AstarWaitlistComparator())

	// We might reach a node with known distance making an upper limit for the closest target and also for the search.
	// Possible cases:
	//   - target in a different subgraph reached by covering edge
	//   - target in the same subgraph which was reached earlier from a different subgraph (by a covering edge)
	var upperLimitValue = -1
	var upperLimitAstarNode: AstarNode<S, A>? = null

	// Only used in check-s
	// TODO check naming (depthFromAStartNode?)
	private var depths = mutableMapOf<ArgNode<S, A>, Int>()

	var reachedExacts = ArrayDeque<AstarNode<S, A>>()//

	fun addToWaitlist(astarNode: AstarNode<S, A>, parentAstarNode: AstarNode<S, A>?, depth: Int) {
		check(astarNode.heuristic.isKnown)
		if (astarNode.heuristic.isInfinite) {
			return
		}
		// TODO document when/why can this happen?
		if (astarNode.distance.isInfinite) {
			return
		}

		if (astarNode in doneSet) {
			check(depths[astarNode.argNode]!! <= depth)
			return
		}

		if (!minDepths.containsKey(astarNode) || minDepths[astarNode]!! > depth) {
			waitlist.add(Edge(astarNode, depth))
			parents[astarNode.argNode] = parentAstarNode?.argNode
			depths[astarNode.argNode] = depth
			minDepths[astarNode] = depth
		}
	}

	val isWaitlistEmpty: Boolean
		get() = waitlist.isEmpty

	fun removeFromWaitlist(): Edge<S, A>? {
		check(!waitlist.isEmpty)
		while (!waitlist.isEmpty) {
			val edge = waitlist.remove()
			val (astarNode, depth) = edge

			// TODO what if we reach already reached target through cover edge in the same iteration? this is not handled!!!! finomodhat-e target node nem targetté => coverelhet e sima nodeot target
			if (astarNode in doneSet) {
				continue
			}
			doneSet += astarNode

			if (!astarNode.distance.isBounded) {
				return edge
			}

			// A node with better upper limit will be processed before current upperlimit is reached.
			// [upperLimitValue] > depth + distance > depth + heuristic
			if (upperLimitValue > depth + astarNode.distance.value || upperLimitValue == -1) {
				// TODO Note for future: if target gets distance immediately then this won't be true <== is not target
				check(AstarAbstractor.heuristicSearchType != AstarAbstractor.HeuristicSearchType.FULL)

				upperLimitValue = depth + astarNode.distance.value
				upperLimitAstarNode = astarNode
			}
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