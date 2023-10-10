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
	private val depths = hashMapOf<AstarNode<S, A>, Int>()

	// Used for debugging
	private val weights = hashMapOf<AstarNode<S, A>, Distance>()

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

	// Contains reached targets ordered by closest to furthest. Not unique and should not be as it is used for setting distances.
	// Non-target nodes the finite values must be from a previous findDistanceForAny call as we set finite distances at the end of iteration.
	// TODO target doesn't have a distance so name can be confusing
	var reachedFinites = mutableListOf<AstarNode<S, A>>()

	fun addToWaitlist(astarNode: AstarNode<S, A>, parentAstarNode: AstarNode<S, A>?, depth: Int) {
		heuristicFinder(astarNode, astarAbstractor, this)

		if (astarNode.distance.isKnown) {
			check(DI.heuristicSearchType === HeuristicSearchType.SEMI_ONDEMAND)
		}

		if (astarNode.heuristic.isInfinite) {
			return
		}

		// TODO rephrase: We can reach into infinite subgraph (cover or even with normal edge as some parts are marked as infinite in optimization)
		if (astarNode.distance.isInfinite) {
			return
		}

		if (astarNode in doneSet) {
			check(depths[astarNode]!! <= depth)
			return
		}

		if (astarNode !in depths || depths[astarNode]!! > depth) {
			parents[astarNode] = parentAstarNode
			depths[astarNode] = depth
			weights[astarNode] = astarNode.getWeight(depth)
			waitlist.add(Edge(astarNode, depth))
		}
	}

	fun removeFromWaitlist(): Edge<S, A>? {
		while (!waitlist.isEmpty && (
			!stopCriterion.canStop(astarArg.arg, reachedFinites.map { it.argNode }) ||
			waitlist.peek().end.reachesTarget
		)) {
			val edge = waitlist.remove()
			val (astarNode, _) = edge

			if (astarNode in doneSet) {
				continue
			}
			doneSet += astarNode

			if (astarNode.reachesTarget) {
				reachedFinites += astarNode
				// TODO pattern
				if (DI.heuristicSearchType != HeuristicSearchType.FULL) {
					// Distance finite case:
					// We know the exact remaining distance until a target, we don't need to visit this subgraph
					// Because of leftover nodes it is also possible to reach a node like this through a normal edge.
					// Only happens when searching in an older [ARG]

					// Target case:
					// TODO rephrase: do not return Target as it is a useless target under a target

					continue
				}
				check(!astarNode.distance.isFinite)
			}

			return edge
		}

		return null
	}

	fun distanceUntilTarget(astarNode: AstarNode<S, A>): Int {
		return if (astarNode.argNode.isTarget) {
			depths[astarNode]!!
		} else {
			depths[astarNode]!! + astarNode.distance.value
		}
	}

	operator fun contains(astarNode: AstarNode<S, A>) = astarNode in depths

	fun toString(astarNode: AstarNode<S, A>): String {
		val depth = depths[astarNode]!!
		val weight = weights[astarNode]!!
		val heuristic = astarNode.heuristic
		return "G($depth) + H($heuristic) = F($weight)"
	}

	// Pair would create .first and .second properties which would be hard to read
	data class Edge<S: State, A: Action>(
		val end: AstarNode<S, A>,
		val depth: Int
	) {
		val weight
			get() = end.getWeight(depth)
	}
}
