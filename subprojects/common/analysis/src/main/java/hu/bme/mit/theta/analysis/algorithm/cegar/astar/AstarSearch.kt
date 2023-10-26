package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist
import kotlin.math.min

/**
 * TODO document early exit
 */
class AstarSearch<S: State, A: Action, P: Prec>(
	val startAstarNodes: List<AstarNode<S, A>>,
	private val stopCriterion: StopCriterion<S, A>,
	var weightStopAfter: Long,
	private val heuristicFinder: HeuristicFinder<S, A, P>,
	private val astarAbstractor: AstarAbstractor<S, A, P>,
) {
	private val doneSet = hashSetOf<AstarNode<S, A>>()

	// Used to know whether the current item is smaller than the one in the [finiteWaitlist] (if not in [doneSet])
	private val depths = hashMapOf<AstarNode<S, A>, Int>()

	// Used for debugging the search <= heuristic can change later
	private val weights = hashMapOf<AstarNode<S, A>, Distance>()

	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	// TODO map always gives null if key is not found, value type is somewhat misleading
	var parents = hashMapOf<AstarNode<S, A>, AstarNode<S, A>?>()

	val astarArg = startAstarNodes.first().astarArg

	// TODO fully on-demand: useless expand if heuristic is zero and weight reaches limit => order it accordingly and use peek() in removeFromWaitlist
	private val finiteWaitlist = PriorityWaitlist.create<Edge<S, A>>(if (DI.disableOptimizations) OldAstarWaitlistComparator() else AstarWaitlistComparator())
	private val unknownWaitlist = PriorityWaitlist.create<Edge<S, A>>(compareBy { it.end.heuristic.value })
	init {
		startAstarNodes.forEach { addToWaitlist(it, null, 0) }
	}

	// Contains reached targets ordered by closest to furthest. Not unique and should not be as it is used for setting distances.
	// Non-target nodes the finite values must be from a previous findDistanceForAny call as we set finite distances at the end of iteration.
	// TODO target doesn't have a distance so name can be confusing
	var reachedFinites = mutableListOf<AstarNode<S, A>>()

	fun addToWaitlist(astarNode: AstarNode<S, A>, parentAstarNode: AstarNode<S, A>?, depth: Int) {
		if (DI.heuristicSearchType != HeuristicSearchType.FULLY_ONDEMAND) {
			// We are either covered into
			// - completed node (was in waitlist => has heuristic)
			// - completed node's child (in waitlist => has heuristic)
			// - leftover from prune:
			//   - decreasing: during copy we call [findHeuristic] => has heuristic
			//   - non-decreasing: we can cover into a leftover node before it is visited (even when starting from init nodes) => may not have
			if (DI.heuristicSearchType == HeuristicSearchType.DECREASING) {
				if (parentAstarNode != null && parentAstarNode.argNode.isCovered) {
					check(astarNode.heuristic.isKnown)
				}
			}
			heuristicFinder(astarNode, astarAbstractor, this, Long.MAX_VALUE)
		} else {
			heuristicFinder.common(astarNode)
		}

		// If we reach a node with a distance in an earlier iteration we can safely put it into the [finiteWaitlist]
		if (astarNode.distance.isKnown) {
			check(DI.heuristicSearchType == HeuristicSearchType.SEMI_ONDEMAND || DI.heuristicSearchType == HeuristicSearchType.FULLY_ONDEMAND)
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

			if (astarNode.heuristic.isUnknown) {
				unknownWaitlist.add(Edge(astarNode, depth))
			} else {
				finiteWaitlist.add(Edge(astarNode, depth))
			}
		}
	}

	fun removeFromWaitlist(): Edge<S, A>? {
		while(
			unknownWaitlist.isNotEmpty &&
			weightStopAfter >= unknownWaitlist.peek().weight.value &&
			(
				finiteWaitlist.isEmpty ||
				unknownWaitlist.peek().weight.value < finiteWaitlist.peek().weight.value
			)
		) {
			val edge = unknownWaitlist.remove()
			val (astarNode, depth) = edge
			val nextValue = min(nextWeight(unknownWaitlist), nextWeight(finiteWaitlist))
			heuristicFinder(astarNode, astarAbstractor, this, nextValue)
			// [astarNode] could receive a known heuristic, which also could be infinite
			depths.remove(astarNode)
			addToWaitlist(astarNode, parents[astarNode], depth)
		}
		if (unknownWaitlist.isNotEmpty && weightStopAfter < unknownWaitlist.peek().weight.value) {
			return null
		}

		while (
			finiteWaitlist.isNotEmpty &&
			weightStopAfter >= finiteWaitlist.peek().weight.value && (
				!stopCriterion.canStop(astarArg.arg, reachedFinites.map { it.argNode }) ||
				(finiteWaitlist.peek().end.reachesTarget && DI.enableOptimizations)
			)
		) {
			val edge = finiteWaitlist.remove()
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
					// TODO rephrase: do not expand target as it is probably useless for the refiner to target under a target (which subgraph will be cut)

					continue
				}
				check(!astarNode.distance.isFinite)
			}

			return edge
		}

		return null
	}

	fun distanceUntilTarget(astarNode: AstarNode<S, A>) =
		if (astarNode.argNode.isTarget) {
			depths[astarNode]!!
		} else {
			check(astarNode.distance.isFinite)
			depths[astarNode]!! + astarNode.distance.value
		}

	fun lowerBoundUntilTarget(): Distance {
		// We shouldn't need lower-bound if we know the distance (finite case)
		check(reachedFinites.isEmpty())
		// We shouldn't need lower-bound if we know the distance (infinite case)
		check(!isUnreachable)

		return if (finiteWaitlist.isEmpty) {
			unknownWaitlist.peek().weight
		} else if (unknownWaitlist.isEmpty) {
			finiteWaitlist.peek().weight
		} else {
			arrayOf(unknownWaitlist, finiteWaitlist).minOf { it.peek().weight }
		}
	}

	val isUnreachable: Boolean
		get() {
			// It is expected that [reachedFinites] is empty, otherwise we would know that this should return false
			check(reachedFinites.isEmpty())
			return finiteWaitlist.isEmpty && unknownWaitlist.isEmpty
		}

	private fun nextWeight(queue: PriorityWaitlist<Edge<S, A>>) =
		if (queue.isNotEmpty) {
			queue.peek().weight.value.toLong()
		} else {
			Long.MAX_VALUE
		}

	operator fun contains(astarNode: AstarNode<S, A>) = astarNode in depths

	fun toString(astarNode: AstarNode<S, A>): String {
		val depth = depths[astarNode]!!
		val weight = weights[astarNode]!!
		val heuristic = astarNode.heuristic
		return "G($depth) + H($heuristic) = F($weight)"
	}

	data class Edge<S: State, A: Action>(
		val end: AstarNode<S, A>,
		val depth: Int
	) {
		val weight
			get() = end.getWeight(depth)

		val heuristic
			get() = end.heuristic

		override fun toString() = "${end.argNode} G($depth) + H($heuristic) = F($weight)"
	}
}
