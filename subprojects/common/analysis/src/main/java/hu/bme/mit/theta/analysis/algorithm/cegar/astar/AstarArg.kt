package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.reachedset.Partition
import java.util.function.Function
import kotlin.jvm.optionals.getOrNull

class AstarArg<S: State, A: Action>(
	val arg: ARG<S, A>,
	private val partialOrd: PartialOrd<S>,
	projection: Function<in S, *>,
	var provider: AstarArg<S, A>?
) {
	// Contains init nodes as well
	var astarNodes = hashMapOf<ArgNode<S, A>, AstarNode<S, A>>()
		private set
	val astarInitNodes
		get() = astarNodes.filter { it.key.isInit }

	// Covering ArgNode is searched from here
	val reachedSet: Partition<AstarNode<S, A>, Any> = Partition.of { projection.apply(it.argNode.state) }

	/**
	 * After underlying ARG is pruned we must remove AstarNodes corresponding to pruned ArgNodes.
	 *
	 * Calling this function could be avoided by creating another (computed) property besides [astarNodes] but
	 * that would leave pruned nodes in [astarNodes] which is not ideal for memory usage.
	 */
	fun pruneApply() {
		// Collect remained nodes
		reachedSet.clear()
		val astarNodesNew = hashMapOf<ArgNode<S, A>, AstarNode<S, A>>()
		arg.nodes().forEach { argNode ->
			val astarNode = this[argNode]

			astarNodesNew[argNode] = astarNode
			reachedSet.add(astarNode)
		}
		astarNodes = astarNodesNew
	}

	fun createSuccAstarNode(argNode: ArgNode<S, A>): AstarNode<S, A> {
		val providerAstarNode = getProviderAstarNode(argNode)
		val astarNode = AstarNode(argNode, providerAstarNode, this)
		reachedSet.add(astarNode)
		put(astarNode)
		return astarNode
	}

	/**
	 * @param parentAstarNode can be null when [argNode] is an init node
	 */
	private fun getProviderAstarNode(argNode: ArgNode<S, A>): AstarNode<S, A>? {
		val provider = provider
		provider ?: return null
		require(!provider.contains(argNode))

		var providerCandidates = getProviderCandidates(argNode)
		providerCandidates ?: return null

		// filter based on partialOrd.isLeq == "<=" == subset of
		providerCandidates = providerCandidates.filter { partialOrd.isLeq(argNode.state, it.state) }

		// If we knew all nodes heuristics then we would choose the largest one as it is the most precise lower bound
		if (providerCandidates.any { provider[it].distance.isKnown }) {
			providerCandidates = providerCandidates
				.filter { provider[it].distance.isKnown }
				.sortedBy { provider[it].distance }
		}

		val providerNode = providerCandidates.firstOrNull()
		if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
			if (providerNode == null) {
				// this fails for test 48,51,61 on Xsts
				check(!argNode.isInit)
				return null
			}
		} else {
			check(providerNode != null)
		}
		return provider[providerNode]
	}

	private fun getProviderCandidates(argNode: ArgNode<S, A>): List<ArgNode<S, A>>? {
		val provider = provider!!
		val parentAstarNode = argNode.parent()?.astarNode
		if (parentAstarNode == null) {
			require(argNode.isInit)
			return provider.astarInitNodes.keys.toList()
		}

		var parentAstarNodeProvider = parentAstarNode.providerAstarNode ?: run {
			// If parent doesn't have provider then we also won't have.
			check(AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING)

			// If we reach a state more general later than a parent provider's child state then it won't get covered.
			// Therefore, there can be a candidate anywhere in the astar arg, so we should return null.
			return provider.arg.nodes()
		}

		// parentAstarNodeProvider can be covered.
		// Because a covered node's children are the covering node's children we must follow the chain.
		parentAstarNodeProvider.argNode.coveringNode.getOrNull()?.let { parentNodeCoveringNodeProvider ->
			// The chain can only contain 1 covering edge because they are compressed in an ARG.

			// TODO check this
			// Even if we checked provider node before setting it whether it is covered, later it still can be covered if it's not yet expanded.
			// There is no guarantee that covering node has been expanded or has distance, we can only know it for the covered node
			//	 The only case when this can happen is when covered node is a target otherwise
			//	 we continue the search in covering node which would make it expanded would also receive a distance.
			//	 That case is handled by expanded target when found.

			// Visualization optimization: set parent's provider to the covering node
			parentAstarNode.providerAstarNode = provider[parentNodeCoveringNodeProvider]

			parentAstarNodeProvider = parentAstarNode.providerAstarNode!!
		}

		// parentAstarNode had to be in waitlist or is a leftover node
		// => it's heuristic is known
		// => (if not decreasing) it's provider's distance is known
		// => must be expanded or covered (even if it's a target)
		// => (because of previous compression) covering node can't have covering node. // TODO surely?
		if (AstarAbstractor.heuristicSearchType != AstarAbstractor.HeuristicSearchType.DECREASING) {
			check(parentAstarNodeProvider.argNode.isExpanded)
		}

		return parentAstarNodeProvider.argNode.succNodes()
	}

	/// ArgNode extension which depend on an AstarArg
	// (these are only used in AstarArgUtil, but there is no double extension methods, so they have to be here)

	/**
	 * Node must not be covered.
	 * If not excluded it returns true.
	 */
	val ArgNode<S, A>.allSuccDistanceKnown: Boolean
		get() {
			require(!isCovered)
			if (!isExpanded) {
				return false
			}
			return succNodes.allMatch { it.astarNode.distance.isKnown }
		}

	/**
	 * Minimum distance if node is not a leaf.
	 *
	 * All children must have known distance.
	 */
	val ArgNode<S, A>.minSuccDistance: Distance?
		get() {
			require(allSuccDistanceKnown)
			return minKnownSuccDistance
		}

	/**
	 * Minimum distance amongst known distances if node is not a leaf.
	 */
	val ArgNode<S, A>.minKnownSuccDistance: Distance?
		get() {
			return succNodes()
				.map { it.astarNode.distance }
				.filter(Distance::isKnown)
				.minOrNull()
		}

	val ArgNode<S, A>.astarNode: AstarNode<S, A>
		get() = get(this)

	/// Helper, Wrapper methods for map

	fun put(astarNode: AstarNode<S, A>) {
		astarNodes[astarNode.argNode] = astarNode
	}

	operator fun contains(argNode: ArgNode<S, A>) = astarNodes.containsKey(argNode)

	operator fun get(argNode: ArgNode<S, A>) = astarNodes[argNode]!!
}
