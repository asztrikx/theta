package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.reachedset.Partition
import kotlin.jvm.optionals.getOrNull

class AstarArg<S: State, A: Action>(
	val arg: ARG<S, A>,
	private val partialOrd: PartialOrd<S>,
	projection: (S) -> Any,
	var provider: AstarArg<S, A>?
) {
	// Contains init nodes as well
	var astarNodes = hashMapOf<ArgNode<S, A>, AstarNode<S, A>>()
		private set
	val astarInitNodes
		get() = astarNodes.filter { it.key.isInit }

	// Covering ArgNode is searched from here
	val reachedSet: Partition<AstarNode<S, A>, *> = Partition.of { projection(it.argNode.state) }

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

	fun <P: Prec> createSuccAstarNode(argNode: ArgNode<S, A>, argBuilder: ArgBuilder<S, A, P>, prec: P?): AstarNode<S, A> {
		val providerAstarNode = argNode.getProviderAstarNode(argBuilder, prec)
		val astarNode = AstarNode(argNode, providerAstarNode, this)
		reachedSet.add(astarNode)
		put(astarNode)
		return astarNode
	}

	private fun <P: Prec> ArgNode<S, A>.getProviderAstarNode(argBuilder: ArgBuilder<S, A, P>, prec: P?): AstarNode<S, A>? {
		val providerArg = provider ?: return null
		var providerCandidates = this.getProviderCandidates(argBuilder, prec!!) ?: return null

		providerCandidates = providerCandidates.filter { partialOrd.isLeq(state, it.state) }

		val providerNode = if (providerCandidates.any { providerArg[it].distance.isKnown }) {
			// TODO when can this happen (see git history maybe it has been deleted) // e.g. provider is target&init

			providerCandidates
				.filter { providerArg[it].distance.isKnown }
				// Largest one is the most precise lower bound
				.maxByOrNull { providerArg[it].distance }!!
		} else {
			providerCandidates.firstOrNull()
		}

		if (DI.heuristicSearchType == HeuristicSearchType.DECREASING) {
			if (isInit && providerNode == null) {
				// Xsts test case 48, 51, 61
				check(DI.analysisBadLeq)

				// Heuristic will be 0 if providerNode is kept null, so we don't have to fail
			}
		} else {
			if (providerNode == null) {
				check(DI.analysisBadLeq)

				// In this case we could extend the candidates to all nodes, *maybe* for one the analysis works correctly.
				// We probably could use decreasing heuristic in this case if the previous also fails.
				check(false)
			}
		}

		providerNode ?: return null
		return providerArg[providerNode]
	}

	private fun <P: Prec> ArgNode<S, A>.getProviderCandidates(argBuilder: ArgBuilder<S, A, P>, prec: P): Collection<ArgNode<S, A>>? {
		val provider = provider!!
		val treeParentAstarNode = parent()?.astarNode ?: run {
			require(isInit)
			return provider.astarInitNodes.keys
		}
		var treeParentAstarNodeProvider = treeParentAstarNode.providerAstarNode ?: run {
			// If [treeParentAstarNode] doesn't have provider then we also won't have.
			check(DI.heuristicSearchType == HeuristicSearchType.DECREASING)
			return provider.arg.nodes()
		}

		// Make sure [treeParentAstarNodeProvider] has children
		// TODO pattern
		if (DI.heuristicSearchType == HeuristicSearchType.SEMI_ONDEMAND) {
			// Recursive call
			treeParentAstarNodeProvider.createChildren(prec, null, argBuilder)
		}

		// [treeParentAstarNodeProvider] can be covered.
		// Because a covered node's children are the covering node's children we must follow the chain.
		treeParentAstarNodeProvider.argNode.coveringNode.getOrNull()?.let { treeParentNodeCoveringNodeProvider ->
			// The chain can only contain 1 covering edge because they are compressed in an ARG.

			// Visualization optimization: set parent's provider to the covering node
			treeParentAstarNode.providerAstarNode = provider[treeParentNodeCoveringNodeProvider]
			treeParentAstarNodeProvider = treeParentAstarNode.providerAstarNode!!
		}

		if (DI.heuristicSearchType != HeuristicSearchType.DECREASING) {
			// [treeParentAstarNode] was in queue or is a leftover node =>
			// [treeParentAstarNode] has heuristic =>
			// (if not decreasing) [treeParentAstarNode]'s provider has distance &&
			// (if createChildren is called and if not decreasing) [treeParentAstarNode]'s provider must be expanded or covered =>
			// (because of compression covering node can't have covering node and covering case has been handled) => [treeParentAstarNode] expanded
			check(treeParentAstarNodeProvider.argNode.isExpanded)
		}

		// TODO small proof: why covered's parents aren't good (probably doesn't have that children >=) https://photos.app.goo.gl/tn1T91rQdVdbZxTH9
		return if (treeParentAstarNodeProvider.argNode.isExpanded) {
			treeParentAstarNodeProvider.argNode.succNodes()
		} else {
			null
		}
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
			require(!isCovered)
			require(allSuccDistanceKnown)
			return minKnownSuccDistance
		}

	/**
	 * Minimum distance amongst known distances if node is not a leaf.
	 */
	val ArgNode<S, A>.minKnownSuccDistance: Distance?
		get() {
			require(!isCovered)
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
