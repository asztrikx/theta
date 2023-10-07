package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgCopier
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler.AstarNodeCopyHandler
import kotlin.jvm.optionals.getOrNull

// TODO make this receive the whole list which checked to be in order, also update documentation
/**
 * Propagate finite distance up from a node ([from]) until any node in a set ([until]) is reached.
 * **If all target in an ARG is known the [setDistanceFromAllTargets] should be used for setting all distances**
 *
 * @param until As we are not always searching from the root it's important to only set distances for nodes involved in the search.
 * @param parents should map from a node to its parent.
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromFiniteDistance(
	from: AstarNode<S, A>,
	until: Set<ArgNode<S, A>>,
	parents: Map<AstarNode<S, A>, AstarNode<S, A>?>,
) {
	require(from.distance.isFinite)

	val conditionalNodes = mutableListOf<ArgNode<S, A>>()

	from.argNode.walkUpParents(from.distance.value, { parents[this@propagateUpDistanceFromFiniteDistance[this]]?.argNode }) { argNode, distance ->
		val astarNode = argNode.astarNode

		// We expand targets therefore we can have a target ancestor.
		// This function should have been called on that target by this point even in full expand or n-cex => it should have distance.
		if (argNode.isTarget && from.argNode !== argNode) {
			check(astarNode.distance.isKnown)
		}

		if (astarNode.distance.isFinite) {
			return@walkUpParents if (from.argNode === argNode) {
				// We start from a known distance
				false
			} else {
				// Multiple targets can be visited during a check => we should not overwrite shorter distances from those targets
				check(astarNode.distance.value <= distance)
				true
			}
		}

		check(astarNode.distance.isUnknown)

		astarNode.distance = Distance.finiteOf(distance)

		if (argNode !== from.argNode && !argNode.isCovered) {
			check(distance == argNode.minKnownSuccDistance!!.value + 1)
		}

		// Save covered nodes
		val parentNode = parents[astarNode]?.argNode
		val nonParentCoveredNodes = argNode.coveredNodes()
			.filter { it !== parentNode }
		conditionalNodes += nonParentCoveredNodes
		check(nonParentCoveredNodes.all { it.astarNode.distance.isUnknown })

		// Parent is a covered argNode
		if (!argNode.isInit && argNode.parent()!! !== parentNode) {
			// Tree parent's other children may have already reached a target with a covered argNode parent.
			// But then tree parent's distance hasn't been set and won't be as there isn't any children left.
			conditionalNodes += argNode.parent()!!
			// [argNode]'s tree parent can already have distance
		}
		return@walkUpParents argNode in until
	}

	propagateUpDistanceFromConditionalNodes(conditionalNodes)
}

/**
 * Propagate finite distance up (also through covering edges) until we find a node with a child with unknown distance.
 * Distance value is derived from the child with minimum distance value.
 *
 * [nodes] may not only be from a covered node but also from non-covered nodes (with > 0 children)
 *
 * @param nodes nodes with distance will be filtered out
 */
private fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromConditionalNodes(nodes: Collection<ArgNode<S, A>>) {
	// Filtering out is just a probably useless optimization
	// (Filtering out in the caller can cause the following edge case of distance changing if filtering too early:)
	// If we had checked for unknown distances in the walkup process
	// then in the following case distance could be set in the meantime:
	// parents: c's is b, b's is a
	//      a
	//    /  \
	//   b- ->c
	val queue = ArrayDeque(nodes.filter { it.astarNode.distance.isUnknown }) // TODO later: do we surely want to filter this and not throw an exception?
	while (queue.isNotEmpty()) {
		val startNode = queue.removeFirst()

		var previousDistance: Distance? = null
		startNode.walkUpParents(0, { parent.getOrNull() }) { node, _ ->
			val astarNode = node.astarNode

			if (node.isTarget) {
				check(astarNode.distance.isKnown)
				return@walkUpParents true
			} else if (astarNode.distance.isKnown) {
				check(astarNode.distance.isFinite)
				if (node !== startNode) {
					if (previousDistance!!.isInfinite) {
						check(astarNode.distance <= Distance.INFINITE)
					} else {
						check(astarNode.distance <= previousDistance!! + 1)
					}
				}
				return@walkUpParents true
			}

			if (node === startNode && node.isCovered) {
				// Copy known distance
				astarNode.distance = node.coveringNode()!!.astarNode.distance
			} else if (node === startNode && node.isLeaf) {
				// This must be infinite distance propagation
				astarNode.distance = Distance.INFINITE
			} else {
				// [node] === [startNode] can hold if [propagateUpDistanceFromKnownDistance]'s parents gone through a covering edge
				// node may not be expanded whether its the [startNode] or not

				if (!node.allSuccDistanceKnown) {
					return@walkUpParents true
				}

				// If there are more than 1 child then all children could have been part of a conditional path
				// now processed and have distance. In this case this conditional path has to determine a distance
				// for the node and must not stop propagating up the distance.
				// This can be the case even if propagating up infinite distance.
				val minSuccDistance = node.minSuccDistance!!
				astarNode.distance = if (minSuccDistance.isInfinite) {
					Distance.INFINITE
				} else {
					minSuccDistance + 1
				}
			}
			previousDistance = astarNode.distance

			node.coveredNodes().forEach {
				check(it.astarNode.distance.isUnknown)
			}
			queue += node.coveredNodes()
			return@walkUpParents false
		}
	}
}

/**
 * Propagate infinite distance down (also through covering edges **in both directions**). Use if startNodes's
 *
 * [nodes] may not only be from a covered node but also from non-covered nodes (with > 0 children)
 *
 * @param nodes nodes with distance will be filtered out
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateDownDistanceFromInfiniteDistance(nodes: Collection<ArgNode<S, A>>) {
	val conditionalNodes = mutableListOf<ArgNode<S, A>>()

	val queue = ArrayDeque(nodes)
	while (!queue.isEmpty()) {
		val node = queue.removeFirst()
		listOf(node).walkSubtree { argNode, _ ->
			val astarNode = argNode.astarNode

			check(!astarNode.distance.isFinite)
			// Unexpanded regions shouldn't exist unless its known it can't reach any target
			if (!argNode.isExpanded && !argNode.isCovered) {
				check(astarNode.heuristic.isInfinite)
			}

			// Some covered nodes may already have distance
			conditionalNodes += argNode.coveredNodes()

			astarNode.distance = Distance.INFINITE
			return@walkSubtree false
		}
	}

	propagateUpDistanceFromConditionalNodes(conditionalNodes)
}

/**
 * Set (*some*) nodes which can't possibly reach target infinite.
 * **This should only be called when search reached a target.**
 * **If all target in an ARG is known the [setDistanceFromAllTargets] should be used for setting all distances**
 *
 * When searching for a target we might explore nodes which provably can't reach target.
 * We need to mark them infinite otherwise we may revisit those nodes. **This function doesn't handle all scenarios.**
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromInfiniteDistance() {
	/*
	e.g. with BFS
		T := target
		L := leaf & non-target
		search from: 1) a 2) b
		 a
		| \
		-  b
		| | \
		- -  -
		| | | \
		- - L  L
		| |
		T T
	*/

	/*
	Cases for nodes which doesn't have a distance => possibly have infinite distance
	- covered
		- coverer marked as infinite (from previous iteration) 1)
		- coverer not marked as infinite
			- covering edge is part of a circle 2) not handled: seems too complex
				- covered to ancestor
				- covered to non-ancestor
			- covering edge is not part of a circle 3) it will be handled if coverer is marked as infinite here
	- not covered
		- expanded
			- leaf (non leaves will be added by their leaves as they must also be infinite) 4)
			- not leaf 5) it will be handled if all children is marked as infinite here
				- has infinite heuristic
				- has non-infinite heuristic
		- not expanded
			- has infinite heuristic 6)
			- has non-infinite heuristic 7) not handled: we can't determine without expanding
	*/
	val conditionalNodes = mutableListOf<ArgNode<S, A>>()
	val excludeKnownDistance = { node: ArgNode<S, A> -> node.astarNode.distance.isUnknown }
	val excludeTarget = { node: ArgNode<S, A> -> !node.isTarget }

	// 1)
	val lateCoveredNodes = arg.coveredNodes().filter { coveredNode ->
		val covererNode = coveredNode.coveringNode()!!
		val astarCovererNode = covererNode.astarNode
		return@filter astarCovererNode.distance.isInfinite
	}
	conditionalNodes += lateCoveredNodes.filter(excludeKnownDistance)

	// 4)
	conditionalNodes += arg.expandedLeafNodes().filter(excludeKnownDistance and excludeTarget)

	// 6)
	val infiniteHeuristicNodes = astarNodes.values
		.filter { it.heuristic.isInfinite }
		.filter {
			// TODO future: if we stop copying infinite subgraph (which also makes less covering available..) (ArgCopy doesn't know about heuristics) then check(!argNode.isExpanded) (and remove filter for it)
			if (it.argNode.isExpanded) {
				// Must have been part of the copied infinite subgraph
				check(
					it.providerAstarNode != null &&
					it.argNode.toString() == it.providerAstarNode!!.argNode.toString() &&
					it.providerAstarNode!!.argNode.isExpanded
				)
			}
			true
		}
		.map { it.argNode }
		.filter { !it.isCovered && !it.isExpanded }
	conditionalNodes += infiniteHeuristicNodes.filter(excludeKnownDistance)

	// Check if previous logic is implemented correctly
	check(conditionalNodes.size == conditionalNodes.toSet().size)

	propagateUpDistanceFromConditionalNodes(conditionalNodes)
}

/**
 * Set **all type of distances** from all targets. **It must only be used if all targets in an ARG is given in [targets].**
 *
 * Non-target nodes with known distances are not handled (no use case currently).
 */
fun <S: State, A: Action> AstarArg<S, A>.setDistanceFromAllTargets(targets: Collection<ArgNode<S, A>>) {
	check(targets.all { it.isTarget })

	// Set finite distances
	targets.walkReverseSubtree skip@ { argNode, distance ->
		// An ancestor can be a target because targets are expanded
		if (argNode.astarNode.distance.isKnown) {
			return@skip true
		}

		argNode.astarNode.distance = Distance.finiteOf(distance)
		return@skip false
	}

	// Set infinite distances
	astarNodes.values
		.filter { !it.distance.isKnown }
		.forEach { it.distance = Distance.INFINITE }

	check(astarNodes.values.all { it.distance.isKnown })

	// [checkShortestDistance] also would do this no need to call
}

// TODO this only works for targets
fun <S: State, A: Action> AstarArg<S, A>.checkShortestDistance(finites: Collection<AstarNode<S, A>>) {
	finites.map { it.argNode }.walkReverseSubtree skip@ { argNode, distance ->
		val astarNode = argNode.astarNode

		// An ancestor can be a target because targets are expanded
		if (distance != 0 && argNode.isTarget) {
			return@skip true
		}

		// Because [AstarArg.propagateUpDistanceFromInfiniteDistance] doesn't handle all cases there might be distance not set.
		if (astarNode.distance.isFinite) {
			check(astarNode.distance.value == distance)
		}
		return@skip false
	}
}


/**
 * Creates a copy of [astarArg]. The original [astarArg] will be connected to this copy.
 *
 * provider <-- [astarArg] becomes provider <-- astarArgCopy <-- [astarArg]
 *
 * The provider's of [astarArg]'s nodes will be matching copied node.
 * The distance and heuristic fields will also be set.
 */
fun <S: State, A: Action, P: Prec> AstarArg<S, A>.createIterationReplacement(
	partialOrd: PartialOrd<S>,
	projection: (S) -> Any,
	astarNodeCopyHandler: AstarNodeCopyHandler<S, A, P>,
	astarAbstractor: AstarAbstractor<S, A, P>,
): AstarArg<S, A> {
	val translation = mutableListOf<Pair<ArgNode<S, A>, ArgNode<S, A>>>()
	val argCopy = ArgCopier.createCopy(arg) { argNode, argNodeCopy ->
		translation += Pair(argNode, argNodeCopy)
	}
	val astarArgCopy = AstarArg(argCopy, partialOrd, projection, provider)
	provider = astarArgCopy

	// Covering edges are created after createCopy finished
	translation.forEach { (argNode, argNodeCopy) ->
		val astarNode = argNode.astarNode

		val astarNodeCopy = AstarNode(argNodeCopy, astarNode.providerAstarNode, astarArgCopy)
		// Heuristic has to be set first otherwise admissibility check fails
		if (astarNode.heuristic.isKnown) {
			astarNodeCopy.heuristic = astarNode.heuristic

			// If it has a distance then is must also have a heuristic
			if (astarNode.distance.isKnown) {
				astarNodeCopy.distance = astarNode.distance
			}
		} else {
			check(astarNode.distance.isUnknown)
		}
		astarArgCopy.put(astarNodeCopy)
		astarArgCopy.reachedSet.add(astarNodeCopy)

		astarNode.providerAstarNode = astarNodeCopy
		astarNode.reset()
		astarNodeCopyHandler(astarNode, astarAbstractor)
	}
	check(arg.nodes().size == astarArgCopy.astarNodes.values.size)
	check(arg.initNodes().size == astarArgCopy.astarInitNodes.size)
	return astarArgCopy
}

