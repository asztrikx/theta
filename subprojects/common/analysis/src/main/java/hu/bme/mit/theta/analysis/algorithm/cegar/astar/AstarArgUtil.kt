package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgCopier
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler.AstarNodeCopyHandler

/**
 * Propagate finite distance up from a node ([from]) until any node in a collection ([until]) is reached.
 * **If all target in an ARG is known the [setDistanceFromAllTargets] should be used for setting all distances**
 *
 * @param until As we are not always searching from the root it's important to only set distances for nodes involved in the search.
 * @param parents should map from a node to its parent.
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromFiniteDistance(
	from: AstarNode<S, A>,
	until: Collection<ArgNode<S, A>>,
	parents: Map<AstarNode<S, A>, AstarNode<S, A>?>,
) {
	require(DI.heuristicSearchType != HeuristicSearchType.FULL)
	require(from.distance.isFinite)

	val conditionalNodes = mutableListOf<ArgNode<S, A>>()

	from.argNode.walkUpParents(from.distance.value, { parents[this[it]]?.argNode }) { argNode, distance ->
		val astarNode = argNode.astarNode

		// In one search we don't have target as a descendant of target
		check(!(argNode.isTarget && from.argNode !== argNode))

		// Conditional covered nodes
		val parentNode = parents[astarNode]?.argNode
		val nonParentCoveredNodes = argNode.coveredNodes()
			.filter { it !== parentNode }
		check(nonParentCoveredNodes.all { it.astarNode.distance.isUnknown })
		conditionalNodes += nonParentCoveredNodes

		// Conditional graph parent
		if (!argNode.isInit && argNode.parent()!! !== parentNode) {
			// Tree parent's other children may have already reached a target with a covered argNode parent.
			// But then tree parent's distance hasn't been set and won't be as there isn't any children left.
			// [argNode]'s tree parent can already have distance.
			conditionalNodes += argNode.parent()!!
		}

		// [from] node case
		// [from] node could also have conditional nodes
		if (astarNode.distance.isFinite) {
			check(from.argNode == argNode)
			return@walkUpParents false
		}
		check(argNode !== from.argNode)
		check(astarNode.distance.isUnknown)

		// Finite distance
		astarNode.distance = Distance.finiteOf(distance)
		if (!argNode.isCovered) {
			check(argNode !== from.argNode) // might not be expanded
			check(argNode.isExpanded)
			check(!argNode.minKnownSuccDistance!!.isInfinite)
			check(distance == argNode.minKnownSuccDistance!!.value + 1)
		}

		return@walkUpParents argNode in until
	}
	checkDistanceProperty()

	propagateUpDistanceFromConditionalNodes(conditionalNodes)
}

/**
 * Propagate finite distance up (also through covering edges) until we find a node with a child with unknown distance.
 * Distance value is derived from the child with minimum distance value if all children has a known distance.
 *
 * [nodes] may not only be from a covered node but also from non-covered nodes (non leaf)
 *
 * Nodes with distance will be filtered out.
 */
private fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromConditionalNodes(nodes: Collection<ArgNode<S, A>>) {
	// Filtering out in the caller can cause the following edge case of distance change:
	// parents: c's is b, b's is a
	//      a
	//    /  \
	//   b- ->c
	val queue = ArrayDeque(nodes.filter { it.astarNode.distance.isUnknown }) // TODO later: do we surely want to filter this and not throw an exception?
	// TODO could be rewritten with [walkReverseSubtree]
	while (queue.isNotEmpty()) {
		val startNode = queue.removeFirst()

		startNode.walkUpParents(0, { it.parent() }) { argNode, _ ->
			val astarNode = argNode.astarNode

			if (argNode.isTarget) {
				//check(astarNode.distance.isKnown) // TODO not sure
				return@walkUpParents true
			} else if (astarNode.distance.isKnown) {
				check(astarNode.distance.isFinite)
				return@walkUpParents true
			}

			astarNode.distance = if (argNode === startNode && argNode.isCovered) {
				argNode.coveringNode()!!.astarNode.distance
			} else if (argNode === startNode && argNode.isLeaf) {
				check(astarNode.distance.isUnknown)
				Distance.INFINITE
			} else {
				// [argNode] === [startNode] can hold if [propagateUpDistanceFromKnownDistance]'s parents gone through a covering edge
				// [argNode] may not be expanded whether it's the [startNode] or not.
				check(!astarNode.argNode.isCovered)

				if (!argNode.allSuccDistanceKnown) {
					return@walkUpParents true
				}

				// If there are more than 1 child then all children could have been part of a conditional path
				// now processed and have distance. In this case this conditional path has to determine a distance
				// for the argNode and must not stop propagating up the distance.
				// This can be the case even if propagating up infinite distance.
				val minSuccDistance = argNode.minSuccDistance!!
				if (minSuccDistance.isInfinite) {
					Distance.INFINITE
				} else {
					minSuccDistance + 1
				}
			}

			argNode.coveredNodes().forEach {
				check(it.astarNode.distance.isUnknown)
			}
			queue += argNode.coveredNodes()
			return@walkUpParents false
		}
	}

	checkDistanceProperty()
}

/**
 * Propagate infinite distance down (also through covering edges **in both directions**).
 *
 * @param nodes doesn't need to have infinite distance set
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateDownDistanceFromInfiniteDistance(nodes: Collection<ArgNode<S, A>>) {
	val conditionalNodes = mutableListOf<ArgNode<S, A>>()

	val queue = ArrayDeque(nodes)
	while (queue.isNotEmpty()) {
		val node = queue.removeFirst()
		listOf(node).walkSubtree { argNode, _ ->
			val astarNode = argNode.astarNode

			check(!astarNode.distance.isFinite)
			// Unvisited regions shouldn't exist unless its known it can't reach any target
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
					it.argNode.id == it.providerAstarNode!!.argNode.id &&
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
 * Set **know distances** from all targets. **It must only be used if all targets in an ARG is given in [targetAstarNodes].**
 */
fun <S: State, A: Action> AstarArg<S, A>.setDistanceFromAllTargets(targetAstarNodes: List<AstarNode<S, A>>) {
	require(DI.heuristicSearchType == HeuristicSearchType.FULL)

	val targets = targetAstarNodes.map { it.argNode }
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
		.filter { it.distance.isUnknown }
		.forEach { it.distance = Distance.INFINITE }

	// [checkShortestDistance] also would do this no need to call
}

// TODO this only works for targets, have to be fixed before usage
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

	translation.forEach { (argNode, argNodeCopy) ->
		val astarNode = argNode.astarNode

		val astarNodeCopy = AstarNode(argNodeCopy, astarNode.providerAstarNode, astarArgCopy)
		// Heuristic has to be set first otherwise admissibility check fails
		if (astarNode.heuristic.isKnown) {
			astarNodeCopy.heuristic = astarNode.heuristic
		}
		// If it was a covered leftover node it can have distance without having a heuristic
		if (astarNode.distance.isKnown) {
			astarNodeCopy.distance = astarNode.distance
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

fun <S: State, A: Action> AstarArg<S, A>.checkDistanceProperty() = arg.nodes().map{ this[it] }.forEach {
	if (it.distance.isUnknown) {
		return@forEach
	}

	if (it.argNode.parent() != null) {
		val parentAstarNode = it.argNode.parent()!!.astarNode
		if (it.distance.isFinite && parentAstarNode.distance.isFinite) {
			check((parentAstarNode.distance - it.distance).value <= 1)
		} else if (parentAstarNode.distance.isKnown) {
			check(!(parentAstarNode.distance.isInfinite && it.distance.isFinite))
		}
	}

	if (it.argNode.coveringNode() != null) {
		val coveringAstarNode = it.argNode.coveringNode()!!.astarNode
		if (it.distance.isFinite && coveringAstarNode.distance.isFinite) {
			check(coveringAstarNode.distance == it.distance)
		} else if (coveringAstarNode.distance.isKnown) {
			check(coveringAstarNode.distance.isInfinite && it.distance.isInfinite)
		}
	}
}

val <S: State, A: Action> AstarArg<S, A>.isAstarComplete
	get() = arg.isInitialized && astarNodes.values.all { it.argNode.isComplete || it.heuristic == Distance.INFINITE }
