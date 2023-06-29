package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import kotlin.jvm.optionals.getOrNull

/**
 * Propagate exact distance up from a node ([from]) until any node in a set ([until]) is reached.
 * **If all target in an ARG is known the [setDistanceFromAllTargets] should be used for setting all distances**
 *
 * As we are not always searching from the root it's important to only set distances for nodes involved in the search.
 * @param parents should map from a node to its parent.
 */
fun <S: State, A: Action> AstarArg<S, A>.propagateUpDistanceFromKnownDistance(
	from: AstarNode<S, A>,
	until: Set<ArgNode<S, A>>,
	parents: Map<ArgNode<S, A>, ArgNode<S, A>?>,
) {
	require(from.distance.isKnown)

	val conditionalNodes = mutableListOf<ArgNode<S, A>>()

	from.argNode.walkUpParents(from.distance.value, parents::get) { node, distance -> // TODO relativeDistance is 2 when coveringnode is also 2
		val astarNode = get(node)

		// We expand targets therefore we can have a target ancestor.
		// This function should have been called on that target by this point even in full expand or n-cex => it should have distance.
		if (node.isTarget && from.argNode !== node) {
			check(astarNode.distance.isKnown)
		}

		if (astarNode.distance.isBounded) {
			return@walkUpParents if (from.argNode === node) {
				// We start from a known distance
				false
			} else {
				// Multiple targets can be visited during a check => we should not overwrite shorter distances from those targets
				check(astarNode.distance.value <= distance)
				true
			}
		}

		check(astarNode.distance.isUnknown)

		astarNode.distance = Distance(Distance.Type.BOUNDED, distance)

		// if true it may be a target without known succ distance or a non-target covering node with distance
		if (node !== from.argNode && parents[node] === node.coveringNode.getOrNull()) {
			check(distance == node.minKnownSuccDistance!!.value + 1)
		}

		// Save covered nodes
		val parentNode = parents[node]
		val nonParentCoveredNodes = node.coveredNodes()
			.filter { it !== parentNode }
		conditionalNodes += nonParentCoveredNodes

		// Parent is a covered node
		if (!node.isInit && node.parent()!! !== parentNode) {
			// Graph parent's other children may have already reached a target with a covered node parent.
			// But then graph parent's distance hasn't been set and won't be as there isn't any children left.
			conditionalNodes += node.parent()!!
		}
		return@walkUpParents until.contains(node)
	}

	propagateUpDistanceFromConditionalNodes(conditionalNodes)
}

// TODO infinitre ugyanolyan jól működik-e még: esetleg korábbi verzió printelje ki mennit talált illetve a mostani is (elég gyorsan kiderül így)
/**
 * Propagate exact distance up (also through covering edges) until we find a node with a child with unknown distance.
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
	val queue = ArrayDeque(nodes.filter { get(it).distance.isUnknown })
	while (!queue.isEmpty()) {
		val startNode = queue.removeFirst()

		var previousDistance: Distance? = null
		startNode.walkUpParents(0, { parent.getOrNull() }) { node, _ ->
			val astarNode = get(node)

			if (node.isTarget) {
				check(astarNode.distance.isKnown) // TODO comment to not remove
				return@walkUpParents true
			} else if (astarNode.distance.isKnown) {
				check(astarNode.distance.isBounded)
				if (node !== startNode) {
					if (previousDistance!!.isInfinite) {
						check(astarNode.distance <= previousDistance!!)
					} else {
						check(astarNode.distance <= Distance(Distance.Type.BOUNDED, previousDistance!!.value + 1))
					}
				}
				return@walkUpParents true
			}

			if (node === startNode && node.isCovered) {
				// Copy known distance
				astarNode.distance = get(node.coveringNode()!!).distance
			} else if (node === startNode && node.isLeaf) {
				// This must be infinite distance propagation
				astarNode.distance = Distance(Distance.Type.INFINITE)
			} else {
				// node === startNode can hold if propagateUpDistanceFromKnownDistance's parents gone through a covering edge

				if (!node.allSuccDistanceKnown) {
					return@walkUpParents true
				}

				// If there are more than 1 child then all children could have been part of a conditional path
				// now processed and have distance. In this case this conditional path has to determine a distance
				// for the node and must not stop propagating up the distance.
				// This can be the case even if propagating up infinite distance.
				val minSuccDistance = node.minSuccDistance!!
				astarNode.distance = if (minSuccDistance.isInfinite) {
					Distance(Distance.Type.INFINITE)
				} else {
					Distance(Distance.Type.BOUNDED, minSuccDistance.value + 1)
				}
			}
			previousDistance = astarNode.distance

			node.coveredNodes().forEach {
				check(get(it).distance.isUnknown)
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
			val astarNode = get(argNode)

			check(!astarNode.distance.isBounded)
			// Unexpanded regions shouldn't exist unless its known it can't reach any target
			if (!argNode.isExpanded && !argNode.isCovered) {
				check(astarNode.heuristic.isInfinite)
			}

			conditionalNodes += argNode.coveredNodes()

			astarNode.distance = Distance(Distance.Type.INFINITE)
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
	Cases for nodes which doesn't have a distance => possibly have infinite distance *which could have been set during search*
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
	val excludeKnownDistance = { node: ArgNode<S, A> -> get(node).distance.isUnknown }
	val excludeTarget = { node: ArgNode<S, A> -> !node.isTarget }

	// 1)
	val lateCoveredNodes = arg.coveredNodes().filter { coveredNode ->
		val covererNode = coveredNode.coveringNode()!!
		val astarCovererNode = get(covererNode)
		return@filter astarCovererNode.distance.isInfinite
	}
	conditionalNodes += lateCoveredNodes.filter(excludeKnownDistance) // TODO can this be a target?

	// 4)
	conditionalNodes += arg.expandedLeafNodes().filter(excludeKnownDistance and excludeTarget)

	// 6)
	val infiniteHeuristicNodes = astarNodes.values
		.filter { it.heuristic.isInfinite }
		.filter { astarNode ->
			val argNode = astarNode.argNode

			// TODO if we stop copying infinite subgraph then swap if branches and move .map above this
			if (false) {
				check(!argNode.isExpanded)
			} else {
				if (argNode.isExpanded) {
					check( // TODO recheck
						astarNode.providerAstarNode != null &&
						argNode.toString() == astarNode.providerAstarNode!!.argNode.toString() &&
						astarNode.providerAstarNode!!.argNode.isExpanded
					)
				}
			}

			return@filter !argNode.isCovered && !argNode.isExpanded
		}
		.map { it.argNode }
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

	// Set exact distances
	targets.walkReverseSubtree skip@ { argNode, distance ->
		// An ancestor can be a target because targets are expanded
		// TODO targets can't have a distance when function is called
		if (argNode.astarNode.distance.isKnown) {
			return@skip true // TODO check why old coded didn't failed
		}

		argNode.astarNode.distance = Distance(Distance.Type.BOUNDED, distance)
		return@skip false
	}

	// Set infinite distances
	astarNodes.values
		.filter { !it.distance.isKnown }
		.forEach { it.distance = Distance(Distance.Type.INFINITE) }

	check(astarNodes.values.all { it.distance.isKnown })

	// [checkShortestDistance] also would do this no need to call
}

fun <S: State, A: Action> AstarArg<S, A>.checkShortestDistance() {
	arg.targetNodes().walkReverseSubtree skip@ { argNode, distance ->
		val astarNode = argNode.astarNode

		// An ancestor can be a target because targets are expanded
		// TODO check why old coded didn't failed
		if (distance != 0 && argNode.isTarget) {
			return@skip true
		}

		check(astarNode.distance.isBounded) // TODO this can fail
		check(astarNode.distance.value == distance)
		return@skip false
	}
}
