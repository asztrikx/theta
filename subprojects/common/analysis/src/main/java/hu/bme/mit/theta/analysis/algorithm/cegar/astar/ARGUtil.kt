package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode

// Kotlin doesn't allow bounds here
// Second parameter: distance
typealias Skip<S, A> = (ArgNode<S, A>, Int) -> Boolean
typealias NewVisits<S, A> = (Visit<S, A>) -> Collection<Visit<S, A>>
typealias Parents<S, A> = HashMap<ArgNode<S, A>, ArgNode<S, A>?>

data class Visit<S: State, A: Action>(
	val argNode: ArgNode<S, A>,
	val distance: Int,
)

/**
 * Visits nodes from start nodes with the help of [newVisitsFunc] so that each node is reached in the shortest distance.
 *
 * @receiver start nodes
 *
 * @param newVisitsFunc determines the neighbour nodes of a node and their distance.
 *
 * @param skip is called on all nodes visited which determines whether to skip processing the neighbours of a node.
 * This also should be used to receive and process the visited nodes with the parameters: node, shortest distance from any start node.
 */
inline fun <S: State, A: Action> Collection<ArgNode<S, A>>.walk(
	skip: Skip<S, A>,
	newVisitsFunc: NewVisits<S, A>,
): Parents<S, A> {
	val doneSet = hashSetOf<ArgNode<S, A>>()
	val parents: Parents<S, A> = hashMapOf()
	forEach { parents[it] = null }
	val queue = ArrayDeque(map {
		Visit(it, 0)
	})
	while (queue.isNotEmpty()) {
		check(queue.last().distance - queue.first().distance in 0..1)
		val visit = queue.removeFirst()
		val (argNode, distance) = visit

		// covering edges can point to already visited [ArgNode]s
		if (argNode in doneSet) {
			continue
		}
		doneSet += argNode

		if (skip(argNode, distance)) {
			continue
		}

		val newVisits = newVisitsFunc(visit)
		for (newVisit in newVisits) {
			if (newVisit.argNode in doneSet) {
				continue
			}
			// TODO we could also skip if a cover edge have already reached a node (needs a separate doneSet)
			if (newVisit.distance == distance) {
				// Covering edge has zero weight which would break BFS if we did not push it to a correct place.
				queue.addFirst(newVisit)
			} else {
				require(newVisit.distance == distance + 1)
				queue.addLast(newVisit)
			}
			parents[newVisit.argNode] = argNode
		}
	}
	return parents
}

inline fun <S: State, A: Action> Collection<ArgNode<S, A>>.walk(
	newVisitsFunc: NewVisits<S, A>
) = walk({_, _ -> false}, newVisitsFunc)

/**
 * Calls [walk] with a newVisitFunc that only visits children or the covering node.
 */
inline fun <S: State, A: Action> Collection<ArgNode<S, A>>.walkSubtree(skip: Skip<S, A>): Parents<S, A> {
	return walk(skip) newVisits@ { (argNode, distance) ->
		argNode.coveringNode.getOrNull()?.let {
			return@newVisits listOf(Visit(it, distance))
		}

		val succNodes = argNode.succNodes()
		// Capacity could be given here (currently it is **NOT** given)
		val newVisits = MutableList<Visit<S, A>>(succNodes.size) { i ->
			Visit(succNodes[i], distance + 1)
		}
		return@newVisits newVisits
	}
}

/**
 * Calls [walk] with a newVisitFunc that only visits parent and covered nodes.
 */
inline fun <S: State, A: Action> Collection<ArgNode<S, A>>.walkReverseSubtree(skip: Skip<S, A>): Parents<S, A> {
	return walk(skip) newVisits@ { (argNode, distance) ->
		val newVisits = mutableListOf<Visit<S, A>>()
		if (!argNode.isInit) {
			newVisits += Visit(argNode.parent()!!, distance + 1)
		}
		newVisits += argNode.coveredNodes().map { Visit(it, distance) }

		return@newVisits newVisits
	}
}

/**
 * Visits parents defined by [parentOf] until it returns null or [skip] returns true.
 * For each node the distance is given from @receiver.
 */
inline fun <S: State, A: Action> ArgNode<S, A>.walkUpParents(
	startDistance: Int,
	parentOf: (ArgNode<S, A>) -> ArgNode<S, A>?,
	skip: Skip<S, A>,
) {
	var current: ArgNode<S, A>? = this
	var distance = startDistance
	while (current != null) {
		if (skip(current, distance)) {
			break
		}
		val parent = parentOf(current)
		if (parent != null) {
			if (current.parent() === parent) {
				distance++
			} else {
				check(current.coveredNodes.anyMatch { it === parent })
			}
		}
		current = parent
	}
}
