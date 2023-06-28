package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import kotlin.jvm.optionals.getOrNull

// Kotlin doesn't allow bounds here
typealias Skip<S, A> = (ArgNode<S, A>, Int) -> Boolean
typealias NewVisit<S, A> = (Visit<S, A>) -> Collection<Visit<S, A>>

// Pair would create .first and .second properties which would be hard to read
class Visit<S: State, A: Action>(
	val argNode: ArgNode<S, A>,
	val distance: Int,
) {
	operator fun component1() = argNode
	operator fun component2() = distance
}

/**
 * Calls [skip] on all nodes visited with [newVisitsFunc].
 *
 * If [skip] returns true on a node `n` then the result of [newVisitsFunc]`(n)` will not be processed.
 * [skip] also should be used to process nodes, it receives the node and the distance calculated by [newVisitsFunc].
 */
fun <S: State, A: Action> Collection<ArgNode<S, A>>.walk(
	skip: Skip<S, A>,
	newVisitsFunc: NewVisit<S, A>
) {
	val doneSet = hashSetOf<ArgNode<S, A>>()
	val queue = ArrayDeque(this.map {
		Visit(it, 0)
	})
	while (!queue.isEmpty()) {
		val (argNode, distance) = queue.removeFirst()

		// covering edges can point to already visited ArgNode
		if (doneSet.contains(argNode)) {
			continue
		}
		doneSet += argNode

		if (skip(argNode, distance)) {
			continue
		}

		val newVisits = newVisitsFunc(Visit(argNode, distance))
		for (newVisit in newVisits) {
			if (newVisit.argNode in doneSet) {
				continue
			}
			if (newVisit.distance == distance) {
				// Covering edge has zero weight which would break BFS if we did not push it to a correct place.
				// The front is always a correct place.
				queue.addFirst(newVisit)
			} else {
				check(newVisit.distance == distance + 1)
				queue.addLast(newVisit)
			}
		}
	}
}

fun <S: State, A: Action> Collection<ArgNode<S, A>>.walk(
	newVisitsFunc: NewVisit<S, A>
) {
	walk({_, _ -> false}, newVisitsFunc)
}

/**
 * Calls [walk] with a newVisitFunc that only visits children or the coveringNode.
 *
 * The difference between this and [ArgNode::getNodes] is that this gives the distances for each node from the [startNodes]
 */
fun <S: State, A: Action> Collection<ArgNode<S, A>>.walkSubtree(skip: Skip<S, A>) {
	walk(skip) newVisits@ { (argNode, distance) ->
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
fun <S: State, A: Action> Collection<ArgNode<S, A>>.walkReverseSubtree(skip: Skip<S, A>) {
	walk(skip) newVisits@ { (argNode, distance) ->
		val newVisits = mutableListOf<Visit<S, A>>()
		if (!argNode.isInit) {
			newVisits += Visit(argNode.parent()!!, distance + 1)
		}
		// Currently capacity can't be given in kotlin: argNode.coveredNodes().size + 1
		newVisits += argNode.coveredNodes().map { Visit(it, distance) }

		return@newVisits newVisits
	}
}

/**
 * Visits parents defined by [walkUpParent] until it returns null or [skip] returns true.
 */
fun <S: State, A: Action> ArgNode<S, A>.walkUpParents(
	startDistance: Int,
	walkUpParent: ArgNode<S, A>.() -> ArgNode<S, A>?,
	skip: Skip<S, A>,
) {
	var current: ArgNode<S, A>? = this
	var distance = startDistance
	while (current != null) {
		if (skip(current, distance)) {
			break
		}
		val parent = current.walkUpParent()
		if (parent != null) {
			if (current.parent.getOrNull() === parent) {
				distance++
			} else {
				check(current.coveredNodes.anyMatch { it === parent })
			}
		}
		current = parent
	}
}
