package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State

fun <S: State, A: Action> AstarNode<S, A>.checkConsistency(child: AstarNode<S, A>) {
    val parent = this
    require(!parent.heuristic.isInfinite)
    if (child.heuristic.isInfinite) {
        check(parent.heuristic.isKnown) // why? only exact
        return
    }

    val heuristicDistanceValue = parent.heuristic.value - child.heuristic.value
    val edgeWeight = if (parent.argNode.isCovered) 0 else 1
    check(heuristicDistanceValue <= edgeWeight)
}
