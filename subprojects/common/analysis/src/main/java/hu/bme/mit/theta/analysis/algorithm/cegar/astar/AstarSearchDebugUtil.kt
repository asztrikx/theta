package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State

fun <S: State, A: Action, P: Prec> AstarSearch<S, A, P>.printParents(astarNode: AstarNode<S, A>) {
	var current: AstarNode<S, A>? = astarNode
	while(current != null) {
		println(current)
		current = parents[current]
	}
}