package hu.bme.mit.theta.analysis.algorithm.cegar.astar.distanceSetter

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch

interface DistanceSetter<S: State, A: Action> {
	fun setInnerNodesDistances(search: AstarSearch<S, A>)
}
