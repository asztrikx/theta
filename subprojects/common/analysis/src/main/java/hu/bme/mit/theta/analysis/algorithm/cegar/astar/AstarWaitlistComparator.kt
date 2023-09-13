package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge

class AstarWaitlistComparator<S: State, A: Action> : Comparator<Edge<S, A>> {
	override fun compare(edge1: Edge<S, A>, edge2: Edge<S, A>): Int {
		val weightComparator = compareBy { it: Edge<S, A> ->
			it.end.getWeight(it.depthFromAStartNode).value
		}

		/*
		TODO https://photos.app.goo.gl/wguQ7K9opyLqTUPa7
		Monotonicity
		Indirect proof
		- target selection

		dokumentálás: cserébe 0-ásakat nem coverelhetjük: okoz e ez gondot (ha target a kövi nem, ha más heurisztikájú akkor csak +1 csúcs és a kövit biztosan lehet coverelni, ha több 0-ás van egymás után akkor árthat csak)
		*/
		val earlyExitComparator = predicateOrderedComparator<Edge<S,A>>(
			{ it.end.argNode.isCovered && it.end.heuristic == Distance.ZERO },
			{ !it.end.argNode.isCovered && it.end.heuristic == Distance.ONE },
			{ !it.end.argNode.isCovered && it.end.heuristic == Distance.ZERO },
		)

		return (weightComparator then earlyExitComparator).compare(edge1, edge2)
	}
}
