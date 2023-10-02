package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgCopier
import hu.bme.mit.theta.analysis.algorithm.ArgNode

object AstarIterator {
	/**
	 * Creates a copy of [astarArg]. The original [astarArg] will be connected to this copy.
	 *
	 * provider <-- [astarArg] becomes provider <-- astarArgCopy <-- [astarArg]
	 *
	 * The provider's of [astarArg]'s nodes will be matching copied node.
	 * The distance and heuristic fields will also be set.
	 */
	fun <S: State, A: Action, P: Prec> createIterationReplacement(
		astarArg: AstarArg<S, A>,
		partialOrd: PartialOrd<S>,
		projection: (S) -> Any,
		astarAbstractor: AstarAbstractor<S, A, P>
	): AstarArg<S, A> {
		val translation = mutableListOf<Pair<ArgNode<S, A>, ArgNode<S, A>>>()
		val argCopy = ArgCopier.createCopy(astarArg.arg) { argNode, argNodeCopy ->
			translation += Pair(argNode, argNodeCopy)
		}
		val astarArgCopy = AstarArg(argCopy, partialOrd, projection, astarArg)
		astarArgCopy.provider = astarArg.provider
		astarArg.provider = astarArgCopy

		// Covering edges are created after createCopy finished
		translation.forEach { (argNode, argNodeCopy) ->
			val astarNode = astarArg[argNode]
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
			astarNode.providerAstarNode = astarNodeCopy
			astarNode.reset()
			astarArgCopy.put(astarNodeCopy)
			astarArgCopy.reachedSet.add(astarNodeCopy)

			// TODO pattern
			// Fully and Semi-ondemand will always give the same heuristic for covering- and covered node (=> consistent) as they are based on distance
			if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
				handleAstarDecreasing(astarNode, astarAbstractor)
			}
		}
		check(astarArg.arg.nodes.count() == astarArgCopy.astarNodes.values.size.toLong())
		check(astarArg.arg.initNodes.count() == astarArgCopy.astarInitNodes.size.toLong())
		return astarArgCopy
	}

	private fun <S: State, A: Action, P: Prec> handleAstarDecreasing(
		astarNode: AstarNode<S, A>,
		astarAbstractor: AstarAbstractor<S, A, P>,
	) {
		// Nodes in the next iteration already have covering edges which can break the consistency requirement with the decreasing heuristics.
		// Therefore, we remove those here so that we can check for consistency during search.
		// See XstsTest 48, 51 testcase fail without this.

		val astarArg = astarNode.astarArg
		val argNode = astarNode.argNode

		// TODO why do we call this recursive function here ????
		astarAbstractor.findHeuristic(astarNode)

		argNode.coveringNode()?.let {
			astarArg.handleDecreasingCoverEdgeConsistency(argNode, it)
		}
		// Avoid concurrent modification exception by copying stream to list
		argNode.coveredNodes().forEach { coveredNode ->
			astarArg.handleDecreasingCoverEdgeConsistency(coveredNode, argNode)
		}
	}

	// There is no guarantee that cover edges will still be consistent
	private fun <S: State, A: Action> AstarArg<S, A>.handleDecreasingCoverEdgeConsistency(
		coveredNode: ArgNode<S, A>,
		coveringNode: ArgNode<S, A>,
	) {
		// Other astar node (covering or covered) may not exist when trying to handle consistency
		// It will be checked from the other end of the edge later.
		if (coveredNode !in this || coveringNode !in this) {
			return
		}

		val astarCoveredNode = this[coveredNode]
		val astarCoveringNode = this[coveringNode]
		check(astarCoveredNode.heuristic.isKnown && astarCoveringNode.heuristic.isKnown)
		if (astarCoveredNode.heuristic > astarCoveringNode.heuristic) {
			coveredNode.unsetCoveringNode()
		}
	}
}
