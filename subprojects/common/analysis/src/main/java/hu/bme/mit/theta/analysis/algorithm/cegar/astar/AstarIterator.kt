package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgCopier
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import java.util.function.Function
import kotlin.jvm.optionals.getOrNull

object AstarIterator {
	/**
	 * Creates a copy of [astarArg]. The original [astarArg] will be connected to this copy.
	 *
	 * provider <-- [astarArg] becomes provider <-- astarArgCopy <-- [astarArg]
	 *
	 * The provider's of [astarArg]'s nodes will be matching copied node.
	 * The distance and heuristic fields will also be set.
	 */
	@JvmStatic
	fun <S: State, A: Action, P: Prec> createIterationReplacement(
		astarArg: AstarArg<S, A, P>,
		prec: P,
		partialOrd: PartialOrd<S>,
		projection: Function<in S, *>, // TODO replace this with lambda type
		astarAbstractor: AstarAbstractor<S, A, P>
	): AstarArg<S, A, P> {
		val translation = mutableListOf<Pair<ArgNode<S, A>, ArgNode<S, A>>>()
		val newInitNodes = hashSetOf<ArgNode<S, A>>()
		val argCopy = ArgCopier.createCopy(astarArg.arg, { argNode, argNodeCopy ->
			translation += Pair(argNode, argNodeCopy)
		}, { initArgNode, initArgNodeCopy ->
			newInitNodes.add(initArgNodeCopy)
		})
		val astarArgCopy = AstarArg(argCopy, prec, partialOrd, projection, astarArg)
		astarArgCopy.provider = astarArg.provider
		astarArg.provider = astarArgCopy

		// Covering edges are created after createCopy finished
		//  TODO this is bad design
		translation.forEach { (argNode, argNodeCopy) ->
			val astarNode = astarArg.get(argNode)
			val astarNodeCopy = AstarNode(argNodeCopy, astarNode.providerAstarNode)
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
			if (argNodeCopy in newInitNodes) {
				astarArgCopy.putInit(astarNodeCopy)
			}
			astarArgCopy.reachedSet.add(astarNodeCopy.argNode)

			if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
				handleAstarDecreasing(astarNode, astarArg, astarAbstractor)
			}
		}
		check(astarArg.arg.nodes.count() == astarArgCopy.all.values.size.toLong())
		check(astarArg.arg.initNodes.count() == astarArgCopy.allInit.size.toLong())
		return astarArgCopy
	}

	private fun <S: State, A: Action, P: Prec> handleAstarDecreasing(
		astarNode: AstarNode<S, A>,
		astarArg: AstarArg<S, A, P>,
		astarAbstractor: AstarAbstractor<S, A, P>,
	) {
		// Nodes in the next iteration already have covering edges which can break the consistency requirement
		// with the decreasing method described in the abstractor.
		// therefore we remove it here so that we can have a strict check for consistency during search.
		// See XstsTest 48th, 51th testcase failing at cover() because of consistency check before covering.
		// Full and Semiondemand will always give the same heuristic for covering- and covered node as they are based on distance
		// therefore the heuristic (which is based on the distance values) will be consistent.

		val argNode = astarNode.argNode
		val parentArgNode = astarNode.argNode.parent.getOrNull()
		val parentAstarNode = astarArg.get(parentArgNode)
		parentAstarNode ?: check(argNode.isInit)
		astarAbstractor.findHeuristic(astarNode, astarArg, parentAstarNode)

		argNode.coveringNode.getOrNull()?.let {
			handleDecreasingCoverEdgeConsistency(argNode, it, astarArg)
		}
		// Avoid concurrent modification exception by copying stream to list
		argNode.coveredNodes.toList().forEach { coveredNode ->
			handleDecreasingCoverEdgeConsistency(coveredNode, argNode, astarArg)
		}
	}

	// There is no guarantee that cover edges will still be consistent TODO which tests will fail
	private fun <S: State, A: Action, P: Prec> handleDecreasingCoverEdgeConsistency(
		coveredNode: ArgNode<S, A>,
		coveringNode: ArgNode<S, A>,
		astarArg: AstarArg<S, A, P>,
	) {
		val astarCoveredNode = astarArg.get(coveredNode)
		val astarCoveringNode = astarArg.get(coveringNode)

		// Other astar node (covering or covered) may not exist when trying to handle consistency
		// It will be checked from the other end of the edge later.
		astarCoveredNode ?: return
		astarCoveringNode ?: return

		check(astarCoveredNode.heuristic.isKnown && astarCoveringNode.heuristic.isKnown)
		if (astarCoveredNode.heuristic != astarCoveringNode.heuristic) {
			coveredNode.unsetCoveringNode()
		}
	}
}
