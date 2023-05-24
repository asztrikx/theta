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
		val argCopy = ArgCopier.createCopy(astarArg.arg) { argNode, argNodeCopy ->
			translation += Pair(argNode, argNodeCopy)
		}
		val astarArgCopy = AstarArg(argCopy, prec, partialOrd, projection, astarArg)
		astarArgCopy.provider = astarArg.provider
		astarArg.provider = astarArgCopy

		// Covering edges are created after createCopy finished
		translation.forEach { (argNode, argNodeCopy) ->
			val astarNode = astarArg[argNode]
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
			astarArgCopy.reachedSet.add(astarNodeCopy)

			if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
				handleAstarDecreasing(astarNode, astarArg, astarAbstractor)
			}
		}
		check(astarArg.arg.nodes.count() == astarArgCopy.astarNodes.values.size.toLong())
		check(astarArg.arg.initNodes.count() == astarArgCopy.astarInitNodes.size.toLong())
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

		val parentArgNode = argNode.parent.getOrNull()
		val parentAstarNode = if (parentArgNode == null) {
			check(argNode.isInit)
			null
		} else astarArg[parentArgNode]

		// TODO why do we call this recursive function here ????
		astarAbstractor.findHeuristic(astarNode, astarArg, parentAstarNode)

		argNode.coveringNode.getOrNull()?.let {
			handleDecreasingCoverEdgeConsistency(argNode, it, astarArg)
		}
		// Avoid concurrent modification exception by copying stream to list
		argNode.coveredNodes().forEach { coveredNode ->
			handleDecreasingCoverEdgeConsistency(coveredNode, argNode, astarArg)
		}
	}

	// There is no guarantee that cover edges will still be consistent
	// TODO which tests would fail?
	private fun <S: State, A: Action, P: Prec> handleDecreasingCoverEdgeConsistency(
		coveredNode: ArgNode<S, A>,
		coveringNode: ArgNode<S, A>,
		astarArg: AstarArg<S, A, P>,
	) {
		// Other astar node (covering or covered) may not exist when trying to handle consistency
		// It will be checked from the other end of the edge later.
		if (coveredNode !in astarArg || coveringNode !in astarArg) {
			return
		}

		// TODO use .astarArg extension when this method is moved to util (and changed to extension method)
		val astarCoveredNode = astarArg[coveredNode]
		val astarCoveringNode = astarArg[coveringNode]
		check(astarCoveredNode.heuristic.isKnown && astarCoveringNode.heuristic.isKnown)
		if (astarCoveredNode.heuristic != astarCoveringNode.heuristic) {
			coveredNode.unsetCoveringNode()
		}
	}
}
