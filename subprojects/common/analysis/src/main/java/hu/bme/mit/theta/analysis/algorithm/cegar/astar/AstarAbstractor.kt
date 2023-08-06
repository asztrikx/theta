/*
 *  Copyright 2017 Budapest University of Technology and Economics
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.PartialOrd
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder
import hu.bme.mit.theta.analysis.algorithm.cegar.Abstractor
import hu.bme.mit.theta.analysis.algorithm.cegar.AbstractorResult
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions.AtLeastNCexs
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions.FullExploration
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarIterator.createIterationReplacement
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.CegarHistoryStoragePrevious
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.common.logging.Logger
import hu.bme.mit.theta.common.logging.NullLogger
import java.util.function.Function
import kotlin.math.max

/**
 * Astar implementation for [Abstractor], relying on an [ArgBuilder].
 *
 * @param initialStopCriterion When we are exploring an arg first time we use the StopCriterion provided in constructor
 */
class AstarAbstractor<S: State, A: Action, P: Prec> private constructor(
	private val argBuilder: ArgBuilder<S, A, P>,
	private val projection: (S) -> Any,
	private val initialStopCriterion: StopCriterion<S, A>,
	private val logger: Logger,
	private val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
	// Used for debugging here
	private val partialOrd: PartialOrd<S>,
) : Abstractor<S, A, P> {
	private val astarFileVisualizer = AstarFileVisualizer(true, cegarHistoryStorage, logger)

	init {
		if (heuristicSearchType == HeuristicSearchType.FULL) {
			require(initialStopCriterion is FullExploration<S, A>)
		}
		if (heuristicSearchType != HeuristicSearchType.FULL) {
			// "multitarget"
			// If we are looking for n targets then it is possible that we reached [1,n) target
			require(initialStopCriterion !is AtLeastNCexs<S, A>)
		}
		if (heuristicSearchType == HeuristicSearchType.FULL || heuristicSearchType == HeuristicSearchType.DECREASING) {
			require(cegarHistoryStorage is CegarHistoryStoragePrevious<S, A, P>)
		}
	}

	private fun Collection<AstarNode<S, A>>.findDistanceForAny(
		stopCriterion: StopCriterion<S, A>,
		visualizerState: String,
		prec: P,
	) {
		var startAstarNodes = this
		//if (stopCriterion.canStop(arg))
		if (startAstarNodes.any { it.distance.isBounded }) {
			logger.infoLine("|  |  Skipping AstarArg: startAstarNodes already have a distance")
			return
		}

		val astarArg = startAstarNodes.first().astarArg
		val arg = astarArg.arg
		logger.detailLine("|  |  Precision: $prec")
		logger.infoLine("|  |  Starting ARG: ${arg.nodes.count()} nodes, ${arg.incompleteNodes.count()} incomplete, ${arg.unsafeNodes.count()} unsafe")
		logger.substepLine("|  |  Starting AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
		logger.substepLine("|  |  Building ARG...")

		astarFileVisualizer.visualize("start $visualizerState", cegarHistoryStorage.indexOf(astarArg))

		val search = AstarSearch<S, A>()

		// expanded targets' children may not have heuristic
		startAstarNodes.forEach {
			findHeuristic(it)
			search.addToWaitlist(it, null, 0)
		}
		startAstarNodes = startAstarNodes.filter { !it.heuristic.isInfinite }

		// Non-target nodes the exact values must be from a previous findDistanceForAny call as we set exact distances at the end of iteration.
		// May contain same node multiple times: normal node covers into target already visited
		val reachedExacts = ArrayDeque<AstarNode<S, A>>()

		while (!search.isWaitlistEmpty) {
			val (astarNode, depth) = search.removeFromWaitlist() ?: break

			// TODO move to search class?
			// Currently, this only happens when in an older iteration.
			if (astarNode.getWeight(depth).value >= (search.weightSupremumValue ?: Int.MAX_VALUE)) {
				// Otherwise we might miss shorter weightSupremumValue overwritten before first weightSupremumValue process
				check(reachedExacts.size == 0)
				reachedExacts += search.weightSupremumAstarNode!!
				search.weightSupremumValue = null
				require(stopCriterion.canStop(astarArg.arg, listOf(astarNode.argNode)))
				break
			}

			if (astarNode.argNode.isTarget) {
				if (astarNode.distance.isUnknown) {
					reachedExacts += astarNode
					if (stopCriterion.canStop(astarArg.arg, listOf(astarNode.argNode))) {
						break
					}
				}

				if (heuristicSearchType != HeuristicSearchType.FULL) {
					expandTarget(astarNode, prec)
					astarNode.distance = Distance.ZERO
					continue
				}
			}

			visitNode(search, astarNode, depth, astarArg, prec)
		}

		// If we can't reach a depth greater than [weightSupremumValue] then other target is not reachable.
		if (search.isWaitlistEmpty && search.weightSupremumValue != null) {
			check(heuristicSearchType !== HeuristicSearchType.FULL)
			check(reachedExacts.size == 0)
			reachedExacts += search.weightSupremumAstarNode!!
		}

		// [reachedExacts] are expected to be ordered by depth
		check(reachedExacts.asSequence().zipWithNext { a, b -> search.minDepths[a]!! <= search.minDepths[b]!! }.all { it })

		if (heuristicSearchType === HeuristicSearchType.FULL) {
			astarArg.setDistanceFromAllTargets(reachedExacts.map { it.argNode })
		} else {
			val startNodes = startAstarNodes.map { it.argNode }
			reachedExacts.apply {
				forEach { astarArg.propagateUpDistanceFromKnownDistance(it, startNodes.toSet(), search.parents) }
				clear()
			}

			if (startAstarNodes.none { it.distance.isBounded }) {
				astarArg.propagateDownDistanceFromInfiniteDistance(startNodes)
			} else {
				astarArg.propagateUpDistanceFromInfiniteDistance()
			}
			astarArg.checkShortestDistance()
		}
		check(startAstarNodes.any { it.distance.isBounded } || startAstarNodes.all { it.distance.isInfinite })

		astarFileVisualizer.visualize("end $visualizerState", cegarHistoryStorage.indexOf(astarArg))

		logger.substepLine("done")
		logger.infoLine("|  |  Finished ARG: ${arg.nodes.count()} nodes, ${arg.incompleteNodes.count()} incomplete, ${arg.unsafeNodes.count()} unsafe")
		logger.infoLine("|  |  Finished AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
	}

	private fun visitNode(
		search: AstarSearch<S, A>,
		astarNode: AstarNode<S, A>,
		depth: Int,
		astarArg: AstarArg<S, A>,
		prec: P,
	) {
		val argNode = astarNode.argNode

		// After prune node may have children but not fully expanded (isExpanded false).
		// If node has no children it still can already be expanded, therefore expanded is already set (should not be covered).
		// If node already has covering node, close cloud still choose another one, therefore avoid.
		if (!argNode.isExpanded && argNode.isLeaf && !argNode.isCovered) {
			astarNode.close(astarArg.reachedSet[astarNode])
		}
		if (argNode.isCovered) {
			val coveringNode = argNode.coveringNode()!!
			val coveringAstarNode = astarArg[coveringNode]

			// We are either covered into
			// - completed node (was in waitlist => has heuristic)
			// - completed node's child (in waitlist => has heuristic)
			// - leftover from prune (after copy we apply decreasing for all nodes => has heuristic)
			if (heuristicSearchType == HeuristicSearchType.DECREASING) {
				check(coveringAstarNode.heuristic.isKnown)
			}

			// If astarNode's parent is also covered then covering edges have been redirected. (see ArgNode::cover)
			// We have to update parents map according to that.
			//  1) a - - -> b (argNode,coveredAstarNode)
			//  2) a - - -> b - - -> c (coveringNode)
			//  3) a        b - - -> c
			//	   |                 ^
			//     | - - - - - - - - |
			val parentArgNode = search.parents[astarNode.argNode]
			val parentAstarNode = parentArgNode?.let { astarArg[it] }
			val coveredAstarNode = if (parentArgNode != null && parentArgNode.isCovered && parentArgNode.coveringNode()!! === argNode) {
				// Because argNode is covered it can only reach coveringNode with the same distance as it's new parent
				// therefore we can safely remove it
				search.parents.remove(argNode)

				// Update to new parent if we this node is the current parent
				// as coveringAstarNode may already have a better parent or already in doneSet
				parentAstarNode
			} else {
				astarNode
			}

			// New cover edge's consistency (b -> c).
			// If rewiring happened we don't need to check the rewired edge (a -> c) for consistency
			// as it is distributive property for this case.
			findHeuristic(coveringAstarNode)
			astarNode.checkConsistency(coveringAstarNode)
			// Covering edge has 0 weight therefore depth doesn't increase
			search.addToWaitlist(coveringAstarNode, coveredAstarNode, depth)
			// Covering node is not a new node therefore it's already in reachedSet
			return
		}

		if (!argNode.isFeasible) {
			return
		}

		if (heuristicSearchType != HeuristicSearchType.FULL) {
			check(!argNode.isTarget)
		}
		check(!argNode.isCovered)

		// expand: create nodes
		if (!argNode.isExpanded) {
			val newArgNodes = argBuilder.expand(argNode, prec)
			for (newArgNode in newArgNodes) {
				astarArg.createSuccAstarNode(newArgNode) // TODO why wasn't it here?
			}
		}

		// go over recreated and remained nodes
		for (succArgNode in argNode.succNodes()) {
			val succAstarNode = astarArg[succArgNode]

			// already existing succAstarNode, e.g:
			// 		although we only add leaves to startAstarNodes and findHeuristic is called upon them
			//		they may get covered with a non leaf which has succAstarNode (because of how copy works)
			//		but its provider doesn't have distance, therefore there is no heuristic
			findHeuristic(succAstarNode) // maybe don't do this if one of child is target?
			astarNode.checkConsistency(succAstarNode)
			search.addToWaitlist(succAstarNode, astarNode, depth + 1)
		}
	}

	// Expands the target (future provider node) so that the children of the provided node can also have provider nodes to choose from.
	// Target should not already be expanded.
	// This could be merged into normal loop, but it may make it harder to read
	// TODO say that findHeuristic isn't called
	// TODO say this avoids adding to waitlist
	private fun expandTarget(astarNode: AstarNode<S, A>, prec: P) {
		check(astarNode.argNode.isTarget)
		var astarNode = astarNode
		val astarArg = astarNode.astarArg
		var argNode = astarNode.argNode
		// astarNode can be a coverer node for another target, therefore it can already be expanded (directly or indirectly)
		if (argNode.isExpanded || argNode.isCovered) {
			return
		}
		// TODO ask this
		assert(argNode.isLeaf)
		do {
			// Can be already expanded
			//assert(!argNode.isExpanded)
			//assert(argNode.isLeaf)
			assert(argNode.isCovered)
			astarNode.close(astarArg.reachedSet[astarNode])
			if (argNode.coveringNode() == null) {
				//// We can get covered into already expanded node
				//// Covering target may have been after astarNode in waitlist therefore it may not already be expanded
				argBuilder.expand(argNode, prec)

				// expand: create astar nodes
				argNode.succNodes.forEach {
					assert(!astarArg.contains(it))
					astarArg.createSuccAstarNode(it)
				}

				// Children can be either target or not.
				// Do not set distance for target as they will be filtered out when adding them to waitlist
				// therefore they won't be expanded.
				break
			}

			// Covered node's children are the covering node's children, therefore we have to expand the covering node
			argNode = argNode.coveringNode()!!
			astarNode = astarArg[argNode]
			assert(argNode.isTarget)
			// optimization: we know the distance for a target node
			astarNode.distance = Distance.ZERO
		} while (!argNode.isExpanded) // We can cover into an already expanded target (it can't be covered, see close())
	}

	/**
	 * Calculates the heuristic for @receiver. Depending on HeuristicSearchType this may be recursive.
	 *
	 * @param astarNode should already have providerNode if not in the first arg
	 */
	fun findHeuristic(astarNode: AstarNode<S, A>) {
		// TODO strategy pattern maybe here?

		val astarArg = astarNode.astarArg
		val parentAstarNode = astarNode.argNode.parent()?.let { astarArg[it] }

		// Do not return (?set distance to?) EXACT(0) when node is target as that would not create the side effect of expanding the node

		if (astarNode.heuristic.isKnown) {
			return
		}

		if (astarArg.provider == null) {
			// Only lower bound we can say to satisfy a*
			astarNode.heuristic = Distance.ZERO
			return
		}
		val providerAstarArg = astarArg.provider!!

		check(heuristicSearchType != HeuristicSearchType.FULL)

		// We don't have heuristic from provider therefore we decrease parent's
		// astarArg.provider == null case could also be handled by this
		if (heuristicSearchType == HeuristicSearchType.DECREASING) {
			// init node as we are always starting from startNodes
			if (parentAstarNode == null) {
				astarNode.heuristic = Distance.ZERO
				return
			}
			check(parentAstarNode.argNode.coveringNode() == null)
			check(parentAstarNode.heuristic.isKnown)
			var parentHeuristicValue = parentAstarNode.heuristic.value
			parentHeuristicValue = max(parentHeuristicValue - 1, 0)
			astarNode.heuristic = Distance.boundedOf(parentHeuristicValue)
			return
		}

		// Provider AstarNode can be null
		check(astarNode.providerAstarNode != null)
		val providerAstarNode = astarNode.providerAstarNode!!
		if (providerAstarNode.heuristic.isInfinite) {
			check(providerAstarNode.distance.isInfinite)
		}

		val (_, prec) = cegarHistoryStorage.find(providerAstarArg)

		// Visualize current
		astarFileVisualizer.visualize("paused ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Paused AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")

		// get the heuristic with findDistance in parent arg
		listOf(providerAstarNode).findDistanceForAny(AstarDistanceKnown(providerAstarNode), "${providerAstarNode.argNode}", prec)
		check(astarNode.heuristic.isKnown)

		// Visualize current (redundant)
		astarFileVisualizer.visualize("resumed ${astarNode.argNode}", cegarHistoryStorage.indexOf(astarArg))
		logger.substepLine("|  |  Resumed AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
	}

	private fun AstarNode<S, A>.close(candidates: Collection<AstarNode<S, A>>) {
		require(!argNode.isCovered)
		require(!argNode.isExpanded)
		for (astarCandidate in candidates) {
			val candidate = astarCandidate.argNode
			if (!candidate.mayCover(argNode)) {
				continue
			}

			// TODO why?
			check(!heuristic.isInfinite)

			if (heuristic <= astarCandidate.heuristic) {
				// breakpoint here (removeFromWaitlist reaching target again with cover edge)
				if (!argNode.isTarget && candidate.isTarget) { println("test") }

				argNode.cover(candidate)
				return
			}
		}
	}

	// uses previous AstarArg then calls checkFromNode with root=null
	// it is assumed that last AstarArg in astarArgStore should be used if exists
	override fun check(arg: ARG<S, A>, prec: P): AbstractorResult {
		val astarArg: AstarArg<S, A>
		if (cegarHistoryStorage.size == 0) {
			astarArg = AstarArg(arg, partialOrd, projection, null)
			cegarHistoryStorage.add(astarArg, prec)
		} else {
			// TODO use parameter arg and create astarArg here (means we have to create connections here)?
			val pair = cegarHistoryStorage.last
			astarArg = pair.first
			// prec is not modified but copied after prune by the CegarChecker
			cegarHistoryStorage.setLast(astarArg, prec)
			// prune was only applied to arg by the CegarChecker
			astarArg.pruneApply()
		}

		// initialize: prune can keep initialized state
		if (!arg.isInitialized) {
			logger.substep("|  |  (Re)initializing ARG...")
			argBuilder.init(arg, prec).forEach { newNode ->
				check(!newNode.isCovered)
				astarArg.createSuccAstarNode(newNode)
			}
			logger.substepLine("done")
			check(arg.isInitialized)
		}

		// If we start from incomplete nodes then we have to know their shortest depth from an init node.
		// Unexpanded child might have shorter distance from a covered node.
		astarArg.astarInitNodes.values.findDistanceForAny(initialStopCriterion, "", prec)

		// AstarArg has to be copied as arg will be modified after refinement
		val astarArgCopy = createIterationReplacement(astarArg, partialOrd, projection, this)
		cegarHistoryStorage.setLast(astarArgCopy, prec)

		// prec will be overwritten
		cegarHistoryStorage.add(astarArg, prec)

		// found and isSafe is different: e.g. full expand
		return if (arg.isSafe) {
			// Arg may not be expanded as we can get INFINITE heuristic avoiding expansion
			// require(arg.isComplete) { "Returning incomplete ARG as safe" };
			AbstractorResult.safe()
		} else {
			AbstractorResult.unsafe()
		}
	}

	override fun createArg(): ARG<S, A> = argBuilder.createArg()

	enum class HeuristicSearchType {
		FULL,
		SEMI_ONDEMAND,
		DECREASING,
	}

	companion object {
		// TODO: make it non singleton
		lateinit var heuristicSearchType: HeuristicSearchType
		fun <S: State, A: Action, P: Prec> builder(argBuilder: ArgBuilder<S, A, P>) = Builder(argBuilder)
	}

	/*@Override
    public String toString() {
        return Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString();
    }*/
	override fun toString() = "Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString()"

	class Builder<S: State, A: Action, P: Prec>(private val argBuilder: ArgBuilder<S, A, P>) {
		private var projection: (S) -> Any = { 0 }
		private var stopCriterion = StopCriterions.firstCex<S, A>()
		private var logger: Logger = NullLogger.getInstance()
		private lateinit var cegarHistory: CegarHistoryStorage<S, A, P>
		private lateinit var partialOrd: PartialOrd<S>

		fun projection(projection: Function<in S, *>) = apply { this.projection = { s -> projection.apply(s) } }

		//fun projection(projection: (S) -> Any) = apply { this.projection = projection }

		fun stopCriterion(stopCriterion: StopCriterion<S, A>) = apply { this.stopCriterion = stopCriterion }

		fun logger(logger: Logger) = apply { this.logger = logger }

		fun cegarHistoryStorage(cegarHistoryStorage: CegarHistoryStorage<S, A, P>) = apply { this.cegarHistory = cegarHistoryStorage }

		fun partialOrder(partialOrd: PartialOrd<S>) = apply { this.partialOrd = partialOrd }

		fun build() = AstarAbstractor(argBuilder, projection, stopCriterion, logger, cegarHistory, partialOrd)
	}
}
