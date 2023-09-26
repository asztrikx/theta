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
import hu.bme.mit.theta.analysis.Analysis
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
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.distanceSetter.DistanceSetter
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.heuristicFinder.HeuristicFinder
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.analysis.prod2.prod2explpred.Prod2ExplPredAnalysis
import hu.bme.mit.theta.common.logging.Logger
import hu.bme.mit.theta.common.logging.NullLogger
import java.util.function.Function

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
	private val partialOrd: PartialOrd<S>,
	private val heuristicFinder: HeuristicFinder<S, A, P>,
	private val distanceSetter: DistanceSetter<S, A>,
): Abstractor<S, A, P> {
	private val astarFileVisualizer = AstarFileVisualizer(true, cegarHistoryStorage, logger)

	/**
	 * Determines the closest target to any of the node or determines that no node can reach target.
	 *
	 * Kotlin can't create a reference of a function which is a member and is an extension
	 */
	fun findDistanceForAny(
		startAstarNodes: Collection<AstarNode<S, A>>,
		stopCriterion: StopCriterion<S, A>,
		visualizerState: String,
		prec: P,
	) {
		var startAstarNodes = startAstarNodes
		val astarArg = startAstarNodes.first().astarArg
		val arg = astarArg.arg

		if (startAstarNodes.any { it.distance.isBounded }) {
			logger.infoLine("|  |  Skipping AstarArg: startAstarNodes already have a distance")
			return
		}

		logger.detailLine("|  |  Precision: $prec")
		logger.infoLine("|  |  Starting ARG: ${arg.nodes.count()} nodes, ${arg.incompleteNodes.count()} incomplete, ${arg.unsafeNodes.count()} unsafe")
		logger.substepLine("|  |  Starting AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
		logger.substepLine("|  |  Building ARG...")
		astarFileVisualizer.visualize("start $visualizerState", cegarHistoryStorage.indexOf(astarArg))

		if (startAstarNodes.any { it.argNode.isTarget }) {
			startAstarNodes
				.filter { it.argNode.isTarget }
				.forEach { it.distance = Distance.ZERO }
		} else {
			// expanded targets' children may not have heuristic
			startAstarNodes.forEach { heuristicFinder(it, this@AstarAbstractor) }
			startAstarNodes = startAstarNodes.filter { !it.heuristic.isInfinite }

			val search = AstarSearch(startAstarNodes, stopCriterion)
			while (true) {
				val (astarNode, depth) = search.removeFromWaitlist() ?: break
				visitNode(search, astarNode, depth, astarArg, prec)
			}
			distanceSetter(search)
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
		var astarNode = astarNode
		var argNode = astarNode.argNode

		astarNode.close(astarArg.reachedSet[astarNode], search)?.let {
			astarNode.checkConsistency(astarArg[argNode.coveringNode()!!])
			astarNode = it
			argNode = astarNode.argNode
		}
		if (argNode.isCovered) {
			val coveringAstarNode = astarArg[argNode.coveringNode()!!]

			// We are either covered into
			// - completed node (was in waitlist => has heuristic)
			// - completed node's child (in waitlist => has heuristic)
			// - leftover from prune:
			//   - decreasing: during copy we call [findHeuristic] => has heuristic
			//   - non-decreasing: we can cover into a leftover node before it is visited (even when starting from init nodes) => may not have
			if (heuristicSearchType == HeuristicSearchType.DECREASING) {
				check(coveringAstarNode.heuristic.isKnown)
			}
			heuristicFinder(coveringAstarNode, this)
			astarNode.checkConsistency(coveringAstarNode)
			// Covering edge has 0 weight therefore depth doesn't increase
			search.addToWaitlist(coveringAstarNode, astarNode, depth)
		} else if (argNode.isFeasible) {
			if (!argNode.isExpanded) {
				val newArgNodes = argBuilder.expand(argNode, prec)
				for (newArgNode in newArgNodes) {
					// TODO this should definitely be a pattern: blocking expanding older arg
					if (astarArg.provider != null && heuristicSearchType == HeuristicSearchType.SEMI_ONDEMAND) {
						astarNode.providerAstarNode!!.createChildren(prec, search)
					}
					astarArg.createSuccAstarNode(newArgNode)
				}
			}

			// go over recreated and remained nodes
			for (succArgNode in argNode.succNodes()) {
				val succAstarNode = astarArg[succArgNode]

				heuristicFinder(succAstarNode, this)
				astarNode.checkConsistency(succAstarNode)
				search.addToWaitlist(succAstarNode, astarNode, depth + 1)
			}
		}
	}

	/**
	 * [astarNode] is the provider node of a different node. This is used to creates children for [astarNode],
	 * so that different node's children will have candidate provider nodes.
	 *
	 * A node's children :=
	 * - if it is/can be expanded: its tree children
	 * - if it is/can be covered: its coverer node's children (recursive definition)
	 *
	 * [heuristicFinder] is not called during this.
	 */
	private fun AstarNode<S, A>.createChildren(prec: P, search: AstarSearch<S, A>) {
		// we could call expand on found target nodes after each search however
		// - the intention would not be as clear as calling it before [createSuccAstarNode]
		// - it could expande more nodes than we would actually need

		var astarNode = this
		var argNode = astarNode.argNode
		val astarArg = astarNode.astarArg
		if (!argNode.isTarget) {
			if (!argNode.isCovered || !argNode.coveringNode()!!.isTarget) {
				// provided astarNode was in queue =>
				// provided astarNode has heuristic =>
				// provider astarNode has distance &&
				// provider astarNode is not a target =>
				// provider astarNode must have been expanded or if covered then the coverer (if non target) must have been expanded
				check(argNode.isExpanded || argNode.coveringNode()!!.isExpanded)
				return
			}

			// target covering node
			argNode = argNode.coveringNode()!!
			astarNode = astarArg[argNode]
		}
		require(argNode.isTarget)

		if (heuristicSearchType == HeuristicSearchType.FULL) {
			require(argNode.isCovered || argNode.isExpanded)
		}

		if (argNode.isCovered) {
			// [createChildren] is already called (directly or indirectly) on this node
			return
		}

		// [createChildren] can be already called on this node through a different edge
		while(!argNode.isExpanded) {
			// optimization (leq uses smt solver): target node can only be covered with a target node
			astarNode.close(astarArg.reachedSet[astarNode].filter { it.argNode.isTarget }, search)?.let {}
			if (argNode.coveringNode() != null) {
				argNode = argNode.coveringNode()!!
				astarNode = astarArg[argNode]
				check(argNode.isTarget)
				check(!argNode.isCovered)
				continue
			}
			argBuilder.expand(argNode, prec).forEach {
				astarArg.createSuccAstarNode(it)
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
			astarArg = cegarHistoryStorage.last.first
			cegarHistoryStorage.setLast(astarArg, prec)
			astarArg.pruneApply()
		}

		// initialize: prune can keep initialized state
		if (!arg.isInitialized) {
			logger.substep("|  |  (Re)initializing ARG...")
			argBuilder.init(arg, prec).forEach {
				astarArg.createSuccAstarNode(it)
			}
			logger.substepLine("done")
		}

		// If we start from incomplete nodes then we have to know their shortest depth from an init node.
		// Unexpanded child might have shorter distance from a covered node.
		findDistanceForAny(astarArg.astarInitNodes.values, initialStopCriterion, "init", prec)

		// prec is not modified but copied after prune by the CegarChecker
		val astarArgCopy = createIterationReplacement(astarArg, partialOrd, projection, this)
		cegarHistoryStorage.setLast(astarArgCopy, prec)
		cegarHistoryStorage.add(astarArg, prec)

		return if (arg.isSafe) {
			// Arg may not be complete (== fully expanded) as INFINITE heuristic can avoid expanding some nodes.
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
		// Only used for assertions // TODO make this true and move this out to somewhere else
		lateinit var heuristicSearchType: HeuristicSearchType
		fun <S: State, A: Action, P: Prec> builder(argBuilder: ArgBuilder<S, A, P>) = Builder(argBuilder)
	}

	/*@Override
    public String toString() {
        return Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString();
    }*/
	override fun toString() = "Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString()"

	class Builder<S: State, A: Action, P: Prec>(private val argBuilder: ArgBuilder<S, A, P>) {
		private lateinit var analysis: Analysis<S, A, P>
		private var projection: (S) -> Any = { 0 }
		private var stopCriterion = StopCriterions.firstCex<S, A>()
		private var logger: Logger = NullLogger.getInstance()
		private lateinit var cegarHistoryStorage: CegarHistoryStorage<S, A, P>
		private lateinit var partialOrd: PartialOrd<S>

		// Only used to warn about bad PartialOrder
		fun analysis(prod2Analysis: Analysis<S, A, P>) = apply {
			require(prod2Analysis !is Prod2ExplPredAnalysis) // TODO 8th failing case, use decreasing heuristic for this when findProviderAstarNode fails
			this.analysis = prod2Analysis
		}

		fun projection(projection: Function<in S, *>) = apply { this.projection = { s -> projection.apply(s) } }

		fun stopCriterion(stopCriterion: StopCriterion<S, A>) = apply {
			if (heuristicSearchType == HeuristicSearchType.FULL) {
				require(stopCriterion is FullExploration<S, A>)
			}
			if (heuristicSearchType != HeuristicSearchType.FULL) {
				// Currently weightSupremumXYZ is a single value not a list so it is unsupported. Also findDistanceFor**Any**
				// If we are looking for n targets then it is possible that we reached [1,n) target
				require(stopCriterion !is AtLeastNCexs<S, A>)
			}
			this.stopCriterion = stopCriterion
		}

		fun logger(logger: Logger) = apply { this.logger = logger }

		fun cegarHistoryStorage(cegarHistoryStorage: CegarHistoryStorage<S, A, P>) = apply {
			if (heuristicSearchType == HeuristicSearchType.FULL || heuristicSearchType == HeuristicSearchType.DECREASING) {
				require(cegarHistoryStorage is CegarHistoryStoragePrevious<S, A, P>)
			}
			this.cegarHistoryStorage = cegarHistoryStorage
		}

		fun partialOrder(partialOrd: PartialOrd<S>) = apply { this.partialOrd = partialOrd }

		fun build() = AstarAbstractor(argBuilder, projection, stopCriterion, logger, cegarHistoryStorage, partialOrd)
	}
}
