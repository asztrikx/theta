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
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.cegarhistorystorage.CegarHistoryStoragePrevious
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
			astarNode.checkConsistency(astarArg[argNode.coveringNode()!!])
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
					astarArg.createSuccAstarNode(newArgNode, argBuilder, prec)
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

	// TODO document: arg must be the same reference in every call
	override fun check(arg: ARG<S, A>, prec: P): AbstractorResult {
		require(arg.targetNodes().isEmpty())

		val astarArg: AstarArg<S, A>
		if (cegarHistoryStorage.size == 0) {
			astarArg = AstarArg(arg, partialOrd, projection, null)
			cegarHistoryStorage.add(astarArg, prec)
		} else {
			astarArg = cegarHistoryStorage.last.first
			// update Prec
			cegarHistoryStorage.setLast(astarArg, prec)
			astarArg.pruneApply()
		}

		// initialize: prune can keep initialized state
		if (!arg.isInitialized) {
			logger.substep("|  |  (Re)initializing ARG...")
			argBuilder.init(arg, prec).forEach {
				astarArg.createSuccAstarNode(it, argBuilder, prec)
				// TODO later (currently there is only one init node): check if they can't cover each other as it is used // check if it even used anywhere
			}
			logger.substepLine("done")
		}

		findDistanceForAny(astarArg.astarInitNodes.values, initialStopCriterion, "init", prec)

		val astarArgCopy = createIterationReplacement(astarArg, partialOrd, projection, heuristicFinder, this)
		cegarHistoryStorage.setLast(astarArgCopy, prec)

		// old Prec
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
		// Only used for assertions
		lateinit var heuristicSearchType: HeuristicSearchType
		var analysisBadLeq = false
		fun <S: State, A: Action, P: Prec> builder(argBuilder: ArgBuilder<S, A, P>) = Builder(argBuilder)
	}

	/*@Override
    public String toString() {
        return Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString();
    }*/
	override fun toString() = "Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString()"

	class Builder<S: State, A: Action, P: Prec>(private val argBuilder: ArgBuilder<S, A, P>) {
		private var analysisSet = false
		private var projection: (S) -> Any = { 0 }
		private var stopCriterion = StopCriterions.firstCex<S, A>()
		private var logger: Logger = NullLogger.getInstance()
		private lateinit var cegarHistoryStorage: CegarHistoryStorage<S, A, P>
		private lateinit var partialOrd: PartialOrd<S>

		fun analysis(analysis: Analysis<S, A, P>) = apply {
			analysisBadLeq = analysis is Prod2ExplPredAnalysis
			analysisSet = true
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

		fun build(): AstarAbstractor<S, A, P> {
			require(analysisSet)
			return AstarAbstractor(argBuilder, projection, stopCriterion, logger, cegarHistoryStorage, partialOrd)
		}
	}
}
