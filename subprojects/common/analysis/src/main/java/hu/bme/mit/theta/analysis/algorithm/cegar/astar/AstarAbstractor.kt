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
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions.FullExploration
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.Strategy
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStoragePrevious
import hu.bme.mit.theta.analysis.prod2.Prod2Analysis
import hu.bme.mit.theta.analysis.prod2.prod2explpred.Prod2ExplPredAnalysis
import hu.bme.mit.theta.common.Utils
import java.util.function.Function

/**
 * Astar implementation for [Abstractor], relying on an [ArgBuilder].
 *
 * @param initialStopCriterion Used when exploring an arg for the first time
 */
class AstarAbstractor<S: State, A: Action, P: Prec> private constructor(
	private val argBuilder: ArgBuilder<S, A, P>,
	private val projection: (S) -> Any,
	private val initialStopCriterion: StopCriterion<S, A>,
	private val partialOrd: PartialOrd<S>,
	private val strategy: Strategy<S, A, P>,
): Abstractor<S, A, P> {
	private val cegarHistoryStorage = strategy.cegarHistoryStorage
	private val heuristicFinder = strategy.heuristicFinder
	private val distanceSetter = strategy.distanceSetter
	private val astarNodeCopyHandler = strategy.astarNodeCopyHandler
	private val astarFileVisualizer = strategy.astarFileVisualizer
	private val logger = DI.logger
	init {
		astarFileVisualizer.enabled = false
	}

	/**
	 * Determines the closest target to any of the node or determines that no node can reach target.
	 *
	 * Kotlin can't create a reference of a function which is a member and is an extension
	 */
	fun findDistanceForAny(
		startAstarNodes: List<AstarNode<S, A>>,
		stopCriterion: StopCriterion<S, A>,
		visualizerState: String,
		prec: P,
	) {
		var startAstarNodes = startAstarNodes
		val astarArg = startAstarNodes.first().astarArg
		val arg = astarArg.arg
//		val initAstarNodes = arg.initNodes().map { astarArg[it] }

		if (startAstarNodes.any { it.distance.isFinite }) {
			logger.infoLine("|  |  Skipping AstarArg: startAstarNodes already have a distance")
			return
		}

//		if (startAstarNodes == initAstarNodes) {
//			ArgCexCheckHandler.instance.setCurrentArg<P>(AbstractArg(arg, prec))
//		}
		logger.detailLine("|  |  Precision: $prec")
		logger.infoLine("|  |  Starting ARG: ${arg.nodes.count()} nodes, ${arg.incompleteNodes.count()} incomplete, ${arg.unsafeNodes.count()} unsafe")
		logger.substepLine("|  |  Starting AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")
		logger.substepLine("|  |  Building ARG...")

		val search = AstarSearch(startAstarNodes, stopCriterion, heuristicFinder, this)
		astarFileVisualizer.visualize("start $visualizerState", astarArg, search)
		while (true) {
			val (astarNode, depth) = search.removeFromWaitlist() ?: break
			visitNode(search, astarNode, depth, astarArg, prec)
//			if (startAstarNodes == initAstarNodes) {
//				ArgCexCheckHandler.instance.setCurrentArg<P>(AbstractArg(arg, prec))
//			}
		}

		distanceSetter(search)
		astarArg.checkDistanceProperty()

		if (DI.disableOptimizations) {
			search.reachedFinites.forEach {
				it.createChildren(search, argBuilder, heuristicFinder, this, cegarHistoryStorage)
			}
		}

		astarFileVisualizer.visualize("end $visualizerState", astarArg, search)
		logger.substepLine("done")
		logger.infoLine("|  |  Finished ARG: ${arg.nodes.count()} nodes, ${arg.incompleteNodes.count()} incomplete, ${arg.unsafeNodes.count()} unsafe")
		logger.infoLine("|  |  Finished AstarArg: ${astarFileVisualizer.getTitle("", cegarHistoryStorage.indexOf(astarArg))}")

		check(startAstarNodes.any { it.distance.isFinite } || startAstarNodes.all { it.distance.isInfinite })
		// check(arg.unsafeNodes().map { astarArg[it] }.all { it.distance.isKnown })  // this hits however it works without failing here
	}

	private fun visitNode(
		search: AstarSearch<S, A, P>,
		astarNode: AstarNode<S, A>,
		depth: Int,
		astarArg: AstarArg<S, A>,
		prec: P,
	) {
		var astarNode = astarNode
		var argNode = astarNode.argNode

		if (DI.heuristicSearchType == HeuristicSearchType.DECREASING) {
			if (argNode.isCovered) {
				val astarCoveringNode = astarArg[argNode.coveringNode()!!]
				if (astarCoveringNode.heuristic < astarNode.heuristic || (DI.disableOptimizations && astarCoveringNode.heuristic != astarNode.heuristic)) {
					check(argNode.isLeftover)
					argNode.unsetCoveringNode()
				}
			}
		}

		astarNode.close(search, heuristicFinder, this)?.let {
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
			if (DI.heuristicSearchType == HeuristicSearchType.DECREASING) {
				check(coveringAstarNode.heuristic.isKnown)
			}

			// Covering edge has 0 weight therefore depth doesn't increase
			search.addToWaitlist(coveringAstarNode, astarNode, depth)
		} else if (argNode.isFeasible) {
			if (!argNode.isExpanded) {
				// There can be leftover nodes => newArgNodes != succNodes
				val newArgNodes = argBuilder.expand(argNode, prec)
				for (newArgNode in newArgNodes) {
					astarArg.createSuccAstarNode(newArgNode, argBuilder, cegarHistoryStorage, heuristicFinder, this)
				}
			}

			// TODO optimization: do not call findHeuristic on all nodes if one of them is a target and we can stop
			val succAstarNodes = argNode.succNodes().map { astarArg[it] }
			for (succAstarNode in succAstarNodes) {
				search.addToWaitlist(succAstarNode, astarNode, depth + 1)
			}
		}
	}

	private var nextAstarArg: AstarArg<S, A>? = null

	/**
	 * @param arg Can still contain a decreased amount of target nodes
	 */
	override fun check(arg: ARG<S, A>, prec: P): AbstractorResult {
		nextAstarArg?.let { require(it.arg === arg)}

		val astarArg = if (cegarHistoryStorage.size == 0) {
			AstarArg(arg, partialOrd, projection, null)
		} else {
			nextAstarArg!!.apply {
				pruneApply()
			}
		}
		cegarHistoryStorage.add(astarArg, prec)

//		val metrics = Metrics()
//		if (logger !is NullLogger) {
//			metrics.iteration = cegarHistoryStorage.indexOf(astarArg)
//			metrics.leftoverNodes = arg.nodes().size
//			metrics.leftoverCoverings = arg.nodes().filter { it.isCovered }.size
//		}

		// initialize: prune can keep initialized state
		if (!arg.isInitialized) {
			logger.substep("|  |  (Re)initializing ARG...")
			argBuilder.init(arg, prec).forEach {
				astarArg.createSuccAstarNode(it, argBuilder, cegarHistoryStorage, heuristicFinder, this)
			}
			logger.substepLine("done")
		}

		findDistanceForAny(astarArg.astarInitNodes.values.toList(), initialStopCriterion, "init", prec)

//		if (logger !is NullLogger) {
//			metrics.cover = arg.nodes().filter { it.isCovered }.size
//			metrics.cover -= metrics.leftoverCoverings
//			metrics.expand = arg.nodes().size + 1
//			metrics.expand -= metrics.leftoverNodes + 1
//			metrics.distance = astarArg.astarInitNodes.values.filter { it.distance.isKnown }.maxOf { it.distance }
//			metrics.targets = astarArg.astarNodes.filter { it.key.isTarget }.size
//			metrics.targetWithKnownDistance = astarArg.astarNodes.filter { it.key.isTarget && it.value.distance.isKnown }.size
//			metrics.infiniteDistances = astarArg.astarNodes.filter { it.value.distance.isInfinite }.size
//			metrics.finiteDistances = astarArg.astarNodes.filter { it.value.distance.isFinite }.size
//			logger.mainstep(metrics.toString()) // TODO line
//		}

		val astarArgCopy = astarArg.createIterationReplacement(partialOrd, projection, astarNodeCopyHandler, this)
		cegarHistoryStorage.setLast(astarArgCopy, prec)

		nextAstarArg = astarArg

		return if (arg.isSafe) {
			require(astarArg.isAstarComplete) { "Returning incomplete ARG as safe" };
			AbstractorResult.safe()
		} else {
			AbstractorResult.unsafe()
		}
	}

	override fun createArg(): ARG<S, A> = argBuilder.createArg()

	companion object {
		fun <S: State, A: Action, P: Prec> builder(argBuilder: ArgBuilder<S, A, P>) = Builder(argBuilder)
	}

	override fun toString() = Utils.lispStringBuilder(this::class.simpleName).toString()

	class Builder<S: State, A: Action, P: Prec>(private val argBuilder: ArgBuilder<S, A, P>) {
		private var analysisSet = false
		private var projection: (S) -> Any = { 0 }
		private var stopCriterion = StopCriterions.firstCex<S, A>()
		private lateinit var partialOrd: PartialOrd<S>
		private lateinit var strategy: Strategy<S, A, P>

		fun analysis(analysis: Analysis<*, *, *>) = apply {
			DI.analysisBadLeq = analysis is Prod2ExplPredAnalysis<*> || analysis is Prod2Analysis<*, *, *, *, *>
			analysisSet = true
		}

		fun projection(projection: Function<in S, *>) = apply { this.projection = { s -> projection.apply(s) } }

		fun stopCriterion(stopCriterion: StopCriterion<S, A>) = apply {
			if (DI.heuristicSearchType == HeuristicSearchType.FULL) {
				require(stopCriterion is FullExploration<S, A>)
			}
			this.stopCriterion = stopCriterion
		}

		fun strategy(strategy: Strategy<S, A, P>) = apply {
			if (DI.heuristicSearchType == HeuristicSearchType.FULL || DI.heuristicSearchType == HeuristicSearchType.DECREASING) {
				require(strategy.cegarHistoryStorage is CegarHistoryStoragePrevious<S, A, P>)
			}
			this.strategy = strategy
		}

		fun partialOrder(partialOrd: PartialOrd<S>) = apply { this.partialOrd = partialOrd }

		fun build(): AstarAbstractor<S, A, P> {
			require(analysisSet)
			return AstarAbstractor(argBuilder, projection, stopCriterion, partialOrd, strategy)
		}
	}
}
