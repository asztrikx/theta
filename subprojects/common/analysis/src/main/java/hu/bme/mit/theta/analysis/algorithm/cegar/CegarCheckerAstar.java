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
package hu.bme.mit.theta.analysis.algorithm.cegar;

import static com.google.common.base.Preconditions.checkNotNull;

import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

import com.google.common.base.Stopwatch;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.SafetyChecker;
import hu.bme.mit.theta.analysis.algorithm.SafetyResult;
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.analysis.utils.ArgVisualizer;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.common.Utils;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

/**
 * Counterexample-Guided Abstraction Refinement (CEGAR) loop implementation,
 * that uses an Abstractor to explore the abstract state space and a Refiner to
 * check counterexamples and refine them if needed. It also provides certain
 * statistics about its execution.
 */
public final class CegarCheckerAstar<S extends State, A extends Action, P extends Prec> implements SafetyChecker<S, A, P> {

	private final Abstractor<S, A, P> abstractor;
	private final Refiner<S, A, P> refiner;
	private final Logger logger;

	// a* specific
	// TODO should these change during run? or event should there be local versions
	private final AstarComparator astarComparator;
	private final int depthWeight = 1;
	private final int heuristicsWeight = 2;

	private final class AstarComparator implements Comparator<ArgNode<S, A>> {
		protected final PartialOrd<S> partialOrd;
		private final int depthWeight;
		private final int heuristicsWeight;
		private final DistanceHeuristicStore distanceHeuristicStore;

		private final class DistanceHeuristicStore {
			private final Map<ArgNode<S,A>, Integer> distanceHeuristics = new HashContainerFactory().createMap();
			Partition<ArgNode<S, A>, ?> indexer;
			final Function<? super S, ?> projection;

			private DistanceHeuristicStore(final Function<? super S, ?> projection) {
				this.projection = projection;
				this.indexer = Partition.of(n -> projection.apply(n.getState()));
			}

			public DistanceHeuristicStore create(final Function<? super S, ?> projection) {
				return new DistanceHeuristicStore(projection);
			}

			// put saves argNode with specified distance to target
			public void put(final ArgNode<S, A> argNode, final int distance) {
				distanceHeuristics.put(argNode, distance);
				indexer.add(argNode);
			}

			public void putAll(final Map<ArgNode<S,A>, Integer> distances) {
				distanceHeuristics.putAll(distances);
				indexer.addAll(distances.keySet());
			}

			// get searches for an ArgNode which is <= then argNode and returns its distance.
			// If no ArgNode is suitable then -1 is returned.
			public int get(final ArgNode<S, A> argNode) {
				// get keys which are <= then current key
				final AtomicInteger distance = new AtomicInteger(-1);

				final List<ArgNode<S, A>> argNodeCandidates = indexer.get(argNode);
				argNodeCandidates.forEach((argNodeCandidate)->{
					final int distanceHeuristic = distanceHeuristics.get(argNodeCandidate);
					if (distance.get() != -1 && distance.get() <= distanceHeuristic) {
						return;
					}
					if(partialOrd.isLeq(argNode.getState(), argNodeCandidate.getState())){
						distance.set(distanceHeuristic);
					}
				});

				return distance.get();
			}

			public boolean contains(final ArgNode<S,A> argNode){
				final List<ArgNode<S, A>> argNodeCandidates = indexer.get(argNode);
				return argNodeCandidates.size() != 0;
			}

			public void clear() {
				distanceHeuristics.clear();
				indexer = Partition.of(n -> projection.apply(n.getState()));
			}
		}

		public AstarComparator(final PartialOrd<S> partialOrd, final Function<? super S, ?> projection, final int depthWeight, final int heuristicsWeight){
			this.partialOrd = partialOrd;
			this.distanceHeuristicStore = new DistanceHeuristicStore(projection);
			this.depthWeight = depthWeight;
			this.heuristicsWeight = heuristicsWeight;
		}

		public int compare(final ArgNode<S, A> argNode1, final ArgNode<S, A> argNode2) {
			final boolean distance1Exists = distanceHeuristicStore.contains(argNode1);
			final boolean distance2Exists = distanceHeuristicStore.contains(argNode2);

			// not reachable in more abstract domain => won't be reachable in refined => give 'infinite' weight
			if (!distance1Exists && !distance2Exists) {
				return 0;
			} else if (!distance1Exists) {
				return 1;
			} else if (!distance2Exists) {
				return -1;
			}

			final int distance1 = distanceHeuristicStore.get(argNode1);
			final int distance2 = distanceHeuristicStore.get(argNode2);

			// calculate a* heuristics
			final int weight1 = depthWeight * argNode1.getDepth() + heuristicsWeight * distance1;
			final int weight2 = depthWeight * argNode2.getDepth() + heuristicsWeight * distance2;

			return weight1 - weight2;
		}

		// Keeps track of previous states to use heuristics in next arg creation
		public void store(final ARG<S, A> arg){
			// clear previous results as currents have better refinement so they are more accurate
			distanceHeuristicStore.clear();
			final Map<ArgNode<S,A>, Integer> distances = arg.getDistances();
			distanceHeuristicStore.putAll(distances);
		}
	}

	private CegarCheckerAstar(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Logger logger
	) {
		astarComparator = new AstarComparator(partialOrd, projection, depthWeight, heuristicsWeight);
		final Abstractor<S, A, P> abstractor = BasicAbstractor
			.builder(argBuilder)
			.projection(projection)
			.waitlist(PriorityWaitlist.create(astarComparator))
			.stopCriterion(StopCriterions.fullExploration())
			.logger(logger)
			.build();

		this.abstractor = checkNotNull(abstractor);
		this.refiner = checkNotNull(refiner);
		this.logger = checkNotNull(logger);
	}

	public static <S extends State, A extends Action, P extends Prec> CegarCheckerAstar<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd) {
		return new CegarCheckerAstar<>(argBuilder, projection, refiner, partialOrd, NullLogger.getInstance());
	}

	public static <S extends State, A extends Action, P extends Prec> CegarCheckerAstar<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Logger logger) {
		return new CegarCheckerAstar<>(argBuilder, projection, refiner, partialOrd, logger);
	}

	@Override
	public SafetyResult<S, A> check(final P initPrec) {
		logger.write(Level.INFO, "Configuration: %s%n", this);
		final Stopwatch stopwatch = Stopwatch.createStarted();
		long abstractorTime = 0;
		long refinerTime = 0;
		RefinerResult<S, A, P> refinerResult = null;
		AbstractorResult abstractorResult = null;
		final ARG<S, A> arg = abstractor.createArg();
		P prec = initPrec;
		int iteration = 0;
		do {
			++iteration;

			logger.write(Level.MAINSTEP, "Iteration %d%n", iteration);
			logger.write(Level.MAINSTEP, "| Checking abstraction...%n");
			final long abstractorStartTime = stopwatch.elapsed(TimeUnit.MILLISECONDS);
			abstractorResult = abstractor.check(arg, prec);
			abstractorTime += stopwatch.elapsed(TimeUnit.MILLISECONDS) - abstractorStartTime;
			logger.write(Level.MAINSTEP, "| Checking abstraction done, result: %s%n", abstractorResult);

			//System.out.println(GraphvizWriter.getInstance().writeString(ArgVisualizer.getDefault().visualize(arg)));
			astarComparator.store(arg);

			if (abstractorResult.isUnsafe()) {
				logger.write(Level.MAINSTEP, "| Refining abstraction...%n");
				final long refinerStartTime = stopwatch.elapsed(TimeUnit.MILLISECONDS);
				refinerResult = refiner.refine(arg, prec);
				refinerTime += stopwatch.elapsed(TimeUnit.MILLISECONDS) - refinerStartTime;
				logger.write(Level.MAINSTEP, "Refining abstraction done, result: %s%n", refinerResult);

				if (refinerResult.isSpurious()) {
					prec = refinerResult.asSpurious().getRefinedPrec();
				}
			}
		} while (!abstractorResult.isSafe() && !refinerResult.isUnsafe());

		stopwatch.stop();
		SafetyResult<S, A> cegarResult = null;
		final CegarStatistics stats = new CegarStatistics(stopwatch.elapsed(TimeUnit.MILLISECONDS), abstractorTime,
				refinerTime, iteration);

		assert abstractorResult.isSafe() || (refinerResult != null && refinerResult.isUnsafe());

		if (abstractorResult.isSafe()) {
			cegarResult = SafetyResult.safe(arg, stats);
		} else if (refinerResult.isUnsafe()) {
			cegarResult = SafetyResult.unsafe(refinerResult.asUnsafe().getCex(), arg, stats);
		}

		assert cegarResult != null;
		logger.write(Level.RESULT, "%s%n", cegarResult);
		logger.write(Level.INFO, "%s%n", stats);
		return cegarResult;
	}

	@Override
	public String toString() {
		return Utils.lispStringBuilder(getClass().getSimpleName()).add(abstractor).add(refiner).toString();
	}
}
