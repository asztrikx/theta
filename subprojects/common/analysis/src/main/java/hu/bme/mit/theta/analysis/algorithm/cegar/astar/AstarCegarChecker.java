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
package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import static com.google.common.base.Preconditions.checkNotNull;

import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.google.common.base.Stopwatch;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder;
import hu.bme.mit.theta.analysis.algorithm.SafetyChecker;
import hu.bme.mit.theta.analysis.algorithm.SafetyResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.Abstractor;
import hu.bme.mit.theta.analysis.algorithm.cegar.AbstractorResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.Refiner;
import hu.bme.mit.theta.analysis.algorithm.cegar.RefinerResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.CegarStatistics;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions;
import hu.bme.mit.theta.common.Utils;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.logging.Logger.Level;

/**
 * Counterexample-Guided Abstraction Refinement (CEGAR) loop implementation,
 * that uses an Abstractor to explore the abstract state space and a Refiner to
 * check counterexamples and refine them if needed. It also provides certain
 * statistics about its execution.
 */
public final class AstarCegarChecker<S extends State, A extends Action, P extends Prec> implements SafetyChecker<S, A, P> {
	private final Abstractor<S, A, P> abstractor;
	private final Refiner<S, A, P> refiner;
	private final Logger logger;

	// Blocking
	// TODO uncomplete arg stopped => exception did this happen because heur inf?
	// TODO fix test failing (maybe the astarArgStore null exception error related? check stacktrace)
	// TODO fix astarviz (+ add title)
	// 	astarArgStore null exception error when trying to print back state with astarviz
	// TODO assert doesnt throw exception: see telegram photo (in compare when checking descendant heur unknown)

	// Last checks
	// TODO parallel run when going back
	// TODO measure without debug!!!
	// TODO fix intellij yellow warnings
	// TODO final for func arguments
	// TODO checkNotNull for func arguments
	// TODO use Collection (where no indexing required)
	// TODO log: root,etc
	// TODO create common store for argnodes until they modified (copy on write)
	// TODO fix imports
	// TODO what if prune deletes an init node, is it possible?
	// TODO covering edges to descendants: when can we give inf heur
	// TODO should we check descendant != null (intellij helps) or state != DESCENDANT_UNAVAILABLE
	// TODO Collections.Emptylist instead of new Arr.. ?
	// TODO create testing env with some test cases
	// TODO comment one parent abstract should be
	// TODO use javadocs
	// TODO use new HashContainerFactory().createMap()
	// TODO check if covered => has to check coverer
	// TODO rename Last Current to unified name
	// TODO eliminate hashmap, create edges etc?

	final AstarArgStore<S, A, P> astarArgStore;

	private AstarCegarChecker(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Logger logger
	) {
		this.astarArgStore = AstarArgStore.create(partialOrd);
		final Abstractor<S, A, P> abstractor = AstarAbstractor
			.builder(argBuilder)
			.projection(projection)
			.stopCriterion(StopCriterions.firstCex())
			.logger(logger)
			.AstarArgStore(astarArgStore)
			.build();

		this.abstractor = checkNotNull(abstractor);
		this.refiner = checkNotNull(refiner);
		this.logger = checkNotNull(logger);
	}

	public static <S extends State, A extends Action, P extends Prec> AstarCegarChecker<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd) {
		return new AstarCegarChecker<>(argBuilder, projection, refiner, partialOrd, NullLogger.getInstance());
	}

	public static <S extends State, A extends Action, P extends Prec> AstarCegarChecker<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Logger logger) {
		return new AstarCegarChecker<>(argBuilder, projection, refiner, partialOrd, logger);
	}

	@Override
	public SafetyResult<S, A> check(final P initPrec) {
		logger.write(Level.INFO, "Configuration: %s%n", this);
		final Stopwatch stopwatch = Stopwatch.createStarted();
		long abstractorTime = 0;
		long refinerTime = 0;
		RefinerResult<S, A, P> refinerResult = null;
		AbstractorResult abstractorResult = null;
		ARG<S, A> arg = abstractor.createArg();
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

			// copy state of arg and last AstarArg in astarArgStore to be able to go back ondemand for heuristics
			// 	always has last as check will create if there isn't
			astarArgStore.addLastCopied();
			arg = astarArgStore.getLast().arg;

			if (abstractorResult.isUnsafe()) {
				logger.write(Level.MAINSTEP, "| Refining abstraction...%n");
				final long refinerStartTime = stopwatch.elapsed(TimeUnit.MILLISECONDS);
				refinerResult = refiner.refine(arg, prec);
				// throw out pruned ArgNode's AstarNodes
				astarArgStore.getLast().prune();
				refinerTime += stopwatch.elapsed(TimeUnit.MILLISECONDS) - refinerStartTime;
				logger.write(Level.MAINSTEP, "Refining abstraction done, result: %s%n", refinerResult);

				if (refinerResult.isSpurious()) {
					prec = refinerResult.asSpurious().getRefinedPrec();
					astarArgStore.getLast().prec = prec;
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
