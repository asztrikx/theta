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

import com.google.common.base.Stopwatch;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.algorithm.*;
import hu.bme.mit.theta.analysis.algorithm.cegar.Abstractor;
import hu.bme.mit.theta.analysis.algorithm.cegar.AbstractorResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.Refiner;
import hu.bme.mit.theta.analysis.algorithm.cegar.RefinerResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.CegarStatistics;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStoreFull;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStoreSemiOndemand;
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
	private final Function<? super S, ?> projection;
	private final PartialOrd<S> partialOrd;

	public enum Type {
		FULL, SEMI_ONDEMAND
	}

	final AstarArgStore<S, A, P> astarArgStore;

	private AstarCegarChecker(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Logger logger, final Type type
	) {
		StopCriterion<S, A> stopCriterion;
		switch (type) {
			case FULL:
				this.astarArgStore = new AstarArgStoreFull<>();
				stopCriterion = StopCriterions.fullExploration();
				break;
			case SEMI_ONDEMAND:
				this.astarArgStore = new AstarArgStoreSemiOndemand<>();
				stopCriterion = StopCriterions.firstCex();
				break;
			default:
				throw new IllegalArgumentException("Unknown AstarCegarChecker.Type");
		}
		final Abstractor<S, A, P> abstractor = AstarAbstractor
			.builder(argBuilder)
			.projection(projection) //
			.stopCriterion(stopCriterion)
			.logger(logger)
			.AstarArgStore(astarArgStore)
			.type(type)
			.partialOrder(partialOrd)
			.build();

		this.abstractor = checkNotNull(abstractor);
		this.refiner = checkNotNull(refiner);
		this.logger = checkNotNull(logger);
		this.projection = projection;
		this.partialOrd = partialOrd;
	}

	public static <S extends State, A extends Action, P extends Prec> AstarCegarChecker<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final PartialOrd<S> partialOrd, final Type type) {
		return new AstarCegarChecker<>(argBuilder, projection, refiner, partialOrd, NullLogger.getInstance(), type);
	}

	public static <S extends State, A extends Action, P extends Prec> AstarCegarChecker<S, A, P> create(
			final ArgBuilder<S, A, P> argBuilder, final Function<? super S, ?> projection, final Refiner<S, A, P> refiner,
			final Logger logger, final PartialOrd<S> partialOrd, final Type type) {
		return new AstarCegarChecker<>(argBuilder, projection, refiner, partialOrd, logger, type);
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
		AstarArg<S, A, P> astarArg = new AstarArg<>(arg, prec, partialOrd, projection);
		int iteration = 0;
		do {
			++iteration;

			astarArgStore.add(astarArg);

			logger.write(Level.MAINSTEP, "Iteration %d%n", iteration);
			logger.write(Level.MAINSTEP, "| Checking abstraction...%n");
			final long abstractorStartTime = stopwatch.elapsed(TimeUnit.MILLISECONDS);
			abstractorResult = abstractor.check(arg, prec);
			abstractorTime += stopwatch.elapsed(TimeUnit.MILLISECONDS) - abstractorStartTime;
			logger.write(Level.MAINSTEP, "| Checking abstraction done, result: %s%n", abstractorResult);

			if (abstractorResult.isUnsafe()) {
				// AstarArg has to be copied as arg will be modified after refinement
				astarArg = AstarCopier.createCopy(astarArgStore.getLast(), prec, partialOrd, projection);
				arg = astarArg.arg;

				logger.write(Level.MAINSTEP, "| Refining abstraction...%n");
				final long refinerStartTime = stopwatch.elapsed(TimeUnit.MILLISECONDS);
				refinerResult = refiner.refine(arg, prec);
				astarArg.pruneApply();
				refinerTime += stopwatch.elapsed(TimeUnit.MILLISECONDS) - refinerStartTime;
				logger.write(Level.MAINSTEP, "Refining abstraction done, result: %s%n", refinerResult);

				if (refinerResult.isSpurious()) {
					// Pruned version of an ARG is the next iteration of ARG.
					prec = refinerResult.asSpurious().getRefinedPrec();
					astarArg.prec = prec;
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
