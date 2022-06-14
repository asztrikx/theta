package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder;
import hu.bme.mit.theta.analysis.algorithm.SafetyChecker;
import hu.bme.mit.theta.analysis.algorithm.cegar.CegarChecker;
import hu.bme.mit.theta.analysis.algorithm.cegar.Refiner;
import hu.bme.mit.theta.analysis.pred.PredState;
import hu.bme.mit.theta.common.logging.Logger;

import java.util.function.Function;

public class AstarSafetyChecker {
	public static <S extends State, A extends Action, P extends Prec> SafetyChecker<S, A, P> getAstarSafetyChecker(
			ArgBuilder<S, A, P> argBuilder,
			Refiner<S, A, P> refiner,
			PartialOrd<S> partialOrd,
			Logger logger,
			Function<? super S, ?> projection,
			boolean isMultiSeq
	) {
		AstarCegarChecker.Type astarCegarCheckerType;
		// Like BasicAbstractor (which is not used) created above the caller statment
		if (isMultiSeq) {
			astarCegarCheckerType = AstarCegarChecker.Type.FULL;
		} else {
			astarCegarCheckerType = AstarCegarChecker.Type.SEMI_ONDEMAND;
		}
		return AstarCegarChecker
				.create(argBuilder, projection, refiner, logger, partialOrd, astarCegarCheckerType);
	}
}
