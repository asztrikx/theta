package hu.bme.mit.theta.xsts.analysis;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Trace;
import hu.bme.mit.theta.analysis.expl.ExplState;
import hu.bme.mit.theta.analysis.expr.refinement.ExprTraceChecker;
import hu.bme.mit.theta.analysis.expr.refinement.ExprTraceFwBinItpChecker;
import hu.bme.mit.theta.analysis.expr.refinement.ExprTraceStatus;
import hu.bme.mit.theta.analysis.expr.refinement.ItpRefutation;
import hu.bme.mit.theta.core.model.Valuation;
import hu.bme.mit.theta.solver.SolverFactory;
import hu.bme.mit.theta.xsts.XSTS;

import java.util.ArrayList;
import java.util.List;

import static com.google.common.base.Preconditions.checkArgument;
import static hu.bme.mit.theta.core.type.booltype.BoolExprs.Not;

public class XstsTraceConcretizer {

	private XstsTraceConcretizer() {
	}

	public static Trace<XstsState<ExplState>, XstsAction> concretize(
			final Trace<XstsState<?>, XstsAction> trace, SolverFactory solverFactory, final XSTS xsts) {

		final ExprTraceChecker<ItpRefutation> checker = ExprTraceFwBinItpChecker.create(xsts.getInitFormula(),
				Not(xsts.getProp()), solverFactory.createItpSolver());
		final ExprTraceStatus<ItpRefutation> status = checker.check(trace);
		checkArgument(status.isFeasible(), "Infeasible trace.");
		final Trace<Valuation, ? extends Action> valuations = status.asFeasible().getValuations();

		assert valuations.getStates().size() == trace.getStates().size();

		final List<XstsState<ExplState>> xstsStates = new ArrayList<>();
		for (int i = 0; i < trace.getStates().size(); ++i) {
			xstsStates.add(XstsState.of(ExplState.of(valuations.getState(i)),trace.getState(i).lastActionWasEnv(),trace.getState(i).isInitialized()));
		}

		return Trace.of(xstsStates, trace.getActions());
	}
}