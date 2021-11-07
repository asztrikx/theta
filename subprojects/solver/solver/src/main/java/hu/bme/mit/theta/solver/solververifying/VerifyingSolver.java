package hu.bme.mit.theta.solver.solververifying;

import hu.bme.mit.theta.core.model.Valuation;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.booltype.BoolType;
import hu.bme.mit.theta.solver.Interpolant;
import hu.bme.mit.theta.solver.ItpMarker;
import hu.bme.mit.theta.solver.ItpMarkerTree;
import hu.bme.mit.theta.solver.ItpPattern;
import hu.bme.mit.theta.solver.ItpSolver;
import hu.bme.mit.theta.solver.Solver;
import hu.bme.mit.theta.solver.SolverManager;
import hu.bme.mit.theta.solver.SolverStatus;

import java.util.Collection;

import static com.google.common.base.Preconditions.checkState;
import static hu.bme.mit.theta.core.type.booltype.BoolExprs.True;

public class VerifyingSolver implements Solver {
	private final Solver solver;
	VerifyingSolver(String solver) throws Exception {
		this.solver = SolverManager.resolveSolverFactory(solver).createSolver();
	}

	@Override
	public void add(Expr<BoolType> assertion) {
		solver.add(assertion);
	}

	@Override
	public SolverStatus check()
	{
		SolverStatus check = solver.check();
		if(check.isSat()) {
			final Valuation model = solver.getModel();
			for (Expr<BoolType> assertion : solver.getAssertions()) {
				checkState(assertion.eval(model).equals(True()), "Solver problem: " + assertion);
			}
		}
		return check;
	}

	@Override
	public void push() {
		solver.push();
	}

	@Override
	public void pop(int n) {
		solver.pop();
	}

	@Override
	public void reset() {
		solver.reset();
	}

	@Override
	public SolverStatus getStatus() {
		return solver.getStatus();
	}

	@Override
	public Valuation getModel() {
		return solver.getModel();
	}

	@Override
	public Collection<Expr<BoolType>> getAssertions() {
		return solver.getAssertions();
	}

	@Override
	public void close() throws Exception {
		solver.close();
	}
}
