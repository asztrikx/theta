package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;

import java.util.ArrayList;
import java.util.List;

public class AstarArgStoreSemiOndemand<S extends State, A extends Action, P extends Prec> implements AstarArgStore<S, A, P> {
	protected final List<AstarArg<S, A, P>> astarArgs = new ArrayList<>();

	@Override
	public void add(final AstarArg<S, A, P> astarArg) {
		astarArgs.add(astarArg);
	}

	@Override
	public AstarArg<S, A, P> get(final int index) {
		return astarArgs.get(index);
	}

	@Override
	public AstarArg<S, A, P> getLast() {
		return astarArgs.get(size() - 1);
	}

	@Override
	public int size() {
		return astarArgs.size();
	}

	@Override
	public int getIndex(AstarArg<S, A, P> astarArg) {
		// Most of the time the requested astarArg is at the back therefore use lastIndexOf to search from back
		return astarArgs.lastIndexOf(astarArg);
	}
}
