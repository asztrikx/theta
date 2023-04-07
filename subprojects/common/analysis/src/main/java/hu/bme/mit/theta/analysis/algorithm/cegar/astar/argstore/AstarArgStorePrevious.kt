package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;

public class AstarArgStorePrevious<S extends State, A extends Action, P extends Prec> implements AstarArgStore<S, A, P> {
	private AstarArg<S, A, P> previous = null;
	private AstarArg<S, A, P> current = null;
	private int size = 0;

	private static final String unstoredAstarArg = "The requested AstarArg is not stored as it shouldn't be needed when doing full expand.";

	@Override
	public void add(final AstarArg<S, A, P> astarArg) {
		previous = current;
		current = astarArg;

		size++;
	}

	@Override
	public AstarArg<S, A, P> get(int index) {
		if (index == size - 1) {
			return current;
		} else if (index == size - 2) {
			return previous;
		} else {
			throw new RuntimeException(unstoredAstarArg);
		}
	}

	@Override
	public AstarArg<S, A, P> getLast() {
		return current;
	}

	@Override
	public int size() {
		return size;
	}

	@Override
	public int getIndex(AstarArg<S, A, P> astarArg) {
		if (current == astarArg) {
			return size - 1;
		} else if (previous == astarArg) {
			return size - 2;
		} else {
			throw new RuntimeException(unstoredAstarArg);
		}
	}

	@Override
	public void setLast(AstarArg<S, A, P> astarArg) {
		current = astarArg;
	}
}
