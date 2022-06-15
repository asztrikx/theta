package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;

public interface AstarArgStore<S extends State, A extends Action, P extends Prec> {
	void add(AstarArg<S, A, P> astarArg);
	AstarArg<S, A, P> get(int index);
	AstarArg<S, A, P> getLast();
	int size();
	int getIndex(AstarArg<S, A, P> astarArg);
}
