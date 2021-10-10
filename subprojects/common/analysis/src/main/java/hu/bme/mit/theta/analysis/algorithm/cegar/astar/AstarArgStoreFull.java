package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;

public class AstarArgStoreFull<S extends State, A extends Action, P extends Prec> extends AstarArgStore<S, A, P>  {
    public AstarArgStoreFull(final PartialOrd<S> partialOrd) {
        super(partialOrd);
    }

    public void add(final AstarArg<S, A, P> astarArg) {
        assert astarArgs.size() <= 2;
        if (astarArgs.size() == 2) {
            astarArgs.remove(0);
        }
        // as we delete previous AstarArgs iteration based on astarArgs size is wrong
        int iteration = 1;
        if (astarArgs.size() != 0) {
            iteration = getLastIteration() + 1;
        }

        super.add(astarArg);
        astarArg.iteration = iteration;
    }

    public AstarArg<S, A, P> getIteration(int iteration) {
        return super.getIteration(astarArgs.size() - (getLastIteration() - iteration));
    }
}
