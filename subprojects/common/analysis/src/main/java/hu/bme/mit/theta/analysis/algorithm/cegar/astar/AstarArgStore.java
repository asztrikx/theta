package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.reachedset.Partition;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

// AstarArgStore
//  this should be a class as later we might make this abstract and create NoStore implementation
public class AstarArgStore<S extends State, A extends Action, P extends Prec> {
    public final PartialOrd<S> partialOrd;
    protected final List<AstarArg<S, A, P>> astarArgs = new ArrayList<>();

    public AstarArgStore(final PartialOrd<S> partialOrd) {
        this.partialOrd = partialOrd;
    }

    public void add(final AstarArg<S, A, P> astarArg) {
        // set parent
        AstarArg<S, A, P> parent;
        if (astarArgs.isEmpty()) {
            parent = null;
        } else {
            parent = getLast();
        }
        astarArg.parent = parent;
        // iteration is started from 1 in logging
        astarArg.iteration = astarArgs.size() + 1;

        astarArgs.add(astarArg);
    }

    public boolean isEmpty() {
        return astarArgs.isEmpty();
    }

    public AstarArg<S, A, P> getIteration(final int iteration) {
        return astarArgs.get(iteration - 1);
    }

    public AstarArg<S, A, P> getLast() {
        return astarArgs.get(astarArgs.size() - 1);
    }

    public int getLastIteration() {
        return getLast().iteration;
    }
}
