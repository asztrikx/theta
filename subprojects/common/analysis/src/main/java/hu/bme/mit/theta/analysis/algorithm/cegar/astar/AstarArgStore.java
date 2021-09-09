package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;

import java.util.ArrayList;
import java.util.List;

// AstarArgStore
//  this should be a class as later we might make this abstract and create NoStore implementation
public final class AstarArgStore<S extends State, A extends Action, P extends Prec> {
    // TODO make not protected
    protected final PartialOrd<S> partialOrd;
    private final List<AstarArg<S, A, P>> astarArgs = new ArrayList<>();

    private AstarArgStore(final PartialOrd<S> partialOrd) {
        this.partialOrd = partialOrd;
    }

    public static <S extends State, A extends Action, P extends Prec> AstarArgStore<S, A, P> create(final PartialOrd<S> partialOrd) {
        return new AstarArgStore<>(partialOrd);
    }

    public void add(final AstarArg<S, A, P> astarArg) {
        astarArgs.add(astarArg);
    }

    public int size() {
        return astarArgs.size();
    }

    public AstarArg<S, A, P> get(int index) {
        return astarArgs.get(index);
    }
}
