package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.Prec;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

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
        // set descendant
        AstarArg<S, A, P> descendant;
        if (size() == 0) {
            descendant = null;
        } else {
            descendant = getLast();
        }
        astarArg.descendant = descendant;

        astarArgs.add(astarArg);
    }

    public int size() {
        return astarArgs.size();
    }

    public AstarArg<S, A, P> get(int index) {
        return astarArgs.get(index);
    }

    public AstarArg<S, A, P> getLast() {
        return astarArgs.get(astarArgs.size() - 1);
    }

    public void addLastCopied() {
        final AstarArg<S, A, P> argLast = getLast();
        final AstarArg<S, A, P> astarArgLast = getLast();

        // copy ARG
        final ARG.ARGCopyResult<S, A> argCopyResult = argLast.arg.cloneWithResult();
        final ARG<S, A> argNew = argCopyResult.argCopied;
        final Map<ArgNode<S, A>, ArgNode<S, A>> oldToNew = argCopyResult.oldToNew;

        // new AstarArg
        final AstarArg<S, A, P> astarArgNew = AstarArg.create(argNew, argLast.prec, argLast, partialOrd);

        // do not copy heuristics as we will reach error in new arg
        //  which will give more accurate heuristics for next arg

        // init nodes
        for (Map.Entry<ArgNode<S, A>, AstarNode<S, A>> entry : astarArgLast.getAllInitNode().entrySet()) {
            ArgNode<S, A> argNodeLast = entry.getKey();
            ArgNode<S, A> argNodeNew = oldToNew.get(argNodeLast);
            AstarNode<S, A> astarNodeLast = entry.getValue();

            AstarNode<S, A> astarNodeNew = AstarNode.create(argNodeNew, astarNodeLast);
            astarArgNew.putInitNode(astarNodeNew);
            // do not add as it will be added when getAll is called
            //  astarArgNew.put(astarNodeNew);
        }

        // nodes
        for (Map.Entry<ArgNode<S, A>, AstarNode<S, A>> entry : astarArgLast.getAll().entrySet()) {
            ArgNode<S, A> argNodeLast = entry.getKey();
            ArgNode<S, A> argNodeNew = oldToNew.get(argNodeLast);
            AstarNode<S, A> astarNodeLast = entry.getValue();

            AstarNode<S, A> astarNodeNew = AstarNode.create(argNodeNew, astarNodeLast);
            astarArgNew.put(astarNodeNew);
        }

        add(astarArgNew);
    }
}
