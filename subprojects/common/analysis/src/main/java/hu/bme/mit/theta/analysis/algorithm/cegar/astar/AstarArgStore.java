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
public class AstarArgStore<S extends State, A extends Action, P extends Prec> {
    // TODO make not protected
    protected final PartialOrd<S> partialOrd;
    protected final List<AstarArg<S, A, P>> astarArgs = new ArrayList<>();

    public AstarArgStore(final PartialOrd<S> partialOrd) {
        this.partialOrd = partialOrd;
    }

    public void add(final AstarArg<S, A, P> astarArg) {
        // set descendant
        AstarArg<S, A, P> descendant;
        if (astarArgs.isEmpty()) {
            descendant = null;
        } else {
            descendant = getLast();
        }
        astarArg.descendant = descendant;
        // iteration is started from 1 in logging
        astarArg.iteration = astarArgs.size() + 1;

        astarArgs.add(astarArg);
    }

    public boolean isEmpty() {
        return astarArgs.isEmpty();
    }

    public AstarArg<S, A, P> getIteration(int iteration) {
        return astarArgs.get(iteration - 1);
    }

    public AstarArg<S, A, P> getLast() {
        return astarArgs.get(astarArgs.size() - 1);
    }

    public AstarArg<S, A, P> lastCopy() {
        final AstarArg<S, A, P> astarArgLast = getLast();

        // copy ARG
        final ARG.ARGCopyResult<S, A> argCopyResult = astarArgLast.arg.cloneWithResult();
        final ARG<S, A> argNew = argCopyResult.argCopied;
        final Map<ArgNode<S, A>, ArgNode<S, A>> oldToNew = argCopyResult.oldToNew;

        // new AstarArg
        final AstarArg<S, A, P> astarArgNew = AstarArg.create(argNew, astarArgLast.prec, astarArgLast, partialOrd);

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

        return astarArgNew;
    }

    public void addLastCopied() {
        add(lastCopy());
    }

    public int getLastIteration() {
        return getLast().iteration;
    }
}
