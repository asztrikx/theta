package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;

import java.util.ArrayList;
import java.util.Collection;
import java.util.stream.Collectors;

public interface AstarStopCriterion<S extends State, A extends Action, P extends Prec> {
    // This should not be used by a real AstarStopCriterion
    default boolean canStop(AstarArg<S, A, P> astarArg) {
        throw new RuntimeException();
    }

    boolean canStop(AstarArg<S, A, P> astarArg, AstarNode<S, A> astarNode);

    static <S extends State, A extends Action, P extends Prec> AstarStopCriterion<S, A, P> of(StopCriterion<S, A> stopCriterion) {
        return new AstarStopCriterion<>() {
            @Override
            public boolean canStop(AstarArg<S, A, P> astarArg) {
                return stopCriterion.canStop(astarArg.arg);
            }

            // Astar can't stop when currently handled node has a stopping children, we have to wait until it is the current node
            // therefore we wrap current node into a list as "newNodes"
            @Override
            public boolean canStop(AstarArg<S, A, P> astarArg, AstarNode<S, A> astarNode) {
                Collection<ArgNode<S, A>> nodes = new ArrayList<>();
                nodes.add(astarNode.argNode);
                return stopCriterion.canStop(astarArg.arg, nodes);
            }
        };
    }
}
