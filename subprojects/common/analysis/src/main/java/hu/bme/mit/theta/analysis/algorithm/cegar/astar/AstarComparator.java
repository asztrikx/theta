package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import static com.google.common.base.Preconditions.checkNotNull;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import java.util.Comparator;

public final class AstarComparator<S extends State, A extends Action, P extends Prec> implements Comparator<AstarNode<S, A>> {
    private AstarComparator() {
    }

    public static <S extends State, A extends Action, P extends Prec> AstarComparator<S, A, P> create() {
        return new AstarComparator<>();
    }

    @Override
    public int compare(final AstarNode<S, A> astarNode1, final AstarNode<S, A> astarNode2) {
        final ArgNode<S, A> argNode1 = astarNode1.argNode;
        final ArgNode<S, A> argNode2 = astarNode2.argNode;

        // one heuristic is not available <=> other heuristic is not available
        assert astarNode1.parent != null && astarNode2.parent != null ||
                astarNode1.parent == null && astarNode2.parent == null;

        // no heuristic from previous arg as no previous arg exists
        //  because of previous assert we only need to check one AstarNode's state
        if (astarNode1.parent == null) {
            // calculate bfs weights
            final int weight1 = argNode1.getDepth();
            final int weight2 = argNode2.getDepth();

            return weight1 - weight2;
        }

        // parent's heuristics should be known
        assert astarNode1.heuristicState != AstarNode.HeuristicState.PARENT_UNKNOWN;
        assert astarNode2.heuristicState != AstarNode.HeuristicState.PARENT_UNKNOWN;

        // use parents to get heuristics
        final AstarNode<S, A> parent1 = astarNode1.parent;
        final AstarNode<S, A> parent2 = astarNode2.parent;

        // not reachable in more abstract domain => won't be reachable in refined => treat as 'infinite' weight
        final boolean unreachable1 = parent1.heuristicState == AstarNode.HeuristicState.INFINITE;
        final boolean unreachable2 = parent2.heuristicState == AstarNode.HeuristicState.INFINITE;
        if (unreachable1 && unreachable2) {
            return 0;
        } else if (unreachable1) {
            return 1;
        } else if (unreachable2) {
            return -1;
        }

        // catch missing State handle for later developments
        if (
            parent1.heuristicState != AstarNode.HeuristicState.EXACT &&
            parent2.heuristicState != AstarNode.HeuristicState.EXACT
        ) {
            throw new IllegalArgumentException(AstarNode.IllegalState);
        }

        final int distanceToError1 = checkNotNull(parent1.distanceToError);
        final int distanceToError2 = checkNotNull(parent2.distanceToError);

        // calculate a* heuristics
        final int weight1 = argNode1.getDepth() + distanceToError1;
        final int weight2 = argNode2.getDepth() + distanceToError2;

        return weight1 - weight2;
    }
}

