package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import static com.google.common.base.Preconditions.checkNotNull;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import java.util.Comparator;

public final class AstarComparator<S extends State, A extends Action, P extends Prec> implements Comparator<ArgNode<S, A>> {
    private final AstarArg<S, A, P> astarArg;

    private AstarComparator(final AstarArg<S, A, P> astarArg){
        this.astarArg = astarArg;
    }

    public static <S extends State, A extends Action, P extends Prec> AstarComparator<S, A, P> create(
            final AstarArg<S, A, P> astarArg
    ) {
        return new AstarComparator<>(astarArg);
    }

    @Override
    public int compare(final ArgNode<S, A> argNode1, final ArgNode<S, A> argNode2) {
        final AstarNode<S, A> astarNode1 = checkNotNull(astarArg.get(argNode1));
        final AstarNode<S, A> astarNode2 = checkNotNull(astarArg.get(argNode2));

        // one heuristic is not available <=> other heuristic is not available
        assert astarNode1.state != AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE && astarNode2.state != AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE ||
                astarNode1.state == AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE && astarNode2.state == AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE;

        // no heuristic from previous arg as no previous arg exists
        //  because of previous assert we only need to check one AstarNode's state
        if (astarNode1.state == AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE) {
            // calculate bfs weights
            final int weight1 = argNode1.getDepth();
            final int weight2 = argNode2.getDepth();

            return weight1 - weight2;
        }

        // descendant's heuristics should be known
        // TODO getheuristics for descendant should be called here (for later not exact heuristics)
        assert astarNode1.state != AstarNode.State.DESCENDANT_HEURISTIC_UNKNOWN;
        assert astarNode2.state != AstarNode.State.DESCENDANT_HEURISTIC_UNKNOWN;

        // use descendants to get heuristics
        final AstarNode<S, A> descendant1 = astarNode1.descendant;
        final AstarNode<S, A> descendant2 = astarNode2.descendant;

        // not reachable in more abstract domain => won't be reachable in refined => treat as 'infinite' weight
        final boolean unreachable1 = descendant1.state == AstarNode.State.HEURISTIC_INFINITE;
        final boolean unreachable2 = descendant2.state == AstarNode.State.HEURISTIC_INFINITE;
        if (unreachable1 && unreachable2) {
            return 0;
        } else if (unreachable1) {
            return 1;
        } else if (unreachable2) {
            return -1;
        }

        // catch missing State handle for later developments
        if (descendant1.state != AstarNode.State.HEURISTIC_EXACT) {
            throw new IllegalArgumentException(AstarNode.IllegalState);
        }
        if (descendant2.state != AstarNode.State.HEURISTIC_EXACT) {
            throw new IllegalArgumentException(AstarNode.IllegalState);
        }

        final int distanceToError1 = checkNotNull(descendant1.distanceToError);
        final int distanceToError2 = checkNotNull(descendant2.distanceToError);

        // calculate a* heuristics
        final int weight1 = argNode1.getDepth() + distanceToError1;
        final int weight2 = argNode2.getDepth() + distanceToError2;

        return weight1 - weight2;
    }
}

