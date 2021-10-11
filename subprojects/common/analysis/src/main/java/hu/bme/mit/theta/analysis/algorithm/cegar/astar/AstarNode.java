package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import java.util.Objects;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarNode<S extends hu.bme.mit.theta.analysis.State, A extends Action> {
    public final ArgNode<S, A> argNode;
    public final AstarNode<S, A> descendant;
    public HeuristicState heuristicState;
    // Has to be integer in order to know when its not set
    public Integer distanceToError;
    public static final String IllegalState = "unknown AstarNode.State";

    // heuristics is a value in distanceToError which can be used for next arg
    public enum HeuristicState {
        // DESCENDANT_UNAVAILABLE,
        //  this goes into HEURISTIC_UNKNOWN as it can be replaced by e.g. HEURISTIC_EXACT
        //  which is a problem when

        // have to calculate descendant heuristic in order to walk in current node to get heuristic
        DESCENDANT_UNKNOWN,

        // descendant heuristic is available (or doesn't exists) => we can walk in arg to get heuristic
        UNKNOWN,

        INFINITE,
        EXACT
    }

    private AstarNode(final ArgNode<S, A> argNode, final AstarNode<S, A> descendant) {
        this.argNode = checkNotNull(argNode);
        // can be null if it is the first arg
        this.descendant = descendant;
        recalculateState();
        this.distanceToError = null;
    }

    public void recalculateState() {
        if (descendant != null) {
            switch (descendant.heuristicState) {
                case DESCENDANT_UNKNOWN:
                case UNKNOWN:
                    heuristicState = HeuristicState.DESCENDANT_UNKNOWN;
                    break;
                case EXACT:
                    heuristicState = HeuristicState.UNKNOWN;
                    break;
                case INFINITE:
                    heuristicState = HeuristicState.INFINITE;
                    break;
                default:
                    throw new IllegalArgumentException(IllegalState);
            }
        } else {
            heuristicState = HeuristicState.UNKNOWN;
        }
    }

    public static <S extends State, A extends Action> AstarNode<S, A> create(
            final ArgNode<S, A> argNode,
            final AstarNode<S, A> descendant) {
        return new AstarNode<>(argNode, descendant);
    }

    @Override
    public String toString() {
        return String.format("(%s, %s, %s)",
                heuristicState.toString(),
                Objects.toString(distanceToError, "-1"),
                String.format("N%d", argNode.getId())
        );
    }
}
