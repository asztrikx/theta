package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import static com.google.common.base.Preconditions.checkNotNull;

// TODO rename to AstarArgNode
public final class AstarNode<S extends hu.bme.mit.theta.analysis.State, A extends Action> {
    public final ArgNode<S, A> argNode;
    public final AstarNode<S, A> descendant;
    // TODO rename this & enum <= import is dead for State
    public State state;
    // TODO use optional
    // Has to be integer in order to know when its not set
    public Integer distanceToError;
    public static String IllegalState = "unknown AstarNode.State";

    // heuristics is a value in distanceToError which can be used for next arg
    public enum State {
        // no arg exists before this one from which we could have heuristic
        DESCENDANT_HEURISTIC_UNAVAILABLE,
        DESCENDANT_HEURISTIC_UNKNOWN,
        HEURISTIC_UNKNOWN,
        HEURISTIC_INFINITE,
        HEURISTIC_EXACT
    }

    private AstarNode(final ArgNode<S, A> argNode, final AstarNode<S, A> descendant) {
        this.argNode = checkNotNull(argNode);
        // can be null if it is the first arg
        this.descendant = descendant;
        if (descendant != null) {
            switch (descendant.state) {
                case DESCENDANT_HEURISTIC_UNKNOWN:
                case HEURISTIC_UNKNOWN:
                case DESCENDANT_HEURISTIC_UNAVAILABLE:
                    state = State.DESCENDANT_HEURISTIC_UNKNOWN;
                    break;
                case HEURISTIC_EXACT:
                    state = State.HEURISTIC_UNKNOWN;
                    break;
                case HEURISTIC_INFINITE:
                    state = State.HEURISTIC_INFINITE;
                    break;
                default:
                    throw new IllegalArgumentException(IllegalState);
            }
        } else {
            state = State.DESCENDANT_HEURISTIC_UNAVAILABLE;
        }
        this.distanceToError = null;
    }

    public static <S extends hu.bme.mit.theta.analysis.State, A extends Action> AstarNode<S, A> create(
            final ArgNode<S, A> argNode,
            final AstarNode<S, A> descendant) {
        return new AstarNode<>(argNode, descendant);
    }
}
