package hu.bme.mit.theta.xsts.analysis;

import hu.bme.mit.theta.analysis.expr.ExprState;
import hu.bme.mit.theta.common.Utils;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.booltype.BoolType;

public class XstsState<S extends ExprState> implements ExprState {

    private static final int HASH_SEED = 4413;
    private volatile int hashCode = 0;

    private final S state;
    private final boolean lastActionWasEnv;
    private final boolean initialized;

        private XstsState(S state, boolean lastActionWasEnv, boolean initialized) {
        this.state = state;
        this.lastActionWasEnv = lastActionWasEnv;
        this.initialized = initialized;
    }

    public static <S extends ExprState> XstsState<S> of(final S state, final boolean lastActionWasEnv, boolean initialized) {
        return new XstsState<>(state, lastActionWasEnv, initialized);
    }

    public S getState() {
        return state;
    }

    public boolean isLastActionWasEnv() {
        return lastActionWasEnv;
    }

    public boolean isInitialized() { return initialized; }

    @Override
    public Expr<BoolType> toExpr() {
        return state.toExpr();
    }

    @Override
    public boolean isBottom() {
        return state.isBottom();
    }

//    @Override
//    public int hashCode() {
//        int result = hashCode;
//        if (result == 0) {
//            result = HASH_SEED;
//            result = 31 * result + (lastActionWasEnv?0:1);
//            result = 31 * result + state.hashCode();
//            hashCode = result;
//        }
//        return result;
//    }
//
//    @Override
//    public boolean equals(final Object obj) {
//        if (this == obj) {
//            return true;
//        } else if (obj instanceof XstsState) {
//            final XstsState<?> that = (XstsState<?>) obj;
//            return this.lastActionWasEnv==that.lastActionWasEnv && this.initialized == that. initialized && this.state.equals(that.state);
//        } else {
//            return false;
//        }
//    }

    @Override
    public String toString() {
        return Utils.lispStringBuilder(getClass().getSimpleName()).aligned().add(initialized?"":"UNINITIALIZED").add(lastActionWasEnv?"ENV":"INTERNAL").body().add(state).toString();
    }
}
