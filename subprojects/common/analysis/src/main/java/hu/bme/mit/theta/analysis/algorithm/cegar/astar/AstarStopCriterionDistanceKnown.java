package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;

import java.util.Collection;

public final class AstarStopCriterionDistanceKnown<S extends State, A extends Action, P extends Prec> implements AstarStopCriterion<S, A, P> {
    @Override
    public boolean canStop(AstarArg<S, A, P> astarArg, AstarNode<S, A> astarNode) {
        return astarNode.distance.isKnown();
    }
}
