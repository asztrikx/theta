package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;

import java.util.Collection;

public class AstarDistanceKnown<S extends State, A extends Action> implements StopCriterion<S, A> {
	private final AstarNode<S, A> astarNode;

	public AstarDistanceKnown(AstarNode<S, A> astarNode) {
		this.astarNode = astarNode;
	}

	@Override
	public boolean canStop(ARG<S, A> arg) {
		return true; // TODO
	}

	@Override
	public boolean canStop(ARG<S, A> arg, Collection<ArgNode<S, A>> newNodes) {
		return canStop(arg);
	}
}
