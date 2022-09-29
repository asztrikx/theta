package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import javax.annotation.Nullable;

import java.util.List;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarNode<S extends State, A extends Action> {
	public final ArgNode<S, A> argNode;
	public @Nullable AstarNode<S, A> providerAstarNode;
	private Distance distance = new Distance(Distance.Type.UNKNOWN);
	private Distance heuristic = new Distance(Distance.Type.UNKNOWN);

	// providerAstarNode: can be null if it is the first arg
	AstarNode(final ArgNode<S, A> argNode, @Nullable final AstarNode<S, A> providerAstarNode) {
		this.argNode = checkNotNull(argNode);
		this.providerAstarNode = providerAstarNode;
		// providerAstarNode's distance can change after ctor ran
	}

	public void setDistance(Distance distance) {
		//assertAdmissability(distance);
		/*if (argNode.isTarget()) {
			assert distance.getType() == Distance.Type.EXACT && distance.getValue() == 0;
		}*/
		this.distance = distance;
	}

	public Distance getDistance() {
		return distance;
	}

	// It is guaranteed that once it returns a known value it won't change.
	public Distance getHeuristic() {
		if (heuristic.isKnown()) {
			// Provider distance can't change
			/*if (providerAstarNode != null && providerAstarNode.getDistance().isKnown()) {
				// Provider can change if original provider is covered => use equals
				assert heuristic.equals(providerAstarNode.getDistance());
			}*/
		} else {
			if (providerAstarNode != null && providerAstarNode.getDistance().isKnown()) {
				heuristic = providerAstarNode.getDistance();
			}
		}
		return heuristic;
	}

	public void setHeuristic(Distance heuristic) {
		//assert heuristic.isKnown();
		// Requirement for astar h(target) == 0
		/*if (argNode.isTarget()) {
			assert heuristic.getType() == Distance.Type.EXACT && heuristic.getValue() == 0;
		}*/
		this.heuristic = heuristic;
	}

	public Distance getWeight(int depth) {
		Distance heuristic = getHeuristic();
		Distance.Type type = heuristic.getType();
		if (type == Distance.Type.INFINITE) {
			return heuristic;
		}
		return new Distance(type, heuristic.getValue() + depth);
	}

	@Override
	public String toString() {
		// Can't print depth nor weight as it is dependent on the search (where it is started from)
		return String.format("%s D%s H%s", argNode.toString(), distance.toString(), heuristic.toString());
	}
}
