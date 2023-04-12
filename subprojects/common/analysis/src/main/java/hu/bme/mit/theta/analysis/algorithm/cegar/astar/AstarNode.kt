package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import javax.annotation.Nullable;

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

	// Do not call with unknown distance, use reset().
	public void setDistance(Distance distance) {
		assertAdmissibility(distance);
		if (argNode.isTarget()) {
			assert distance.getType() == Distance.Type.EXACT && distance.getValue() == 0;
		}
		this.distance = distance;
	}

	public Distance getDistance() {
		return distance;
	}

	// It is guaranteed that once it returns a known value it won't change unless setHeuristic is called with unknown (AstarIterator).
	public Distance getHeuristic() {
		if (heuristic.isKnown()) {
			// Provider distance can't change
			if (providerAstarNode != null && providerAstarNode.getDistance().isKnown()) {
				// Provider can change if original provider is covered => use equals
				assert heuristic.equals(providerAstarNode.getDistance());
			}
		} else {
			if (providerAstarNode != null && providerAstarNode.getDistance().isKnown()) {
				// Sideeffect is neccessary as astarNode-providerAstarNode relation is one directional therefore
				// we can't update astarNode when providerAstarNode's distance changes.
				heuristic = providerAstarNode.getDistance();
			}
		}
		return heuristic;
	}

	// Do not call with unknown distance, use reset().
	public void setHeuristic(Distance heuristic) {
		assert heuristic.isKnown();

		// Requirement for astar h(target) == 0
		if (argNode.isTarget()) {
			assert heuristic.getType() == Distance.Type.EXACT && heuristic.getValue() == 0;
		}
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

	private void assertAdmissibility(Distance distance) {
		// called from updateDistancesFromConditionalNodes:
		// 		this's heuristic can not be unknown as we are always starting from nodes with known heuristic and for all successor nodes we call findHeuristic.
		//		This won't be the case when we start searching from leaf nodes.
		assert !(getHeuristic().getType() == Distance.Type.INFINITE && distance.getType() != Distance.Type.INFINITE);
		assert getHeuristic().isKnown();
		assert distance.isKnown();
		if (getHeuristic().getType() != Distance.Type.INFINITE && distance.getType() != Distance.Type.INFINITE) {
			assert getHeuristic().getValue() <= distance.getValue();
		}
	}

	public void reset() {
		this.heuristic = new Distance(Distance.Type.UNKNOWN);
		this.distance = new Distance(Distance.Type.UNKNOWN);
	}

	@Override
	public String toString() {
		// Can't print depth nor weight as it is dependent on the search (where it is started from)
		return String.format("%s D%s H%s", argNode.toString(), distance.toString(), heuristic.toString());
	}
}
