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
	public Distance distance;

	//// TODO this should be through astarArg as they are coupled together, this should be disabled (protected:)
	// providerAstarNode: can be null if it is the first arg
	public AstarNode(final ArgNode<S, A> argNode, @Nullable final AstarNode<S, A> providerAstarNode) {
		this.argNode = checkNotNull(argNode);
		this.providerAstarNode = providerAstarNode;
		this.distance = new Distance(Distance.Type.UNKNOWN);
	}

	// If heuristic is unknown then it *won't* find it as it is just a getter
	public Distance getHeuristic() {
		if (providerAstarNode == null) {
			return new Distance(Distance.Type.EXACT, 0);
		}
		return providerAstarNode.distance;
	}

	public Distance getWeight(int depth) {
		assert getHeuristic().isKnown();
		Distance f = getHeuristic();
		if (f.getType() == Distance.Type.INFINITE) {
			return f;
		}
		return new Distance(Distance.Type.EXACT, f.getValue() + depth);
	}

	@Override
	public String toString() {
		return String.format("H(%s), D(%s)",
				getHeuristic().toString(),
				distance.toString()
		);
	}
}
