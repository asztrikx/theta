package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge;

import java.util.Comparator;

public class AstarWaitlistComparator<S extends State, A extends Action> implements Comparator<Edge<S, A>> {
	@Override
	public int compare(Edge<S, A> edge1, Edge<S, A> edge2) {
		AstarNode<S, A> astarNode1 = edge1.end;
		AstarNode<S, A> astarNode2 = edge2.end;
		ArgNode<S, A> argNode1 = astarNode1.argNode;
		ArgNode<S, A> argNode2 = astarNode2.argNode;

		final int weight1 = astarNode1.getWeight(edge1.depthFromAStartNode).getValue();
		final int weight2 = astarNode2.getWeight(edge2.depthFromAStartNode).getValue();

		// optimization
		if (weight1 == weight2) {
			if (argNode1.isTarget() && argNode2.isTarget()) {
				return 0;
			} else if (argNode1.isTarget()) {
				return -1;
			} else if (argNode2.isTarget()) {
				return 1;
			}
			return 0;
		}
		return weight1 - weight2;
	}
}
