package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge;

import java.util.Comparator;
import java.util.Map;

public class AstarWaitlistComparator<S extends State, A extends Action> implements Comparator<Edge<S, A>> {
	/*
		// in astarExtend we do not have INFINITE nodes so this part is only for generality
		final boolean unreachable1 = weight1.getType() == AstarNode.DistanceType.INFINITE;
		final boolean unreachable2 = weight2.getType() == AstarNode.DistanceType.INFINITE;

		if (unreachable1 && unreachable2) {
			return 0;
		} else if (unreachable1) {
			return 1;
		} else if (unreachable2) {
			return -1;
		}
	*/

	@Override
	public int compare(Edge<S, A> edge1, Edge<S, A> edge2) {
		// optimization
		if (edge1.end.argNode.isTarget() && edge2.end.argNode.isTarget()) {
			return 0;
		} else if (edge1.end.argNode.isTarget()) {
			return -1;
		} else if (edge2.end.argNode.isTarget()) {
			return 1;
		}

		final Distance weight1 = edge1.end.getWeight(edge1.depthFromAStartNode);
		final Distance weight2 = edge2.end.getWeight(edge2.depthFromAStartNode);
		return weight1.getValue() - weight2.getValue();
	}
}
