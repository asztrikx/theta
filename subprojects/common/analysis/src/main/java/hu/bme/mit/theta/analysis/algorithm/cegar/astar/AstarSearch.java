package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.ArrayDeque;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

public class AstarSearch<S extends State, A extends Action> {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	private Set<AstarNode<S, A>> doneSet = new HashContainerFactory().createSet();
	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	private Map<AstarNode<S, A>, Integer> minWeights = new HashContainerFactory().createMap();;
	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	public Map<ArgNode<S, A>, ArgNode<S, A>> parents = new HashContainerFactory().createMap();
	private Waitlist<Edge<S, A>> waitlist = PriorityWaitlist.create(new AstarWaitlistComparator<>());
	// We might reach a node with known distance making an upper limit for the closest target
	public int upperLimitValue = -1;
	public AstarNode<S, A> upperLimitAstarNode = null;
	// debug
	public Map<ArgNode<S, A>, Integer> depths = new HashContainerFactory().createMap();
	public Queue<AstarNode<S, A>> reachedExacts = new ArrayDeque<>();

	public void addToWaitlist(AstarNode<S, A> astarNode, AstarNode<S, A> parentAstarNode, int depth) {
		if (astarNode.getHeuristic().getType() == Distance.Type.INFINITE) {
			return;
		}

		if (astarNode.getDistance().getType() == Distance.Type.INFINITE) {
			return;
		}

		assert astarNode.getHeuristic().isKnown();

		// When is this possible:
		//   - in a different subgraph reached by covering edge
		//   - same subgraph which was reached from a different subgraph by a covering edge
		// We have a target in x distance therefore we have an upper bound
		// Node can already be marked done therefore
		// Can be in doneSet if not firstCex
		//	- otherwise can move to if statement below
		//	- we can't put into waitlist as it will drop it (in doneSet) otherwise it's an optimization to handle here
		if (astarNode.getDistance().getType() == Distance.Type.EXACT) {
			assert !doneSet.contains(astarNode);

			if (upperLimitValue > depth + astarNode.getDistance().getValue() || upperLimitValue == -1) {
				upperLimitValue = depth + astarNode.getDistance().getValue();
				upperLimitAstarNode = astarNode;
				// Only happens if a startAstarNode already have distance
				assert parentAstarNode != null;
				// [Multi target?]
				// Multiple node can be covered into the same node, therefore in that way it can have multiple parent
				if (parents.containsKey(astarNode.argNode)) {
					assert parents.get(astarNode.argNode).getCoveringNode().get() == astarNode.argNode;
				}
				parents.put(astarNode.argNode, parentAstarNode.argNode);
			}
			return;
		}

		if (!doneSet.contains(astarNode)) {
			Distance distance = astarNode.getWeight(depth);
			if (!minWeights.containsKey(astarNode) || minWeights.get(astarNode) > distance.getValue()) {
				waitlist.add(new Edge<>(astarNode, depth));
				parents.put(astarNode.argNode, parentAstarNode == null ? null : parentAstarNode.argNode);
				depths.put(astarNode.argNode, depth);
				minWeights.put(astarNode, distance.getValue());
			}
		} else {
			assert depths.get(astarNode.argNode) <= depth;
		}
	}

	public boolean isWaitlistEmpty() {
		return waitlist.isEmpty();
	}

	public @Nullable Edge<S, A> removeFromWaitlist() {
		@Nullable Edge<S, A> edge;
		do {
			edge = waitlist.remove();
		} while (edge != null && doneSet.contains(edge.end));

		if (edge != null) {
			doneSet.add(edge.end);
		}

		return edge;
	}

	public static class Edge<S extends State, A extends Action> {
		public AstarNode<S, A> end;
		public int depthFromAStartNode;

		public Edge(AstarNode<S, A> end, int depthFromAStartNode) {
			this.end = end;
			this.depthFromAStartNode = depthFromAStartNode;
		}
	}
}
