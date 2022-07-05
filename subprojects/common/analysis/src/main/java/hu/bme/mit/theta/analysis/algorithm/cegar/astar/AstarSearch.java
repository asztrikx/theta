package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.Map;
import java.util.Set;

public class AstarSearch<S extends State, A extends Action> {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	public Set<AstarNode<S, A>> doneSet = new HashContainerFactory().createSet();
	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	public Map<AstarNode<S, A>, Integer> minWeights = new HashContainerFactory().createMap();;
	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	public Map<ArgNode<S, A>, ArgNode<S, A>> parents = new HashContainerFactory().createMap();
	private Waitlist<Edge<S, A>> waitlist = PriorityWaitlist.create(new AstarWaitlistComparator<>());
	// debug
	public Map<ArgNode<S, A>, Integer> depths = new HashContainerFactory().createMap();

	public void addToWaitlist(AstarNode<S, A> astarNode, AstarNode<S, A> parentAstarNode, int depth) {
		if (astarNode.getHeuristic().getType() == Distance.Type.INFINITE) {
			return;
		}

		if (astarNode.distance.getType() == Distance.Type.INFINITE) {
			return;
		}

		if (!doneSet.contains(astarNode)) {
			Distance distance = astarNode.getWeight(depth);
			if (!minWeights.containsKey(astarNode) || minWeights.get(astarNode) > distance.getValue()) {
				waitlist.add(new Edge<>(parentAstarNode, astarNode, depth));
				parents.put(astarNode.argNode, parentAstarNode == null ? null : parentAstarNode.argNode);
				depths.put(astarNode.argNode, depth);
			}
		} else {
			assert depths.get(astarNode.argNode) <= depth;
		}
	}

	public boolean isWaitlistEmpty() {
		return waitlist.isEmpty();
	}

	public Edge<S, A> removeFromWaitlist() {
		return waitlist.remove();
	}

	public static class Edge<S extends State, A extends Action> {
		public @Nullable AstarNode<S, A> start; // TODO this is redundant as we can get it from parents however this is faster
		public AstarNode<S, A> end;
		public int depthFromAStartNode;

		// start: can be null when init node is added to waitlist
		public Edge(@Nullable AstarNode<S, A> start, AstarNode<S, A> end, int depthFromAStartNode) {
			this.start = start;
			this.end = end;
			this.depthFromAStartNode = depthFromAStartNode;
		}
	}
}