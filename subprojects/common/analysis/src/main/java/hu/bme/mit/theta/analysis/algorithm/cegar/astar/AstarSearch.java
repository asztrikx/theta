package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class AstarSearch<S extends State, A extends Action> {
	// We could already have started to explore a subgraph therefore do not use global doneSet variable
	public Set<AstarNode<S, A>> doneSet = new HashSet<>();
	// Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
	public Map<AstarNode<S, A>, Integer> minWeights = new HashMap<>();
	// After we reach target we know the distance for all nodes between root and target which is visitable by parent entries
	// This is needed because with covering edges there can be multiple in-edges
	// node -> parent
	public Map<ArgNode<S, A>, ArgNode<S, A>> parents = new HashContainerFactory().createMap();
	////public Map<AstarNode<S, A>, Integer> depths = new HashContainerFactory().createMap();
	public Waitlist<Edge<S, A>> waitlist = PriorityWaitlist.create(new AstarWaitlistComparator<>());

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