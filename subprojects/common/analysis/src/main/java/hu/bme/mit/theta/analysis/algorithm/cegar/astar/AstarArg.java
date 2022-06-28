package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarArg<S extends State, A extends Action, P extends Prec> {
	public final ARG<S, A> arg;
	public P prec;
	public @Nullable AstarArg<S, A, P> provider;

	// contains init nodes as well
	//  TODO use partition
	private Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
	private Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
	private final PartialOrd<S> partialOrd;
	// Covering ArgNode is searched from here
	public final Partition<ArgNode<S, A>, ?> reachedSet;

	public AstarArg(
			final ARG<S, A> arg, P prec, final PartialOrd<S> partialOrd,
			final Function<? super S, ?> projection
	) {
		this.arg = checkNotNull(arg);
		this.prec = prec;
		this.partialOrd = checkNotNull(partialOrd);
		this.reachedSet = Partition.of(n -> projection.apply(n.getState()));
	}

	public void setUnknownDistanceInfinite() {
		getAll().values().forEach(astarNode -> {
			if (astarNode.distance.getType() != Distance.Type.EXACT) {
				astarNode.distance = new Distance(Distance.Type.INFINITE);
			}
		});
	}

	// parents: node -> parent
	public void updateDistancesFromTargetUntil(
			AstarNode<S, A> target, Set<ArgNode<S, A>> until, Map<ArgNode<S, A>, ArgNode<S, A>> parents
	) {
		//// TODO: rewrite comment
		//// A* property allows us to say that all nodes which was *involved in the search* and reaches target are
		//// the closest to that target

		//// walk up the tree: middle nodes expansion were a subset of waitlist therefore we know their distance
		////  do not follow *all* covering edge as those node were not involved in search
		////  however the path could go through several covering nodes

		int startDistance;
		if (target.distance.getType() == Distance.Type.EXACT) {
			startDistance = target.distance.getValue();
		} else {
			startDistance = 0;
		}

		arg.walkUpParents(target.argNode, parents, (node, distance) -> {
			AstarNode<S, A> astarNode = get(node);

			// Multiple targets can be visited during a check, therefore we should only keep distances from the first found target
			if (astarNode.distance.getType() != Distance.Type.EXACT) {
				astarNode.distance = new Distance(Distance.Type.EXACT, distance + startDistance);
				return until.contains(node);
			} else {
				assert astarNode.distance.getValue() <= distance + startDistance;

				if (target.argNode != node) {
					return true;
				} else {
					// We would not start a search from a node with known distance
					assert !until.contains(node);
					return false;
				}

				// We can skip once we reach a known distance:
				//  - if it was within the same search then parents is the same
				//      - for common nodes we walk up through the same nodes
				//      - those nodes only have distance by reaching at least once the target in this search
				//      - otherwise the distance was already there and we wouldn't explore here
				//      - therefore all nodes above the first node with distance have a distance
				//  - if it wasn't then we would stop when we reach a node with known distance therefore this scenario is not possible
			}
		});
	}

	public void updateDistancesFromRootInfinite(AstarNode<S, A> from) {
		arg.walk(List.of(from.argNode), (argNode, distance) -> {
			AstarNode<S, A> astarNode = get(argNode);

			// if we reach a part where target is reachable then root should also reach it
			assert(astarNode.distance.getType() != Distance.Type.EXACT);
			astarNode.distance = new Distance(Distance.Type.INFINITE);

			return false;
		});
	}

	// After underlying ARG is pruned we must remove AstarNodes corresponding to pruned ArgNodes
	public void pruneApply() {
		// Collect remained init nodes
		final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodesNew = new HashContainerFactory().createMap();
		arg.getInitNodes().forEach(initNode -> {
			final AstarNode<S, A> astarInitNode = astarInitNodes.get(initNode);
			assert astarInitNode != null;
			astarInitNodesNew.put(initNode, astarInitNode);
		});
		astarInitNodes = astarInitNodesNew;

		// Collect remained nodes
		final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodesNew = new HashContainerFactory().createMap();
		reachedSet.clear();

		arg.walk(arg.getInitNodes().collect(Collectors.toList()), (argNode, distance) -> {
			final AstarNode<S, A> astarNode = astarNodes.get(argNode);
			assert astarNode != null;
			astarNodesNew.put(argNode, astarNode);
			reachedSet.add(argNode);
			return false;
		});
		astarNodes = astarNodesNew;
	}

	public AstarNode<S, A> createSuccAstarNode(ArgNode<S, A> argNode, AstarNode<S, A> parentAstarNode) {
		AstarNode<S, A> providerAstarNode = findProviderAstarNode(argNode, parentAstarNode, provider);
		AstarNode<S, A> astarNode = new AstarNode<>(argNode, providerAstarNode);
		put(astarNode);
		reachedSet.add(argNode);
		return astarNode;
	}

	public AstarNode<S, A> createInitAstarNode(ArgNode<S, A> initArgNode) {
		AstarNode<S, A> providerNode = findProviderAstarNode(initArgNode, null, provider);
		AstarNode<S, A> newInitAstarNode = new AstarNode<>(initArgNode, providerNode);
		putInit(newInitAstarNode);
		reachedSet.add(newInitAstarNode.argNode);
		return newInitAstarNode;
	}

	// parentAstarNode: can be null when argNode is an init node
	private AstarNode<S, A> findProviderAstarNode(
			ArgNode<S, A> argNode,
			@Nullable AstarNode<S, A> parentAstarNode,
			AstarArg<S, A, P> providerAstarArg
	) {
		// No previous arg therefore no provider node
		if (providerAstarArg == null) {
			return null;
		}

		// Parent of argNode should be given
		assert !providerAstarArg.containsArg(argNode); // TODO maybe change to astarArg

		Stream<ArgNode<S, A>> providerCandidates;

		// Init nodes don't have parents
		if (parentAstarNode == null) {
			providerCandidates = providerAstarArg.getAllInitArg().stream();
		} else {
			AstarNode<S, A> parentProviderAstarNode = parentAstarNode.providerAstarNode;
			ArgNode<S, A> parentProviderNode = parentProviderAstarNode.argNode;

			// parentAstarNode had to be in waitlist
			// 		therefore it's heuristic is known
			// 		therefore it's provider's distance is known
			assert parentProviderAstarNode.distance.isKnown();
			// 		therefore it's heuristic is known
			//		therefore it must be expanded or covered (targets are also expanded)
			assert parentProviderNode.isExpanded() || parentProviderNode.isCovered();

			// Even if we checked provider node before setting it whether it is covered,
			// later it still can be covered if it's not yet expanded.
			// The node's children are the covering node's children.
			// There is no guarantee that covering node has been expanded or has distance, we can only know it for the covered node
			//	 The only case when this can happen is when covered node is a target otherwise
			//	 we continue the search in covering node which would make it expanded would also receive a distance.
			//	 That case is handled by expanded target when found.
			// Covering node chains are compressed therefore covering node can't have covering node.
			if (parentProviderNode.getCoveringNode().isPresent()) {
				ArgNode<S, A> parentProviderCoveringNode = parentProviderNode.getCoveringNode().get();
				// Optimization: only handle covering case once
				// We can't do this when setting parentAstarNode's provider node as covering can happen after that
				// This change will affect visualizer midpoint
				parentProviderAstarNode = parentAstarNode.providerAstarNode = providerAstarArg.get(parentProviderCoveringNode);
				parentProviderNode = parentProviderAstarNode.argNode;

				assert parentProviderAstarNode.distance.isKnown();
				assert parentProviderNode.isExpanded();
			}

			providerCandidates = parentProviderNode.getOutEdges().map(ArgEdge::getTarget);
		}

		// filter based on partialOrd: isLeq == "<=" == subset of
		providerCandidates = providerCandidates.filter(providerCandidate ->
				partialOrd.isLeq(argNode.getState(), providerCandidate.getState())
		);
		Optional<ArgNode<S,A>> providerNode = providerCandidates.findAny();
		assert providerNode.isPresent();
		return providerAstarArg.get(providerNode.get());
	}

	/// ARG Wrappers

	public Stream<AstarNode<S, A>> getIncompleteNodes() {
		return arg.getIncompleteNodes().map(this::get);
	}

	/// Collection wrappers

	public void put(final AstarNode<S, A> astarNode) {
		astarNodes.put(astarNode.argNode, astarNode);
	}

	public boolean containsArg(final ArgNode<S, A> argNode) {
		return astarNodes.containsKey(argNode);
	}

	public AstarNode<S, A> get(final ArgNode<S, A> argNode) {
		return astarNodes.get(argNode);
	}

	public void putAll(final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodeMap) {
		astarNodes.putAll(astarNodeMap);
	}

	public Map<ArgNode<S, A>, AstarNode<S, A>> getAll() {
		return astarNodes;
	}

	public Collection<AstarNode<S, A>> getAllInit() {
		return astarInitNodes.values();
	}

	public Collection<ArgNode<S, A>> getAllInitArg() {
		return astarInitNodes.keySet();
	}

	/*public void putAllInitNode(final Map<ArgNode<S, A>, AstarNode<S, A>> mapping) {
		astarInitNodes.putAll(mapping);
		astarNodes.putAll(mapping);
	}*/

	public void putInit(final AstarNode<S, A> astarInitNode) {
		astarInitNodes.put(astarInitNode.argNode, astarInitNode);
		astarNodes.put(astarInitNode.argNode, astarInitNode);
	}
}
