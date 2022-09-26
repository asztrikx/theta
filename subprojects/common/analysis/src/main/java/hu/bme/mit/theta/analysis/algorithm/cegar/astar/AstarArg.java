package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.common.Tuple2;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarArg<S extends State, A extends Action, P extends Prec> {
	public final ARG<S, A> arg;
	public P prec;
	public @Nullable AstarArg<S, A, P> provider;

	// contains init nodes as well
	private Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
	private Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
	private final PartialOrd<S> partialOrd;
	// Covering ArgNode is searched from here
	public final Partition<ArgNode<S, A>, ?> reachedSet; // TODO check whether reachset can hold AstarNodes

	public AstarArg(
			final ARG<S, A> arg, P prec, final PartialOrd<S> partialOrd,
			final Function<? super S, ?> projection
	) {
		this.arg = checkNotNull(arg);
		this.prec = prec;
		this.partialOrd = checkNotNull(partialOrd);
		this.reachedSet = Partition.of(n -> projection.apply(n.getState()));
	}

	// Propagate exact distance up from target until a node in a set is reached
	// parents: node -> parent
	public void updateDistancesFromTargetUntil(
			AstarNode<S, A> from, Set<ArgNode<S, A>> until, Map<ArgNode<S, A>, ArgNode<S, A>> parents
	) {
		int startDistance;
		if (from.getDistance().getType() == Distance.Type.EXACT) {
			startDistance = from.getDistance().getValue();
		} else {
			startDistance = 0;
		}

		// Represents all paths starting from a node where we can only set distance for a node in this path
		// if it only has a single child. Distance value is derived from the child by incrementing it by one.
		List<Tuple2<ArgNode<S, A>, Integer>> conditionalNodes = new ArrayList<>();

		// [Multitarget] if we cover into a subgraph explored in this iteration we should have two parents (we can update distances up to the roots)

		// Parents is good even when we have a covered node pointing to a node with an exact distance
		// as that exact distance can only be from a previous iteration
		// as we set distances at the end of the iterations
		// so the parent of the covering node will be the covered node.
		// ([Multitarget] would obselete this comment with multiple parents, still keep it)
		arg.walkUpParents(from.argNode, parents::get, (node, distance) -> {
			AstarNode<S, A> astarNode = get(node);
			distance = distance + startDistance;

			// We expand targets therefore we can have a target ancestor.
			if (node.isTarget() && from.argNode != node) {
				assert astarNode.getDistance().isKnown();
			}

			// Multiple targets can be visited during a check, therefore we should only keep distances from the first found target
			if (astarNode.getDistance().getType() != Distance.Type.EXACT) {
				assert !astarNode.getDistance().isKnown();
				astarNode.setDistance(new Distance(Distance.Type.EXACT, distance));

				// Save covered nodes
				Integer lambdaDistance = distance;
				ArgNode<S, A> parentAstarNode = parents.get(node);
				List<Tuple2<ArgNode<S, A>, Integer>> nonParentCoveredNodes = node.getCoveredNodes()
						// Node's parent is a covered node, do not call updateDistancesFromNodes as it's distance will be known
						.filter(coveredNode -> coveredNode != parentAstarNode)
						.map(coveredNode -> {
							assert !get(coveredNode).getDistance().isKnown();
							return Tuple2.of(coveredNode, lambdaDistance);
						})
						.toList();
				conditionalNodes.addAll(nonParentCoveredNodes);

				// Parent is a covered node
				// therefore graph parent's children may all have distance after setting this node (one of its child).
				// This case is handled by conditionalNodes list.
				if (node.getCoveredNodes().count() != nonParentCoveredNodes.size()) {
					// Can be an init node.
					if (node.getParent().isPresent()) {
						ArgNode<S, A> graphParent = node.getParent().get();
						AstarNode<S, A> astarGraphParent = get(graphParent);
						// Graph parent may already have distance
						if (!astarGraphParent.getDistance().isKnown()) {
							conditionalNodes.add(Tuple2.of(graphParent, Integer.MIN_VALUE));
						}
					}
				}

				return until.contains(node);
			} else {
				if (from.argNode != node) {
					assert astarNode.getDistance().getValue() <= distance;
					return true;
				} else {
					//// [multiparent] this maybe false as startNode may just hava a distance from different target
					//// Upperlimit case
					assert !until.contains(node);
					return false;
				}

				// TODO recheck this
				// We can skip once we reach a known distance:
				//  - if it was within the same search then parents is the same
				//      - for common nodes we walk up through the same nodes
				//      - those nodes only have distance by reaching at least once the target in this search
				//      - otherwise the distance was already there and we wouldn't explore here
				//      - therefore all nodes above the first node with distance have a distance
				//  - if it wasn't then we would stop when we reach a node with known distance therefore this scenario is not possible
			}
		});

		updateDistancesFromConditionalNodes(conditionalNodes);
	}

	// Propagate exact distance up
	// nodes: all should be either covered or have > 0 children
	public void updateDistancesFromConditionalNodes(List<Tuple2<ArgNode<S, A>, Integer>> nodes) {
		// TODO reword, recheck
		// distance can already be known:
		//	parents: c's is b, b's is a
		//  updateDistancesFromTargetUntil calls with a as in general case inedge's source is not the covering node's ancestor
		//  see comment in that function
		//      a
		//    /  \
		//   b- ->c
		nodes = nodes.stream().filter(t -> !get(t.get1()).getDistance().isKnown()).toList();
		Queue<Tuple2<ArgNode<S, A>, Integer>> queue = new ArrayDeque<>(nodes);

		// We also know the distance to all covered nodes as there is no other path for shorter distance.
		// TODO reword
		// Also all ancestors until one has an unknown dist child
		while (!queue.isEmpty()) {
			var item = queue.remove();
			ArgNode<S, A> coveredNode = item.get1();
			int distanceBase = item.get2();
			arg.walkUpParents(coveredNode, node -> {
				if (node.getParent().isPresent()) {
					return node.getParent().get();
				} else {
					return null;
				}
			}, (node, _distance) -> {
				AstarNode<S, A> astarNode = get(node);

				if (node.isTarget()) {
					assert astarNode.getDistance().getType() == Distance.Type.EXACT;
				}
				// Covered nodes are added to the queue and arg.walkUpParents is started from them
				if (node.isCovered()) {
					assert node == coveredNode && _distance == 0;
				} else {
					assert node.getSuccNodes().findAny().isPresent();
				}

				// This can't be the start node as we filter the ones with a distance.
				if (astarNode.getDistance().getType() == Distance.Type.EXACT) {
					assert !node.isCovered();
					// The previously set node will have an exact distance therefore get() won't fail.
					Distance minDistance = node.getSuccNodes().map(child -> get(child).getDistance()).filter(Distance::isKnown).min(Distance::compareTo).get();
					if (node.isTarget()) {
						assert astarNode.getDistance().getValue() == 0;
					} else {
						assert astarNode.getDistance().getValue() == minDistance.getValue() + 1;
					}
					return true;
				}

				// Is the node covered
				if (node.getCoveringNode().isPresent()) {
					astarNode.setDistance(new Distance(Distance.Type.EXACT, distanceBase));
				} else {
					if (!allSuccDistanceKnown(node)) {
						return true;
					}

					// e.g. Other child can also have been in queue: it stopped at this node as it had a child with unknown distance
					// therefore we manually have to choose the one with better distance
					Distance minDistance = node.getSuccNodes().map(child -> get(child).getDistance()).min(Distance::compareTo).get();
					assert minDistance.getType() == Distance.Type.EXACT;
					astarNode.setDistance(new Distance(Distance.Type.EXACT, minDistance.getValue() + 1));
				}
				// astarNode.distance != _distance, see above
				Integer lambdaDistance = astarNode.getDistance().getValue();
				queue.addAll(node.getCoveredNodes().map(newCoveredNode -> Tuple2.of(newCoveredNode, lambdaDistance)).toList());

				return false;
			});
		}
	}

	public void updateDistancesFromRootInfinite(AstarNode<S, A> from) {
		Queue<ArgNode<S, A>> queue = new ArrayDeque<>();
		queue.add(from.argNode);

		while (!queue.isEmpty()) {
			var item = queue.remove();
			arg.walk(List.of(item), (argNode, distance) -> {
				AstarNode<S, A> astarNode = get(argNode);

				// if we reach a part where target is reachable then root should also reach it
				assert astarNode.getDistance().getType() != Distance.Type.EXACT;
				assert argNode.isCovered() || argNode.isExpanded() || astarNode.getHeuristic().getType() == Distance.Type.INFINITE;
				astarNode.setDistance(new Distance(Distance.Type.INFINITE));

				return false;
			}, arg::walkDefault);
		}
	}

	// Set all nodes which will never reach target infinite
	public void updateDistanceInfinite() {
		Queue<ArgNode<S, A>> queue = new ArrayDeque<>();
		Predicate<ArgNode<S, A>> excludeKnownDistance = node -> {
			// Can't reach target && not already marked as infinite
			return !get(node).getDistance().isKnown();
		};
		Predicate<ArgNode<S, A>> excludeTarget = node -> !node.isTarget();

		// Naive solution would be to set all nodes reachable from startNodes infinite when startNodes did not reach
		// any target, but that would lead to the following case:
		// 		b,d,f: targets
		//		-: nodes don't matter in this case
		//		we call findHeuristic on a then c then e
		//		e-g is visited 3 times
		//
		//	The edges closer to the left side will enter waitlist first
		//     a
		//    | \
		//    -  c
		//	  | | \
		//	  - -  e
		//	  | | | \
		//	  - - -  g (no more children)
		//    | | |
		//    b d f

		// Not a target, Doesn't have distance
		//	- covered
		//		1) covered to ancestor: not handled
		//		- not covered to ancestor
		//			2) coverer marked as infinite
		//			3) coverer not marked as infinite (will be added in while loop if it gets marked as infinite)
		//	- not covered
		//		- expanded
		//			4) leaf (non leaves will be added by their leaves as they must also be infinite)
		//		- not expanded
		//			5) has infinite heuristic

		// 2) Covered, coverer marked as infinite
		// Node can get covered into already infinite node, therefore it won't be set to infinite
		Stream<ArgNode<S, A>> lateCoveredNodes = arg.getCoveredNodes().filter(coveredNode -> {
			assert coveredNode.getCoveringNode().isPresent();
			ArgNode<S, A> covererNode = coveredNode.getCoveringNode().get();
			AstarNode<S, A> astarCovererNode = get(covererNode);
			return astarCovererNode.getDistance().getType() == Distance.Type.INFINITE;
		});
		queue.addAll(lateCoveredNodes.filter(excludeKnownDistance).toList());

		// 4) Not covered > expanded > leaf
		queue.addAll(arg.getCompleteLeafNodes().filter(excludeKnownDistance.and(excludeTarget)).toList());

		// 5) Not covered > not expanded
		// AstarNode's with infinite heuristic distances won't be expanded therefore they won't get set distance inf
		Stream<ArgNode<S, A>> infiniteHeuristicNodes = astarNodes.values().stream()
				.filter(astarNode -> astarNode.getHeuristic().getType() == Distance.Type.INFINITE)
				.filter(astarNode -> {
					ArgNode<S, A> argNode = astarNode.argNode;
					// Infinite heuristic can be a leaf (therefore expanded) it was copied from previous iteration
					if (argNode.isExpanded()) {
						// TODO if we stop copying infinite subgraph then uncomment this
						//assert argNode.isLeaf();
						assert astarNode.providerAstarNode != null && argNode.toString().equals(astarNode.providerAstarNode.argNode.toString());
					}
					//assert !argNode.isCovered(); // TODO if we stop copying infinite subgraph then uncomment this
					return !argNode.isExpanded() && !argNode.isCovered();
				})
				.map(astarNode -> astarNode.argNode);

		queue.addAll(infiniteHeuristicNodes.filter(excludeKnownDistance).toList());

		// TODO filter heuristic known and is target here

		// The cases should be disjoint
		assert queue.size() == new HashSet<>(queue).size();

		while (!queue.isEmpty()) {
			ArgNode<S, A> argNode = queue.remove();

			// Parent nodes with non inf heuristic can be infinite distance after refinement
			arg.walkUpParents(argNode, node -> {
				if (node.getParent().isPresent()) {
					return node.getParent().get();
				} else {
					return null;
				}
			}, (node, _distance) -> {
				AstarNode<S, A> astarNode = get(node);

				// We can have target ancestor
				if (node.isTarget()) {
					// target may or may not have a distance
					return true;
				}

				// When updating nodes reaching a target, they can already have distance without all children having distance
				if (astarNode.getDistance().isKnown()) {
					if (astarNode.getDistance().getType() == Distance.Type.INFINITE) {
						assert astarNode.getHeuristic().getType() == Distance.Type.INFINITE;
					}
					return true;
				}

				// Covered nodes added to queue have covering node with infinite distance.
				// Nodes with infinite heuristic are not expanded therefore we have to check it before allSuccNodeDistanceInfinite.
				if (node.isCovered() || astarNode.getHeuristic().getType() == Distance.Type.INFINITE || allSuccNodeDistanceInfinite(node)) {
					// Infinite distance can't be set until all children have infinite distance.
					// We won't revisit infinite paths so allSuccNodeDistanceInfinite will only be true 1 time.
					assert !astarNode.getDistance().isKnown();
					astarNode.setDistance(new Distance(Distance.Type.INFINITE));

					// 3) Add covered nodes to queue
					node.getCoveredNodes().forEach(coveredNode -> {
						assert !get(coveredNode).getDistance().isKnown();
						queue.add(coveredNode);
					});

					return false;
				} else {
					// updateDistanceFromNodes depends on all children having a known distance,
					// and some nodes' infinite distances may only be discovered when calling findDistance again
					// therefore we have to call updateDistanceFromNodes because exact distances are only set from a target
					if (allSuccDistanceKnown(node)) {
						// distance won't be used as node is not covered
						updateDistancesFromConditionalNodes(List.of(Tuple2.of(node, Integer.MIN_VALUE)));
					}

					return true;
				}
			});
		}
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

		arg.walk(arg.getInitNodes().toList(), (argNode, distance) -> {
			final AstarNode<S, A> astarNode = astarNodes.get(argNode);
			assert astarNode != null;
			astarNodesNew.put(argNode, astarNode);
			reachedSet.add(argNode);
			return false;
		}, arg::walkDefault);
		astarNodes = astarNodesNew;
	}

	public AstarNode<S, A> createSuccAstarNode(ArgNode<S, A> argNode, AstarNode<S, A> parentAstarNode) {
		AstarNode<S, A> providerAstarNode = findProviderAstarNode(argNode, parentAstarNode, provider);
		AstarNode<S, A> astarNode = new AstarNode<>(argNode, providerAstarNode);
		put(astarNode);
		return astarNode;
	}

	public AstarNode<S, A> createInitAstarNode(ArgNode<S, A> initArgNode) {
		AstarNode<S, A> providerAstarNode = findProviderAstarNode(initArgNode, null, provider);
		AstarNode<S, A> newInitAstarNode = new AstarNode<>(initArgNode, providerAstarNode);
		putInit(newInitAstarNode);
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
			assert argNode.isInit();
			providerCandidates = providerAstarArg.getAllInitArg().stream();
		} else {
			AstarNode<S, A> parentProviderAstarNode = parentAstarNode.providerAstarNode;
			ArgNode<S, A> parentProviderNode = parentProviderAstarNode.argNode;

			// parentAstarNode had to be in waitlist
			// 		therefore it's heuristic is known
			// 		therefore it's provider's distance is known
			assert parentProviderAstarNode.getDistance().isKnown();
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

				assert parentProviderAstarNode.getDistance().isKnown();
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

	// Node can't be covered.
	// If node is not expanded it will return false.
	public boolean allSuccDistanceKnown(ArgNode<S, A> argNode) {
		assert !argNode.isCovered();
		if (!argNode.isExpanded()) {
			return false;
		}
		return argNode.getSuccNodes().allMatch(child -> get(child).getDistance().isKnown());
	}

	public boolean allSuccNodeDistanceInfinite(ArgNode<S, A> argNode) {
		assert !argNode.isCovered();
		if (!argNode.isExpanded()) {
			return false;
		}
		return argNode.getSuccNodes().allMatch(childNode -> {
			AstarNode<S, A> childAstarNode = get(childNode);
			return childAstarNode.getDistance().getType() == Distance.Type.INFINITE;
		});
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
