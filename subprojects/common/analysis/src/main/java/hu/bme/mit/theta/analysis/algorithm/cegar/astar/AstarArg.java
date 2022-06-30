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
import java.util.stream.Collectors;
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

	// Propagate exact distance up from target until a node in a set is reached
	// parents: node -> parent
	public void updateDistancesFromTargetUntil(
			AstarNode<S, A> target, Set<ArgNode<S, A>> until, Map<ArgNode<S, A>, ArgNode<S, A>> parents
	) {
		int startDistance;
		if (target.distance.getType() == Distance.Type.EXACT) {
			startDistance = target.distance.getValue();
		} else {
			startDistance = 0;
		}

		List<Tuple2<ArgNode<S, A>, Integer>> coveredNodes = new ArrayList<>();

		arg.walkUpParents(target.argNode, parents::get, (node, distance) -> {
			AstarNode<S, A> astarNode = get(node);
			distance = distance + startDistance;

			// Multiple targets can be visited during a check, therefore we should only keep distances from the first found target
			if (astarNode.distance.getType() != Distance.Type.EXACT) {
				astarNode.distance = new Distance(Distance.Type.EXACT, distance);

				// Save covered nodes
				Integer lambdaDistance = distance;
				Stream<Tuple2<ArgNode<S, A>, Integer>> coveredNodesStream = astarNode.argNode.getCoveredNodes().map(coveredNode -> {
					assert get(coveredNode).distance.getType() == Distance.Type.UNKNOWN;
					return Tuple2.of(coveredNode, lambdaDistance);
				});
				coveredNodes.addAll(coveredNodesStream.toList());

				return until.contains(node);
			} else {
				assert astarNode.distance.getValue() <= distance;

				// TODO we also can set distance to covered node and set it as target, then we would not have this edge case
				if (target.argNode != node) {
					return true;
				} else {
					// We would not start a search from a node with known distance
					// therefore it must be a node which we covered into and got upperlimit
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

		updateDistancesFromNodes(coveredNodes);
	}

	// Propagate exact distance up
	// nodes: all should be either covered or have > 0 children and must not have a distance already
	public void updateDistancesFromNodes(List<Tuple2<ArgNode<S, A>, Integer>> nodes) {
		assert nodes.stream().noneMatch(t -> get(t.get1()).distance.isKnown());
		Queue<Tuple2<ArgNode<S, A>, Integer>> queue = new ArrayDeque<>(nodes);

		// We also know the distance to all covered nodes as they can't reach target otherwise
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

				if (astarNode.distance.getType() == Distance.Type.EXACT) {
					assert _distance + distanceBase >= astarNode.distance.getValue();
					return true;
				}

				// Is the node covered
				if (node.getCoveredNodes().findAny().isPresent()) {
					assert _distance == 0;
					astarNode.distance = new Distance(Distance.Type.EXACT, distanceBase);
				} else {
					assert node.getSuccNodes().findAny().isPresent();
					if (!allSuccDistanceKnown(node)) {
						return true;
					}

					// Other child can also have been in queue: it stopped at this node as it had a child with unknown distance
					// therefore we manually have to choose the one with better distance
					// TODO this case also happens when we expand a new arg from unexpanded nodes (update distance only goes to start nodes not all the way until an init node)
					Distance minDistance = node.getSuccNodes().map(child -> get(child).distance).min(Distance::compareTo).get();
					assert minDistance.getType() == Distance.Type.EXACT;
					astarNode.distance = new Distance(Distance.Type.EXACT, minDistance.getValue() + 1);
				}
				// astarNode.distance != distance, see above
				Integer lambdaDistance = astarNode.distance.getValue();
				queue.addAll(node.getCoveredNodes().map(newCoveredNode -> Tuple2.of(newCoveredNode, lambdaDistance)).toList());

				return false;
			});
		}
	}

	// Set all nodes which will never reach target infinite
	public void updateDistanceInfinite() {
		Queue<ArgNode<S, A>> queue = new ArrayDeque<>();
		Predicate<ArgNode<S, A>> excludeByDistance = node -> {
			AstarNode<S, A> astarNode = get(node);
			// Can't reach target && not already marked as infinite
			return astarNode.distance.getType() == Distance.Type.UNKNOWN;
		};

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

		// All cases
		//	- visited
		//		- covered
		//			- covered to ancestor (handled in queue)
		//			- not covered to ancestor
		//				- covered marked as infinite
		//				- covered not marked as infinite (will be added in while loop (if can't reach target))
		//		- not covered
		//			- leaf
		//	- not visited
		//		- has infinite heuristic

		// Node can get covered into already infinite node, therefore it won't be set to infinite
		Stream<ArgNode<S, A>> lateCoveredNodes = arg.getCoveredNodes().filter(coveredNode -> {
			assert coveredNode.getCoveringNode().isPresent();
			ArgNode<S, A> covererNode = coveredNode.getCoveringNode().get();
			AstarNode<S, A> astarCovererNode = get(covererNode);
			return astarCovererNode.distance.getType() == Distance.Type.INFINITE;
		});
		queue.addAll(lateCoveredNodes.filter(excludeByDistance).toList());

		// Not covered leaf
		queue.addAll(arg.getCompleteLeafNodes().filter(excludeByDistance).toList());

		// AstarNode's with infinite heuristic distances won't be expanded therefore they won't get set distance inf
		Stream<ArgNode<S, A>> infiniteHeuristicNodes = astarNodes.values().stream()
				.filter(astarNode -> astarNode.getHeuristic().getType() == Distance.Type.INFINITE)
				.map(astarNode -> astarNode.argNode);
		queue.addAll(infiniteHeuristicNodes.filter(excludeByDistance).toList());

		while (!queue.isEmpty()) {
			ArgNode<S, A> argNode = queue.remove();
			arg.walkUpParents(argNode, node -> {
				if (node.getParent().isPresent()) {
					return node.getParent().get();
				} else {
					return null;
				}
			}, (node, _distance) -> {
				AstarNode<S, A> astarNode = get(node);

				// From target, we can already have distance without to wait for all children to have distance
				if (astarNode.distance.isKnown()) {
					assert astarNode.distance.getType() != Distance.Type.INFINITE;
					assert astarNode.distance.getType() == Distance.Type.EXACT;
					return true;
				}

				// Add covered nodes to queue
				node.getCoveredNodes().forEach(coveredNode -> {
					assert !get(coveredNode).distance.isKnown();
					queue.add(coveredNode);
				});

				// Covered nodes added to queue have covering node with infinite distance
				// TODO for first node we are checking but maybe these check filter out some startNodes
				if (node.isCovered() || allSuccNodeDistanceInfinite(node)) {
					assert astarNode.distance.getType() == Distance.Type.UNKNOWN;
					astarNode.distance = new Distance(Distance.Type.INFINITE);

					return false;
				} else {
					// updateDistanceFromNodes depends on all children having a known distance,
					// and nodes with infinite distances are only set later therefore we have to recall updateDistanceFromNodes
					if (allSuccDistanceKnown(node)) {
						// distance won't be used as node is not covered
						updateDistancesFromNodes(List.of(Tuple2.of(node, Integer.MIN_VALUE)));
					}

					return true;
				}
			});

			if (queue.isEmpty()) {
				// Covered to ancestor still can reach target TODO
				// 		Mivel T-t később is megtalálhatja így nem tudjuk biztosra, meg kéne nézni h expanded-e minden leszármazott vagy coverelt
				//       |
				//       a <- -
				//       |      \
				//       b      |
				//     / |      |
				//    T  c - - /
				//
				//       |
				//       a <- -
				//     / |      \
				//    d  b      |
				//       |      |
				//       c - - /
				// case
				//queue.addAll(arg.getAncestorCoveredNodes().filter(excludeByDistance).toList());

			}
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
		AstarNode<S, A> providerAstarNode = findProviderAstarNode(initArgNode, null, provider);
		AstarNode<S, A> newInitAstarNode = new AstarNode<>(initArgNode, providerAstarNode);
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
			// TODO for loop? comments may get messy
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

	public boolean allSuccDistanceKnown(ArgNode<S, A> argNode) {
		assert !argNode.isCovered();
		if (!argNode.isExpanded()) {
			return false;
		}
		return argNode.getSuccNodes().allMatch(child -> get(child).distance.isKnown());
	}

	public boolean allSuccNodeDistanceInfinite(ArgNode<S, A> argNode) {
		assert !argNode.isCovered();
		if (!argNode.isExpanded()) {
			return false;
		}
		return argNode.getSuccNodes().allMatch(childNode -> {
			AstarNode<S, A> childAstarNode = get(childNode);
			return childAstarNode.distance.getType() == Distance.Type.INFINITE;
		});
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
