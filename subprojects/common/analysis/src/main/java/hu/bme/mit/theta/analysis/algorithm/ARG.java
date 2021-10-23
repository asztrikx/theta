/*
 *  Copyright 2017 Budapest University of Technology and Economics
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package hu.bme.mit.theta.analysis.algorithm;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;
import static com.google.common.base.Preconditions.checkState;
import static java.util.stream.Collectors.toList;

import hu.bme.mit.theta.analysis.waitlist.FifoWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.Containers;

import java.util.Collection;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.BiFunction;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import java.util.ArrayList;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

/**
 * Represents an abstract reachability graph (ARG). See the related class
 * ArgBuilder.
 */
public final class ARG<S extends State, A extends Action> {

	private final Collection<ArgNode<S, A>> initNodes;
	protected boolean initialized; // Set by ArgBuilder
	private int nextId = 0;
	final PartialOrd<S> partialOrd;

	private ARG(final PartialOrd<S> partialOrd) {
		initNodes = Containers.createSet();
		this.partialOrd = partialOrd;
		this.initialized = false;
	}

	public static <S extends State, A extends Action> ARG<S, A> create(final PartialOrd<S> partialOrd) {
		return new ARG<>(partialOrd);
	}

	////

	/**
	 * copies ARG and their ArgNode shallowly (keeping action, state)
	 * this should be checked every time ARG or ArgNode changes
	 */
	@Override
	public Object clone() {
		return cloneWithResult().argCopied;
	}

	public static final class ARGCopyResult<S extends State, A extends Action> {
		public final ARG<S, A> argCopied;
		public final Map<ArgNode<S, A>, ArgNode<S, A>> oldToNew;

		public ARGCopyResult(final ARG<S, A> argCopied, final Map<ArgNode<S, A>, ArgNode<S, A>> oldToNew) {
			this.argCopied = argCopied;
			this.oldToNew = oldToNew;
		}
	}

	public ARGCopyResult<S, A> cloneWithResult() {
		ARG<S, A> arg = new ARG<>(partialOrd);
		arg.initialized = initialized;
		if (!initialized) {
			return new ARGCopyResult<>(arg, new HashContainerFactory().createMap());
		}

		// clone ArgNodes and their connection
		//	don't copy state as it can be large
		//	don't use ArgBuilder as we already know the partially expanded state of ARG
		Map<ArgNode<S, A>, ArgNode<S, A>> oldToNew = new HashContainerFactory().createMap();
		for (ArgNode<S, A> currentInitArgNode: initNodes) {
			ArgNode<S, A> newInitArgNode = arg.createInitNode(currentInitArgNode.getState(), currentInitArgNode.isTarget());
			newInitArgNode.expanded = currentInitArgNode.expanded;
			assert !newInitArgNode.isCovered();

			oldToNew.put(currentInitArgNode, newInitArgNode);
		}

		walk(oldToNew.keySet(), (currentArgNode, distance) -> {
			ArgNode<S, A> newArgNode = oldToNew.get(currentArgNode);
			assert newArgNode != null;

			// create new children from old node
			currentArgNode.getOutEdges().forEach((ArgEdge<S, A> argEdge) -> {
				ArgNode<S, A> currentSuccArgNode = argEdge.getTarget();
				ArgNode<S, A> newSuccArgNode = arg.createSuccNode(newArgNode, argEdge.getAction(), currentSuccArgNode.getState(), currentSuccArgNode.isTarget());
				newSuccArgNode.expanded = currentSuccArgNode.expanded;
				if (currentSuccArgNode.isCovered()) {
					assert currentSuccArgNode.coveringNode.isPresent();
					ArgNode<S, A> currentCoveringNode = currentSuccArgNode.coveringNode.get();
					ArgNode<S, A> newCoveringNode = oldToNew.get(currentCoveringNode);
					if (newCoveringNode != null) {
						newSuccArgNode.setCoveringNode(newCoveringNode);
					} else {
						assert currentCoveringNode.getParent().isPresent() &&
								currentCoveringNode.getParent().get() == currentArgNode;
					}
				}
				// covering node can be created before or at the same time (has the same parent) when covered node is
				// 	in the latter case the covered node can be created first
				//		first they both get added to arg
				//		then the covered nodes get out of waitlist and close will get called on it
				if (currentSuccArgNode.coveredNodes.size() != 0) {
					for (ArgNode<S, A> currentSuccCoveredArgNode: currentSuccArgNode.coveredNodes) {
						if (currentSuccCoveredArgNode.getCoveringNode().isEmpty()) {
							ArgNode<S, A> newCoveredArgNode = oldToNew.get(currentSuccCoveredArgNode);
							newCoveredArgNode.setCoveringNode(currentSuccArgNode);
						}
					}
				}

				assert !oldToNew.containsKey(currentSuccArgNode);
				oldToNew.put(currentSuccArgNode, newSuccArgNode);
			});
			return false;
		});

		return new ARGCopyResult<>(arg, oldToNew);
	}

	public Stream<ArgNode<S, A>> getInitNodes() {
		return initNodes.stream();
	}

	public Stream<S> getInitStates() {
		return getInitNodes().map(ArgNode::getState);
	}

	public Stream<ArgNode<S, A>> getNodes() {
		return getInitNodes().flatMap(ArgNode::descendants);
	}

	public Stream<ArgNode<S, A>> getUnsafeNodes() {
		return getInitNodes().flatMap(ArgNode::unexcludedDescendants).filter(ArgNode::isTarget);
	}

	public Stream<ArgNode<S, A>> getIncompleteNodes() {
		return getInitNodes().flatMap(ArgNode::unexcludedDescendants).filter(n -> !n.isExpanded());
	}

	////

	/**
	 * Checks if the ARG is complete, i.e., whether it is initialized and all of
	 * its nodes are complete.
	 */
	public boolean isComplete() {
		return isInitialized() && getNodes().allMatch(ArgNode::isComplete);
	}

	/**
	 * Checks if the ARG is safe, i.e., whether all of its nodes are safe.
	 */
	public boolean isSafe() {
		return getNodes().allMatch(ArgNode::isSafe);
	}

	/**
	 * Checks if the ARG is initialized, i.e., all of its initial nodes are
	 * present.
	 */
	public boolean isInitialized() {
		return initialized;
	}

	////

	public ArgNode<S, A> createInitNode(final S initState, final boolean target) {
		checkNotNull(initState);
		final ArgNode<S, A> initNode = createNode(initState, 0, target);
		initNodes.add(initNode);
		return initNode;
	}

	public ArgNode<S, A> createSuccNode(final ArgNode<S, A> node, final A action, final S succState,
										final boolean target) {
		checkNotNull(node);
		checkNotNull(action);
		checkNotNull(succState);
		checkArgument(node.arg == this, "Node does not belong to this ARG");
		checkArgument(!node.isTarget(), "Node is target");
		final ArgNode<S, A> succNode = createNode(succState, node.getDepth() + 1, target);
		createEdge(node, action, succNode);
		return succNode;
	}

	private ArgNode<S, A> createNode(final S state, final int depth, final boolean target) {
		final ArgNode<S, A> node = new ArgNode<>(this, state, nextId, depth, target);
		nextId = nextId + 1;
		return node;
	}

	private ArgEdge<S, A> createEdge(final ArgNode<S, A> source, final A action, final ArgNode<S, A> target) {
		final ArgEdge<S, A> edge = new ArgEdge<>(source, action, target);
		source.outEdges.add(edge);
		target.inEdge = Optional.of(edge);
		return edge;
	}

	/**
	 * Removes a node along with its subtree.
	 */
	public void prune(final ArgNode<S, A> node) {
		checkNotNull(node);
		checkArgument(node.arg == this, "Node does not belong to this ARG");
		if (node.getInEdge().isPresent()) {
			final ArgEdge<S, A> edge = node.getInEdge().get();
			final ArgNode<S, A> parent = edge.getSource();
			parent.outEdges.remove(edge);
			parent.expanded = false;
		} else {
			assert initNodes.contains(node);
			initNodes.remove(node);
			this.initialized = false;
		}
		node.descendants().forEach(ArgNode::unsetCoveringNode);
		node.descendants().forEach(ArgNode::clearCoveredNodes);
	}

	/**
	 * Prune the whole ARG, making it uninitialized.
	 */
	public void pruneAll() {
		initNodes.clear();
		this.initialized = false;
	}

	public void minimize() {
		initNodes.forEach(this::minimizeSubTree);
	}

	private void minimizeSubTree(final ArgNode<S, A> node) {
		final Stream<ArgNode<S, A>> children = node.children().collect(toList()).stream();
		if (node.isExcluded()) {
			children.forEach(this::prune);
		} else {
			children.forEach(this::minimizeSubTree);
		}
	}

	////

	/**
	 * Gets all counterexamples, i.e., traces leading to target nodes.
	 */
	public Stream<ArgTrace<S, A>> getCexs() {
		return getUnsafeNodes().map(ArgTrace::to);
	}

	/**
	 * Distances for argNodes which reached target even through covering.
	 */
	public Map<ArgNode<S, A>, Integer> getDistances() {
		final Map<ArgNode<S,A>, Integer> distances = new HashContainerFactory().createMap();
		class DistanceSearchResult<S extends State, A extends Action> {
			final public ArgNode<S,A> argNode;
			final public int distance;
			DistanceSearchResult(final ArgNode<S,A> argNode, final int distance) {
				this.argNode = argNode;
				this.distance = distance;
			}
		}

		// have to go from all unsafe nodes at once to get shortest path for nodes reaching 2+ unsafe nodes
		final Waitlist<DistanceSearchResult<S,A>> waitlist = FifoWaitlist.create();
		getUnsafeNodes().forEach((final ArgNode<S, A> target) -> {
			waitlist.add(new DistanceSearchResult<S, A>(target, 0));
			distances.put(target, 0);
		});

		// arg without covering edges are trees
		// 	we walk up the cex trace and all other covered traces which are closed with nodes in cex trace or in any other covered trace
		// use BFS so size of waitlist won't be too large
		while (!waitlist.isEmpty()) {
			final DistanceSearchResult<S, A> distanceSearchResult = waitlist.remove();
			final ArgNode<S, A> argNode = distanceSearchResult.argNode;

			BiFunction<ArgNode<S,A>, Integer, Void> expand = (succArgNode, distanceNext) -> {
				// cex traces and covered traces can have common nodes with other cex or covered traces
				if (distances.containsKey(succArgNode) && distanceNext >= distances.get(succArgNode)){
					return null;
				}
				waitlist.add(new DistanceSearchResult<S, A>(succArgNode, distanceNext));
				distances.put(succArgNode, distanceNext);
				return null;
			};

			// covered nodes will have the same distance => has to be before regular nodes (+1 distance) (bfs)
			argNode.getCoveredNodes().forEach((succArgNode)->{
				expand.apply(succArgNode, distanceSearchResult.distance);
			});
			if (argNode.getInEdge().isPresent()){
				final ArgNode<S, A> succArgNode = argNode.getInEdge().get().getSource();
				expand.apply(succArgNode, distanceSearchResult.distance + 1);
			}
		}

		return distances;
	}

	/**
	 * See walk with multiple roots
	 */
	public void walk(ArgNode<S, A> root, BiFunction<ArgNode<S, A>, Integer, Boolean> skip) {
		Collection<ArgNode<S, A>> roots = new ArrayList<>();
		roots.add(root);
		walk(roots, skip);
	}

	/**
	 * Calls skip on all nodes reachable from root even through coverings.
	 * If skip returns true then the children of the ArgNode is not added to waitlist
	 * ArgNode and it's distance from root is given to consumer.
	 */
	public void walk(Collection<ArgNode<S, A>> roots, BiFunction<ArgNode<S, A>, Integer, Boolean> skip) {
		for (ArgNode<S, A> root : roots) {
			checkNotNull(root);
		}
		checkNotNull(skip);

		class DistanceSearchResult<S extends State, A extends Action> {
			final public ArgNode<S,A> argNode;
			final public int distance;
			DistanceSearchResult(final ArgNode<S,A> argNode, final int distance) {
				this.argNode = argNode;
				this.distance = distance;
			}
		}

		// arg without covering edges are trees
		// 	we walk down the tree and also follow covering edges
		// use BFS so size of waitlist won't be too large
		final Map<ArgNode<S,A>, Integer> distances = new HashContainerFactory().createMap();
		Collection<DistanceSearchResult<S, A>> distanceSearchResults = new ArrayList<>();
		for (ArgNode<S, A> root : roots) {
			distances.put(root, 0);
			distanceSearchResults.add(new DistanceSearchResult<>(root, 0));
		}

		final Waitlist<DistanceSearchResult<S,A>> waitlist = FifoWaitlist.create();
		waitlist.addAll(distanceSearchResults);

		while (!waitlist.isEmpty()) {
			final DistanceSearchResult<S, A> distanceSearchResult = waitlist.remove();
			final ArgNode<S, A> argNode = distanceSearchResult.argNode;

			if (skip.apply(argNode, distanceSearchResult.distance)) {
				continue;
			}

			BiConsumer<ArgNode<S,A>, Integer> expand = (succArgNode, distanceNext) -> {
				// cex traces and covered traces can have common nodes with other cex or covered traces
				if (distances.containsKey(succArgNode) && distanceNext >= distances.get(succArgNode)){
					return;
				}
				waitlist.add(new DistanceSearchResult<>(succArgNode, distanceNext));
				distances.put(succArgNode, distanceNext);
			};

			// TODO inf loop detecting here??
			// covering nodes will have the same distance => has to be before regular nodes (+1 distance) (bfs)
			if (argNode.getCoveringNode().isPresent()) {
				expand.accept(argNode.getCoveringNode().get(), distanceSearchResult.distance);
			}

			argNode.getOutEdges().map(ArgEdge::getTarget).forEach(argNodeSuccessor -> {
				expand.accept(argNodeSuccessor, distanceSearchResult.distance + 1);
			});
		}
	}

	public void walkUpParents(ArgNode<S, A> root, BiFunction<ArgNode, Integer, Boolean> skip) {
		checkNotNull(root);
		checkNotNull(skip);

		ArgNode<S, A> current = root;
		int pseudoDistance = 0;
		while (true) {
			if (skip.apply(current, pseudoDistance)) {
				break;
			}
			if (current.getParent().isEmpty()){
				break;
			}

			pseudoDistance++;
			current = current.getParent().get();
		}
	}

	/**
	 * Gets the size of the ARG, i.e., the number of nodes.
	 */
	public long size() {
		return getNodes().count();
	}

	/**
	 * Gets the depth of the ARG, i.e., the maximal depth of its nodes. Depth
	 * starts (at the initial nodes) from 0. Depth is undefined for an empty
	 * ARG.
	 */
	public int getDepth() {
		final OptionalInt maxOpt = getNodes().mapToInt(ArgNode::getDepth).max();
		checkState(maxOpt.isPresent(), "Depth is undefined for an empty ARG.");
		return maxOpt.getAsInt();
	}

	/**
	 * Gets the mean branching factor of the expanded nodes.
	 */
	public double getMeanBranchingFactor() {
		final Stream<ArgNode<S, A>> nodesToCalculate = getNodes().filter(ArgNode::isExpanded);
		final double mean = nodesToCalculate.mapToDouble(n -> n.getOutEdges().count()).average().orElse(0);
		return mean;
	}

}
