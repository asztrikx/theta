/*
 *  Copyright 2022 Budapest University of Technology and Economics
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

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.waitlist.FifoWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.Containers;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import java.util.Collection;
import java.util.Set;
import java.util.Optional;
import java.util.OptionalInt;

import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;
import static com.google.common.base.Preconditions.checkState;
import static java.util.stream.Collectors.toList;

/**
 * Represents an abstract reachability graph (ARG). See the related class
 * ArgBuilder.
 */
public final class ARG<S extends State, A extends Action> {

	private final Collection<ArgNode<S, A>> initNodes;
	boolean initialized; // Set by ArgBuilder
	int nextId = 0;
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

	// Get expanded (not covered) nodes with no outedge
	public Stream<ArgNode<S, A>> getCompleteLeafNodes() {
		return getInitNodes().flatMap(ArgNode::unexcludedDescendants).filter(n -> n.isExpanded() && n.getSuccNodes().findAny().isEmpty() && !n.isCovered());
	}

	public Stream<ArgNode<S, A>> getAncestorCoveredNodes() {
		return getCoveredNodes().filter(coveredNode -> {
			assert coveredNode.coveringNode.isPresent();
			ArgNode<S, A> coveringNode = coveredNode.coveringNode.get();
			return coveredNode.properAncestors().anyMatch(properAncestor -> properAncestor == coveringNode);
		});
	}

	public Stream<ArgNode<S, A>> getCoveredNodes() {
		return getInitNodes().flatMap(ArgNode::unexcludedDescendants).filter(ArgNode::isCovered);
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
		// We need to expand target // TODO better comment based on edge case 3
		//checkArgument(!node.isTarget(), "Node is target");
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
	 * Calls skip on all nodes reachable from root even through covering edges.
	 * If skip returns true then the children of the ArgNode is not added to waitlist.
	 * ArgNode and it's distance from a root is given to skip().
	 */
	public void walk(Collection<ArgNode<S, A>> roots, BiFunction<ArgNode<S, A>, Integer, Boolean> skip) {
		for (ArgNode<S, A> root : roots) {
			checkNotNull(root);
		}
		checkNotNull(skip);

		class Visit {
			final public ArgNode<S, A> argNode;
			final public int distance;

			public Visit(ArgNode<S, A> argNode, int distance) {
				this.argNode = argNode;
				this.distance = distance;
			}
		}

		Set<ArgNode<S, A>> doneSet = new HashContainerFactory().createSet();
		Waitlist<Visit> waitlist = FifoWaitlist.create();
		for (ArgNode<S, A> root : roots) {
			waitlist.add(new Visit(root, 0));
		}

		// Covering edge has zero weight which would break BFS if we did not push it to its correct place in waitlist
		//	e.g. to the front which is always a correct place
		Visit coveringVisit = null;
		while (!waitlist.isEmpty()) {
			Visit visit;
			if (coveringVisit == null) {
				visit = waitlist.remove();
			} else {
				visit = coveringVisit;
				coveringVisit = null;
			}
			ArgNode<S, A> argNode = visit.argNode;
			int distance = visit.distance;

			// covering edges can point to already visited ArgNode
			if (doneSet.contains(argNode)) {
				continue;
			}
			doneSet.add(argNode);

			// skip
			if (skip.apply(argNode, distance)) {
				continue;
			}

			// covered: add to front of waitlist
			if (argNode.getCoveringNode().isPresent()) {
				ArgNode<S, A> coveringNode = argNode.getCoveringNode().get();
				coveringVisit = new Visit(coveringNode, distance);
				continue;
			}

			argNode.getSuccNodes().forEach(succNode -> {
				if (!doneSet.contains(succNode)) {
					waitlist.add(new Visit(succNode, distance + 1));
				}
			});
		}
	}

	// parents: node -> parent
	public void walkUpParents(ArgNode<S, A> start, Function<ArgNode<S, A>, ArgNode<S, A>> getParent, BiFunction<ArgNode<S, A>, Integer, Boolean> skip) {
		checkNotNull(start);
		checkNotNull(skip);

		ArgNode<S, A> current = start;
		int distance = 0;
		while (current != null) {
			if (skip.apply(current, distance)) {
				break;
			}

			ArgNode<S, A> parent = getParent.apply(current);
			if (parent != null) { // TODO how did this not failed
				if (current.inEdge.isPresent() && current.inEdge.get().getSource() == parent) {
					distance++;
				} else {
					assert current.coveredNodes.contains(parent);
				}
			}
			current = parent;
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
