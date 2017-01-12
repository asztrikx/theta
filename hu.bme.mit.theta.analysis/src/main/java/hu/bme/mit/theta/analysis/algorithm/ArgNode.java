package hu.bme.mit.theta.analysis.algorithm;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkNotNull;
import static hu.bme.mit.theta.common.ObjectUtils.toStringBuilder;
import static hu.bme.mit.theta.common.Utils.anyMatch;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Optional;
import java.util.stream.Stream;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;

public final class ArgNode<S extends State, A extends Action> {

	private static final int HASH_SEED = 8543;
	private volatile int hashCode = 0;

	final ARG<S, A> arg;

	private final int id;
	private final int depth;
	private final boolean target;

	private S state;

	Optional<ArgEdge<S, A>> inEdge; // Set by ARG
	final Collection<ArgEdge<S, A>> outEdges;

	Optional<ArgNode<S, A>> coveringNode; // Set by ARG
	final Collection<ArgNode<S, A>> coveredNodes;

	boolean expanded; // Set by ArgBuilder

	ArgNode(final ARG<S, A> arg, final S state, final int id, final int depth, final boolean target) {
		this.arg = arg;
		this.state = state;
		this.id = id;
		this.depth = depth;
		this.target = target;
		inEdge = Optional.empty();
		outEdges = new ArrayList<>();
		coveringNode = Optional.empty();
		coveredNodes = new HashSet<>();
		expanded = false;
	}

	////

	public int getId() {
		return id;
	}

	/**
	 * Gets the depth of the node, which is 0 if the node has no parent, and
	 * depth(parent) + 1 otherwise.
	 */
	public int getDepth() {
		return depth;
	}

	public S getState() {
		return state;
	}

	public void setState(final S state) {
		checkNotNull(state);
		this.state = state;
	}

	public boolean mayCover(final ArgNode<S, A> node) {
		if (arg.domain.isLeq(node.getState(), this.getState())) {
			return ancestors().noneMatch(n -> n.equals(node) || n.isSubsumed());
		}
		return false;
	}

	public void setCoveringNode(final ArgNode<S, A> node) {
		checkNotNull(node);
		checkArgument(node.arg == this.arg);
		unsetCoveringNode();
		coveringNode = Optional.of(node);
		node.coveredNodes.add(this);
	}

	public void unsetCoveringNode() {
		if (coveringNode.isPresent()) {
			coveringNode.get().coveredNodes.remove(this);
			coveringNode = Optional.empty();
		}
	}

	public void clearCoveredNodes() {
		coveredNodes.forEach(n -> n.coveringNode = Optional.empty());
		coveredNodes.clear();
	}

	public void cover(final ArgNode<S, A> node) {
		checkArgument(!node.isExcluded());
		setCoveringNode(node);
		descendants().forEach(ArgNode::clearCoveredNodes);
	}

	////

	public Optional<ArgNode<S, A>> getParent() {
		return inEdge.map(ArgEdge::getSource);
	}

	public Optional<ArgEdge<S, A>> getInEdge() {
		return inEdge;
	}

	public Stream<ArgEdge<S, A>> getOutEdges() {
		return outEdges.stream();
	}

	public Optional<ArgNode<S, A>> getCoveringNode() {
		return coveringNode;
	}

	public Stream<ArgNode<S, A>> getCoveredNodes() {
		return coveredNodes.stream();
	}

	////

	public Stream<ArgNode<S, A>> getSuccNodes() {
		return getOutEdges().map(ArgEdge::getTarget);
	}

	public Stream<S> getSuccStates() {
		return getSuccNodes().map(ArgNode::getState);
	}

	////

	/**
	 * Checks if the node is covered, i.e., there is a covering edge for the
	 * node.
	 */
	public boolean isCovered() {
		return coveringNode.isPresent();
	}

	/**
	 * Checks if the node is not a bottom state.
	 */
	public boolean isFeasible() {
		return !arg.domain.isBottom(state);
	}

	/**
	 * Checks if the node is subsumed, i.e., the node is covered or not
	 * feasible.
	 */
	public boolean isSubsumed() {
		return isCovered() || !isFeasible();
	}

	/**
	 * Checks if the node is excluded, i.e., the node is subsumed or has an
	 * excluded parent.
	 */
	public boolean isExcluded() {
		return isSubsumed() || anyMatch(getParent(), ArgNode::isExcluded);
	}

	/**
	 * Checks if the node is target, i.e., the target predicate holds (e.g., it
	 * is an error state).
	 */
	public boolean isTarget() {
		return target;
	}

	/**
	 * Checks if the node is expanded, i.e., all of its successors are present.
	 */
	public boolean isExpanded() {
		return expanded;
	}

	/**
	 * Checks if the node is leaf, i.e., it has no successors.
	 */
	public boolean isLeaf() {
		return outEdges.isEmpty();
	}

	/**
	 * Checks if the node is safe, i.e., not target or excluded.
	 */
	public boolean isSafe() {
		return !isTarget() || isExcluded();
	}

	/**
	 * Checks if the node is complete, i.e., expanded or excluded.
	 */
	public boolean isComplete() {
		return isExpanded() || isExcluded();
	}

	////

	public Stream<ArgNode<S, A>> properAncestors() {
		final Optional<ArgNode<S, A>> parent = getParent();
		if (parent.isPresent()) {
			return Stream.concat(Stream.of(parent.get()), parent.get().properAncestors());
		} else {
			return Stream.empty();
		}
	}

	public Stream<ArgNode<S, A>> ancestors() {
		return Stream.concat(Stream.of(this), this.properAncestors());
	}

	public Stream<ArgNode<S, A>> children() {
		return outEdges.stream().map(e -> e.getTarget());
	}

	public Stream<ArgNode<S, A>> properDescendants() {
		return Stream.concat(this.children(), this.children().flatMap(ArgNode::properDescendants));
	}

	public Stream<ArgNode<S, A>> descendants() {
		return Stream.concat(Stream.of(this), this.properDescendants());
	}

	public Stream<ArgNode<S, A>> unexcludedDescendants() {
		if (this.isSubsumed()) {
			return Stream.empty();
		} else {
			return Stream.concat(Stream.of(this), this.children().flatMap(ArgNode::unexcludedDescendants));
		}
	}

	////

	@Override
	public int hashCode() {
		int result = hashCode;
		if (result == 0) {
			result = HASH_SEED;
			result = 31 * result + id;
			hashCode = result;
		}
		return result;
	}

	@Override
	public boolean equals(final Object obj) {
		return super.equals(obj);
	}

	@Override
	public String toString() {
		return toStringBuilder("ArgNode").add(id).add(state).toString();
	}

}
