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
package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.Abstractor;
import hu.bme.mit.theta.analysis.algorithm.cegar.AbstractorResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarCegarChecker.Type;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;

import javax.annotation.Nullable;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;
import static com.google.common.base.Preconditions.checkState;

/**
 * Astar implementation for the abstractor, relying on an ArgBuilder.
 */
public final class AstarAbstractor<S extends State, A extends Action, P extends Prec> implements Abstractor<S, A, P> {
	private final ArgBuilder<S, A, P> argBuilder;
	private final Function<? super S, ?> projection;
	// can't have waitlist common in AstarAbstractor instance as checkFromNode is recursively called

	// When we are exploring an arg first time we use the StopCriterion provided in constructor
	private final StopCriterion<S, A> initialStopCriterion;
	// We can't stop when a target node is children of a node because there might be another
	private final Logger logger;
	private final AstarArgStore<S, A, P> astarArgStore;
	private final AstarFileVisualizer<S, A, P> astarFileVisualizer;
	private final Type type;
	private PartialOrd<S> partialOrd;

	private AstarAbstractor(final ArgBuilder<S, A, P> argBuilder,
							final Function<? super S, ?> projection,
							final StopCriterion<S, A> initialStopCriterion,
							final Logger logger,
							final AstarArgStore<S, A, P> astarArgStore,
							final Type type,
							final PartialOrd<S> partialOrd
	) {
		this.argBuilder = checkNotNull(argBuilder);
		this.projection = checkNotNull(projection);
		this.initialStopCriterion = checkNotNull(initialStopCriterion);
		this.logger = checkNotNull(logger);
		this.astarArgStore = checkNotNull(astarArgStore);
		this.type = type;
		this.astarFileVisualizer = new AstarFileVisualizer<>(logger, astarArgStore);
		this.partialOrd = partialOrd;
	}

	public static <S extends State, A extends Action, P extends Prec> Builder<S, A, P> builder(
			final ArgBuilder<S, A, P> argBuilder) {
		return new Builder<>(argBuilder);
	}

	@Override
	public ARG<S, A> createArg() {
		return argBuilder.createArg();
	}

	// return: whether stopCriterion stopped it
	private void findDistanceInner(
			AstarArg<S, A, P> astarArg,
			StopCriterion<S, A> stopCriterion,
			Collection<AstarNode<S, A>> startAstarNodes
	) {
		// create search
		AstarSearch<S, A> search = new AstarSearch<>();
		Waitlist<Edge<S, A>> waitlist = search.waitlist; // can't extract value => do not be a parameter
		Set<AstarNode<S, A>> doneSet = search.doneSet;
		Map<AstarNode<S, A>, Integer> minWeights = search.minWeights;
		Map<ArgNode<S, A>, ArgNode<S, A>> parents = search.parents;
		//Map<AstarNode<S, A>, Integer> depths = search.depths;

		// Waitlist requires heuristic for nodes
		// 	  node for which distance we are going back may not have heuristic
		startAstarNodes.forEach(startNode -> findHeuristic(startNode, astarArg));
		startAstarNodes = startAstarNodes.stream()
				// Occurrence: node hasn't been expanded as it had infinite heuristic and is copied to new arg
				.filter(startNode -> startNode.getHeuristic().getType() != Distance.Type.INFINITE)
				.collect(Collectors.toList());

		Set<ArgNode<S, A>> startNodes = startAstarNodes.stream().map(astarNode -> astarNode.argNode).collect(Collectors.toSet());

		// start nodes to search
		startAstarNodes.forEach(startNode -> waitlist.add(new Edge<>(null, startNode, 0)));

		int upperLimitValue = -1;
		AstarNode<S, A> upperLimitAstarNode = null;
		while (!waitlist.isEmpty()) {
			Edge<S, A> edge = waitlist.remove();
			AstarNode<S, A> astarNode = edge.end;
			int depth = edge.depthFromAStartNode;
			ArgNode<S, A> argNode = astarNode.argNode;
			assert astarNode.getHeuristic().getType() != Distance.Type.INFINITE;

			// lazy propagation
			if (doneSet.contains(astarNode)) {
				continue;
			}
			doneSet.add(astarNode);

			// reached upper limit
			if (depth >= upperLimitValue && upperLimitValue != -1) {
				astarArg.updateDistancesFromTargetUntil(upperLimitAstarNode, startNodes, parents);
				if (stopCriterion.canStop(astarArg.arg, List.of(astarNode.argNode))) {
					return;
				}
			}

			// reached target
			if (argNode.isTarget()) {
				astarArg.updateDistancesFromTargetUntil(astarNode, startNodes, parents);
				if (stopCriterion.canStop(astarArg.arg, List.of(astarNode.argNode))) {
					return;
				}
				continue;
			}

			// When is this possible:
			//   - in a different subgraph reached by covering edge
			//   - same subgraph which was reached from a different subgraph by a covering edge
			// We have a target in x distance therefore we have an upper bound
			// Node can already be marked done therefore
			if (astarNode.distance.isKnown()) {
				//// put this into correct place: because of FULL in the same iteration it can be marked done
				//// this case can also handle covering node's case

				//// do not mark as done for other nodes to use this? this will be useful when leftovers put in reachedset in init (<= if newNode done do not add)
				//// however if FULL this can still be marked as done so this will fail
				//// if we check if newNodes contains a node with known distance will that be correct? as we already do like that when we say that n-1 depths will be considered
				if (upperLimitValue > depth + astarNode.distance.getValue() || upperLimitValue == -1) {
					upperLimitValue = depth + astarNode.distance.getValue();
					upperLimitAstarNode = astarNode;
				}
				continue;
			}

			close(argNode, astarArg.reachedSet.get(argNode));
			if (argNode.getCoveringNode().isPresent()) {
				ArgNode<S, A> coveringNode = argNode.getCoveringNode().get();
				AstarNode<S, A> coveringAstarNode = astarArg.get(coveringNode);
				findHeuristic(coveringAstarNode, astarArg);

				// Covering edge has 0 weight
				//// going over covering edge does not break heuristic monotonicity
				addToWaitList(coveringAstarNode, astarNode, search, depth);
				// covering node is already found therefore already in reachedSet
				continue;
			}

			if (!argNode.isSubsumed() && !argNode.isTarget()) {
				// expand: create nodes
				if (!argNode.isExpanded()) {
					argBuilder.expand(argNode, astarArg.prec);
				}

				// go over recreated and remained nodes
				argNode.getOutEdges().forEach(outEdge -> {
					ArgNode<S, A> succArgNode = outEdge.getTarget();
					AstarNode<S, A> succAstarNode = astarArg.get(succArgNode);

					// expand: create astar nodes
					if (succAstarNode == null) {
						AstarNode<S, A> providerAstarNode = findProviderAstarNode(succArgNode, outEdge.getAction(), astarNode, astarArg.parent);
						succAstarNode = new AstarNode<>(succArgNode, providerAstarNode);
						astarArg.put(succAstarNode);
						astarArg.reachedSet.add(succArgNode);
						findHeuristic(succAstarNode, astarArg);
					}

					if (succAstarNode.getHeuristic().getType() == Distance.Type.INFINITE) {
						return;
					}

					/*if (doneSet.contains(succAstarNode)) {
						// either: reach through covering edge, or this reached through covering edge (multi init nodes)
						assert depths.get(newAstarNode) <= depth + 1;
					}*/

					addToWaitList(succAstarNode, astarNode, search, depth + 1);
				});
			}

			// TODO: stopcriterion for children: can we be sure that if the parent of target is found first then target
			//  would be found first if let the expanding continue?
		}

		// upper limit was not reached (no more nodes left)
		if (upperLimitValue != -1) {
			astarArg.updateDistancesFromTargetUntil(upperLimitAstarNode, startNodes, parents);
		}

		return;
	}

	private void addToWaitList(AstarNode<S, A> astarNode, AstarNode<S, A> parentAstarNode, AstarSearch<S, A> search, int depth) {
		Waitlist<Edge<S, A>> waitlist = search.waitlist;
		Set<AstarNode<S, A>> doneSet = search.doneSet;
		Map<AstarNode<S, A>, Integer> minWeights = search.minWeights;
		Map<ArgNode<S, A>, ArgNode<S, A>> parents = search.parents;

		Distance distance = astarNode.getWeight(depth);
		if (!doneSet.contains(astarNode)) {
			if (!minWeights.containsKey(astarNode) || minWeights.get(astarNode) > distance.getValue()) {
				waitlist.add(new Edge<>(parentAstarNode, astarNode, depth));
				parents.put(astarNode.argNode, parentAstarNode.argNode);
			}
		}
	}

	// return: whether stopCriterion stopped it
	private void findDistance(
			AstarArg<S, A, P> astarArg, StopCriterion<S, A> stopCriterion, Collection<AstarNode<S, A>> startAstarNodes,
			String visualizerState
	) {
		final ARG<S, A> arg = astarArg.arg;

		logger.write(Level.INFO, "|  |  Starting ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.SUBSTEP, "|  |  Building ARG...");
		astarFileVisualizer.visualize(String.format("start%s", visualizerState), astarArgStore.getIndex(astarArg));

		if (!stopCriterion.canStop(arg)) { // this is at max only for leftover nodes??
			findDistanceInner(astarArg, stopCriterion, startAstarNodes);
		}

		logger.write(Level.SUBSTEP, "done%n");
		logger.write(Level.INFO, "|  |  Finished AstarARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		astarFileVisualizer.visualize(String.format("end%s", visualizerState), astarArgStore.getIndex(astarArg));
	}

	private void initAstarArg(final AstarArg<S, A, P> astarArg) {
		astarArg.arg.getInitNodes().forEach(initArgNode -> {
			assert !initArgNode.isCovered();

			// find provider node
			AstarNode<S, A> providerNode = findProviderAstarNode(initArgNode, null, null, astarArg.parent);
			assert astarArg.parent == null || providerNode != null;

			// create init astar node
			AstarNode<S, A> newInitAstarNode = new AstarNode<>(initArgNode, providerNode);
			astarArg.putInit(newInitAstarNode);

			astarArg.reachedSet.add(newInitAstarNode.argNode);
		});
	}

	// astarNode should already have coveringNode
	// astarArg: the one in which for a node we look for heuristic
	private void findHeuristic(AstarNode<S, A> astarNode, AstarArg<S, A, P> astarArg) {
		// no previous astar arg exists: getHeuristic returns (EXACT, 0)
		if (astarArg.parent == null) {
			return;
		}

		// already know provider node's distance
		if (astarNode.getHeuristic().isKnown()) {
			return;
		}
		assert type != Type.FULL;

		// visualize current before going back to previous astarArg
		String visualizerState = AstarFileVisualizer.getVisualizerState(List.of(astarNode));
		astarFileVisualizer.visualize(String.format("paused %s", visualizerState), astarArgStore.getIndex(astarArg));

		// get the heuristic with findDistance in parent arg
		AstarNode<S, A> providerAstarNode = astarNode.providerAstarNode;
		AstarArg<S, A, P> parentAstarArg = astarArg.parent;

		String visualizerStateProvider = " " + AstarFileVisualizer.getVisualizerState(List.of(astarNode.providerAstarNode));
		findDistance(parentAstarArg, new AstarDistanceKnown<>(providerAstarNode), List.of(providerAstarNode), visualizerStateProvider);
		assert astarNode.getHeuristic().isKnown();

		// visualize current after going back to previous astarArg
		astarFileVisualizer.visualize(String.format("resumed %s", visualizerState), astarArgStore.getIndex(astarArg));
	}

	// action, parentAstarNode: can be null when argNode is an init node
	private AstarNode<S, A> findProviderAstarNode(
			ArgNode<S, A> argNode,
			@Nullable A action,
			@Nullable AstarNode<S, A> parentAstarNode,
			AstarArg<S, A, P> parentAstarArg
	) {
		// No previous arg therefore no provider node
		if (parentAstarArg == null) {
			return null;
		}

		// Parent of argNode should be given
		assert !parentAstarArg.containsArg(argNode);

		Stream<ArgNode<S, A>> providerCandidates;

		// Init nodes don't have parents
		if (parentAstarNode == null) {
			providerCandidates = parentAstarArg.getAllInitArg().stream();
		} else {
			AstarNode<S, A> parentProviderAstarNode = parentAstarNode.providerAstarNode;
			//// as parent's child is explored parent had to be in waitlist => distance exists => parent expanded / covered

			// parentAstarNode had to be in waitlist
			// 		therefore it's heuristic is known
			// 		therefore it's provider's distance is known
			assert parentProviderAstarNode.distance.isKnown();
			//		therefore it must be expanded or covered
			assert parentProviderAstarNode.argNode.isExpanded() || parentAstarNode.argNode.isCovered();

			// Although we don't choose covered provider node after it has been set it still can be covered
			// (when the provider node is a fresh node and not have)
			// The node's children are at the covering node
			if (parentProviderAstarNode.argNode.getCoveringNode().isPresent()) {
				ArgNode<S, A> parentProviderCoveringNode = parentProviderAstarNode.argNode.getCoveringNode().get();
				// This change will affect visualizer midpoint
				parentProviderAstarNode = parentAstarNode.providerAstarNode = parentAstarArg.get(parentProviderCoveringNode);

				assert parentProviderAstarNode.distance.isKnown();
				assert parentProviderAstarNode.argNode.isExpanded();
			}

			logger.write(Level.VERBOSE, "Count before filtering for action %d%n", parentProviderAstarNode.argNode.getOutEdges().count());
			providerCandidates = parentProviderAstarNode.argNode.getOutEdges()
					// TODO test if equals work
					.filter(edge -> edge.getAction().equals(action))
					.map(ArgEdge::getTarget);
			logger.write(Level.VERBOSE, "Count after filtering for action %d%n",
					parentProviderAstarNode.argNode.getOutEdges().filter(edge -> edge.getAction().equals(action)).count()
			);
		}

		// filter based on partialOrd: isLeq == "<=" == subset of
		providerCandidates = providerCandidates.filter(providerCandidate ->
				partialOrd.isLeq(argNode.getState(), providerCandidate.getState())
		);
		Optional<ArgNode<S,A>> providerNode = providerCandidates.findAny();
		assert providerNode.isPresent();
		return parentAstarArg.get(providerNode.get());
	}

	@Override
	// uses previous AstarArg then calls checkFromNode with root=null
	// it is assumed that last AstarArg in astarArgStore should be used if exists
	public AbstractorResult check(final ARG<S, A> arg, final P prec) {
		checkNotNull(arg);
		checkNotNull(prec);
		logger.write(Level.DETAIL, "|  |  Precision: %s%n", prec);

		AstarArg<S, A, P> astarArg = astarArgStore.getLast();

		// initialize: prune can keep initialized state
		if (!arg.isInitialized()) {
			logger.write(Level.SUBSTEP, "|  |  (Re)initializing ARG...");
			argBuilder.init(arg, prec);
			initAstarArg(astarArg);
			logger.write(Level.SUBSTEP, "done%n");
		}

		assert arg.isInitialized();

		//// parents + (parents & covering edges) make this difficult: arg.getIncompleteNodes().map(astarArg::get).filter(n -> n.distance.getType() != DistanceType.INFINITE)
		//// 		add to reachedSet if implemented
		Collection<AstarNode<S, A>> incompleteAstarNodes = astarArg.getIncompleteNodes().collect(Collectors.toList());
		findDistance(astarArg, initialStopCriterion, incompleteAstarNodes, "");

		// found and isSafe is different: e.g. full expand
		if (arg.isSafe()) {
			// Arg may won't be expanded as we can get INFINITE heuristic avoiding expansion
			//   therefore we don't need this: checkState(arg.isComplete(), "Returning incomplete ARG as safe");
			return AbstractorResult.safe();
		} else {
			if (type == AstarCegarChecker.Type.FULL) {
				astarArg.setUnknownDistanceInfinite();
			}
			return AbstractorResult.unsafe();
		}
	}

	private void close(final ArgNode<S, A> node, final Collection<ArgNode<S, A>> candidates) {
		if (!node.isLeaf()) {
			return;
		}
		for (final ArgNode<S, A> candidate : candidates) {
			if (candidate.mayCover(node)) {
				node.cover(candidate);
				return;
			}
		}
	}

	/*@Override
	public String toString() {
		return Utils.lispStringBuilder(getClass().getSimpleName()).add(waitlist).toString();
	}*/

	public static final class Builder<S extends State, A extends Action, P extends Prec> {
		private final ArgBuilder<S, A, P> argBuilder;
		private Function<? super S, ?> projection;
		private StopCriterion<S, A> stopCriterion;
		private Logger logger;
		private AstarArgStore<S, A, P> astarArgStore;
		private Type type;
		private PartialOrd<S> partialOrd;

		private Builder(final ArgBuilder<S, A, P> argBuilder) {
			this.argBuilder = argBuilder;
			this.projection = s -> 0;
			this.stopCriterion = StopCriterions.firstCex();
			this.logger = NullLogger.getInstance();
		}

		public Builder<S, A, P> projection(final Function<? super S, ?> projection) {
			this.projection = projection;
			return this;
		}

		public Builder<S, A, P> stopCriterion(final StopCriterion<S, A> stopCriterion) {
			this.stopCriterion = stopCriterion;
			return this;
		}

		public Builder<S, A, P> logger(final Logger logger) {
			this.logger = logger;
			return this;
		}

		public Builder<S, A, P> AstarArgStore(final AstarArgStore<S, A, P> astarArgStore) {
			this.astarArgStore = astarArgStore;
			return this;
		}

		public Builder<S, A, P> type(final Type type) {
			this.type = type;
			return this;
		}

		public Builder<S, A, P> partialOrder(final PartialOrd partialOrd) {
			this.partialOrd = partialOrd;
			return this;
		}

		public AstarAbstractor<S, A, P> build() {
			assert astarArgStore != null;
			return new AstarAbstractor<>(argBuilder, projection, stopCriterion, logger, astarArgStore, type, partialOrd);
		}
	}

}
