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
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.Abstractor;
import hu.bme.mit.theta.analysis.algorithm.cegar.AbstractorResult;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterion;
import hu.bme.mit.theta.analysis.algorithm.cegar.abstractor.StopCriterions;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch.Edge;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarCegarChecker.Type;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStoreFull;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

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
		assert initialStopCriterion instanceof StopCriterions.FirstCex<S,A>;
		this.logger = checkNotNull(logger);
		this.astarArgStore = checkNotNull(astarArgStore);
		this.type = type;
		this.astarFileVisualizer = new AstarFileVisualizer<>(false, astarArgStore);
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
		Set<AstarNode<S, A>> doneSet = search.doneSet;
		Map<ArgNode<S, A>, ArgNode<S, A>> parents = search.parents;

		// Waitlist requires heuristic for nodes
		// 	  node for which distance we are going back may not have heuristic
		startAstarNodes.forEach(startNode -> findHeuristic(startNode, astarArg));
		startAstarNodes = startAstarNodes.stream()
				.filter(startNode -> startNode.getHeuristic().getType() != Distance.Type.INFINITE)
				.collect(Collectors.toList());
		startAstarNodes.forEach(startAstarNode -> search.addToWaitlist(startAstarNode, null, 0));
		assert startAstarNodes.stream().allMatch(startAstarNode -> startAstarNode.distance.getType() != Distance.Type.EXACT);

		// Implementation assumes that lower distance is set first therefore store reached targets in the order we reach them.
		// We save targets and nodes with exact value. In the latter the exact values must be from a previous findDistance as we set exact distances at the end of iteration.
		Queue<AstarNode<S, A>> reachedExacts = new ArrayDeque<>();

		while (!search.isWaitlistEmpty()) {
			Edge<S, A> edge = search.removeFromWaitlist();
			AstarNode<S, A> parentAstarNode = edge.start;
			AstarNode<S, A> astarNode = edge.end;
			int depth = edge.depthFromAStartNode;
			ArgNode<S, A> argNode = astarNode.argNode;
			assert astarNode.getHeuristic().getType() != Distance.Type.INFINITE;
			assert astarNode.distance.getType() != Distance.Type.INFINITE;

			// lazy propagation
			if (doneSet.contains(astarNode)) {
				continue;
			}
			doneSet.add(astarNode);

			// debug: is parent state managed correctly
			// TODO: remove this state management
			if (parentAstarNode == null) {
				assert parents.get(astarNode.argNode) == null;
			} else {
				assert parentAstarNode.argNode == parents.get(astarNode.argNode);
			}

			// reached upper limit: depth + heuristic distance
			//astarNode.getWeight(depth).getValue() // TODO
			if (depth >= search.upperLimitValue && search.upperLimitValue != -1) {
				reachedExacts.add(search.upperLimitAstarNode);
				if (stopCriterion.canStop(astarArg.arg, List.of(astarNode.argNode))) {
					break;
				}
			}

			// reached target
			if (argNode.isTarget()) {
				reachedExacts.add(astarNode);
				if (stopCriterion.canStop(astarArg.arg, List.of(astarNode.argNode))) {
					break;
				}
				continue;
			}

			// After prune node may have children but not fully expanded (isExpanded false).
			// If node has no children it still can already be expanded, therefore expanded is already set (should not be covered).
			// If node already has covering node, close cloud still choose another one, therefore avoid.
			if (!argNode.isExpanded() && argNode.getSuccNodes().findAny().isEmpty() && argNode.getCoveringNode().isEmpty()) {
				close(argNode, astarArg.reachedSet.get(argNode));
			}
			if (argNode.getCoveringNode().isPresent()) {
				ArgNode<S, A> coveringNode = argNode.getCoveringNode().get();
				AstarNode<S, A> coveringAstarNode = astarArg.get(coveringNode);
				findHeuristic(coveringAstarNode, astarArg);

				// If astarNode's parent is also a coveredNode then covering edges have been redirected.
				// We have to update parents map according to that. (see ArgNode::cover)
				//  1) a - - -> b
				//  2) a - - -> b - - -> c
				//  3) a        b - - -> c
				//	   |                 ^
				//     | - - - - - - - - |

				AstarNode<S, A> coveredAstarNode;
				if (parentAstarNode != null && parentAstarNode.argNode.getCoveringNode().isPresent()) {
					// Because argNode is covered it can only reach coveringNode with the same distance as it's new parent
					// therefore we can safely remove it
					parents.remove(argNode);

					// Update to new parent if we this node is the current parent
					// as coveringAstarNode may already have a better parent or already in doneSet
					coveredAstarNode = parentAstarNode;
				} else {
					coveredAstarNode = astarNode;
				}

				// Covering edge has 0 weight
				search.addToWaitlist(coveringAstarNode, coveredAstarNode, depth);
				// Covering node is already found therefore already in reachedSet

				continue;
			}

			if (argNode.isFeasible()) {
				assert !argNode.isTarget() && !argNode.isCovered();

				// expand: create nodes
				if (!argNode.isExpanded()) {
					Collection<ArgNode<S, A>> newNodes = argBuilder.expand(argNode, astarArg.prec);
					astarArg.reachedSet.addAll(newNodes);
				}

				// go over recreated and remained nodes
				for (ArgNode<S, A> succArgNode : argNode.getSuccNodes().toList()) {
					AstarNode<S, A> succAstarNode = astarArg.get(succArgNode);

					// expand: create astar nodes
					if (succAstarNode == null) {
						succAstarNode = astarArg.createSuccAstarNode(succArgNode, astarNode);
					}

					// already existing succAstarNode
					// 		although we only add leaves to startAstarNodes and findHeuristic is called upon them
					//		they may get covered with a non leaf which has succAstarNode (from copy)
					//		but it's provider doesn't have distance, therefore there is no heuristic
					findHeuristic(succAstarNode, astarArg);

					search.addToWaitlist(succAstarNode, astarNode, depth + 1);
				}
			}
		}

		// upper limit was not reached (no more nodes left)
		if (search.upperLimitValue != -1) {
			reachedExacts.add(search.upperLimitAstarNode);
		}

		// If we are looking for n targets then it is possible that we reached [1,n) target when reaching this line

		// We need to have heuristic for startAstarNodes therefore we need to set distance.
		// updateDistancesFromNodes depends on infinite distances being already set therefore set infinite distances first.
		astarArg.updateDistanceInfinite();
		Set<ArgNode<S, A>> startNodes = startAstarNodes.stream().map(astarNode -> astarNode.argNode).collect(Collectors.toSet());
		while(!reachedExacts.isEmpty()) {
			AstarNode<S, A> target = reachedExacts.remove();
			astarArg.updateDistancesFromTargetUntil(target, startNodes, parents);

			// All provider nodes are expected to be expanded, targets as well
			expandTarget(target, astarArg);
		}

		// Updating distance doesn't handle loops as they are hard to implement and probably don't have the same problem
		// of visiting nodes multiple times as other cases.
		// However, we need to guarantee that startAstarNodes have a distance which is not yet true if a startNode is
		// in a loop and doesn't reach target.
		if (startAstarNodes.stream().noneMatch(a -> a.distance.isKnown())) {
			startAstarNodes.forEach(astarArg::updateDistancesFromRootInfinite);
		}
	}

	// Expands the target (future provider node) so that the children of the provided node can also have provider nodes to choose from.
	// Target should not already be expanded.
	private void expandTarget(AstarNode<S, A> astarNode, AstarArg<S, A, P> astarArg) {
		ArgNode<S, A> argNode = astarNode.argNode;
		// astarNode can be a coverer node for another target, therefore it can already be expanded (directly or indirectly)
		if (argNode.isExpanded() || argNode.getCoveringNode().isPresent()) {
			return;
		}
		assert argNode.getSuccNodes().findAny().isEmpty(); // TODO ask this

		do {
			assert !argNode.isExpanded();
			assert argNode.getSuccNodes().findAny().isEmpty();
			assert argNode.getCoveringNode().isEmpty();
			close(argNode, astarArg.reachedSet.get(argNode));
			if (argNode.getCoveringNode().isEmpty()) {
				//// We can get covered into already expanded node
				//// Covering target may have been after astarNode in waitlist therefore it may not already be expanded
				Collection<ArgNode<S, A>> newNodes = argBuilder.expand(argNode, astarArg.prec);
				astarArg.reachedSet.addAll(newNodes);

				// expand: create astar nodes
				AstarNode<S, A> lambdaAstarNode = astarNode;
				argNode.getSuccNodes().forEach(succArgNode -> {
					AstarNode<S, A> succAstarNode = astarArg.get(succArgNode);
					assert succAstarNode == null;
					astarArg.createSuccAstarNode(succArgNode, lambdaAstarNode);
					// We don't add it to waitlist therefore we don't need to find heuristic
				});

				break;
			}

			// Covered node's children are the covering node's children, therefore we have to expand the covering node
			argNode = argNode.getCoveringNode().get();
			astarNode = astarArg.get(argNode);
			// Target node's covering node must be a target.
			assert argNode.isTarget();
			// optimization: we know the distance for a target node
			astarNode.distance = new Distance(Distance.Type.EXACT, 0);
		} while(!argNode.isExpanded()); // We can cover into an already expanded target (it can't be covered, see close())
	}

	private void findDistance(
			AstarArg<S, A, P> astarArg, StopCriterion<S, A> stopCriterion, Collection<AstarNode<S, A>> startAstarNodes,
			String visualizerState
	) {
		final ARG<S, A> arg = astarArg.arg;

		logger.write(Level.INFO, "|  |  Starting ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.SUBSTEP,"|  |  Starting AstarArg: %s%n", astarFileVisualizer.getTitle("", astarArgStore.getIndex(astarArg)));
		logger.write(Level.SUBSTEP, "|  |  Building ARG...");
		astarFileVisualizer.visualize(String.format("start%s", visualizerState), astarArgStore.getIndex(astarArg));

		// TODO: is this only for init nodes? if so delete it (DRY for target distance update)
		//if (!stopCriterion.canStop(arg)) {
			findDistanceInner(astarArg, stopCriterion, startAstarNodes);
		//}

		logger.write(Level.SUBSTEP, "done%n");
		logger.write(Level.INFO, "|  |  Finished ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.INFO, "|  |  Finished AstarArg: %s%n", astarFileVisualizer.getTitle("", astarArgStore.getIndex(astarArg)));
		astarFileVisualizer.visualize(String.format("end%s", visualizerState), astarArgStore.getIndex(astarArg));
	}

	private void initAstarArg(final AstarArg<S, A, P> astarArg) {
		astarArg.arg.getInitNodes().forEach(initArgNode -> {
			assert !initArgNode.isCovered();
			astarArg.createInitAstarNode(initArgNode);
		});
	}

	// astarNode: should already have providerNode if not in the first arg
	// astarArg: the one in which for a node we look for heuristic
	private void findHeuristic(AstarNode<S, A> astarNode, AstarArg<S, A, P> astarArg) {
		// no previous astar arg exists: getHeuristic returns (EXACT, 0)
		if (astarArg.provider == null) {
			return;
		}
		checkNotNull(astarNode.providerAstarNode);

		// already know provider node's distance
		if (astarNode.getHeuristic().isKnown()) {
			return;
		}
		assert type != Type.FULL;

		// Do not return EXACT(0) when node is target as that would not create the side effect of expanding the node

		if (astarNode.providerAstarNode.getHeuristic().getType() == Distance.Type.INFINITE) {
			assert astarNode.providerAstarNode.distance.getType() == Distance.Type.INFINITE;
		}

		// visualize current before going back to previous astarArg
		String visualizerState = AstarFileVisualizer.getVisualizerState(astarNode);
		astarFileVisualizer.visualize(String.format("paused %s", visualizerState), astarArgStore.getIndex(astarArg));
		logger.write(Level.SUBSTEP, "|  |  Paused AstarArg: %s%n", astarFileVisualizer.getTitle("", astarArgStore.getIndex(astarArg)));

		// get the heuristic with findDistance in parent arg
		AstarNode<S, A> providerAstarNode = astarNode.providerAstarNode;
		AstarArg<S, A, P> providerAstarArg = astarArg.provider;

		String visualizerStateProvider = " " + AstarFileVisualizer.getVisualizerState(astarNode.providerAstarNode);
		findDistance(providerAstarArg, new AstarDistanceKnown<>(providerAstarNode), List.of(providerAstarNode), visualizerStateProvider);
		assert astarNode.getHeuristic().isKnown();

		// visualize current after going back to previous astarArg
		astarFileVisualizer.visualize(String.format("resumed %s", visualizerState), astarArgStore.getIndex(astarArg));
		logger.write(Level.SUBSTEP, "|  |  Resumed AstarArg: %s%n", astarFileVisualizer.getTitle("", astarArgStore.getIndex(astarArg)));
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
			Collection<ArgNode<S, A>> newNodes = argBuilder.init(arg, prec);
			astarArg.reachedSet.addAll(newNodes);
			initAstarArg(astarArg);
			logger.write(Level.SUBSTEP, "done%n");
		}
		assert arg.isInitialized();

		//// parents + (parents & covering edges) make this difficult: arg.getIncompleteNodes().map(astarArg::get).filter(n -> n.distance.getType() != DistanceType.INFINITE)
		//// 		add to reachedSet if implemented
		Collection<AstarNode<S, A>> incompleteAstarNodes = astarArg.getIncompleteNodes().collect(Collectors.toList());
		// If we start from incomplete nodes then we have to know their shortest depth from an init node.
		findDistance(astarArg, initialStopCriterion, astarArg.getAllInit(), "");

		// found and isSafe is different: e.g. full expand
		if (arg.isSafe()) {
			// Arg may won't be expanded as we can get INFINITE heuristic avoiding expansion
			//   therefore we don't need this: checkState(arg.isComplete(), "Returning incomplete ARG as safe");
			return AbstractorResult.safe();
		} else {
			/*if (type == AstarCegarChecker.Type.FULL) {
				astarArg.setUnknownDistanceInfinite();
			}*/
			return AbstractorResult.unsafe();
		}
	}

	private void debug(AstarArg<S, A, P> astarArg) {
		boolean enabled = astarFileVisualizer.getEnabled();
		astarFileVisualizer.setEnabled(true);
		astarFileVisualizer.visualize("debug", astarArgStore.getIndex(astarArg));
		astarFileVisualizer.setEnabled(enabled);
	}

	private void close(final ArgNode<S, A> node, final Collection<ArgNode<S, A>> candidates) {
		assert node.getCoveringNode().isEmpty();
		assert node.getSuccNodes().findAny().isEmpty();
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

		public Builder<S, A, P> astarArgStore(final AstarArgStore<S, A, P> astarArgStore) {
			this.astarArgStore = astarArgStore;
			return this;
		}

		public Builder<S, A, P> partialOrder(final PartialOrd<S> partialOrd) {
			this.partialOrd = partialOrd;
			return this;
		}

		public AstarAbstractor<S, A, P> build() {
			assert astarArgStore != null;
			return new AstarAbstractor<>(argBuilder, projection, stopCriterion, logger, astarArgStore, astarArgStore instanceof AstarArgStoreFull<S,A,P> ? Type.FULL : Type.SEMI_ONDEMAND, partialOrd);
		}
	}

}
