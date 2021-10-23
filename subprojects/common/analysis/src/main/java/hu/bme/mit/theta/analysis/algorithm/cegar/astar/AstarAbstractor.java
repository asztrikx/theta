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
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 * Astar implementation for the abstractor, relying on an ArgBuilder.
 */
public final class AstarAbstractor<S extends State, A extends Action, P extends Prec> implements Abstractor<S, A, P> {
	private final ArgBuilder<S, A, P> argBuilder;
	private final Function<? super S, ?> projection;
	// can't have waitlist common in AstarAbstractor instance as checkFromNode is recursively called
	private final StopCriterion<S, A> stopCriterion;
	private final Logger logger;
	private final AstarArgStore<S, A, P> astarArgStore;
	private final Type type;

	private AstarAbstractor(final ArgBuilder<S, A, P> argBuilder,
							final Function<? super S, ?> projection,
							final StopCriterion<S, A> stopCriterion,
							final Logger logger,
							final AstarArgStore<S, A, P> astarArgStore,
							final Type type
	) {
		this.argBuilder = checkNotNull(argBuilder);
		this.projection = checkNotNull(projection);
		this.stopCriterion = checkNotNull(stopCriterion);
		this.logger = checkNotNull(logger);
		this.astarArgStore = checkNotNull(astarArgStore);
		this.type = type;
	}

	public static <S extends State, A extends Action, P extends Prec> Builder<S, A, P> builder(
			final ArgBuilder<S, A, P> argBuilder) {
		return new Builder<>(argBuilder);
	}

	@Override
	public ARG<S, A> createArg() {
		return argBuilder.createArg();
	}

	// checkFromNode
	//	if root is null then new arg will be created
	public AbstractorResult checkFromNode(final AstarArg<S, A, P> astarArg, final P prec, final AstarNode<S, A> astarRoot) {
		checkNotNull(astarArg);
		checkNotNull(prec);

		final ARG<S, A> arg = astarArg.arg;
		ArgNode<S, A> root = null;
		if (astarRoot != null) {
			root = astarRoot.argNode;
		}

		logger.write(Level.DETAIL, "|  |  Precision: %s%n", prec);
		String visualizerState = getVisualizerState(root);
		visualize(String.format("start %s", visualizerState),  astarArg.iteration);

		// initialize Arg
		assert root == null || arg.isInitialized();
		if (root == null && !arg.isInitialized()) {
			initializeArg(astarArg, prec);
		}
		assert arg.isInitialized();

		logger.write(Level.INFO, "|  |  Starting ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.SUBSTEP, "|  |  Building ARG...");

		final Partition<ArgNode<S, A>, ?> reachedSet = Partition.of(n -> projection.apply(n.getState()));
		final AstarComparator<S, A, P> astarComparator = AstarComparator.create();
		final Waitlist<AstarNode<S, A>> waitlist = PriorityWaitlist.create(astarComparator);

		// if it will be used for more than findig covering node then recheck
		reachedSet.addAll(arg.getNodes());
		if (root != null) {
			waitlist.add(astarRoot);
		} else {
			waitlist.addAll(arg.getIncompleteNodes().map(astarArg::get));
		}

		// if root == null go until target reached
		// if root != null go until from node target is reached again
		boolean targetReachedAgain = false;
		if (root != null) {
			assert stopCriterion.canStop(arg);
		}
		if (!stopCriterion.canStop(arg) || root != null) {
			while (!waitlist.isEmpty()) {
				final AstarNode<S, A> astarNode = waitlist.remove();
				final ArgNode<S, A> node = astarNode.argNode;

				// only infinite AstarNodes
				//	if only those nodes are left in waitlist which can't reach error then stop
				//	by not adding infinite state nodes only init nodes can cause this
				if (astarNode.parent != null) {
					if (astarNode.parent.heuristicState == AstarNode.HeuristicState.INFINITE) {
						assert arg.getInitNodes().collect(Collectors.toList()).contains(node);
						break;
					}
				}

				// we close nodes as we add them to arg => we can and should exit early
				if (node.isCovered()) {
					// no need to check stopCriterion.canStop as its newNodes parameter will be empty resulting in false
					// 	however we need to check whether coverer reaches error if root != null
					if (root == null) {
						continue;
					}

					assert node.getCoveringNode().isPresent();
					ArgNode<S, A> coverer = node.getCoveringNode().get();
					AstarNode<S, A> astarCoverer = astarArg.get(coverer);
					assert astarCoverer.heuristicState != AstarNode.HeuristicState.PARENT_UNKNOWN;

					if (astarCoverer.heuristicState == AstarNode.HeuristicState.EXACT) {
						targetReachedAgain = true;
						break;
					}
					// we must also continue expanding the covering node
					if (astarCoverer.heuristicState == AstarNode.HeuristicState.UNKNOWN) {
						waitlist.add(astarCoverer);
						continue;
					}

					assert astarCoverer.heuristicState == AstarNode.HeuristicState.INFINITE;
					continue;
				}

				// in already expanded Arg part
				// 	this will only happen if root != null as if it was null then we would only add incomplete nodes
				if (node.isExpanded()) {
					assert root != null;

					final Collection<ArgNode<S, A>> succNodes = handleExpanded(astarArg, waitlist, astarNode);
					if (stopCriterion.canStop(arg, succNodes)) {
						targetReachedAgain = true;
						break;
					}
					continue;
				}

				// expand
				Collection<ArgNode<S, A>> newNodes = Collections.emptyList();
				// only left here to cover initNodes or root TODO place this elsewhere, replace !node.isSubsumed with node.isFeasible()
				close(node, reachedSet.get(node));
				if (!node.isSubsumed() && !node.isTarget()) {
					// node is not expanded
					assert node.getOutEdges().count() == 0;

					newNodes = argBuilder.expand(node, prec);
					// add to reachedSet before:
					// 	filtering
					// 	covering (like in BasicAbstractor)
					reachedSet.addAll(newNodes);

					// when we expand a node which will be covered, then we can't find its parent from current node
					//	therefore we should find coverer before and find parent in coverer
					newNodes.forEach(newNode -> close(newNode, reachedSet.get(newNode)));

					Collection<AstarNode<S, A>> newAstarNodes = getNewAstarNodes(astarArg, astarNode)
						.filter(
								// do not add nodes with already known infinite distance
								newAstarNode -> newAstarNode.heuristicState != AstarNode.HeuristicState.INFINITE
						).collect(Collectors.toList());
					// we only need INFINITE heuristic new nodes to be added to reachedSet
					newNodes = newAstarNodes.stream().map(newAstarNode -> newAstarNode.argNode).collect(Collectors.toList());

					// add covered nodes as well in order to check whether coverer reaches error
					waitlist.addAll(newAstarNodes);
				}
				if (stopCriterion.canStop(arg, newNodes)) {
					targetReachedAgain = true;
					break;
				}
				// when walking in previous arg we can only reach error from
				// 	- expanding to target
				//	- getting a covering edge to a node already reaching target
				if (root != null) {
					if (node.getCoveringNode().isPresent()) {
						final ArgNode<S, A> coveringArgNode = node.getCoveringNode().get();
						final AstarNode<S, A> coveringAstarNode = astarArg.get(coveringArgNode);
						assert coveringAstarNode != null;
						if (coveringAstarNode.heuristicState == AstarNode.HeuristicState.EXACT) {
							targetReachedAgain = true;
							break;
						}
					}
				}
			}
		} else {
			targetReachedAgain = true;
		}

		logger.write(Level.SUBSTEP, "done%n");
		logger.write(Level.INFO, "|  |  Finished ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());

		// waitlist.clear(); // Optimization

		if (arg.isSafe()) {
			visualize(String.format("end %s", visualizerState),  astarArg.iteration);
			// checkState(arg.isComplete(), "Returning incomplete ARG as safe");
			return AbstractorResult.safe();
		} else {
			updateDistances(astarArg, root, targetReachedAgain);
			visualize(String.format("end %s", visualizerState),  astarArg.iteration);
			return AbstractorResult.unsafe();
		}
	}

	public String getVisualizerState(ArgNode<S, A> root) {
		String visualizerState = "";
		if (root != null) {
			visualizerState += String.format("N%d", root.getId());
		} else {
			visualizerState += "root";
		}
		return visualizerState;
	}

	public void initializeArg(final AstarArg<S, A, P> astarArg, final P prec) {
		final ARG<S, A> arg = astarArg.arg;

		logger.write(Level.SUBSTEP, "|  |  (Re)initializing ARG...");
		argBuilder.init(arg, prec);

		// create AstarNodes for init ArgNodes
		//	output of argBuilder.init(...) is not used for clarity
		final List<ArgNode<S, A>> initArgNodes = arg.getInitNodes().collect(Collectors.toList());
		for (ArgNode<S, A> initArgNode : initArgNodes) {
			assert !initArgNode.isCovered();
		}
		// on first arg it will be empty
		//	putAllFromCandidates sets parent null
		//	and sets state to HEURISTIC_UNKNOWN
		final Collection<AstarNode<S, A>> initAstarNodeCandidates = new ArrayList<>();
		if (astarArg.parent != null) {
			initAstarNodeCandidates.addAll(astarArg.parent.getAllInit().values());
		}

		for (ArgNode<S, A> initArgNode : initArgNodes) {
			// TODO duplicated code
			AstarNode<S, A> initAstarNodeParent = null;
			if (astarArg.parent != null) {
				initAstarNodeParent = astarArg.getParentFromCandidates(initArgNode, initAstarNodeCandidates);
				assert initAstarNodeParent != null;
			}
			final AstarNode<S, A> newInitAstarNode = AstarNode.create(initArgNode, initAstarNodeParent);
			astarArg.putInit(newInitAstarNode);
			// must be called after adding AstarNode
			//	- when we go back to previous arg to calculate heuristic: we need the node to be in visualization
			if (astarArg.parent != null) {
				calculateHeuristic(initAstarNodeParent, astarArg.parent);
				newInitAstarNode.recalculateState();
			}
		}

		logger.write(Level.SUBSTEP, "done%n");
	}

	public Collection<ArgNode<S, A>> handleExpanded(
			final AstarArg<S, A, P> astarArg,
			final Waitlist<AstarNode<S, A>> waitlist,
			final AstarNode<S, A> astarNode
	) {
		final ArgNode<S, A> node = astarNode.argNode;

		// when walking from an already expanded node just add outedges to waitlist

		// do not add nodes with already known infinite distance
		final Collection<AstarNode<S, A>> succAstarNodes = node.getOutEdges()
				.map(succEdge -> astarArg.get(succEdge.getTarget()))
				.filter(succAstarNode -> succAstarNode.heuristicState != AstarNode.HeuristicState.INFINITE)
				.collect(Collectors.toList());
		// astarArgStore already contains them
		// reached set already contains them
		waitlist.addAll(succAstarNodes);

		// if target was (as it is expanded) reached from this node => we should already have heuristics
		// 	but this function is not specific to heuristic search so don't assert this
		// succNodes is subset of all nodes => must have reached target already
		final Collection<ArgNode<S, A>> succNodes = succAstarNodes.stream()
				.map(succAstarNode -> astarNode.argNode)
				.collect(Collectors.toList());
		return succNodes;
	}

	public Stream<AstarNode<S, A>> getNewAstarNodes(AstarArg<S, A, P> astarArg, AstarNode<S, A> astarNode) {
		ArgNode<S, A> node = astarNode.argNode;

		// a node can have children b and c. b can get covered by c. b can be before c in outEdges.
		// 	getParentAstarNodeCandidates will look at coverer's parent which does not exists => reverse order
		// TODO faster? or move reachedSet add later
		List<ArgEdge<S, A>> outEdges = node.getOutEdges().collect(Collectors.toList());
		Collections.reverse(outEdges);
		return outEdges.stream().map(newArgEdge -> {
			final ArgNode<S, A> newNode = newArgEdge.getTarget();
			// newNode is really new
			assert astarArg.get(newNode) == null;

			Collection<AstarNode<S, A>> succAstarNodeCandidates = getParentAstarNodeCandidates(astarArg, astarNode, newArgEdge);

			// parent AstarNode map
			// 	TODO duplicated code in init nodes
			AstarNode<S, A> newAstarNodeParent = null;
			if (astarArg.parent != null) {
				newAstarNodeParent = astarArg.getParentFromCandidates(newNode, succAstarNodeCandidates);
				assert newAstarNodeParent != null;
			}
			final AstarNode<S, A> newAstarNode = AstarNode.create(newNode, newAstarNodeParent);
			astarArg.put(newAstarNode);

			// parent heurisitcs calculate to be used in waitlist
			// must be called after adding AstarNode
			//	- when we go back to previous arg to calculate heuristic: we need the node to be in visualization
			if (astarArg.parent != null) {
				calculateHeuristic(newAstarNodeParent, astarArg.parent);
				newAstarNode.recalculateState();
			}

			return newAstarNode;
		});
	}

	public Collection<AstarNode<S, A>> getParentAstarNodeCandidates(
			final AstarArg<S, A, P> astarArg,
			final AstarNode<S, A> astarNode,
			final ArgEdge<S, A> newArgEdge
	) {
		// if covered then we have look for parent in coverer
		if (newArgEdge.getTarget().isCovered()) {
			assert newArgEdge.getTarget().getCoveringNode().isPresent();
			final ArgNode<S, A> coverer = newArgEdge.getTarget().getCoveringNode().get();
			final AstarNode<S, A> astarCoverer = astarArg.get(coverer);
			assert astarCoverer != null;
			return Collections.singletonList(astarCoverer.parent);
		}

		if (astarNode.parent == null) {
			return new ArrayList<>();
		}
		assert astarArg.parent != null;

		// covered nodes are expanded in their covering nodes
		ArgNode<S, A> parentNode = astarNode.parent.argNode;
		if (parentNode.isCovered()) {
			assert parentNode.getCoveringNode().isPresent();
			parentNode = parentNode.getCoveringNode().get();
		}

		final Collection<AstarNode<S, A>> parentAstarNodeCandidates = parentNode.getOutEdges().
				filter(succArgEdgeCandidate -> succArgEdgeCandidate.getAction().equals(newArgEdge.getAction()))
				.map(succArgEdgeCandidate -> astarArg.parent.get(succArgEdgeCandidate.getTarget()))
				.collect(Collectors.toList());
		// add parent as nodes may can be split
		parentAstarNodeCandidates.add(astarNode.parent);

		// check if ::get was successful
		for (AstarNode<S, A> succAstarNodeCandidate: parentAstarNodeCandidates) {
			assert succAstarNodeCandidate != null;
		}
		return parentAstarNodeCandidates;
	}

	public void updateDistances(final AstarArg<S, A, P> astarArg, final ArgNode<S, A> root, boolean targetReachedAgain) {
		ARG<S, A> arg = astarArg.arg;

		// distance to error as new heuristic

		// arg unsafe can because
		// - we just expanded the arg until error is reached
		// - we went back to a previous arg (this) and expanded other part of it
		// in the latter case arg must be unsafe otherwise algorithm would have ended there

		if (type == Type.FULL) {
			assert root == null;
		}

		if (root == null) {
			final Map<ArgNode<S, A>, Integer> distances = arg.getDistances();
			distances.forEach((argNode, distance) -> {
				AstarNode<S, A> astarNode = astarArg.get(argNode);
				astarNode.heuristicState = AstarNode.HeuristicState.EXACT;
				astarNode.distanceToError = distance;
			});

			if (type == Type.FULL) {
				astarArg.getAll().values().forEach(astarNode -> {
					if (astarNode.heuristicState != AstarNode.HeuristicState.EXACT) {
						astarNode.heuristicState = AstarNode.HeuristicState.INFINITE;
					}
				});
			}

			return;
		}

		// Apply now available distance to found error to all nodes reaching error, not only root
		//	if not searched from init nodes then parent nodes will also get updated

		if (targetReachedAgain) {
			final Map<ArgNode<S, A>, Integer> distances = arg.getDistances();
			distances.forEach((argNode, distance) -> {
				AstarNode<S, A> astarNode = astarArg.get(argNode);
				astarNode.heuristicState = AstarNode.HeuristicState.EXACT;
				astarNode.distanceToError = distance;
			});
		} else {
			// if a cover edge pointed to a node reaching target then root would also have reached target in the first arg expand
			// 	=> we wouldn't have gone back to this arg from this root

			arg.walk(root, (argNode, integer) -> {
				AstarNode<S, A> astarNode = astarArg.get(argNode);

				// if we reach a part where target is reachable then root shouldn't be unreachable
				assert(astarNode.heuristicState != AstarNode.HeuristicState.EXACT);

				astarNode.heuristicState = AstarNode.HeuristicState.INFINITE;

				return false;
			});
		}
	}

	@Override
	// uses previous AstarArg then calls checkFromNode with root=null
	// it is assumed that last AstarArg in astarArgStore should be used if exists
	public AbstractorResult check(final ARG<S, A> arg, final P prec) {
		checkNotNull(arg);
		checkNotNull(prec);

		AstarArg<S, A, P> astarArg;
		if (astarArgStore.isEmpty()) {
			astarArg = AstarArg.create(arg, prec, null, astarArgStore.partialOrd);
			astarArgStore.add(astarArg);
		} else {
			astarArg = astarArgStore.getLast();
		}

		return checkFromNode(astarArg, prec, null);
	}

	// calculate distance to error node for it to be used as heuristic for next arg
	public void calculateHeuristic(final AstarNode<S, A> astarNode, final AstarArg<S, A, P> astarArg) {
		checkNotNull(astarNode);
		checkNotNull(astarArg);

		boolean backPrinted = false;
		switch (astarNode.heuristicState) {
			case EXACT:
			case INFINITE:
				return;
			case PARENT_UNKNOWN:
				assert astarNode.parent != null;

				// notify that we are going back
				visualize(String.format("back for N%d", astarNode.argNode.getId()), astarArg.iteration + 1);
				backPrinted = true;

				calculateHeuristic(astarNode.parent, astarArg.parent);
				assert astarNode.parent.heuristicState == AstarNode.HeuristicState.EXACT || astarNode.parent.heuristicState == AstarNode.HeuristicState.INFINITE;

				// do not update current node's heuristic (like: exact => unknown, inf => inf)
				// - outside caller won't update also
				// 		(a call which doesn't go through this part but could update it's heuristic:
				// 		e.g. call from a node whose parent is null and heuristic is unknown but will get infinite after search)
				// - when new AstarNode will be created it will check it's parent and set it there
				astarNode.heuristicState = AstarNode.HeuristicState.UNKNOWN;
				// no break as we want to calculate as we needed heuristic to walk in parent's arg from which we get heuristic for current arg
			case UNKNOWN:
				// infinite heuristic parent
				// - don't walk arg
				// - set state here
				if (astarNode.parent != null && astarNode.parent.heuristicState == AstarNode.HeuristicState.INFINITE) {
					astarNode.heuristicState = AstarNode.HeuristicState.INFINITE;
					break;
				}

				// notify that we are going back
				if (!backPrinted) {
					visualize(String.format("back for N%d", astarNode.argNode.getId()), astarArg.iteration + 1);
				}

				// parent has heuristics to walk from astarNode in astarArg
				checkFromNode(astarArg, astarArg.prec, astarNode);

				assert astarNode.heuristicState == AstarNode.HeuristicState.EXACT || astarNode.heuristicState == AstarNode.HeuristicState.INFINITE;
				break;
			default:
				throw new IllegalArgumentException(AstarNode.IllegalState);
		}
	}

	private static final String nowText = getNowText();
	private void visualize(String state, int iteration) {
		checkNotNull(state);

		if (logger == NullLogger.getInstance()) {
			return;
		}

		StringBuilder title = new StringBuilder();
		for (int i = astarArgStore.getLastIteration(); i >= iteration ; i--) {
			title.append(String.format("%d.", i));
		}
		title.append(String.format(" %s", state));

		try {
			File directory = new File(String.format("%s/theta/%s", System.getProperty("java.io.tmpdir"), nowText));
			if (!directory.exists()) {
				boolean successful = directory.mkdirs();
				assert successful;
			}

			File file = new File(directory.getCanonicalPath());
			// '∣' != '|' (for Windows)
			File[] subfiles = file.listFiles();
			assert subfiles != null;
			String filename = String.format("%s/%d∣ %s.png", directory.getCanonicalPath(), subfiles.length + 1, title);

			GraphvizWriter.getInstance().writeFileAutoConvert(AstarArgVisualizer.getDefault().visualize(astarArgStore.getIteration(iteration), title.toString()), filename);
		} catch (IOException | InterruptedException e) {
			throw new RuntimeException(e);
		}
	}

	private static String getNowText() {
		LocalDateTime now = LocalDateTime.now();
		DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyy_MM_dd HH_mm_ss");
		return dateTimeFormatter.format(now);
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

		public AstarAbstractor<S, A, P> build() {
			assert astarArgStore != null;
			return new AstarAbstractor<>(argBuilder, projection, stopCriterion, logger, astarArgStore, type);
		}
	}

}
