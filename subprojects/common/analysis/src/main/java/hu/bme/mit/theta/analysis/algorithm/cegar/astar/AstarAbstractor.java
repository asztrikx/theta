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
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg.Search;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode.DistanceType;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode.Distance;

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
	private final AstarVisualizer<S, A, P> astarVisualizer;
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
		this.astarVisualizer = new AstarVisualizer<>(logger, astarArgStore);
	}

	public static <S extends State, A extends Action, P extends Prec> Builder<S, A, P> builder(
			final ArgBuilder<S, A, P> argBuilder) {
		return new Builder<>(argBuilder);
	}

	@Override
	public ARG<S, A> createArg() {
		return argBuilder.createArg();
	}

	public boolean astarExpand(AstarArg<S, A, P> astarArg, AstarStopCriterion<S, A, P> astarStopCriterion, AstarNode<S, A> startNode) {
		Collection<AstarNode<S, A>> startNodes = new ArrayList<AstarNode<S, A>>();
		startNodes.add(startNode);
		return astarExpand(astarArg, astarStopCriterion, startNodes);
	}

	// return: whether stopCriterion stopped it
	public boolean astarExpandHelper(AstarArg<S, A, P> astarArg, AstarStopCriterion<S, A, P> stopCriterion, Collection<AstarNode<S, A>> startNodes) {
		// create search
		Search<S, A> search = new Search<>();
		Waitlist<AstarNode<S, A>> waitlist = search.waitlist; // can't extract value => do not be a parameter
		Set<AstarNode<S, A>> doneSet = search.doneSet; // search.doneSet: we could already have started to explore this subgraph => do not use global doneSet
		Map<ArgNode<S, A>, ArgNode<S, A>> parents = search.parents; // local
		Map<AstarNode<S, A>, Integer> depths = search.depths;

		// start nodes to search
		startNodes.forEach(startNode -> depths.put(startNode, 0)); // before putting into waitlist
		waitlist.addAll(startNodes);

		// after prune!!!!: astarArg.reachedSet.addAll(arg.getNodes()); or in init?
		throw new RuntimeException();
		// put parents

		while (!waitlist.isEmpty()) {
			AstarNode<S, A> astarNode = waitlist.remove();
			ArgNode<S, A> argNode = astarNode.argNode;
			Integer depth = depths.get(astarNode);
			assert astarNode.getHeuristic().getType() != DistanceType.INFINITE;

			// reached target
			if (argNode.isTarget()) {
				astarArg.updateDistancesFromTarget(astarNode, );
				if (stopCriterion.canStop(astarArg, astarNode)) {
					return true;
				}
				continue;
			}

			if (stopCriterion.canStop(astarArg, astarNode)) {
				return true;
			}

			// lazy propagation
			if (doneSet.contains(astarNode)) {
				continue;
			}
			doneSet.add(astarNode);

			// covering handle
			if (/*cover check*/) {
				ArgNode<S, A> coveringNode = argNode.getCoveringNode().get();
				assert !doneSet.contains(coveringNode.arg??) || coveringNode.getDepth() <= argNode.getDepth();
				assert astarArg.get(coveringNode).distance.isKnown();
				// close(node, reachedSet.get(node));
				throw new RuntimeException(); // what is the depth of covering node? it is LIKELY different from what we need for a*
				// also special case: covering node already has distance
				waitlist.add(astarArg.get(coveringNode));
				continue;

				throw new RuntimeException();
				// covering subgraph has distance
			}
			//if (!node.isSubsumed() && !node.isTarget())

			throw new RuntimeException(); // node has distance

			// expand
			Collection<ArgNode<S, A>> newNodes = argBuilder.expand(argNode, astarArg.prec);

			// some nodes may already have been expanded and remained after prune => we can't use result of expand
			argNode.getSuccNodes().forEach(newArgNode -> {
				AstarNode<S, A> ancestor = astarArg.findAstarCoveringNode(newArgNode, astarNode);

				AstarNode<S, A> newAstarNode = AstarNode.create(newArgNode, ancestor);
				astarArg.put(newAstarNode);
				astarArg.reachedSet.add(newArgNode);

				// Do not add node which has no chance in reaching target
				if (ancestor.distance.getType() == DistanceType.EXACT) {
					// depths is used for waitlist ordering => before waitlist
					depths.put(newAstarNode, depth + 1);
					waitlist.add(newAstarNode);
					// parents is used for trace back from target
					parents.put(newArgNode, argNode);
				}
			});
		}

		return false;
	}

	// startNodes should already be in astarArg.reachedSet
	// return: whether stopCriterion stopped it
	public boolean astarExpand(AstarArg<S, A, P> astarArg, AstarStopCriterion<S, A, P> stopCriterion, Collection<AstarNode<S, A>> startNodes) {
		final P prec = astarArg.prec;
		final ARG<S, A> arg = astarArg.arg;

		logger.write(Level.DETAIL, "|  |  Precision: %s%n", prec);
		logger.write(Level.INFO, "|  |  Starting ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.SUBSTEP, "|  |  Building ARG...");
		String visualizerState = AstarVisualizer.getVisualizerState(startNodes);
		astarVisualizer.visualize(String.format("start %s", visualizerState), astarArg.iteration);

		// have heuristic for start nodes <= node for which distance we are going back may not have heuristic
		startNodes.forEach(startNode -> findHeuristic(startNode, astarArg, visualizerState));
		startNodes = startNodes.stream()
				.filter(startNode -> startNode.getHeuristic().getType() != DistanceType.INFINITE)
				.collect(Collectors.toList());

		boolean found = false;
		if (!stopCriterion.canStop(astarArg)) { // this is at max only for leftover nodes??
			found = astarExpandHelper(astarArg, stopCriterion, startNodes);
		}

		logger.write(Level.SUBSTEP, "done%n");
		logger.write(Level.INFO, "|  |  Finished AstarARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		astarVisualizer.visualize(String.format("end %s", visualizerState), astarArg.iteration);

		if (/*target reached, did it had the chance? bc of stop criterion? should */) {
			astarArg.updateDistancesInfinite(startNodes);
		}

		return found;
	}

	public void initAstarArg(final AstarArg<S, A, P> astarArg) {
		astarArg.arg.getInitNodes().forEach(initArgNode -> {
			// TODO wrong assumption: (leftover from prune) init nodes can not be covered
			assert !initArgNode.isCovered();

			// find astar covering
			AstarNode<S, A> coveringNode = astarArg.findAstarCoveringNode(initArgNode, null);
			assert astarArg.parent == null || coveringNode != null;

			// create init astar node
			AstarNode<S, A> newInitAstarNode = AstarNode.create(initArgNode, coveringNode);
			astarArg.putInit(newInitAstarNode);

			// astarExpand will find heuristic for it, it's not this function's job
		});
	}

	// astarNode should already have coveringNode
	public void findHeuristic(AstarNode<S, A> astarNode, AstarArg<S, A, P> astarArg, String visualizerState) {
		// no previous astar arg exists => nothing to find (getHeuristic returns (EXACT, 0))
		if (astarArg.parent == null) {
			return;
		}

		// already found (stopCriterion precheck)
		if (astarNode.getHeuristic().isKnown()) {
			return;
		}
		assert type != Type.FULL;

		// parent infinite => current is also infinite
		if (astarNode.getHeuristic().getType() == DistanceType.INFINITE) {
			// this is a bit out of place? should we update subgraph? like what if go back for this node to expand? that is not possible
			// but this should be documented that why we do not do anything
			astarNode.distance = new Distance(DistanceType.INFINITE);
			return;
		}

		// visualize current before going back to previous astarArg
		astarVisualizer.visualize(String.format("paused %s", visualizerState), astarArg.iteration);

		// reachedSet already contains argNode
		//assert coveringAstarArg.reachedSet.get(coveringAstarNode.argNode) != null;
		//assert coveringAstarArg.reachedSet.get(coveringAstarNode.argNode).contains(coveringAstarNode.argNode);

		// findHeuristic == findDistance in parent
		AstarNode<S, A> coveringAstarNode = astarNode.coveringAstarNode;
		AstarArg<S, A, P> coveringAstarArg = astarArg.parent;
		astarExpand(coveringAstarArg, new AstarStopCriterionDistanceKnown<>(), coveringAstarNode);

		assert astarNode.getHeuristic().isKnown();

		// visualize current after going back to previous astarArg
		astarVisualizer.visualize(String.format("resumed %s", visualizerState), astarArg.iteration);
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
			return Collections.singletonList(astarCoverer.coveringAstarNode);
		}

		if (astarNode.coveringAstarNode == null) {
			return new ArrayList<>();
		}
		assert astarArg.parent != null;

		// covered nodes are expanded in their covering nodes
		ArgNode<S, A> parentNode = astarNode.coveringAstarNode.argNode;
		if (parentNode.isCovered()) {
			assert parentNode.getCoveringNode().isPresent();
			parentNode = parentNode.getCoveringNode().get();
		}

		final Collection<AstarNode<S, A>> parentAstarNodeCandidates = parentNode.getOutEdges().
				filter(succArgEdgeCandidate -> succArgEdgeCandidate.getAction().equals(newArgEdge.getAction()))
				.map(succArgEdgeCandidate -> astarArg.parent.get(succArgEdgeCandidate.getTarget()))
				.collect(Collectors.toList());
		// add parent as nodes may can be split
		parentAstarNodeCandidates.add(astarNode.coveringAstarNode);

		// check if ::get was successful
		for (AstarNode<S, A> succAstarNodeCandidate: parentAstarNodeCandidates) {
			assert succAstarNodeCandidate != null;
		}
		return parentAstarNodeCandidates;
	}

	@Override
	// uses previous AstarArg then calls checkFromNode with root=null
	// it is assumed that last AstarArg in astarArgStore should be used if exists
	public AbstractorResult check(final ARG<S, A> arg, final P prec) {
		checkNotNull(arg);
		checkNotNull(prec);

		AstarArg<S, A, P> astarArg = astarArgStore.getLast();

		// initialize: prune can keep initialized state
		if (!arg.isInitialized()) {
			logger.write(Level.SUBSTEP, "|  |  (Re)initializing ARG...");

			argBuilder.init(arg, prec);
			initAstarArg(astarArg);

			logger.write(Level.SUBSTEP, "done%n");
		}
		assert arg.isInitialized();

		// reachedSet fill
		astarArg.reachedSet.addAll(arg.getNodes()); // init nodes + leftover from prune

		// we have to keep pruned parents, ... :/
		//throw new RuntimeException(); //arg.getIncompleteNodes().map(astarArg::get).filter(n -> n.distance.getType() != DistanceType.INFINITE)
			// is found used correctly?

		boolean found = astarExpand(astarArg, AstarStopCriterion.of(stopCriterion), astarArg.getAllAstarInit());

		assert arg.isSafe() && !found || !arg.isSafe() && found;
		if (found) {
			if (type == AstarCegarChecker.Type.FULL) {
				astarArg.setUnknownDistanceInfinite();
			}

			return AbstractorResult.unsafe();
		} else {
			// INFINITE heuristic nodes won't be expanded
			// 		checkState(arg.isComplete(), "Returning incomplete ARG as safe");
			return AbstractorResult.safe();
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
