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
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;

import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;
import static com.google.common.base.Preconditions.checkState;

/**
 * Basic implementation for the abstractor, relying on an ArgBuilder.
 */
public final class AstarAbstractor<S extends State, A extends Action, P extends Prec> implements Abstractor<S, A, P> {
	private final ArgBuilder<S, A, P> argBuilder;
	private final Function<? super S, ?> projection;
	// can't have waitlist common in AstarAbstractor instance as checkFromNode is recursively called
	private final StopCriterion<S, A> stopCriterion;
	private final Logger logger;
	private final AstarArgStore<S, A, P> astarArgStore;

	// a* specific
	// TODO should these change during run? or event should there be local versions

	private AstarAbstractor(final ArgBuilder<S, A, P> argBuilder,
							final Function<? super S, ?> projection,
							final StopCriterion<S, A> stopCriterion,
							final Logger logger,
							final AstarArgStore<S, A, P> astarArgStore) {
		this.argBuilder = checkNotNull(argBuilder);
		this.projection = checkNotNull(projection);
		this.stopCriterion = checkNotNull(stopCriterion);
		this.logger = checkNotNull(logger);
		this.astarArgStore = checkNotNull(astarArgStore);
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
	public AbstractorResult checkFromNode(final AstarArg<S, A, P> astarArg, final P prec, final ArgNode<S, A> root) {
		ARG<S, A> arg = astarArg.arg;
		checkNotNull(prec);
		logger.write(Level.DETAIL, "|  |  Precision: %s%n", prec);

		// initialize Arg
		assert root == null || arg.isInitialized();
		if (root == null && !arg.isInitialized()) {
			logger.write(Level.SUBSTEP, "|  |  (Re)initializing ARG...");
			argBuilder.init(arg, prec);

			// create AstarNodes for init ArgNodes
			//	output of argBuilder.init(...) is not used for clarity
			List<ArgNode<S, A>> initArgNodes = arg.getInitNodes().collect(Collectors.toList());
			// init nodes shouldn't be covered
			for (ArgNode<S, A> initArgNode : initArgNodes) {
				assert !initArgNode.isCovered();
			}
			// on first arg it will be empty
			//	putAllFromCandidates sets descendant null
			//	and sets state to DESCENDANT_HEURISTIC_UNAVAILABLE
			Collection<AstarNode<S, A>> initAstarNodeCandidates = new ArrayList<>();
			if (astarArg.descendant != null) {
				initAstarNodeCandidates.addAll(astarArg.descendant.getAllInitNode().values());
			}
			Collection<AstarNode<S, A>> initAstarNodes = astarArg.putAllFromCandidates(initArgNodes, initAstarNodeCandidates, true);
			initAstarNodes.forEach(astarNode -> calculateHeuristic(astarNode.descendant, astarArg.descendant));

			logger.write(Level.SUBSTEP, "done%n");
		}
		assert arg.isInitialized();

		logger.write(Level.INFO, "|  |  Starting ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());
		logger.write(Level.SUBSTEP, "|  |  Building ARG...");

		final Partition<ArgNode<S, A>, ?> reachedSet = Partition.of(n -> projection.apply(n.getState()));
		final AstarComparator<S, A, P> astarComparator = AstarComparator.create(astarArg);
		final Waitlist<ArgNode<S, A>> waitlist = PriorityWaitlist.create(astarComparator);

		// if it will be used for more than findig covering node then recheck
		reachedSet.addAll(arg.getNodes());
		if (root != null) {
			waitlist.add(root);
		} else {
			waitlist.addAll(arg.getIncompleteNodes());
		}

		// if root == null go until target reached
		// if root != null go until from node target is reached again
		boolean targetReachedAgain = false;
		if (root != null) {
			assert stopCriterion.canStop(arg);
		}
		if (!stopCriterion.canStop(arg) || root != null) {
			while (!waitlist.isEmpty()) {
				final ArgNode<S, A> node = waitlist.remove();
				final AstarNode<S, A> astarNode = astarArg.get(node);

				// only infinite AstarNodes
				//	if only those nodes are left in waitlist which can't reach error then stop
				//	by not adding infinite state nodes only init nodes can cause this
				if (astarNode.state != AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE) {
					if (astarNode.descendant.state == AstarNode.State.HEURISTIC_INFINITE) {
						assert arg.getInitNodes().collect(Collectors.toList()).contains(node);
						break;
					}
				}

				// in already expanded Arg part
				//	can happen if root != null
				//	when walking from an already expanded node just add outedges to waitlist
				if (node.isExpanded()) {
					assert root != null;
					// do not add nodes with already known infinite distance
					Collection<ArgNode<S, A>> succNodes = node.getOutEdges().map(ArgEdge::getTarget)
							.filter(succNode -> {
								AstarNode<S, A> succAstarNode = astarArg.get(succNode);
								return succAstarNode.state != AstarNode.State.HEURISTIC_INFINITE;
							})
							.collect(Collectors.toList());
					// astarArgStore already contains them
					// reached set already contains them
					waitlist.addAll(succNodes);

					// if target was (as it is expanded) reached from this node => we should already have heuristics
					// 	but this function is not specific to heuristic search so don't assert this
					// succNodes is subset of all nodes => must have reached target already
					if (stopCriterion.canStop(arg, succNodes)) {
						targetReachedAgain = true;
						break;
					}
					continue;
				}

				// expand
				Collection<ArgNode<S, A>> newNodes = Collections.emptyList();
				close(node, reachedSet.get(node));
				if (!node.isSubsumed() && !node.isTarget()) {
					newNodes = argBuilder.expand(node, prec);

					// descendant AstarNode map
					// descendant heurisitcs calculate to be used in waitlist
					for (ArgNode<S, A> newArgNode : newNodes) {
						Collection<AstarNode<S, A>> succAstarNodeCandidates = new ArrayList<>();
						if (astarNode.descendant != null) {
							// covered nodes are expanded in their covering nodes
							ArgNode<S, A> descendantArgNode = astarNode.descendant.argNode;
							if (descendantArgNode.isCovered()) {
								assert descendantArgNode.getCoveringNode().isPresent();
								descendantArgNode = descendantArgNode.getCoveringNode().get();
							}

							Collection<ArgNode<S, A>> succArgNodeCandidates = descendantArgNode.getSuccNodes().collect(Collectors.toList());
							succAstarNodeCandidates = succArgNodeCandidates.stream()
									.map(astarArg.descendant::get).collect(Collectors.toList());

							// check if ::get was successful
							for (AstarNode<S, A> succAstarNodeCandidate: succAstarNodeCandidates) {
								assert succAstarNodeCandidate != null;
							}
						}

						AstarNode<S, A> newAstarNode = astarArg.putFromCandidates(newArgNode, succAstarNodeCandidates, false);
						calculateHeuristic(newAstarNode.descendant, astarArg.descendant);
					};

					// do not add nodes with already known infinite distance
					newNodes = newNodes.stream().filter(newNode -> {
						AstarNode<S, A> newAstarNode = astarArg.get(newNode);
						return newAstarNode.state != AstarNode.State.HEURISTIC_INFINITE;
					}).collect(Collectors.toList());

					reachedSet.addAll(newNodes);
					waitlist.addAll(newNodes);
				}
				if (stopCriterion.canStop(arg, newNodes)) {
					targetReachedAgain = true;
					break;
				}
			}
		} else {
			targetReachedAgain = true;
		}

		logger.write(Level.SUBSTEP, "done%n");
		logger.write(Level.INFO, "|  |  Finished ARG: %d nodes, %d incomplete, %d unsafe%n", arg.getNodes().count(),
				arg.getIncompleteNodes().count(), arg.getUnsafeNodes().count());

		waitlist.clear(); // Optimization

		// distance to error as new heuristic
		if (arg.isSafe()) {
			checkState(arg.isComplete(), "Returning incomplete ARG as safe");
			return AbstractorResult.safe();
		} else {
			// arg unsafe can because
			// - we just expanded the arg until error is reached
			// - we went back to a previous arg (this) and expanded other part of it
			// in the latter case arg must be unsafe otherwise algorithm would have ended there

			if (root == null) {
				final Map<ArgNode<S, A>, Integer> distances = arg.getDistances();
				distances.forEach((argNode, distance) -> {
					AstarNode<S, A> astarNode = astarArg.get(argNode);
					astarNode.state = AstarNode.State.HEURISTIC_EXACT;
					astarNode.distanceToError = distance;
				});

				return AbstractorResult.unsafe();
			}

			// Apply now available distance to found error to all nodes reaching error, not only root
			//	if not searched from init nodes then descendant nodes will also get updated
			// TODO if a loop is detected during run == closed to it's descendant => give infinite weight

			if (targetReachedAgain) {
				final Map<ArgNode<S, A>, Integer> distances = arg.getDistances();
				distances.forEach((argNode, distance) -> {
					AstarNode<S, A> astarNode = astarArg.get(argNode);
					astarNode.state = AstarNode.State.HEURISTIC_EXACT;
					astarNode.distanceToError = distance;
				});
			} else {
				// if a cover edge pointed to a node reaching target then root would also have reached target in the first arg expand
				// 	=> we wouldn't have gone back to this arg from this root

				arg.walk(root, (argNode, integer) -> {
					AstarNode<S, A> astarNode = astarArg.get(argNode);

					// if we reach a part where target is reachable then root shouldn't be unreachable
					assert(astarNode.state != AstarNode.State.HEURISTIC_EXACT);

					astarNode.state = AstarNode.State.HEURISTIC_INFINITE;

					return false;
				});
			}

			return AbstractorResult.unsafe();
		}
	}

	@Override
	// uses previous AstarArg then calls checkFromNode with root=null
	// it is assumed that last AstarArg in astarArgStore should be used if exists
	public AbstractorResult check(final ARG<S, A> arg, final P prec) {
		AstarArg<S, A, P> astarArg;
		if (astarArgStore.size() == 0) {
			astarArg = AstarArg.create(arg, prec, null, astarArgStore.partialOrd);
			astarArgStore.add(astarArg);
		} else {
			astarArg = astarArgStore.getLast();
		}

		return checkFromNode(astarArg, prec, null);
	}

	// calculate distance to error node for it to be used as heuristic for next arg
	public void calculateHeuristic(final AstarNode<S, A> astarNode, final AstarArg<S, A, P> astarArg) {
		if (astarNode == null) {
			assert astarArg == null;
			return;
		}
		checkNotNull(astarArg);

		// check for correct State value beforehand to be concise
		if (astarNode.state == AstarNode.State.DESCENDANT_HEURISTIC_UNAVAILABLE) {
			assert astarNode.descendant == null;
		} else {
			assert astarNode.descendant != null;
		}

		switch (astarNode.state) {
			case HEURISTIC_EXACT:
			case HEURISTIC_INFINITE:
				return;
			case DESCENDANT_HEURISTIC_UNKNOWN:
				calculateHeuristic(astarNode.descendant, astarArg.descendant);
				assert astarNode.descendant.state == AstarNode.State.HEURISTIC_EXACT || astarNode.descendant.state == AstarNode.State.HEURISTIC_INFINITE;

				astarNode.state = AstarNode.State.HEURISTIC_UNKNOWN;
				// no break as we want to calculate as we needed heuristic to walk in descendant's arg from which we get heuristic for current arg
			case HEURISTIC_UNKNOWN:
			case DESCENDANT_HEURISTIC_UNAVAILABLE:
				// descendant has heuristics to walk from astarNode in astarArg
				checkFromNode(astarArg, astarArg.prec, astarNode.argNode);
				assert astarNode.state == AstarNode.State.HEURISTIC_EXACT || astarNode.state == AstarNode.State.HEURISTIC_INFINITE;
				break;
			default:
				throw new IllegalArgumentException(AstarNode.IllegalState);
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

		private Builder(final ArgBuilder<S, A, P> argBuilder) {
			this.argBuilder = argBuilder;
			this.projection = s -> 0;
			this.stopCriterion = StopCriterions.firstCex();
			this.logger = NullLogger.getInstance();
			this.astarArgStore = null; // TODO
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

		public AstarAbstractor<S, A, P> build() {
			return new AstarAbstractor<>(argBuilder, projection, stopCriterion, logger, astarArgStore);
		}
	}

}
