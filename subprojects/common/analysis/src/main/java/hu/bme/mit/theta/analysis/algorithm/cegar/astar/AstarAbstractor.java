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
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.Logger.Level;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;

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
	public AbstractorResult checkFromNode(final AstarArg<S, A, P> astarArg, final P prec, final AstarNode<S, A> astarRoot) {
		ARG<S, A> arg = astarArg.arg;
		ArgNode<S, A> root = null;
		if (astarRoot != null) {
			root = astarRoot.argNode;
		}
		checkNotNull(prec);
		logger.write(Level.DETAIL, "|  |  Precision: %s%n", prec);

		String visualizerState = "";
		if (root != null) {
			visualizerState += String.format("N%d", root.getId());
		} else {
			visualizerState += "root";
		}
		visualize(String.format("start %s", visualizerState),  astarArg.iteration);

		// initialize Arg
		assert root == null || arg.isInitialized();
		if (root == null && !arg.isInitialized()) {
			logger.write(Level.SUBSTEP, "|  |  (Re)initializing ARG...");
			argBuilder.init(arg, prec);

			// create AstarNodes for init ArgNodes
			//	output of argBuilder.init(...) is not used for clarity
			List<ArgNode<S, A>> initArgNodes = arg.getInitNodes().collect(Collectors.toList());
			// TODO init nodes shouldn't be covered?
			for (ArgNode<S, A> initArgNode : initArgNodes) {
				assert !initArgNode.isCovered();
			}
			// on first arg it will be empty
			//	putAllFromCandidates sets descendant null
			//	and sets state to HEURISTIC_UNKNOWN
			Collection<AstarNode<S, A>> initAstarNodeCandidates = new ArrayList<>();
			if (astarArg.descendant != null) {
				initAstarNodeCandidates.addAll(astarArg.descendant.getAllInitNode().values());
			}

			for (ArgNode<S, A> initArgNode : initArgNodes) {
				// TODO duplicated code
				AstarNode<S, A> initAstarNodeDescendant = null;
				if (astarArg.descendant != null) {
					initAstarNodeDescendant = astarArg.getDescendantFromCandidates(initArgNode, initAstarNodeCandidates);
					assert initAstarNodeDescendant != null;
				}
				AstarNode<S, A> newInitAstarNode = AstarNode.create(initArgNode, initAstarNodeDescendant);
				astarArg.putInitNode(newInitAstarNode);
				// must be called after adding AstarNode
				//	- when we go back to previous arg to calculate heuristic: we need the node to be in visualization
				if (astarArg.descendant != null) {
					calculateHeuristic(initAstarNodeDescendant, astarArg.descendant);
					newInitAstarNode.recalculateState();
				}
			}

			logger.write(Level.SUBSTEP, "done%n");
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
				if (astarNode.descendant != null) {
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
					Collection<AstarNode<S, A>> succAstarNodes = node.getOutEdges()
							.map(succEdge -> astarArg.get(succEdge.getTarget()))
							.filter(succAstarNode -> succAstarNode.state != AstarNode.State.HEURISTIC_INFINITE)
							.collect(Collectors.toList());
					// astarArgStore already contains them
					// reached set already contains them
					waitlist.addAll(succAstarNodes);

					// if target was (as it is expanded) reached from this node => we should already have heuristics
					// 	but this function is not specific to heuristic search so don't assert this
					// succNodes is subset of all nodes => must have reached target already
					Collection<ArgNode<S, A>> succNodes = succAstarNodes.stream()
							.map(succAstarNode -> astarNode.argNode)
							.collect(Collectors.toList());
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
					node.getOutEdges().forEach((ArgEdge<S, A> newArgEdge) -> {
						ArgNode<S, A> newArgNode = newArgEdge.getTarget();
						Collection<AstarNode<S, A>> succAstarNodeCandidates = new ArrayList<>();
						if (astarNode.descendant != null) {
							assert astarArg.descendant != null;

							// covered nodes are expanded in their covering nodes
							ArgNode<S, A> descendantArgNode = astarNode.descendant.argNode;
							if (descendantArgNode.isCovered()) {
								assert descendantArgNode.getCoveringNode().isPresent();
								descendantArgNode = descendantArgNode.getCoveringNode().get();
							}

							Stream<ArgEdge<S, A>> succArgEdgeCandidates = descendantArgNode.getOutEdges().
									filter((ArgEdge<S, A> succArgEdgeCandidate) -> succArgEdgeCandidate.getAction().equals(newArgEdge.getAction()));
							succAstarNodeCandidates = succArgEdgeCandidates
									.map(succArgEdgeCandidate -> astarArg.descendant.get(succArgEdgeCandidate.getTarget()))
									.collect(Collectors.toList());

							// check if ::get was successful
							for (AstarNode<S, A> succAstarNodeCandidate: succAstarNodeCandidates) {
								assert succAstarNodeCandidate != null;
							}
						}

						// this is logically separate from previous block with same condition (this block appears in init nodes)
						AstarNode<S, A> newAstarNodeDescendant = null;
						if (astarArg.descendant != null) {
							newAstarNodeDescendant = astarArg.getDescendantFromCandidates(newArgNode, succAstarNodeCandidates);
							assert newAstarNodeDescendant != null;
						}
						AstarNode<S, A> newAstarNode = AstarNode.create(newArgNode, newAstarNodeDescendant);
						astarArg.put(newAstarNode);
						// must be called after adding AstarNode
						//	- when we go back to previous arg to calculate heuristic: we need the node to be in visualization
						if (astarArg.descendant != null) {
							calculateHeuristic(newAstarNodeDescendant, astarArg.descendant);
							newAstarNode.recalculateState();
						}
					});

					// do not add nodes with already known infinite distance
					newNodes = newNodes.stream().filter(newNode -> {
						AstarNode<S, A> newAstarNode = astarArg.get(newNode);
						return newAstarNode.state != AstarNode.State.HEURISTIC_INFINITE;
					}).collect(Collectors.toList());

					reachedSet.addAll(newNodes);
					waitlist.addAll(newNodes.stream().map(astarArg::get));
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
						ArgNode<S, A> coveringArgNode = node.getCoveringNode().get();
						AstarNode<S, A> coveringAstarNode = astarArg.get(coveringArgNode);
						assert coveringAstarNode != null;
						if (coveringAstarNode.state == AstarNode.State.HEURISTIC_EXACT) {
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

		// distance to error as new heuristic
		if (arg.isSafe()) {
			visualize(String.format("end %s", visualizerState),  astarArg.iteration);
			// checkState(arg.isComplete(), "Returning incomplete ARG as safe");
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

				visualize(String.format("end %s", visualizerState),  astarArg.iteration);
				return AbstractorResult.unsafe();
			}

			// Apply now available distance to found error to all nodes reaching error, not only root
			//	if not searched from init nodes then descendant nodes will also get updated
			// TODO if a loop is detected during run == closed to it's descendant => give infinite weight?

			if (targetReachedAgain) {
				// TODO maybe we can be more efficient: when coming from covering node just go from there
				// 	when coming from target just go straigth up to root somehow
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

			visualize(String.format("end %s", visualizerState),  astarArg.iteration);
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
		checkNotNull(astarNode);
		checkNotNull(astarArg);

		boolean backPrinted = false;
		switch (astarNode.state) {
			case HEURISTIC_EXACT:
			case HEURISTIC_INFINITE:
				return;
			case DESCENDANT_HEURISTIC_UNKNOWN:
				assert astarNode.descendant != null;

				// notify that we are going back
				visualize(String.format("back for N%d", astarNode.argNode.getId()), astarArg.iteration + 1);
				backPrinted = true;

				calculateHeuristic(astarNode.descendant, astarArg.descendant);
				assert astarNode.descendant.state == AstarNode.State.HEURISTIC_EXACT || astarNode.descendant.state == AstarNode.State.HEURISTIC_INFINITE;

				// do not update current node's heuristic (like: exact => unknown, inf => inf)
				// - outside caller won't update also
				// 		(a call which doesn't go through this part but could update it's heuristic:
				// 		e.g. call from a node whose descendant is null and heuristic is unknown but will get infinite after search)
				// - child node will simply call calculateHeuristic() and  TODO ??
				// - when new AstarNode will be created it will check it's descendant and set it there
				astarNode.state = AstarNode.State.HEURISTIC_UNKNOWN;
				// no break as we want to calculate as we needed heuristic to walk in descendant's arg from which we get heuristic for current arg
			case HEURISTIC_UNKNOWN:
				// infinite heuristic descendant
				// - don't walk arg
				// - set state here
				if (astarNode.descendant != null && astarNode.descendant.state == AstarNode.State.HEURISTIC_INFINITE) {
					astarNode.state = AstarNode.State.HEURISTIC_INFINITE;
					break;
				}

				// notify that we are going back
				if (!backPrinted) {
					visualize(String.format("back for N%d", astarNode.argNode.getId()), astarArg.iteration + 1);
				}

				// descendant has heuristics to walk from astarNode in astarArg
				// TODO color root
				checkFromNode(astarArg, astarArg.prec, astarNode);

				assert astarNode.state == AstarNode.State.HEURISTIC_EXACT || astarNode.state == AstarNode.State.HEURISTIC_INFINITE;
				break;
			default:
				throw new IllegalArgumentException(AstarNode.IllegalState);
		}
	}

	private static final String nowText = getNowText();
	private void visualize(String state, int iteration) {
		if (logger == NullLogger.getInstance()) {
			return;
		}

		StringBuilder title = new StringBuilder();
		for (int i = astarArgStore.size(); i >= iteration ; i--) {
			title.append(String.format("%d.", i));
		}
		title.append(String.format(" %s", state));

		try {
			File directory = new File(String.format("%s/theta/%s", System.getProperty("java.io.tmpdir"), nowText));
			if (!directory.exists()) {
				directory.mkdirs();
			}

			File file = new File(directory.getCanonicalPath());
			String filename = String.format("%s/%d| %s.png", directory.getCanonicalPath(), file.listFiles().length + 1, title);

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
