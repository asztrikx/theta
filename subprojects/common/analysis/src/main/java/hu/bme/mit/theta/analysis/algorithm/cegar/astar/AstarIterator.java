package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgCopier;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.common.Tuple2;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Function;

/**
 * copies ARG and their ArgNode shallowly (keeping action and state)
 */
public class AstarIterator {
	// Creates a copy of the astarArg which will point to the same providerAstarNodes for each node.
	// AstarArg's providerAstarNodes however will point to the copies.
	// The changes also affect the provider field.
	// provider <-- astarArg
	// provider <-- astarArgCopy <-- astarArg
	// astarArg: the source AstarArg from which a copy will be made
	public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> createIterationReplacement(
			AstarArg<S, A, P> astarArg, P prec, final PartialOrd<S> partialOrd,
			final Function<? super S, ?> projection, AstarAbstractor<S, A, P> astarAbstractor
	) {
		Collection<Tuple2<ArgNode<S, A>, ArgNode<S, A>>> translation = new ArrayList<>();
		Set<ArgNode<S, A>> newInitNodes = new HashContainerFactory().createSet();
		ARG<S, A> argCopy = ArgCopier.createCopy(astarArg.arg, (argNode, argNodeCopy) -> {
			translation.add(Tuple2.of(argNode, argNodeCopy));
		}, (initArgNode, initArgNodeCopy) -> {
			newInitNodes.add(initArgNodeCopy);
		});

		AstarArg<S, A, P> astarArgCopy = new AstarArg<>(argCopy, prec, partialOrd, projection, astarArg);
		astarArgCopy.provider = astarArg.provider;
		astarArg.provider = astarArgCopy;

		// Covering edges are created after createCopy finished
		//  TODO this is bad design
		translation.forEach(t -> {
			ArgNode<S, A> argNode = t.get1();
			ArgNode<S, A> argNodeCopy = t.get2();

			AstarNode<S, A> astarNode = astarArg.get(argNode);

			AstarNode<S, A> astarNodeCopy = new AstarNode<>(argNodeCopy, astarNode.getProviderAstarNode());
			// Heuristic has to be set first otherwise admissibility check fails
			if (astarNode.getHeuristic().isKnown()) {
				astarNodeCopy.setHeuristic(astarNode.getHeuristic());

				if (astarNode.getDistance().isKnown()) {
					astarNodeCopy.setDistance(astarNode.getDistance());
				}
			} else {
				assert astarNode.getDistance().isUnknown();
			}

			astarNode.setProviderAstarNode(astarNodeCopy);
			astarNode.reset();

			astarArgCopy.put(astarNodeCopy);
			if (newInitNodes.contains(argNodeCopy)) {
				astarArgCopy.putInit(astarNodeCopy);
			}
			astarArgCopy.reachedSet.add(astarNodeCopy.getArgNode());

			// Nodes in the next iteration already have covering edges which can break the consistency requirement
			// with the decreasing method described in the abstractor.
			// therefore we remove it here so that we can have a strict check for consistency during search.
			// See XstsTest 48th, 51th testcase failing at cover because of consistency check before covering.
			// Full and Semiondemand will always give the same heuristic for covering- and covered node as they are based on distance
			// therefore the heuristic (which is based on the distance values) will be consistent.
			if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
				@Nullable ArgNode<S, A> parentArgNode = astarNode.getArgNode().getParent().orElse(null);
				@Nullable AstarNode<S, A> parentAstarNode = parentArgNode == null ? null : astarArg.get(parentArgNode);
				assert parentAstarNode != null || argNode.isInit();
				astarAbstractor.findHeuristic(astarNode, astarArg, parentAstarNode);

				// There is no guarantee that cover edges will still be consistent
				// - previously the shortest path may have crossed it
				// - decreasing heuristic was consistent there by chance
				BiConsumer<ArgNode<S, A>, ArgNode<S, A>> handleCoverEdgeConsistency = (ArgNode<S, A> coveredNode, ArgNode<S, A> coveringNode) -> {
					@Nullable AstarNode<S, A> astarCoveredNode = astarArgCopy.get(coveredNode);
					@Nullable AstarNode<S, A> astarCoveringNode = astarArgCopy.get(coveringNode);
					if (astarCoveredNode == null || astarCoveringNode == null) {
						return;
					}
					assert astarCoveredNode.getHeuristic().isKnown() && astarCoveringNode.getHeuristic().isKnown();

					if (!astarCoveredNode.getHeuristic().equals(astarCoveringNode.getHeuristic())) {
						coveredNode.unsetCoveringNode();
					}
				};

				if (argNode.getCoveringNode().isPresent()) {
					handleCoverEdgeConsistency.accept(argNode, argNode.getCoveringNode().get());
				}
				argNode.getCoveredNodes().toList().forEach(coveredNodeCopy -> {
					handleCoverEdgeConsistency.accept(coveredNodeCopy, argNode);
				});
			}
		});

		assert astarArg.arg.getNodes().count() == astarArgCopy.getAll().values().size();
		assert astarArg.arg.getInitNodes().count() == astarArgCopy.getAllInit().size();

		return astarArgCopy;
	}
}
