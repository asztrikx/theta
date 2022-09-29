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
public class AstarCopier {
	// astarArg: the source AstarArg from which a copy will be made
	public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> createCopy(
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

		// Covering edges are created after createCopy finished
		//  TODO this is bad design
		translation.forEach(t -> {
			ArgNode<S, A> argNode = t.get1();
			ArgNode<S, A> argNodeCopy = t.get2();

			AstarNode<S, A> astarNode = astarArg.get(argNode);
			AstarNode<S, A> astarNodeCopy = new AstarNode<>(argNodeCopy, astarNode);

			astarArgCopy.put(astarNodeCopy);
			if (newInitNodes.contains(argNodeCopy)) {
				astarArgCopy.putInit(astarNodeCopy);
			}
			astarArgCopy.reachedSet.add(astarNodeCopy.argNode);

			// We need leftovers to have a heuristic otherwise if we would need to decrease heuristic from a covered node
			// which can lead to inconsistency.
			if (AstarAbstractor.heuristicSearchType == AstarAbstractor.HeuristicSearchType.DECREASING) {
				@Nullable ArgNode<S, A> parentArgNode = astarNodeCopy.argNode.getParent().orElse(null);
				@Nullable AstarNode<S, A> parentAstarNode = parentArgNode == null ? null : astarArgCopy.get(parentArgNode);
				assert parentAstarNode != null || argNode.isInit();
				astarAbstractor.findHeuristic(astarNodeCopy, astarArgCopy, parentAstarNode);

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

				if (argNodeCopy.getCoveringNode().isPresent()) {
					handleCoverEdgeConsistency.accept(argNodeCopy, argNodeCopy.getCoveringNode().get());
				}
				argNodeCopy.getCoveredNodes().toList().forEach(coveredNodeCopy -> {
					handleCoverEdgeConsistency.accept(coveredNodeCopy, argNodeCopy);
				});
			}
		});

		assert astarArg.arg.getNodes().count() == astarArgCopy.getAll().values().size();
		assert astarArg.arg.getInitNodes().count() == astarArgCopy.getAllInit().size();

		return astarArgCopy;
	}
}
