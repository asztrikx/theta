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

import java.util.ArrayList;
import java.util.Collection;
import java.util.Set;
import java.util.function.Function;

/**
 * copies ARG and their ArgNode shallowly (keeping action and state)
 */
public class AstarCopier {
	// astarArg: the source AstarArg from which a copy will be made
	public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> createCopy(
			AstarArg<S, A, P> astarArg, P prec, final PartialOrd<S> partialOrd,
			final Function<? super S, ?> projection
	) {
		Collection<Tuple2<ArgNode<S, A>, ArgNode<S, A>>> translation = new ArrayList<>();
		Set<ArgNode<S, A>> newInitNodes = new HashContainerFactory().createSet();
		ARG<S, A> arg = ArgCopier.createCopy(astarArg.arg, (argNode, argNodeCopy) -> {
			translation.add(Tuple2.of(argNode, argNodeCopy));
		}, (initArgNode, initArgNodeCopy) -> {
			newInitNodes.add(initArgNodeCopy);
		});

		AstarArg<S, A, P> astarArgCopy = new AstarArg<>(arg, prec, partialOrd, projection);

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
		});

		return astarArgCopy;
	}
}
