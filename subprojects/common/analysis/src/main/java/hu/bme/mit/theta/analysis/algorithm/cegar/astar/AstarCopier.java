package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgCopier;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.common.Tuple2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Function;

/**
 * copies ARG and their ArgNode shallowly (keeping action and state)
 */
public class AstarCopier {
	public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> createCopy(
			AstarArg<S, A, P> astarArgSource, P prec, final PartialOrd<S> partialOrd,
			final Function<? super S, ?> projection
	) {
		AstarArg<S, A, P> astarArg = astarArgSource;
		Collection<Tuple2<ArgNode<S, A>, ArgNode<S, A>>> translation = new ArrayList<>();
		ARG<S, A> arg = ArgCopier.createCopy(astarArg.arg, (argNode, argNodeCopy) -> {
			translation.add(Tuple2.of(argNode, argNodeCopy));
		});

		AstarArg<S, A, P> astarArgCopy = new AstarArg<>(arg, prec, partialOrd, projection);
		astarArgCopy.putInit(); // TODO: should we use this?
		translation.forEach(t -> {
			ArgNode<S, A> argNode = t.get1();
			ArgNode<S, A> argNodeCopy = t.get2();

			AstarNode<S, A> astarNode = astarArg.get(argNode);
			AstarNode<S, A> astarNodeCopy = new AstarNode<>(argNodeCopy, astarNode);

			astarArgCopy.put(astarNodeCopy);
			astarArg.reachedSet.add(astarNodeCopy.argNode);
		});

		return astarArgCopy;
	}
}
