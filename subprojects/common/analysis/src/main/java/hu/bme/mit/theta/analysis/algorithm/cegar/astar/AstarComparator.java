package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import java.util.Comparator;
import java.util.Map;
import java.util.function.Function;

public final class AstarComparator<S extends State, A extends Action> implements Comparator<ArgNode<S, A>> {
    protected final PartialOrd<S> partialOrd;
    private final int depthWeight;
    private final int heuristicsWeight;
    private final DistanceHeuristicStore distanceHeuristicStore;

    public AstarComparator(final PartialOrd<S> partialOrd, final Function<? super S, ?> projection, final int depthWeight, final int heuristicsWeight){
        this.partialOrd = partialOrd;
        this.distanceHeuristicStore = DistanceHeuristicStore.create(projection, partialOrd);
        this.depthWeight = depthWeight;
        this.heuristicsWeight = heuristicsWeight;
    }

    public int compare(final ArgNode<S, A> argNode1, final ArgNode<S, A> argNode2) {
        final boolean distance1Exists = distanceHeuristicStore.contains(argNode1);
        final boolean distance2Exists = distanceHeuristicStore.contains(argNode2);

        // not reachable in more abstract domain => won't be reachable in refined => give 'infinite' weight
        if (!distance1Exists && !distance2Exists) {
            return 0;
        } else if (!distance1Exists) {
            return 1;
        } else if (!distance2Exists) {
            return -1;
        }

        final int distance1 = distanceHeuristicStore.get(argNode1);
        final int distance2 = distanceHeuristicStore.get(argNode2);

        // calculate a* heuristics
        final int weight1 = depthWeight * argNode1.getDepth() + heuristicsWeight * distance1;
        final int weight2 = depthWeight * argNode2.getDepth() + heuristicsWeight * distance2;

        return weight1 - weight2;
    }

    // Keeps track of previous states to use heuristics in next arg creation
    public void store(final ARG<S, A> arg){
        // clear previous results as currents have better refinement so they are more accurate
        distanceHeuristicStore.clear();
        final Map<ArgNode<S,A>, Integer> distances = arg.getDistances();
        distanceHeuristicStore.putAll(distances);
    }
}

