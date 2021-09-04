package hu.bme.mit.theta.analysis.algorithm.cegar.astar;


import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Function;

public final class DistanceHeuristicStore<S extends State, A extends Action> {
    private final Map<ArgNode<S,A>, Integer> distanceHeuristics = new HashContainerFactory().createMap();
    Partition<ArgNode<S, A>, ?> indexer;
    final Function<? super S, ?> projection;
    private final PartialOrd<S> partialOrd;

    private DistanceHeuristicStore(final Function<? super S, ?> projection, final PartialOrd<S> partialOrd) {
        this.projection = projection;
        this.indexer = Partition.of(n -> projection.apply(n.getState()));
        this.partialOrd = partialOrd;
    }

    public static <S extends State> DistanceHeuristicStore create(final Function<? super S, ?> projection, final PartialOrd<S> partialOrd) {
        return new DistanceHeuristicStore(projection, partialOrd);
    }

    // put saves argNode with specified distance to target
    public void put(final ArgNode<S, A> argNode, final int distance) {
        distanceHeuristics.put(argNode, distance);
        indexer.add(argNode);
    }

    public void putAll(final Map<ArgNode<S,A>, Integer> distances) {
        distanceHeuristics.putAll(distances);
        indexer.addAll(distances.keySet());
    }

    // get searches for an ArgNode which is <= then argNode and returns its distance.
    // If no ArgNode is suitable then -1 is returned.
    public int get(final ArgNode<S, A> argNode) {
        // get keys which are <= then current key
        final AtomicInteger distance = new AtomicInteger(-1);

        final List<ArgNode<S, A>> argNodeCandidates = indexer.get(argNode);
        argNodeCandidates.forEach((argNodeCandidate)->{
            final int distanceHeuristic = distanceHeuristics.get(argNodeCandidate);
            if (distance.get() != -1 && distance.get() <= distanceHeuristic) {
                return;
            }
            if(partialOrd.isLeq(argNode.getState(), argNodeCandidate.getState())){
                distance.set(distanceHeuristic);
            }
        });

        return distance.get();
    }

    public boolean contains(final ArgNode<S,A> argNode){
        final List<ArgNode<S, A>> argNodeCandidates = indexer.get(argNode);
        return argNodeCandidates.size() != 0;
    }

    public void clear() {
        distanceHeuristics.clear();
        indexer = Partition.of(n -> projection.apply(n.getState()));
    }
}
