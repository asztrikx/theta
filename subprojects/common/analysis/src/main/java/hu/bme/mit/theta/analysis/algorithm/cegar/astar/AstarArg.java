package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import javax.annotation.Nullable;
import java.util.*;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarArg<S extends State, A extends Action, P extends Prec> {
    public final ARG<S, A> arg;
    public P prec;
    public @Nullable AstarArg<S, A, P> parent;

    // contains init nodes as well
    //  TODO use partition
    private Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
    private Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
    private final PartialOrd<S> partialOrd;
    // Covering ArgNode is searched from here
    public final Partition<ArgNode<S, A>, ?> reachedSet;

    public AstarArg(
            final ARG<S, A> arg, P prec, final PartialOrd<S> partialOrd,
            final Function<? super S, ?> projection
    ) {
        this.arg = checkNotNull(arg);
        this.prec = prec;
        this.partialOrd = checkNotNull(partialOrd);
        this.reachedSet = Partition.of(n -> projection.apply(n.getState()));
    }

    public void setUnknownDistanceInfinite() {
        getAll().values().forEach(astarNode -> {
            if (astarNode.distance.getType() != Distance.Type.EXACT) {
                astarNode.distance = new Distance(Distance.Type.INFINITE);
            }
        });
    }

    // parents: node -> parent
    public void updateDistancesFromTargetUntil(
            AstarNode<S, A> target, Set<ArgNode<S, A>> until, Map<ArgNode<S, A>, ArgNode<S, A>> parents
    ) {
        //// TODO: rewrite comment
        //// A* property allows us to say that all nodes which was *involved in the search* and reaches target are
        //// the closest to that target

        //// walk up the tree: middle nodes expansion were a subset of waitlist therefore we know their distance
        ////  do not follow *all* covering edge as those node were not involved in search
        ////  however the path could go through several covering nodes

        int startDistance;
        if (target.distance.getType() == Distance.Type.EXACT) {
            startDistance = target.distance.getValue();
        } else {
            startDistance = 0;
        }

        arg.walkUpParents(target.argNode, parents, (node, distance) -> {
            get(node).distance = new Distance(Distance.Type.EXACT, distance + startDistance);

            return until.contains(node);
        });
    }

    public void updateDistancesFromRootInfinite(AstarNode<S, A> from) {
        arg.walk(List.of(from.argNode), (argNode, distance) -> {
            AstarNode<S, A> astarNode = get(argNode);

            // if we reach a part where target is reachable then root should also reach it
            assert(astarNode.distance.getType() != Distance.Type.EXACT);
            astarNode.distance = new Distance(Distance.Type.INFINITE);

            return false;
        });
    }

    // After underlying ARG is pruned we must remove AstarNodes corresponding to pruned ArgNodes
    public void pruneApply() {
        // Collect remained init nodes
        final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodesNew = new HashContainerFactory().createMap();
        arg.getInitNodes().forEach(initNode -> {
            final AstarNode<S, A> astarInitNode = astarInitNodes.get(initNode);
            assert astarInitNode != null;
            astarInitNodesNew.put(initNode, astarInitNode);
        });
        astarInitNodes = astarInitNodesNew;

        // Collect remained nodes
        final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodesNew = new HashContainerFactory().createMap();
        reachedSet.clear();

        arg.walk(arg.getInitNodes().collect(Collectors.toList()), (argNode, distance) -> {
            final AstarNode<S, A> astarNode = astarNodes.get(argNode);
            assert astarNode != null;
            astarNodesNew.put(argNode, astarNode);
            reachedSet.add(argNode);
            return false;
        });
        astarNodes = astarNodesNew;
    }

    /// ARG Wrappers

    public Stream<AstarNode<S, A>> getIncompleteNodes() {
        return arg.getIncompleteNodes().map(this::get);
    }

    /// Collection wrappers

    public void put(final AstarNode<S, A> astarNode) {
        astarNodes.put(astarNode.argNode, astarNode);
    }

    public boolean containsArg(final ArgNode<S, A> argNode) {
        return astarNodes.containsKey(argNode);
    }

    public AstarNode<S, A> get(final ArgNode<S, A> argNode) {
        return astarNodes.get(argNode);
    }

    public void putAll(final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodeMap) {
        astarNodes.putAll(astarNodeMap);
    }

    public Map<ArgNode<S, A>, AstarNode<S, A>> getAll() {
        return astarNodes;
    }

    public Collection<AstarNode<S, A>> getAllInit() {
        return astarInitNodes.values();
    }

    public Collection<ArgNode<S, A>> getAllInitArg() {
        return astarInitNodes.keySet();
    }

    /*public void putAllInitNode(final Map<ArgNode<S, A>, AstarNode<S, A>> mapping) {
        astarInitNodes.putAll(mapping);
        astarNodes.putAll(mapping);
    }*/

    public void putInit(final AstarNode<S, A> astarInitNode) {
        astarInitNodes.put(astarInitNode.argNode, astarInitNode);
        astarNodes.put(astarInitNode.argNode, astarInitNode);
    }
}
