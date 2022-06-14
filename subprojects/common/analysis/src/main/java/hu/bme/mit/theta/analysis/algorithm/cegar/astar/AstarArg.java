package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.reachedset.Partition;
import hu.bme.mit.theta.analysis.waitlist.PriorityWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
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
    public AstarArg<S, A, P> parent;

    // contains init nodes as well
    //  TODO use partition
    private final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
    private final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
    private final PartialOrd<S> partialOrd;
    public int iteration = -1;
    // Covering ArgNode is searched from here
    public final Partition<ArgNode<S, A>, ?> reachedSet;

    public static class Search<S extends State, A extends Action> {
        // We could already have started to explore a subgraph therefore do not use global doneSet variable
        public Set<AstarNode<S, A>> doneSet = new HashSet<>();
        // Useful to know whether the current item is smaller than the one in the waitlist (if not in doneSet)
        public Map<AstarNode<S, A>, Integer> minWeights = new HashMap<>();
        // After we reach target we know the distance for all nodes between root and target
        // which is visitable by parent entries
        public Map<ArgNode<S, A>, ArgNode<S, A>> parents = new HashContainerFactory().createMap();
        public Map<AstarNode<S, A>, Integer> depths = new HashContainerFactory().createMap();
        public Waitlist<Edge<S, A>> waitlist = PriorityWaitlist.create(new AstarWaitlistComparator<>());

        public Search() {
            throw new RuntimeException(); // waitlist to ctor
        }
    }

    public static class Edge<S extends State, A extends Action> {
        public AstarNode<S, A> start; // used for parent => could be ArgNode
        public AstarNode<S, A> end;
        public int depthFromAStartNode;

        public Edge(@Nullable AstarNode<S, A> start, AstarNode<S, A> end, int depthFromAStartNode) {
            this.start = start;
            this.end = end;
            this.depthFromAStartNode = depthFromAStartNode;
        }
    }

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

    public void updateDistancesFromTargetUntil(ArgNode<S, A> target, Set<ArgNode<S, A>> until, Map<ArgNode<S, A>, ArgNode<S, A>> parents) {
        // A* property allows us to say that all nodes which was *involved in the search* and reaches target are
        // the closest to that target

        // walk up the tree: middle nodes expansion were a subset of waitlist therefore we know their distance
        //  do not follow *all* covering edge as those node were not involved in search
        //  however the path could go through several covering nodes

        arg.walkUpParents(target, parents, (node, distance) -> {
            get(node).distance = new Distance(Distance.Type.EXACT, distance);

            return until.contains(node);
        });
    }

    // should be called on astarArg where covering can exists
    // if init node: action, parentAstarNode can be null
    public AstarNode<S, A> findProviderAstarNode(
            ArgNode<S, A> argNode,
            @Nullable A action,
            @Nullable AstarNode<S, A> parentAstarNode,
            AstarArg<S, A, P> astarArg
    ) {
        // parent AstarArg does not exists
        if (parent == null) {
            return null;
        }

        // get candidates based on parent
        Stream<ArgNode<S, A>> coveringCandidates;
        if (parentAstarNode != null) {
            AstarNode<S, A> parentAstarCoveringNode = parentAstarNode.providerAstarNode;
            // as parent's child is explored parent had to be in waitlist => distance exists => parent expanded / covered
            assert parentAstarCoveringNode.distance.isKnown();
            assert parentAstarCoveringNode.argNode.isExpanded() || parentAstarNode.argNode.isCovered();

            // astar covering node can be covered after being set for astarNode
            if (parentAstarCoveringNode.argNode.isCovered()) {
                ArgNode<S, A> coveringNode = parentAstarCoveringNode.argNode.getCoveringNode().get();
                // optimization
                parentAstarNode.providerAstarNode = astarArg.get(coveringNode);
                parentAstarCoveringNode = parentAstarNode.providerAstarNode;

                assert parentAstarCoveringNode.distance.isKnown();
                assert parentAstarCoveringNode.argNode.isExpanded();
            }

            coveringCandidates = parentAstarCoveringNode.providerAstarNode.argNode.getOutEdges()
                .filter(edge -> edge.getAction().equals(action)) // equals works good?
                .map(ArgEdge::getTarget);
        } else {
            coveringCandidates = parent.getAllInit().stream();
        }

        // filter based on partialOrd
        coveringCandidates = coveringCandidates.filter(coveringCandidate ->
            partialOrd.isLeq(argNode.getState(), coveringCandidate.getState())
        );
        Optional<ArgNode<S,A>> covering = coveringCandidates.findAny();
        assert covering.isPresent();
        return parent.get(covering.get());
    }

    public Stream<AstarNode<S, A>> getIncompleteNodes() {
        return arg.getIncompleteNodes().map(this::get);
    }

    public void put(final AstarNode<S, A> astarNode) {
        astarNodes.put(astarNode.argNode, astarNode);
    }

    public boolean contains(final ArgNode<S, A> argNode) {
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

    public Collection<AstarNode<S, A>> getAllAstarInit() {
        return astarInitNodes.values();
    }

    public Collection<ArgNode<S, A>> getAllInit() {
        return astarInitNodes.keySet();
    }

    public void putAllInitNode(final Map<ArgNode<S, A>, AstarNode<S, A>> mapping) {
        astarInitNodes.putAll(mapping);
        astarNodes.putAll(mapping);
    }

    public void putInit(final AstarNode<S, A> astarInitNode) {
        astarInitNodes.put(astarInitNode.argNode, astarInitNode);
        astarNodes.put(astarInitNode.argNode, astarInitNode);
    }

//    public void prune() {
//        // prune init nodes
//        final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodesNew = new HashContainerFactory().createMap();
//        arg.getInitNodes().forEach(argInitNode -> {
//            if (astarInitNodes.containsKey(argInitNode)) {
//                final AstarNode<S, A> astarInitNode = astarInitNodes.get(argInitNode);
//                astarInitNodesNew.put(argInitNode, astarInitNode);
//                // it will also get added to astarNodesNew
//            }
//        });
//        astarInitNodes = astarInitNodesNew;
//
//        // prune all nodes
//        final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodesNew = new HashContainerFactory().createMap();
//        arg.walk(arg.getInitNodes().collect(Collectors.toList()), (argNode, distance) -> {
//            if(astarNodes.containsKey(argNode)) {
//                final AstarNode<S, A> astarNode = astarNodes.get(argNode);
//                astarNodesNew.put(argNode, astarNode);
//                return false;
//            } else {
//                return true;
//            }
//        });
//        astarNodes = astarNodesNew;
//    }

//    /**
//     * when parentAstarNodeCandidates is empty parent will be null
//     */
//    public Collection<AstarNode<S, A>> getAllParentFromCandidates(final List<ArgNode<S, A>> argNodes, List<Collection<AstarNode<S, A>>> parentAstarNodeCandidates) {
//        assert argNodes.size() == parentAstarNodeCandidates.size();
//
//        final List<AstarNode<S, A>> astarNodes = new ArrayList<>();
//        for (int i = 0; i < argNodes.size(); i++) {
//            final AstarNode<S, A> astarNode = getParentFromCandidates(argNodes.get(i), parentAstarNodeCandidates.get(i));
//            astarNodes.add(astarNode);
//        }
//        return astarNodes;
//    }
//
//    /**
//     * the same parentAstarNodeCandidates will be used for all argNodes
//     * when parentAstarNodeCandidates is empty parent will be null
//     */
//    public Collection<AstarNode<S, A>> getAllParentFromCandidates(final List<ArgNode<S, A>> argNodes, Collection<AstarNode<S, A>> parentAstarNodeCandidates) {
//        // create list with the same items to use already written general funcion
//        final List<Collection<AstarNode<S, A>>> list = new ArrayList<>(argNodes.size());
//        for (int i = 0; i < argNodes.size(); i++) {
//            list.add(parentAstarNodeCandidates);
//        }
//
//        return getAllParentFromCandidates(argNodes, list);
//    }
//
//    /**
//     * if parent's parent is covered get parentAstarNodeCandidates from covering node's successor nodes
//     */
//    public AstarNode<S, A> getParentFromCandidates(final ArgNode<S, A> argNode, Collection<AstarNode<S, A>> parentAstarNodeCandidates) {
//        // when a node with no outgoing edge is split and we don't add the node to candidates this assertion fails
//        assert parentAstarNodeCandidates.size() != 0;
//
//        final List<AstarNode<S, A>> parentAstarNodes = parentAstarNodeCandidates.stream()
//                .filter(parentAstarNodeCandidate -> partialOrd.isLeq(argNode.getState(), parentAstarNodeCandidate.argNode.getState()))
//                .collect(Collectors.toList());
//
//        // it is assumed that all edges have unique Action
//        //  otherwise: {a} {b} are reachable with the same action; in next arg child is {a,b,c}; which is the parent?
//        // init nodes may have 1 candidate
//        // not init nodes may have 2 (parent and its child) as child can be covered by parent (or if child is error then it *could* be covered)
//        assert parentAstarNodes.size() == 1 || parentAstarNodes.size() == 2;
//
//        return parentAstarNodes.get(0);
//    }
//
//    public AstarArg<S, A, P> copy() {
//        ARG<S, A> argCopied = arg.copy();
//        throw new UnsupportedOperationException();
//    }
//
}
