package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

// TODO create factory class?
public final class AstarArg<S extends State, A extends Action, P extends Prec> {
    public final ARG<S, A> arg;
    public P prec;
    // contains init nodes as well (it is used out somewhere)
    //  TODO use partition
    private final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
    private final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
    public AstarArg<S, A, P> descendant;
    // TODO make it available only through function parameter
    private final PartialOrd<S> partialOrd;

    private AstarArg(final ARG<S, A> arg, final AstarArg<S, A, P> descendant, final PartialOrd<S> partialOrd) {
        this.arg = checkNotNull(arg);
        this.descendant = descendant;
        this.partialOrd = checkNotNull(partialOrd);
    }

    public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> create(
            final ARG<S, A> arg,
            final P prec,
            final AstarArg<S, A, P> descendant,
            final PartialOrd<S> partialOrd
    ) {
        return new AstarArg<>(arg, descendant, partialOrd);
    }

    // put saves the AstarNode for ArgNode
    public void put(final AstarNode<S, A> astarNode) {
        astarNodes.put(astarNode.argNode, astarNode);
    }

    public boolean contains(final ArgNode<S, A> argNode) {
        return astarNodes.containsKey(argNode);
    }

    // searches the AstarNode for the ArgNode
    public AstarNode<S, A> get(final ArgNode<S, A> argNode) {
        return astarNodes.get(argNode);
    }

    // saves the AstarNodes for ArgNodes
    public void putAll(final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodeMap) {
        astarNodes.putAll(astarNodeMap);
    }

    public Map<ArgNode<S, A>, AstarNode<S, A>> getAll() {
        return astarNodes;
    }

    public Map<ArgNode<S, A>, AstarNode<S, A>> getAllInitNode() {
        return astarInitNodes;
    }

    public void putAllInitNode(final Map<ArgNode<S, A>, AstarNode<S, A>> mapping) {
        astarInitNodes.putAll(mapping);
        astarNodes.putAll(mapping);
    }

    public void putInitNode(final AstarNode<S, A> astarInitNode) {
        astarInitNodes.put(astarInitNode.argNode, astarInitNode);
        astarNodes.put(astarInitNode.argNode, astarInitNode);
    }

    // putAllFromCandidates
    //  when descendantAstarNodeCandidates is empty descendant will be null
    public Collection<AstarNode<S, A>> putAllFromCandidates(final List<ArgNode<S, A>> argNodes, List<Collection<AstarNode<S, A>>> descendantAstarNodeCandidates, boolean init) {
        assert argNodes.size() == descendantAstarNodeCandidates.size();

        List<AstarNode<S, A>> astarNodes = new ArrayList<>();
        for (int i = 0; i < argNodes.size(); i++) {
            AstarNode<S, A> astarNode = putFromCandidates(argNodes.get(i), descendantAstarNodeCandidates.get(i), init);
            astarNodes.add(astarNode);
        }
        return astarNodes;
    }

    // putAllFromCandidates
    //  the same descendantAstarNodeCandidates will be used for all argNodes
    //  when descendantAstarNodeCandidates is empty descendant will be null
    public Collection<AstarNode<S, A>> putAllFromCandidates(final List<ArgNode<S, A>> argNodes, Collection<AstarNode<S, A>> descendantAstarNodeCandidates, boolean init) {
        // create list with the same items to use already written general funcion
        List<Collection<AstarNode<S, A>>> list = new ArrayList<>(argNodes.size());
        for (int i = 0; i < argNodes.size(); i++) {
            list.add(descendantAstarNodeCandidates);
        }

        return putAllFromCandidates(argNodes, list, init);
    }

    // putFromCandidates
    //  when descendantAstarNodeCandidates is empty descendant will be null
    //  if descendant parent is covered get descendantAstarNodeCandidates from covering node's successor nodes
    public AstarNode<S, A> putFromCandidates(final ArgNode<S, A> argNode, Collection<AstarNode<S, A>> descendantAstarNodeCandidates, boolean init) {
        // find descendant
        //  null means no descendant
        AstarNode<S, A> descendantAstarNode = null;

        // when empty candidate given interpret it as no descendant
        if (descendantAstarNodeCandidates.size() != 0) {
            List<AstarNode<S, A>> descendantAstarNodes = descendantAstarNodeCandidates.stream()
                    .filter(descendantArgNodeCandidate -> partialOrd.isLeq(descendantArgNodeCandidate.argNode.getState(), argNode.getState()))
                    .collect(Collectors.toList());
            assert descendantAstarNodes.size() == 1;

            descendantAstarNode = descendantAstarNodes.get(0);
        }

        // create, store astarNode
        AstarNode<S, A> astarNode = AstarNode.create(argNode, descendantAstarNode);
        if (init) {
            putInitNode(astarNode); // TODO atomicity from foreach? (putAllFromCandidates)
        } else {
            put(astarNode); // TODO atomicity from foreach? (putAllFromCandidates)
        }

        return astarNode;
    }
}
