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

public final class AstarArg<S extends State, A extends Action, P extends Prec> {
    public final ARG<S, A> arg;
    public P prec;
    // contains init nodes as well
    //  TODO use partition
    private Map<ArgNode<S, A>, AstarNode<S, A>> astarNodes = new HashContainerFactory().createMap();
    private Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodes = new HashContainerFactory().createMap();
    public AstarArg<S, A, P> parent;
    private final PartialOrd<S> partialOrd;
    public int iteration = -1;

    private AstarArg(final ARG<S, A> arg, P prec, final AstarArg<S, A, P> parent, final PartialOrd<S> partialOrd) {
        this.arg = checkNotNull(arg);
        this.prec = prec;
        this.parent = parent;
        this.partialOrd = checkNotNull(partialOrd);
    }

    public static <S extends State, A extends Action, P extends Prec> AstarArg<S, A, P> create(
            final ARG<S, A> arg,
            final P prec,
            final AstarArg<S, A, P> parent,
            final PartialOrd<S> partialOrd
    ) {
        return new AstarArg<>(arg, prec, parent, partialOrd);
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

    public Map<ArgNode<S, A>, AstarNode<S, A>> getAllInit() {
        return astarInitNodes;
    }

    public void putAllInitNode(final Map<ArgNode<S, A>, AstarNode<S, A>> mapping) {
        astarInitNodes.putAll(mapping);
        astarNodes.putAll(mapping);
    }

    public void putInit(final AstarNode<S, A> astarInitNode) {
        astarInitNodes.put(astarInitNode.argNode, astarInitNode);
        astarNodes.put(astarInitNode.argNode, astarInitNode);
    }

    public void prune() {
        // prune init nodes
        final Map<ArgNode<S, A>, AstarNode<S, A>> astarInitNodesNew = new HashContainerFactory().createMap();
        arg.getInitNodes().forEach(argInitNode -> {
            if (astarInitNodes.containsKey(argInitNode)) {
                final AstarNode<S, A> astarInitNode = astarInitNodes.get(argInitNode);
                astarInitNodesNew.put(argInitNode, astarInitNode);
                // it will also get added to astarNodesNew
            }
        });
        astarInitNodes = astarInitNodesNew;

        // prune all nodes
        final Map<ArgNode<S, A>, AstarNode<S, A>> astarNodesNew = new HashContainerFactory().createMap();
        arg.walk(arg.getInitNodes().collect(Collectors.toList()), (argNode, distance) -> {
            if(astarNodes.containsKey(argNode)) {
                final AstarNode<S, A> astarNode = astarNodes.get(argNode);
                astarNodesNew.put(argNode, astarNode);
                return false;
            } else {
                return true;
            }
        });
        astarNodes = astarNodesNew;
    }

    /**
     * when parentAstarNodeCandidates is empty parent will be null
     */
    public Collection<AstarNode<S, A>> getAllParentFromCandidates(final List<ArgNode<S, A>> argNodes, List<Collection<AstarNode<S, A>>> parentAstarNodeCandidates) {
        assert argNodes.size() == parentAstarNodeCandidates.size();

        final List<AstarNode<S, A>> astarNodes = new ArrayList<>();
        for (int i = 0; i < argNodes.size(); i++) {
            final AstarNode<S, A> astarNode = getParentFromCandidates(argNodes.get(i), parentAstarNodeCandidates.get(i));
            astarNodes.add(astarNode);
        }
        return astarNodes;
    }

    /**
     * the same parentAstarNodeCandidates will be used for all argNodes
     * when parentAstarNodeCandidates is empty parent will be null
     */
    public Collection<AstarNode<S, A>> getAllParentFromCandidates(final List<ArgNode<S, A>> argNodes, Collection<AstarNode<S, A>> parentAstarNodeCandidates) {
        // create list with the same items to use already written general funcion
        final List<Collection<AstarNode<S, A>>> list = new ArrayList<>(argNodes.size());
        for (int i = 0; i < argNodes.size(); i++) {
            list.add(parentAstarNodeCandidates);
        }

        return getAllParentFromCandidates(argNodes, list);
    }

    /**
     * if parent's parent is covered get parentAstarNodeCandidates from covering node's successor nodes
     */
    public AstarNode<S, A> getParentFromCandidates(final ArgNode<S, A> argNode, Collection<AstarNode<S, A>> parentAstarNodeCandidates) {
        // when a node with no outgoing edge is split and we don't add the node to candidates this assertion fails
        assert parentAstarNodeCandidates.size() != 0;

        final List<AstarNode<S, A>> parentAstarNodes = parentAstarNodeCandidates.stream()
                .filter(parentAstarNodeCandidate -> partialOrd.isLeq(argNode.getState(), parentAstarNodeCandidate.argNode.getState()))
                .collect(Collectors.toList());

        // it is assumed that all edges have unique Action
        //  otherwise: {a} {b} are reachable with the same action; in next arg child is {a,b,c}; which is the parent?
        // init nodes may have 1 candidate
        // not init nodes may have 2 (parent and its child) as child can be covered by parent (or if child is error then it *could* be covered)
        assert parentAstarNodes.size() == 1 || parentAstarNodes.size() == 2;

        return parentAstarNodes.get(0);
    }
}
