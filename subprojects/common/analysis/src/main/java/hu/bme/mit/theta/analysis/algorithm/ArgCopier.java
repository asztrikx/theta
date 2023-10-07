package hu.bme.mit.theta.analysis.algorithm;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.waitlist.FifoWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import java.util.Map;
import java.util.function.BiConsumer;

// TODO in kotlin in ArgXYZUtil?
/**
 * copies ARG and their ArgNode shallowly (keeping action and state)
 */
public class ArgCopier {
    /**
     * @param translationHook for all new node and it's copy node it will call with paramters (node, copy node)
     */
    public static <S extends State, A extends Action> ARG<S, A> createCopy(
            ARG<S, A> arg,
            BiConsumer<ArgNode<S, A>, ArgNode<S, A>> translationHook
    ) {
        ARG<S, A> argCopy = ARG.create(arg.partialOrd);

        // BFS: no need to store visited nodes as ARG is a tree as long as we don't visit covering edges
        Waitlist<Visit<S, A>> waitlist = FifoWaitlist.create();

        // Covering node handle: one end of the edge may not exist
        // (covered ArgNode, covering ArgNode copy)
        Map<ArgNode<S, A>, ArgNode<S, A>> shouldSetAsCoveringNode = new HashContainerFactory().createMap();
        // (ArgNode, ArgNode copy)
        Map<ArgNode<S, A>, ArgNode<S, A>> currentToCopyMap = new HashContainerFactory().createMap();

        // copy initNodes
        arg.getInitNodes().forEach(initNode -> {
            ArgNode<S, A> initNodeCopy = argCopy.createInitNode(initNode.getState(), initNode.isTarget());
            initNodeCopy.copyFrom(initNode);

            handleCoveringEdges(initNode, initNodeCopy, shouldSetAsCoveringNode, currentToCopyMap);

            waitlist.add(new Visit<>(initNode, initNodeCopy));
        });

        // walk through ARG and copy nodes
        // ARG is a tree without visiting covering edges
        // TODO use ArgWalkUtil?
        while (!waitlist.isEmpty()) {
            Visit<S, A> visit = waitlist.remove();
            ArgNode<S, A> argNode = visit.original;
            ArgNode<S, A> argNodeCopy = visit.copy;

            translationHook.accept(argNode, argNodeCopy);

            argNode.getOutEdges().forEach(edge -> {
                ArgNode<S, A> succArgNode = edge.getTarget();
                ArgNode<S, A> succArgNodeCopy = argCopy.createSuccNode(argNodeCopy, edge.getAction(), succArgNode.getState(), succArgNode.isTarget());
                succArgNodeCopy.copyFrom(succArgNode);

                handleCoveringEdges(succArgNode, succArgNodeCopy, shouldSetAsCoveringNode, currentToCopyMap);

                waitlist.add(new Visit<>(succArgNode, succArgNodeCopy));
            });
        }

        shouldSetAsCoveringNode.forEach((coveredNode, coveringNodeCopy) -> {
            ArgNode<S, A> coveredNodeCopy = currentToCopyMap.get(coveredNode);
            coveredNodeCopy.setCoveringNode(coveringNodeCopy);
        });

        // Copy nextId after copy happened as other createSuccNode would increase nextId to double
        // Others are either
        //  - final
        //  - already set
        argCopy.initialized = arg.initialized;
        argCopy.nextId = arg.getNodes().map(ArgNode::getId).max(Integer::compareTo).orElse(-1) + 1;

        return argCopy;
    }

    private static class Visit<S extends State, A extends Action> {
        public ArgNode<S, A> original;
        public ArgNode<S, A> copy;

        public Visit(ArgNode<S, A> original, ArgNode<S, A> copy) {
            this.original = original;
            this.copy = copy;
        }
    }

    private static <S extends State, A extends Action> void handleCoveringEdges(
            ArgNode<S, A> argNode,
            ArgNode<S, A> argNodeCopy,
            Map<ArgNode<S, A>, ArgNode<S, A>> shouldSetAsCoveringNode,
            Map<ArgNode<S, A>, ArgNode<S, A>> currentToCopyMap
    ) {
        // covering node: save which to cover
        argNode.getCoveredNodes().forEach(coveredNode -> shouldSetAsCoveringNode.put(coveredNode, argNodeCopy));

        // covered node: save (node, node copy)
        if (argNode.isCovered()) {
            currentToCopyMap.put(argNode, argNodeCopy);
        }
    }
}
