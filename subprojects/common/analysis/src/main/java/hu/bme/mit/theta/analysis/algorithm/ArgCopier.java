package hu.bme.mit.theta.analysis.algorithm;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.PartialOrd;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.waitlist.FifoWaitlist;
import hu.bme.mit.theta.analysis.waitlist.Waitlist;
import hu.bme.mit.theta.common.container.factory.HashContainerFactory;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

/**
 * copies ARG and their ArgNode shallowly (keeping action and state)
 */
public class ArgCopier {
    public static <S extends State, A extends Action> ARG<S, A> createCopy(
            PartialOrd<S> partialOrd,
            ARG<S, A> arg
    ) {
        ARG<S, A> argCopy = ARG.create(partialOrd);
        argCopy.initialized = arg.initialized;
        argCopy.nextId = arg.nextId;

        // BFS: no need to store visited nodes as Arg is a tree as long as we don't visit covering edges
        Waitlist<Copy<S, A>> waitlist = FifoWaitlist.create();

        // Covering nodes may not exists =>
        // 		save (original covering node, copied covered node) pairs
        //		save (original covering node, copied covering node) pairs
        Map<ArgNode<S, A>, Collection<ArgNode<S, A>>> setAsCoveringNode = new HashContainerFactory().createMap();
        Map<ArgNode<S, A>, ArgNode<S, A>> coveringNodeMapping = new HashContainerFactory().createMap();

        // van-e covered node
        //    mindegyikre belerakjuk egy (covered node, covering node copy) listába
        // van-e covering node
        //    megnézzük, hogy szerepel

        // copy initNodes
        arg.getInitNodes().forEach(initNode -> {
            ArgNode<S, A> initNodeCopy = argCopy.createInitNode(initNode.getState(), initNode.isTarget());
            initNodeCopy.copyFrom(initNode);

            waitlist.add(new Copy<>(initNode, initNodeCopy));

            copyCoveringHandler(initNode, initNodeCopy, setAsCoveringNode, coveringNodeMapping);
        });

        // walk through ARG and copy nodes
        while (!waitlist.isEmpty()) {
            Copy<S, A> copy = waitlist.remove();
            ArgNode<S, A> argNode = copy.original;
            ArgNode<S, A> argNodeCopy = copy.copy;

            copyCoveringHandler(argNode, argNodeCopy, setAsCoveringNode, coveringNodeMapping);

            argNode.getOutEdges().forEach(edge -> {
                ArgNode<S, A> succArgNode = edge.getTarget();
                ArgNode<S, A> succArgNodeCopy = argCopy.createSuccNode(argNodeCopy, edge.getAction(), edge.getTarget().getState(), edge.getTarget().isTarget());
                succArgNodeCopy.copyFrom(succArgNode);

                waitlist.add(new Copy<>(succArgNode, succArgNodeCopy));
                throw new RuntimeException(); // is shared state, action modified any time? we have to copy them if so
                // can we use arg.walk?
            });
        }

        assert setAsCoveringNode.size() == 0;
        return argCopy;
    }

    private static class Copy<S extends State, A extends Action> {
        public ArgNode<S, A> original;
        public ArgNode<S, A> copy;

        public Copy(ArgNode<S, A> original, ArgNode<S, A> copy) {
            this.original = original;
            this.copy = copy;
        }
    }

    private static <S extends State, A extends Action> void copyCoveringHandler(
            ArgNode<S, A> argNode,
            ArgNode<S, A> argNodeCopy,
            Map<ArgNode<S, A>, Collection<ArgNode<S, A>>> setAsCoveringNode,
            Map<ArgNode<S, A>, ArgNode<S, A>> coveringNodeMapping
    ) {
        // covering node or covered node cases

        if (argNode.getCoveredNodes().findAny().isPresent()) {
            // set as covering node for nodes reached earlier
            if (setAsCoveringNode.containsKey(argNode)) {
                setAsCoveringNode.get(argNode).forEach(node -> node.setCoveringNode(argNode));
                setAsCoveringNode.remove(argNode);
            }

            // make covering node available for nodes reached later
            coveringNodeMapping.put(argNode, argNodeCopy);
        } else if (argNode.getCoveringNode().isPresent()) {
            ArgNode<S, A> coveringNode = argNode.getCoveringNode().get();

            // covering node reached earlier or not
            if (coveringNodeMapping.containsKey(coveringNode)) {
                ArgNode<S, A> coveringNodeCopy = coveringNodeMapping.get(coveringNode);
                argNodeCopy.setCoveringNode(coveringNodeCopy);
            } else {
                if (!setAsCoveringNode.containsKey(coveringNode)) {
                    Collection<ArgNode<S, A>> copiedCoveredNodes = new ArrayList<>();
                    copiedCoveredNodes.add(argNodeCopy);
                    setAsCoveringNode.put(coveringNode, copiedCoveredNodes);
                } else {
                    setAsCoveringNode.get(coveringNode).add(argNodeCopy);
                }
            }
        }
    }
}
