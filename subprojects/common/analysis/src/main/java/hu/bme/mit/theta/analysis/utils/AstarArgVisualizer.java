/*
 *  Copyright 2017 Budapest University of Technology and Economics
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package hu.bme.mit.theta.analysis.utils;

import static hu.bme.mit.theta.common.visualization.Alignment.LEFT;
import static hu.bme.mit.theta.common.visualization.Shape.RECTANGLE;

import java.awt.Color;

import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode;
import hu.bme.mit.theta.common.container.Containers;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.common.visualization.EdgeAttributes;
import hu.bme.mit.theta.common.visualization.Graph;
import hu.bme.mit.theta.common.visualization.LineStyle;
import hu.bme.mit.theta.common.visualization.NodeAttributes;

public final class AstarArgVisualizer<S extends State, A extends Action, P extends Prec> {

    private static final LineStyle COVER_EDGE_STYLE = LineStyle.DASHED;
    private static final LineStyle SUCC_EDGE_STYLE = LineStyle.NORMAL;
    private static final String ARG_LABEL = "";
    private static final String ARG_ID = "arg";
    private static final String FONT = "courier";
    private static final String NODE_ID_PREFIX = "node_";
    private static final Color FILL_COLOR = Color.WHITE;
    private static final Color LINE_COLOR = Color.BLACK;
    private static final String PHANTOM_INIT_ID = "phantom_init";

    private final Function<S, String> stateToString;
    private final Function<A, String> actionToString;
    private final Function<AstarNode<S, A>, String> descendantAstarNodeToString;
    private final Function<AstarNode<S, A>, String> astarNodeToString;

    private static class LazyHolderDefault {
        static final AstarArgVisualizer<State, Action, Prec> INSTANCE = new AstarArgVisualizer<>(s -> s.toString(), a -> a.toString(), n -> n.toString(), n -> n.toString());
    }

    private static class LazyHolderStructureOnly {
        static final AstarArgVisualizer<State, Action, Prec> INSTANCE = new AstarArgVisualizer<>(s -> "", a -> "", n -> "", n -> "");
    }

    public AstarArgVisualizer(
            final Function<S, String> stateToString,
            final Function<A, String> actionToString,
            final Function<AstarNode<S, A>, String> astarNodeToString,
            final Function<AstarNode<S, A>, String> descendantAstarNodeToString
    ) {
        this.stateToString = stateToString;
        this.actionToString = actionToString;
        this.astarNodeToString = astarNodeToString;
        this.descendantAstarNodeToString = descendantAstarNodeToString;
    }

    public static <S extends State, A extends Action, P extends Prec> AstarArgVisualizer<S, A, P> create(
            final Function<S, String> stateToString,
            final Function<A, String> actionToString,
            final Function<AstarNode<S, A>, String> astarNodeToString,
            final Function<AstarNode<S, A>, String> descendantAstarNodeToString) {
        return new AstarArgVisualizer<>(stateToString, actionToString, astarNodeToString, descendantAstarNodeToString);
    }

    public static AstarArgVisualizer<State, Action, Prec> getDefault() {
        return LazyHolderDefault.INSTANCE;
    }

    public static AstarArgVisualizer<State, Action, Prec> getStructureOnly() {
        return LazyHolderStructureOnly.INSTANCE;
    }

    public Graph visualize(final AstarArg<? extends S, ? extends A, ? extends P> astarArg) {
        ARG<? extends S, ? extends A> arg = astarArg.arg;
        final Graph graph = new Graph(ARG_ID, ARG_LABEL);

        final Set<ArgNode<? extends S, ? extends A>> traversed = Containers.createSet();

        for (final ArgNode<? extends S, ? extends A> initNode : arg.getInitNodes().collect(Collectors.toSet())) {
            traverse(graph, initNode, traversed, astarArg);
            final NodeAttributes nAttributes = NodeAttributes.builder().label("").fillColor(FILL_COLOR)
                    .lineColor(FILL_COLOR).lineStyle(SUCC_EDGE_STYLE).peripheries(1).build();
            graph.addNode(PHANTOM_INIT_ID + initNode.getId(), nAttributes);
            final EdgeAttributes eAttributes = EdgeAttributes.builder().label("").color(LINE_COLOR)
                    .lineStyle(SUCC_EDGE_STYLE).build();
            graph.addEdge(PHANTOM_INIT_ID + initNode.getId(), NODE_ID_PREFIX + initNode.getId(), eAttributes);
        }

        return graph;
    }

    private void traverse(
            final Graph graph,
            final ArgNode<? extends S, ? extends A> node,
            final Set<ArgNode<? extends S, ? extends A>> traversed,
            AstarArg<? extends S, ? extends A, ? extends P> astarArg
    ) {
        if (traversed.contains(node)) {
            return;
        } else {
            traversed.add(node);
        }
        final String nodeId = NODE_ID_PREFIX + node.getId();
        final LineStyle lineStyle = SUCC_EDGE_STYLE;
        final int peripheries = node.isTarget() ? 2 : 1;

        AstarNode<? extends S, ? extends A> astarNode = astarArg.get(node);
        String descendantLabel = "";
        if (astarNode.descendant != null) {
            descendantLabel = descendantAstarNodeToString.apply(astarNode);
        }
        String label = String.format("%s\\l%s\\l%s",
                stateToString.apply(node.getState()),
                astarNodeToString.apply(astarNode),
                descendantLabel
        );

        final NodeAttributes nAttributes = NodeAttributes.builder().label(label)
                .alignment(LEFT).shape(RECTANGLE).font(FONT).fillColor(FILL_COLOR).lineColor(LINE_COLOR)
                .lineStyle(lineStyle).peripheries(peripheries).build();

        graph.addNode(nodeId, nAttributes);

        for (final ArgEdge<? extends S, ? extends A> edge : node.getOutEdges().collect(Collectors.toSet())) {
            traverse(graph, edge.getTarget(), traversed, astarArg);
            final String sourceId = NODE_ID_PREFIX + edge.getSource().getId();
            final String targetId = NODE_ID_PREFIX + edge.getTarget().getId();
            final EdgeAttributes eAttributes = EdgeAttributes.builder().label(actionToString.apply(edge.getAction()))
                    .alignment(LEFT).font(FONT).color(LINE_COLOR).lineStyle(SUCC_EDGE_STYLE).build();
            graph.addEdge(sourceId, targetId, eAttributes);
        }

        if (node.getCoveringNode().isPresent()) {
            traverse(graph, node.getCoveringNode().get(), traversed, astarArg);
            final String sourceId = NODE_ID_PREFIX + node.getId();
            final String targetId = NODE_ID_PREFIX + node.getCoveringNode().get().getId();
            final EdgeAttributes eAttributes = EdgeAttributes.builder().label("").color(LINE_COLOR)
                    .lineStyle(COVER_EDGE_STYLE).weight(0).build();
            graph.addEdge(sourceId, targetId, eAttributes);
        }
    }

}
