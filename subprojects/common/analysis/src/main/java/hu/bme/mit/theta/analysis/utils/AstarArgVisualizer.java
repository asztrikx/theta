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
    private static final String NODE_ID_PREFIX = "N";
    private static final Color FILL_COLOR = Color.WHITE;
    private static final Color LINE_COLOR = Color.BLACK;
    private static final String PHANTOM_INIT_ID = "phantom_init";

    private final Function<S, String> stateToString;
    private final Function<A, String> actionToString;
    private final Function<? super AstarNode<? extends S, ? extends A>, String> astarNodeToString;

    private static class LazyHolderDefault {
        static final AstarArgVisualizer<State, Action, Prec> INSTANCE = new AstarArgVisualizer<>(s -> s.toString(), a -> a.toString(), n -> n.toString());
    }

    private static class LazyHolderStructureOnly {
        static final AstarArgVisualizer<State, Action, Prec> INSTANCE = new AstarArgVisualizer<>(s -> "", a -> "", n -> "");
    }

    public AstarArgVisualizer(
            final Function<S, String> stateToString,
            final Function<A, String> actionToString,
            final Function<? super AstarNode<? extends S, ? extends A>, String> astarNodeToString
    ) {
        this.stateToString = stateToString;
        this.actionToString = actionToString;
        this.astarNodeToString = astarNodeToString;
    }

    public static <S extends State, A extends Action, P extends Prec> AstarArgVisualizer<S, A, P> create(
            final Function<S, String> stateToString,
            final Function<A, String> actionToString,
            final Function<? super AstarNode<? extends S, ? extends A>, String> astarNodeToString) {
        return new AstarArgVisualizer<>(stateToString, actionToString, astarNodeToString);
    }

    public static AstarArgVisualizer<State, Action, Prec> getDefault() {
        return LazyHolderDefault.INSTANCE;
    }

    public static AstarArgVisualizer<State, Action, Prec> getStructureOnly() {
        return LazyHolderStructureOnly.INSTANCE;
    }

    public <S1 extends S, A1 extends A, P1 extends P>  Graph visualize(final AstarArg<S1, A1, P1> astarArg, String title) {
        ARG<S1, A1> arg = astarArg.arg;

        final Graph graph = new Graph(ARG_ID, title);

        final Set<ArgNode<S1, A1>> traversed = Containers.createSet();

        for (final ArgNode<S1, A1> initNode : arg.getInitNodes().collect(Collectors.toSet())) {
            // we might be visualizing a "back" state from a child just expanded => there could be children which don't have yet AstarNodes
            AstarNode<S1, A1> astarInitNode = astarArg.get(initNode);
            if(astarInitNode != null) {
                traverse(graph, astarInitNode, traversed, astarArg);
                final NodeAttributes nAttributes = NodeAttributes.builder().label("").fillColor(FILL_COLOR)
                        .lineColor(FILL_COLOR).lineStyle(SUCC_EDGE_STYLE).peripheries(1).build();
                graph.addNode(PHANTOM_INIT_ID + initNode.getId(), nAttributes);
                final EdgeAttributes eAttributes = EdgeAttributes.builder().label("").color(LINE_COLOR)
                        .lineStyle(SUCC_EDGE_STYLE).build();
                graph.addEdge(PHANTOM_INIT_ID + initNode.getId(), NODE_ID_PREFIX + initNode.getId(), eAttributes);
            }
        }

        return graph;
    }

    private <S1 extends S, A1 extends A, P1 extends P> void traverse(
            final Graph graph,
            final AstarNode<S1, A1> astarNode,
            final Set<ArgNode<S1, A1>> traversed,
            AstarArg<S1, A1, P1> astarArg
    ) {
        final ArgNode<S1, A1> node = astarNode.argNode;
        if (traversed.contains(node)) {
            return;
        } else {
            traversed.add(node);
        }
        final String nodeId = NODE_ID_PREFIX + node.getId();
        final LineStyle lineStyle = SUCC_EDGE_STYLE;
        final int peripheries = node.isTarget() ? 2 : 1;

        // node format: information about node and it's parent (if exists)
        String parentLabel = "-";
        if (astarNode.coveringAstarNode != null) {
            parentLabel = astarNodeToString.apply(astarNode.coveringAstarNode);
        }
        String label = String.format("%s\\l%s\\l%s",
                stateToString.apply(node.getState()),
                String.format("AstarNode: %s", astarNodeToString.apply(astarNode)),
                String.format("Parent: %s", parentLabel)
        );

        final NodeAttributes nAttributes = NodeAttributes.builder().label(label)
                .alignment(LEFT).shape(RECTANGLE).font(FONT).fillColor(FILL_COLOR).lineColor(LINE_COLOR)
                .lineStyle(lineStyle).peripheries(peripheries).build();

        graph.addNode(nodeId, nAttributes);

        for (final ArgEdge<S1, A1> edge : node.getOutEdges().collect(Collectors.toSet())) {
            // we might be visualizing a "back" state from a child just expanded => there could be children which don't have yet AstarNodes
            AstarNode<S1, A1> astarNodeChild = astarArg.get(edge.getTarget());
            if(astarNodeChild != null){
                traverse(graph, astarNodeChild, traversed, astarArg);
                final String sourceId = NODE_ID_PREFIX + edge.getSource().getId();
                final String targetId = NODE_ID_PREFIX + edge.getTarget().getId();
                final EdgeAttributes eAttributes = EdgeAttributes.builder().label(actionToString.apply(edge.getAction()))
                        .alignment(LEFT).font(FONT).color(LINE_COLOR).lineStyle(SUCC_EDGE_STYLE).build();
                graph.addEdge(sourceId, targetId, eAttributes);
            }
        }

        if (node.getCoveringNode().isPresent()) {
            // we might be visualizing a "back" state from a child just expanded => there could be children which don't have yet AstarNodes
            AstarNode<S1, A1> astarNodeChild = astarArg.get(node.getCoveringNode().get());
            if(astarNodeChild != null){
                traverse(graph, astarNodeChild, traversed, astarArg);
                final String sourceId = NODE_ID_PREFIX + node.getId();
                final String targetId = NODE_ID_PREFIX + node.getCoveringNode().get().getId();
                final EdgeAttributes eAttributes = EdgeAttributes.builder().label("").color(LINE_COLOR)
                        .lineStyle(COVER_EDGE_STYLE).weight(0).build();
                graph.addEdge(sourceId, targetId, eAttributes);
            }
        }
    }

}
