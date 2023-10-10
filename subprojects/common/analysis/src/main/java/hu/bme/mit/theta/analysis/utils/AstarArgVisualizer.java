/*
 *  Copyright 2017 Budapest University of Technology and Economics
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package hu.bme.mit.theta.analysis.utils;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgEdge;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch;
import hu.bme.mit.theta.common.container.Containers;
import hu.bme.mit.theta.common.visualization.EdgeAttributes;
import hu.bme.mit.theta.common.visualization.Graph;
import hu.bme.mit.theta.common.visualization.LineStyle;
import hu.bme.mit.theta.common.visualization.NodeAttributes;

import javax.annotation.Nullable;
import java.awt.*;
import java.util.Collection;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import static hu.bme.mit.theta.common.visualization.Alignment.LEFT;
import static hu.bme.mit.theta.common.visualization.Shape.RECTANGLE;

// TODO check this
public final class AstarArgVisualizer<S extends State, A extends Action> {

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
	private final Function<? super ArgNode<? extends S, ? extends A>, String> argNodeToString;

	private static class LazyHolderDefault {
		static final AstarArgVisualizer<State, Action> INSTANCE = new AstarArgVisualizer<>(s -> s.toString(), a -> a.toString(), astarNode -> astarNode.toString(), n -> n.toString());
	}

	private static class LazyHolderStructureOnly {
		static final AstarArgVisualizer<State, Action> INSTANCE = new AstarArgVisualizer<>(s -> "", a -> "", astarNode -> "", n -> "");
	}

	private AstarArgVisualizer(
			final Function<S, String> stateToString,
			final Function<A, String> actionToString,
			final Function<? super AstarNode<? extends S, ? extends A>, String> astarNodeToString,
			final Function<? super ArgNode<? extends S, ? extends A>, String> argNodeToString
	) {
		this.stateToString = stateToString;
		this.actionToString = actionToString;
		this.astarNodeToString = astarNodeToString;
		this.argNodeToString = argNodeToString;
	}

	public static <S extends State, A extends Action> AstarArgVisualizer<S, A> create(
			final Function<S, String> stateToString,
			final Function<A, String> actionToString,
			final Function<? super AstarNode<? extends S, ? extends A>, String> astarNodeToString,
			final Function<? super ArgNode<? extends S, ? extends A>, String> argNodeToString
	) {
		return new AstarArgVisualizer<>(stateToString, actionToString, astarNodeToString, argNodeToString);
	}

	public static AstarArgVisualizer<State, Action> getDefault() {
		return LazyHolderDefault.INSTANCE;
	}

	public static AstarArgVisualizer<State, Action> getStructureOnly() {
		return LazyHolderStructureOnly.INSTANCE;
	}

	public <S1 extends S, A1 extends A> Graph visualize(
		final AstarArg<S1, A1> astarArg,
		Collection<ArgNode<S1, A1>> startNodes,
		@Nullable AstarSearch<S1, A1, ?> search
	) {
		final Graph graph = new Graph(ARG_ID, ARG_LABEL);

		final Set<ArgNode<S1, A1>> traversed = Containers.createSet();

		for (final ArgNode<S1, A1> startNode : startNodes) {
			// we might be visualizing a "back" state from a child just expanded => there could be children which don't have yet AstarNodes
			AstarNode<S1, A1> astarStartNode = astarArg.get(startNode);
			if(astarStartNode != null) {
				traverse(graph, astarStartNode, traversed, astarArg, search);

				final NodeAttributes nAttributes = NodeAttributes.builder().label("").fillColor(FILL_COLOR)
						.lineColor(FILL_COLOR).lineStyle(getLineStyle(astarStartNode)).peripheries(1).build();
				graph.addNode(PHANTOM_INIT_ID + startNode.getId(), nAttributes);
				final EdgeAttributes eAttributes = EdgeAttributes.builder().label("").color(LINE_COLOR)
						.lineStyle(SUCC_EDGE_STYLE).build();
				graph.addEdge(PHANTOM_INIT_ID + startNode.getId(), NODE_ID_PREFIX + startNode.getId(), eAttributes);
			}
		}

		return graph;
	}

	private <S1 extends S, A1 extends A> LineStyle getLineStyle(AstarNode<S1, A1> astarNode) {
		if (astarNode.getArgNode().isExpanded() && astarNode.getArgNode().getSuccNodes().findAny().isEmpty()) {
			return LineStyle.DASHED;
		} else {
			return LineStyle.NORMAL;
		}
	}

	private <S1 extends S, A1 extends A> void traverse(
		final Graph graph,
		final AstarNode<S1, A1> astarNode,
		final Set<ArgNode<S1, A1>> traversed,
		AstarArg<S1, A1> astarArg,
		@Nullable AstarSearch<S1, A1, ?> search
	) {
		final ArgNode<S1, A1> node = astarNode.getArgNode();
		if (traversed.contains(node)) {
			return;
		}
		traversed.add(node);
		final String nodeId = NODE_ID_PREFIX + node.getId();
		final int peripheries = node.isTarget() ? 2 : 1;

		// node format: information about node and it's parent (if exists)
		String label = String.format("%s\\l%s\\l%s\\l%s",
				stateToString.apply(node.getState()),
				String.format("Search:   %s", getSearchAstarNodeDetailsText(astarNode, search)),
				String.format("Current:  %s", getAstarNodeDetailsText(astarNode)),
				String.format("Provider: %s", getAstarNodeDetailsText(astarNode.getProviderAstarNode()))
		);

		Color lineColor = Color.BLACK;
		if (astarNode.getArgNode().isLeftover()) {
			lineColor = Color.GREEN;
		}
		final NodeAttributes nAttributes = NodeAttributes.builder().label(label)
				.alignment(LEFT).shape(RECTANGLE).font(FONT).fillColor(FILL_COLOR).lineColor(lineColor)
				.lineStyle(getLineStyle(astarNode)).peripheries(peripheries).build();

		graph.addNode(nodeId, nAttributes);

		for (final ArgEdge<S1, A1> edge : node.getOutEdges().collect(Collectors.toSet())) {
			// We might be searching for provider node for children nodes created by expand.
			// In that case a visualization will be made of that graph, but those children not have AstarNode because of the latter.
			AstarNode<S1, A1> astarNodeChild = astarArg.get(edge.getTarget());
			if(astarNodeChild != null){
				traverse(graph, astarNodeChild, traversed, astarArg, search);
				createEdge(graph, node, astarNodeChild, SUCC_EDGE_STYLE, ""); //actionToString.apply(edge.getAction())
			}
		}

		if (node.getCoveringNode().isPresent()) {
			// we might be visualizing a "back" state from a child just expanded => there could be children which don't have yet AstarNodes
			AstarNode<S1, A1> astarNodeChild = astarArg.get(node.getCoveringNode().get());
			if(astarNodeChild != null){
				traverse(graph, astarNodeChild, traversed, astarArg, search);
				createEdge(graph, node, astarNodeChild, COVER_EDGE_STYLE, "");
			}
		}
	}

	private <S1 extends S, A1 extends A> String getAstarNodeDetailsText(@Nullable AstarNode<S1, A1> astarNode) {
		if (astarNode == null) {
			return "-";
		}
		return astarNodeToString.apply(astarNode);
	}

	private <S1 extends S, A1 extends A> String getSearchAstarNodeDetailsText(AstarNode<S1, A1> astarNode, AstarSearch<S1, A1, ?> search) {
		if (!search.contains(astarNode)) {
			return "-";
		}
		return search.toString(astarNode);
	}

	private <S1 extends S, A1 extends A> void createEdge(
			Graph graph, ArgNode<S1, A1> parent, AstarNode<S1, A1> child, LineStyle lineStyle, String actionText
	) {
		final String sourceId = NODE_ID_PREFIX + parent.getId();
		final String targetId = NODE_ID_PREFIX + child.getArgNode().getId();
		final EdgeAttributes eAttributes = EdgeAttributes.builder().label(actionText).color(LINE_COLOR)
				.alignment(LEFT).font(FONT).lineStyle(lineStyle).weight(0).build();
		graph.addEdge(sourceId, targetId, eAttributes);
	}
}
