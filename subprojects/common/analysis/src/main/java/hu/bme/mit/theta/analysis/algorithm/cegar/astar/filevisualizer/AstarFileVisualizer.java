package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.CegarHistoryStorage;
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;

import java.util.Collection;

public class AstarFileVisualizer<S extends State, A extends Action, P extends Prec> extends FileVisualizer {
	private final CegarHistoryStorage<S, A, P> cegarHistoryStorage;

	public AstarFileVisualizer(boolean enabled, CegarHistoryStorage<S, A, P> cegarHistoryStorage) {
		super(enabled);
		this.cegarHistoryStorage = cegarHistoryStorage;
	}

	@Override
	public void visualize(String state, int index) {
		AstarArg<S, A, P> astarArg = cegarHistoryStorage.get(index);
		visualize(state, index, astarArg.arg.getInitNodes().toList());
	}

	public void visualize(String state, int index, Collection<ArgNode<S, A>> startNodes) {
		String titleText = getTitle(state, index);
		visualizeBase(state, titleText, () -> AstarArgVisualizer.getDefault().visualize(cegarHistoryStorage.get(index), titleText, startNodes));
	}

	public String getTitle(String state, int index) {
		// To be consistent with Logger outputs iteration should start from 1 but for avoiding confusion during debugging
		// this will start from 0
		return String.format("%d ⁄ %d %s", index, cegarHistoryStorage.getSize() - 1, state);
	}

	// getVisualizerState(Collection<...>) will have same erasure
	public static <S extends State, A extends Action> String getVisualizerState(AstarNode<S, A> astarNode) {
		return getVisualizerState(astarNode.argNode);
	}

	public static <S extends State, A extends Action> String getVisualizerState(ArgNode<S, A> node) {
		return String.format("N%d", node.getId());
	}
}
