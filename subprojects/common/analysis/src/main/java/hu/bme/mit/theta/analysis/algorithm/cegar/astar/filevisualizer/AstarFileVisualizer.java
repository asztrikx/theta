package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;

public class AstarFileVisualizer<S extends State, A extends Action, P extends Prec> extends FileVisualizer {
	private final AstarArgStore<S, A, P> astarArgStore;

	public AstarFileVisualizer(boolean enabled, AstarArgStore<S, A, P> astarArgStore) {
		super(enabled);
		this.astarArgStore = astarArgStore;
	}

	@Override
	public void visualize(String state, int index) {
		String titleText = getTitle(state, index);
		visualizeBase(state, titleText, () -> AstarArgVisualizer.getDefault().visualize(astarArgStore.get(index), titleText));
	}

	public String getTitle(String state, int index) {
		// To be consistent with Logger outputs iteration should start from 1 but for avoiding confusion during debugging
		// this will start from 0
		StringBuilder title = new StringBuilder();
		for (int i = astarArgStore.size() - 1; i >= index ; i--) {
			title.append(String.format("%d.", i));
		}
		title.append(String.format(" %s", state));

		return title.toString();
	}

	// getVisualizerState(Collection<...>) will have same erasure
	public static <S extends State, A extends Action> String getVisualizerState(AstarNode<S, A> astarNode) {
		return getVisualizerState(astarNode.argNode);
	}

	public static <S extends State, A extends Action> String getVisualizerState(ArgNode<S, A> node) {
		return String.format("N%d", node.getId());
	}
}
