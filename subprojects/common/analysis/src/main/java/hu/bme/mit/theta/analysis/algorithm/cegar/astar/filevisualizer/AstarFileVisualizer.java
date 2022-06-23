package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;
import hu.bme.mit.theta.common.logging.Logger;

import java.util.Collection;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

public class AstarFileVisualizer<S extends State, A extends Action, P extends Prec> extends FileVisualizer {
    private final AstarArgStore<S, A, P> astarArgStore;

    public AstarFileVisualizer(boolean enabled, AstarArgStore<S, A, P> astarArgStore) {
        super(enabled);
        this.astarArgStore = astarArgStore;
    }

    @Override
    public void visualize(String state, int index) {
        // To be consistent with Logger outputs iteration should start from 1 but for avoiding confusion during debugging
        // this will start from 0
        StringBuilder title = new StringBuilder();
        for (int i = astarArgStore.size() - 1; i >= index ; i--) {
            title.append(String.format("%d.", i));
        }
        title.append(String.format(" %s", state));

        String titleText = title.toString();
        visualizeBase(state, titleText, () -> AstarArgVisualizer.getDefault().visualize(astarArgStore.get(index), titleText));
    }

    // getVisualizerState(Collection<...>) will have same erasure
    public static <S extends State, A extends Action> String getVisualizerState(Collection<AstarNode<S, A>> astarNodes) {
        Collection<ArgNode<S, A>> nodes = astarNodes.stream().map(startNode -> startNode.argNode).collect(Collectors.toList());
        return getVisualizerStateILoveJava(nodes);
    }

    public static <S extends State, A extends Action> String getVisualizerStateILoveJava(Collection<ArgNode<S, A>> nodes) {
        StringBuilder stringBuilder = new StringBuilder();
        nodes.forEach(startNode -> stringBuilder.append(String.format("N%d,", startNode.getId())));
        String built = stringBuilder.toString();
        return built.substring(0, built.length() - 1);
    }
}
