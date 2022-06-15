package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.Prec;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore.AstarArgStore;
import hu.bme.mit.theta.analysis.utils.AstarArgVisualizer;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Collection;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

public class AstarFileVisualizer<S extends State, A extends Action, P extends Prec> extends FileVisualizer {
    private final AstarArgStore<S, A, P> astarArgStore;

    public AstarFileVisualizer(Logger logger, AstarArgStore<S, A, P> astarArgStore) {
        super(logger);
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
}
