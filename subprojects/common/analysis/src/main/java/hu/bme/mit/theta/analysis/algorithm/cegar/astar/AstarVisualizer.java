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
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

public class AstarVisualizer<S extends State, A extends Action, P extends Prec> {
    private static final String nowText = getNowText();
    private final Logger logger;
    private final AstarArgStore<S, A, P> astarArgStore;

    public AstarVisualizer(Logger logger, AstarArgStore<S, A, P> astarArgStore) {
        this.logger = logger;
        this.astarArgStore = astarArgStore;
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
        return built.substring(0, built.length() - 2);
    }

    public void visualize(String state, int index) {
        checkNotNull(state);

        if (logger == NullLogger.getInstance()) {
            return;
        }

        // To be consistent with Logger outputs iteration should start from 1 but for avoiding confusion during debugging
        // this will start from 0
        StringBuilder title = new StringBuilder();
        for (int i = astarArgStore.size() - 1; i >= index ; i--) {
            title.append(String.format("%d.", i));
        }
        title.append(String.format(" %s", state));

        try {
            File directory = new File(String.format("%s/theta/%s", System.getProperty("java.io.tmpdir"), nowText));
            if (!directory.exists()) {
                boolean successful = directory.mkdirs();
                assert successful;
            }

            File file = new File(directory.getCanonicalPath());
            // '∣' != '|' (for Windows)
            File[] subfiles = file.listFiles();
            assert subfiles != null;
            String filename = String.format("%s/%d∣ %s.png", directory.getCanonicalPath(), subfiles.length + 1, title);

            GraphvizWriter.getInstance().writeFileAutoConvert(AstarArgVisualizer.getDefault().visualize(astarArgStore.get(index), title.toString()), filename);
        } catch (IOException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private static String getNowText() {
        LocalDateTime now = LocalDateTime.now();
        DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyy_MM_dd HH_mm_ss");
        return dateTimeFormatter.format(now);
    }
}
