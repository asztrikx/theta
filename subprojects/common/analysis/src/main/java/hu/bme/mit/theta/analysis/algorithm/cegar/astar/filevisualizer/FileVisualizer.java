package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;
import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.visualization.Graph;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Collection;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static com.google.common.base.Preconditions.checkNotNull;

public abstract class FileVisualizer {
	private static final String nowText = getNowText();
	private final Logger logger;
	private File file;

	public FileVisualizer(Logger logger) {
		this.logger = logger;

		if (logger == NullLogger.getInstance()) {
			return;
		}

		File parentDir = new File(String.format("%s/theta/%s", System.getProperty("java.io.tmpdir"), nowText));
		if (!parentDir.exists()) {
			boolean successful = parentDir.mkdirs();
			assert successful;
		}
		int testCount = parentDir.listFiles().length;
		File directory = new File(String.format("%s/theta/%s/%d", System.getProperty("java.io.tmpdir"), nowText, testCount));
		assert directory.mkdirs();
		try {
			file = new File(directory.getCanonicalPath());
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
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

	public abstract void visualize(String state, int index);

	protected void visualizeBase(String state, String title, Supplier<Graph> graphSupplier) {
		checkNotNull(state);

		if (logger == NullLogger.getInstance()) {
			return;
		}

		try {
			// '∣' != '|' (for Windows)
			File[] subfiles = file.listFiles();
			assert subfiles != null;
			String filename = String.format("%s/%d∣ %s.png", file.getCanonicalPath(), subfiles.length + 1, title);
			GraphvizWriter.getInstance().writeFileAutoConvert(graphSupplier.get(), filename);
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
