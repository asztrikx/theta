package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer;

import hu.bme.mit.theta.common.logging.Logger;
import hu.bme.mit.theta.common.logging.NullLogger;
import hu.bme.mit.theta.common.visualization.Graph;
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.function.Supplier;

import static com.google.common.base.Preconditions.checkNotNull;

public abstract class FileVisualizer {
	private static final String nowText = getNowText();
	private File file;

	private boolean enabled;
	private boolean initialized = false;

	public FileVisualizer(boolean enabled) {
		this.enabled = enabled;

		if (enabled) {
			initialize();
		}
	}

	public void initialize() {
		initialized = true;

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

	public abstract void visualize(String state, int index);

	protected void visualizeBase(String state, String title, Supplier<Graph> graphSupplier) {
		checkNotNull(state);

		if (!enabled) {
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

	public void setEnabled(boolean enabled) {
		this.enabled = enabled;

		if (enabled && !initialized) {
			initialize();
		}
	}

	private static String getNowText() {
		LocalDateTime now = LocalDateTime.now();
		DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyy_MM_dd HH_mm_ss");
		return dateTimeFormatter.format(now);
	}
}
