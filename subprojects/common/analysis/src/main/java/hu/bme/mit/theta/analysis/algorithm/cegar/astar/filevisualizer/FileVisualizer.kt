package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.common.visualization.Graph
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter
import java.nio.file.Path
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import kotlin.io.path.*

abstract class FileVisualizer(var enabled: Boolean) {
	private val testPath: Path by lazy {
		// Run results are kept temporarily distinguished by the start datetime of the run
		val testsPath = Path.of(System.getProperty("java.io.tmpdir"))
			.resolve("theta")
			.resolve(dateTimeOfStart)
			.createDirectories()

		// Create separate directory for each test in the current run
		val testsCount = testsPath.listDirectoryEntries().size
		testsPath.resolve(testsCount.toString())
			.createDirectory()
	}

	protected fun visualizeBase(title: String, graph: Graph) {
		if (!enabled) {
			return
		}

		val visualizationCount = testPath.listDirectoryEntries().size
		val visualizationId = visualizationCount + 1
		// '∣' != '|' (for Windows)
		val filename = testPath.resolve("$visualizationId∣ $title.svg").toString()

		GraphvizWriter.getInstance()
			.writeFileAutoConvert(graph, filename)
	}

	// Must call visualizeBase
	abstract fun visualize(state: String, index: Int)

	companion object {
		// Date should be determined statically as getting to a point where we want to visualize
		// could largely differ from the time of starting the tests
		private val dateTimeOfStart = DateTimeFormatter.ofPattern("yyyy_MM_dd HH_mm_ss")
			.format(LocalDateTime.now())
	}
}