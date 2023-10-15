package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.algorithm.cegar.astar.DI
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.infoLine
import hu.bme.mit.theta.common.visualization.Graph
import hu.bme.mit.theta.common.visualization.writer.GraphvizWriter
import java.io.File
import java.nio.file.Path
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter
import kotlin.io.path.*

abstract class FileVisualizer {
	var enabled = false

	private val testPath: Path by lazy {
		// Run results are kept temporarily distinguished by the start datetime of the run
		val testsPath = Path.of(System.getProperty("java.io.tmpdir"))
			.resolve("theta")
			.resolve(dateTimeOfStart)
			.createDirectories()

		// Create separate directory for each test in the current run
		val testsCount = testsPath.listDirectoryEntries().size
		testsPath.resolve(testsCount.toString()) // TODO pad with zero
			.createDirectory()
	}

	protected fun visualizeBase(title: String, graph: Graph) {
		if (!enabled) {
			return
		}

		val visualizationCount = testPath.listDirectoryEntries().size
		val visualizationId = visualizationCount + 1
		// Filesystem will order 10 after 1 sooner than 2
		val visualizationIdText = "$visualizationId".padStart(5, '0')
		// '∣' != '|' (for Windows)
		val filename = testPath.resolve("$visualizationIdText∣ $title.svg").toString()

		DI.logger.infoLine("|  |  Starting file visualization")
		// TODO make this nice
		File(filename.dropLast(3) + "txt").appendText(GraphvizWriter.getInstance().writeString(graph) + "\n")
		GraphvizWriter.getInstance().writeFileAutoConvert(graph, filename)
		DI.logger.infoLine("|  |  Finished file visualization")
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
