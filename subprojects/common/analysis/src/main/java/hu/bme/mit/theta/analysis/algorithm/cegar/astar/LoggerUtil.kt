package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.common.logging.Logger

fun Logger.detail(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.DETAIL, pattern, *objects)
fun Logger.info(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.INFO, pattern, *objects)
fun Logger.mainstep(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.MAINSTEP, pattern, *objects)
fun Logger.result(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.RESULT, pattern, *objects)
fun Logger.substep(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.SUBSTEP, pattern, *objects)
fun Logger.verbose(pattern: String, vararg objects: Any): Logger = this.write(Logger.Level.VERBOSE, pattern, *objects)

fun Logger.detailLine(pattern: String, vararg objects: Any): Logger = this.detail("$pattern%n", *objects)
fun Logger.infoLine(pattern: String, vararg objects: Any): Logger = this.info("$pattern%n", *objects)
fun Logger.mainstepLine(pattern: String, vararg objects: Any): Logger = this.mainstep("$pattern%n", *objects)
fun Logger.resultLine(pattern: String, vararg objects: Any): Logger = this.result("$pattern%n", *objects)
fun Logger.substepLine(pattern: String, vararg objects: Any): Logger = this.substep("$pattern%n", *objects)
fun Logger.verboseLine(pattern: String, vararg objects: Any): Logger = this.verbose("$pattern%n", *objects)
