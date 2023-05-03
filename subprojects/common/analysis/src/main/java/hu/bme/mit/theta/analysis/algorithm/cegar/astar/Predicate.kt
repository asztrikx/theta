package hu.bme.mit.theta.analysis.algorithm.cegar.astar

typealias Predicate<T> = (T) -> Boolean
infix fun <T> Predicate<T>.and(other: Predicate<T>): Predicate<T> = {
	this(it) && other(it)
}
