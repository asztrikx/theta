package hu.bme.mit.theta.analysis.algorithm.cegar.astar

typealias Predicate<T> = (T) -> Boolean

infix fun <T> Predicate<T>.and(other: Predicate<T>): Predicate<T> = {
	this(it) && other(it)
}

/**
 * Creates a comparator which will order the elements so that
 * if an item matches a preceding predicate in the ordered predicate list [preds]
 * then it will be positioned earlier.
 */
fun <T> predicateOrderedComparator(vararg preds: Predicate<T>): Comparator<T> = Comparator { t1, t2 ->
	for (pred in preds) {
		if (!pred(t1) && !pred(t2)) {
			continue
		}
		return@Comparator compareByDescending<T> { pred(it) }.compare(t1, t2)
	}
	return@Comparator 0
}
