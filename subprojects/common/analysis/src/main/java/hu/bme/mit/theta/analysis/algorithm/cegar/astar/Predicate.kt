package hu.bme.mit.theta.analysis.algorithm.cegar.astar

typealias Predicate<T> = (T) -> Boolean

infix fun <T> Predicate<T>.and(other: Predicate<T>): Predicate<T> = {
	this(it) && other(it)
}

/**
 * Creates a comparator which will order the elements so that
 * if an item matches a preceding predicate in the ordered predicate list [predicates]
 * then it will be positioned earlier.
 */
fun <T> predicateOrderedComparator(vararg predicates: Predicate<T>) = Comparator<T> { t1, t2 ->
	for (predicate in predicates) {
		if (!predicate(t1) && !predicate(t2)) {
			continue
		}
		return@Comparator compareByDescending<T> { predicate(it) }.compare(t1, t2)
	}
	return@Comparator 0
}
