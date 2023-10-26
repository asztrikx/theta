package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch

typealias PausedSearches<S, A, P> = HashMap<AstarNode<S, A>, AstarSearch<S, A, P>>
typealias CegarHistory<S, A, P> = Triple<AstarArg<S, A>, P, PausedSearches<S, A, P>>

interface CegarHistoryStorage<S: State, A: Action, P: Prec> {
	// TODO size vs index, previous doesn't have all the items can't iterate from 0..<size
	val size: Int

	// [PausedSearches] are set to an empty map
	fun add(astarArg: AstarArg<S, A>, prec: P): Boolean

	operator fun get(index: Int): CegarHistory<S, A, P>

	val last: CegarHistory<S, A, P>
		get() = get(size - 1)

	fun indexOf(astarArg: AstarArg<S, A>): Int

	fun find(astarArg: AstarArg<S, A>) = get(indexOf(astarArg))

	fun setLast(astarArg: AstarArg<S, A>)
}
