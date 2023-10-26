package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

class CegarHistoryStorageAll<S: State, A: Action, P: Prec> : CegarHistoryStorage<S, A, P> {
	private val cegarHistories = mutableListOf<Pair<AstarArg<S, A>, P>>()

	override val size: Int
		get() = cegarHistories.size

	override fun add(astarArg: AstarArg<S, A>, prec: P) = cegarHistories.add(Pair(astarArg, prec))

	override operator fun get(index: Int) = cegarHistories[index]

	override fun indexOf(astarArg: AstarArg<S, A>): Int {
		// Most of the time the requested astarArg is at the back therefore use lastIndexOf to search from back
		return cegarHistories.indexOfLast { it.first === astarArg }
	}

	override fun setLast(astarArg: AstarArg<S, A>) {
		val last = cegarHistories.last()
		cegarHistories[cegarHistories.lastIndex] = Triple(astarArg, last.second, last.third)
	}
}
