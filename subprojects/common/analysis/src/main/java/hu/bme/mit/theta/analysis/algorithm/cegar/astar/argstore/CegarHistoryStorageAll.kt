package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

class CegarHistoryStorageAll<S: State, A: Action, P: Prec> : CegarHistoryStorage<S, A, P> {
	private val astarArgs = mutableListOf<AstarArg<S, A, P>>()

	override val size: Int
		get() = astarArgs.size

	override fun add(astarArg: AstarArg<S, A, P>) = astarArgs.add(astarArg)

	override operator fun get(index: Int) = astarArgs[index]

	override fun indexOf(astarArg: AstarArg<S, A, P>): Int {
		// Most of the time the requested astarArg is at the back therefore use lastIndexOf to search from back
		return astarArgs.lastIndexOf(astarArg)
	}

	override fun setLast(astarArg: AstarArg<S, A, P>) {
		astarArgs[astarArgs.lastIndex] = astarArg
	}
}
