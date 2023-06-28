package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

interface CegarHistoryStorage<S: State, A: Action, P: Prec> {
	val size: Int

	fun add(astarArg: AstarArg<S, A>, prec: P): Boolean

	operator fun get(index: Int): Pair<AstarArg<S, A>, P>

	val last: Pair<AstarArg<S, A>, P>
		get() = get(size - 1)

	fun indexOf(astarArg: AstarArg<S, A>): Int

	fun find(astarArg: AstarArg<S, A>): Pair<AstarArg<S, A>, P>

	fun setLast(astarArg: AstarArg<S, A>, prec: P)

	operator fun plusAssign(cegarHistory: Pair<AstarArg<S, A>, P>) {
		this.add(cegarHistory.first, cegarHistory.second)
	}
}
