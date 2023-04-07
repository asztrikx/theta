package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

interface AstarArgStore<S: State, A: Action, P: Prec> {
	val size: Int

	fun add(astarArg: AstarArg<S, A, P>): Boolean

	operator fun get(index: Int): AstarArg<S, A, P>

	val last: AstarArg<S, A, P>
		get() = get(size - 1)

	fun indexOf(astarArg: AstarArg<S, A, P>): Int

	fun setLast(astarArg: AstarArg<S, A, P>)

	operator fun plusAssign(astarArg: AstarArg<S, A, P>) {
		this.add(astarArg)
	}
}
