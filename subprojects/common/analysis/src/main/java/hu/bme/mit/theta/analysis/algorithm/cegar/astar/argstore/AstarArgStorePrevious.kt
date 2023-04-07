package hu.bme.mit.theta.analysis.algorithm.cegar.astar.argstore

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

class AstarArgStorePrevious<S: State, A: Action, P: Prec> : AstarArgStore<S, A, P> {
	private var previous: AstarArg<S, A, P>? = null
	private var current: AstarArg<S, A, P>? = null
	override var size = 0

	override fun add(astarArg: AstarArg<S, A, P>): Boolean {
		previous = current
		current = astarArg
		size++
		return true
	}

	override fun get(index: Int) = when (index) {
		size - 1 -> current!!
		size - 2 -> previous!!
		else -> throw UnstoredAstarArgException()
	}

	override fun indexOf(astarArg: AstarArg<S, A, P>) = when {
		current === astarArg -> size - 1
		previous === astarArg -> size - 2
		else -> throw UnstoredAstarArgException()
	}

	override fun setLast(astarArg: AstarArg<S, A, P>) {
		current = astarArg
	}

	private class UnstoredAstarArgException : Exception("The requested AstarArg is not stored as it shouldn't be needed when doing full expand.")
}
