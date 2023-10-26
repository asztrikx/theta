package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg

class CegarHistoryStoragePrevious<S: State, A: Action, P: Prec> : CegarHistoryStorage<S, A, P> {
	private var previous: CegarHistory<S, A, P>? = null
	private var current: CegarHistory<S, A, P>? = null
	override var size = 0

	override fun add(astarArg: AstarArg<S, A>, prec: P): Boolean {
		previous = current
		current = Triple(astarArg, prec, hashMapOf())
		size++

		// Release references
		previous?.let {
			val (previousAstarArg) = it
			previousAstarArg.provider = null
			previousAstarArg.astarNodes.values.forEach {
				it.providerAstarNode = null
			}
		}

		return true
	}

	override fun get(index: Int) = when (index) {
		size - 1 -> current!!
		size - 2 -> previous!!
		else -> throw UnstoredAstarArgException()
	}

	override fun indexOf(astarArg: AstarArg<S, A>) = when {
		current?.first === astarArg -> size - 1
		previous?.first === astarArg -> size - 2
		else -> throw UnstoredAstarArgException()
	}

	override fun setLast(astarArg: AstarArg<S, A>) {
		require(size != 0)
		current = Triple(astarArg, current!!.second, current!!.third)
	}

	private class UnstoredAstarArgException : Exception("The requested AstarArg is not stored as it shouldn't be needed when doing full expand.")
}
