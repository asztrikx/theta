package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.algorithm.cegar.astar.Distance.Companion.sameTypeOfNoCache

operator fun Distance.plus(otherValue: Int) = sameTypeOfNoCache(this, value + otherValue)
operator fun Distance.plus(otherDistance: Distance) = sameTypeOfNoCache(this, value + otherDistance.value)
operator fun Distance.minus(otherValue: Int) = sameTypeOfNoCache(this, value - otherValue)
operator fun Distance.minus(otherDistance: Distance) = sameTypeOfNoCache(this, value - otherDistance.value)

fun safeAdd(distance: Distance, value: Int) =
	if (distance.isInfinite) {
		distance
	} else {
		distance + value
	}
