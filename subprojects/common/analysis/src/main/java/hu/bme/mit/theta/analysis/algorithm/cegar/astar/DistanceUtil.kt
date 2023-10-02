package hu.bme.mit.theta.analysis.algorithm.cegar.astar

operator fun Distance.plus(otherValue: Int) = Distance.boundedOf(value + otherValue)
operator fun Distance.plus(otherDistance: Distance) = Distance.boundedOf(value + otherDistance.value)
operator fun Distance.minus(otherValue: Int) = this + (-otherValue)
operator fun Distance.minus(otherDistance: Distance) = this + (-otherDistance.value)
