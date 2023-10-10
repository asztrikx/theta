package hu.bme.mit.theta.analysis.algorithm.cegar.astar

operator fun Distance.plus(otherValue: Int) = Distance.finiteOf(value + otherValue)
operator fun Distance.plus(otherDistance: Distance) = Distance.finiteOf(value + otherDistance.value)
operator fun Distance.minus(otherValue: Int) = Distance.finiteOf(value - otherValue)
operator fun Distance.minus(otherDistance: Distance) = Distance.finiteOf(value - otherDistance.value)

val Int.distance: Distance
    get() = Distance.finiteOf(this)
