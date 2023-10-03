package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.common.logging.Logger

object DI {
    // Only used for assertions
    lateinit var heuristicSearchType: HeuristicSearchType
    /*lateinit*/ var analysisBadLeq = false
    lateinit var logger: Logger
}