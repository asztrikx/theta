package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder

open class AstarNodeCopyHandler<S: State, A: Action, P: Prec>(
    val heuristicFinder: HeuristicFinder<S, A, P>,
) {
    // Fully and Semi-ondemand will always give the same heuristic for covering- and covered node (=> consistent) as they are based on distance
    open operator fun invoke(astarNode: AstarNode<S, A>) {}
}