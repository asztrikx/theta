package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.*
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder

class DecreasingAstarNodeCopyHandler<S: State, A: Action, P: Prec>(
    heuristicFinder: HeuristicFinder<S, A, P>
): AstarNodeCopyHandler<S, A, P>(heuristicFinder) {
    override fun invoke(astarNode: AstarNode<S, A>, abstractor: AstarAbstractor<S, A, P>) {
        // Nodes in the next iteration already have covering edges which can break the consistency requirement with the decreasing heuristics.
        // Therefore, we remove those here so that we can check for consistency during search.
        // See XstsTest 48, 51 testcase fail without this.

        heuristicFinder(astarNode, abstractor, null, Long.MAX_VALUE)
    }
}