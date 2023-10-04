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
    override fun invoke(astarNode: AstarNode<S, A>, astarAbstractor: AstarAbstractor<S, A, P>) {
        // Nodes in the next iteration already have covering edges which can break the consistency requirement with the decreasing heuristics.
        // Therefore, we remove those here so that we can check for consistency during search.
        // See XstsTest 48, 51 testcase fail without this.

        val astarArg = astarNode.astarArg
        val argNode = astarNode.argNode

        heuristicFinder(astarNode, astarAbstractor)

        argNode.coveringNode()?.let {
            astarArg.handleDecreasingCoverEdgeConsistency(argNode, it)
        }
        // Avoid concurrent modification exception by copying stream to list
        argNode.coveredNodes().forEach { coveredNode ->
            astarArg.handleDecreasingCoverEdgeConsistency(coveredNode, argNode)
        }
    }

    // There is no guarantee that cover edges will still be consistent
    private fun <S: State, A: Action> AstarArg<S, A>.handleDecreasingCoverEdgeConsistency(
        coveredNode: ArgNode<S, A>,
        coveringNode: ArgNode<S, A>,
    ) {
        // Other astar node (covering or covered) may not exist when trying to handle consistency
        // It will be checked from the other end of the edge later.
        if (coveredNode !in this || coveringNode !in this) {
            return
        }

        val astarCoveredNode = this[coveredNode]
        val astarCoveringNode = this[coveringNode]
        check(astarCoveredNode.heuristic.isKnown && astarCoveringNode.heuristic.isKnown)
        if (astarCoveredNode.heuristic > astarCoveringNode.heuristic) {
            coveredNode.unsetCoveringNode()
        }
    }
}