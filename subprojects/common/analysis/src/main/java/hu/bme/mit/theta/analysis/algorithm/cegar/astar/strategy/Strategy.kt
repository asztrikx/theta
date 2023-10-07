package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.DI
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler.AstarNodeCopyHandler
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter.DistanceSetter
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder
import hu.bme.mit.theta.common.logging.Logger

class Strategy<S: State, A: Action, P: Prec>(
    logger: Logger,
    heuristicSearchType: HeuristicSearchType,
    val cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
    val heuristicFinder: HeuristicFinder<S, A, P>,
    val distanceSetter: DistanceSetter<S, A, P>,
    val astarNodeCopyHandler: AstarNodeCopyHandler<S, A, P>,
    val astarFileVisualizer: AstarFileVisualizer<S, A, P>
) {
    init {
        DI.heuristicSearchType = heuristicSearchType
        DI.logger = logger
        astarFileVisualizer.enabled = true
    }
}
