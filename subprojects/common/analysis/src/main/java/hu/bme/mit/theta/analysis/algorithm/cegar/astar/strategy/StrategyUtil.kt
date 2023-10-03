package hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler.AstarNodeCopyHandler
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.astarNodeCopyHandler.DecreasingAstarNodeCopyHandler
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStorageAll
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStoragePrevious
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter.FullDistanceSetter
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.distanceSetter.NonFullDistanceSetter
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer.AstarFileVisualizer
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.DecreasingHeuristicFinder
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.FullHeuristicFinder
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.SemiOndemandHeuristicFinder
import hu.bme.mit.theta.common.logging.Logger

enum class HeuristicSearchType {
    FULL,
    SEMI_ONDEMAND,
    DECREASING;

    companion object {
        val DEFAULT_NON_FULL = DECREASING
    }
}

fun <S: State, A: Action, P: Prec> from(heuristicSearchType: HeuristicSearchType, logger: Logger): Strategy<S, A, P> = when(heuristicSearchType) {
    HeuristicSearchType.FULL -> fullStrategy(logger)
    HeuristicSearchType.SEMI_ONDEMAND -> semiOndemandStrategy(logger)
    HeuristicSearchType.DECREASING -> decreasingStrategy(logger)
}

fun <S: State, A: Action, P: Prec> fullStrategy(logger: Logger): Strategy<S, A, P> {
    val cegarHistoryStorage = CegarHistoryStoragePrevious<S, A, P>()
    val astarFileVisualizer = AstarFileVisualizer(cegarHistoryStorage)
    val heuristicFinder = FullHeuristicFinder<S, A, P>()
    return Strategy(
        logger,
        HeuristicSearchType.FULL,
        cegarHistoryStorage,
        heuristicFinder,
        FullDistanceSetter(),
        AstarNodeCopyHandler(heuristicFinder),
        astarFileVisualizer,
    )
}

fun <S: State, A: Action, P: Prec> semiOndemandStrategy(logger: Logger): Strategy<S, A, P> {
    val cegarHistoryStorage = CegarHistoryStorageAll<S, A, P>()
    val astarFileVisualizer = AstarFileVisualizer(cegarHistoryStorage)
    val heuristicFinder = SemiOndemandHeuristicFinder(astarFileVisualizer, cegarHistoryStorage)
    return Strategy(
        logger,
        HeuristicSearchType.SEMI_ONDEMAND,
        cegarHistoryStorage,
        heuristicFinder,
        NonFullDistanceSetter(),
        AstarNodeCopyHandler(heuristicFinder),
        astarFileVisualizer,
    )
}

fun <S: State, A: Action, P: Prec> decreasingStrategy(logger: Logger): Strategy<S, A, P> {
    val cegarHistoryStorage = CegarHistoryStoragePrevious<S, A, P>()
    val astarFileVisualizer = AstarFileVisualizer(cegarHistoryStorage)
    val decreasingHeuristicFinder = DecreasingHeuristicFinder<S, A, P>()
    return Strategy(
        logger,
        HeuristicSearchType.DECREASING,
        cegarHistoryStorage,
        decreasingHeuristicFinder,
        NonFullDistanceSetter(),
        DecreasingAstarNodeCopyHandler(decreasingHeuristicFinder),
        astarFileVisualizer,
    )
}
