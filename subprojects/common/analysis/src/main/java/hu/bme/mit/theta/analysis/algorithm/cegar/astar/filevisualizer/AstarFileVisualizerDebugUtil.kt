package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarSearch
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.initNodes

fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugVisualize(
    astarArg: AstarArg<S, A>,
    startNodes: Collection<ArgNode<S, A>>,
    search: AstarSearch<S, A, P>?,
) {
    val enabled = this.enabled
    this.enabled = true
    visualize("debug", cegarHistoryStorage.indexOf(astarArg), startNodes, search)
    this.enabled = enabled
}

fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugInit(astarArg: AstarArg<S, A>, search: AstarSearch<S, A, P>?) {
    debugVisualize(astarArg, astarArg.arg.initNodes(), search)
}

@OptIn(ExperimentalStdlibApi::class)
fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugAll() {
    for (i in 0..<cegarHistoryStorage.size) {
        debugInit(cegarHistoryStorage[i].first, null)
    }
}
