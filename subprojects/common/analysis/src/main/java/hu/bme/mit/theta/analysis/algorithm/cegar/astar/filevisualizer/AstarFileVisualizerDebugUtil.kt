package hu.bme.mit.theta.analysis.algorithm.cegar.astar.filevisualizer

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.AstarArg
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.initNodes

fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugVisualize(astarArg: AstarArg<S, A>, startNodes: Collection<ArgNode<S, A>>) {
    val enabled = this.enabled
    this.enabled = true
    visualize("debug", cegarHistoryStorage.indexOf(astarArg), startNodes)
    this.enabled = enabled
}

fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugInit(astarArg: AstarArg<S, A>) {
    debugVisualize(astarArg, astarArg.arg.initNodes())
}

fun <S: State, A: Action, P: Prec> AstarFileVisualizer<S, A, P>.debugAll() {
    for (i in 0 until cegarHistoryStorage.size) {
        debugInit(cegarHistoryStorage[i].first)
    }
}
