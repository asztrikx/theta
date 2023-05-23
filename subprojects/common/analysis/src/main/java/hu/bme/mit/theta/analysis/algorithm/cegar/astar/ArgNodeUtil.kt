package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import kotlin.jvm.optionals.getOrNull

fun <S: State, A: Action> ArgNode<S, A>.coveringNode() = this.coveringNode.getOrNull()
fun <S: State, A: Action> ArgNode<S, A>.parent() = this.parent.getOrNull()
