package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import kotlin.jvm.optionals.getOrNull

fun <S: State, A: Action> ArgNode<S, A>.parent() = this.parent.getOrNull()
fun <S: State, A: Action> ArgNode<S, A>.inEdge() = this.inEdge.getOrNull()
fun <S: State, A: Action> ArgNode<S, A>.outEdges() = this.outEdges.toList()
fun <S: State, A: Action> ArgNode<S, A>.coveringNode() = this.coveringNode.getOrNull()
fun <S: State, A: Action> ArgNode<S, A>.coveredNodes() = this.coveredNodes.toList()
fun <S: State, A: Action> ArgNode<S, A>.succNodes() = this.succNodes.toList()
fun <S: State, A: Action> ArgNode<S, A>.succStates() = this.succStates.toList()

val <S: State, A: Action> ArgNode<S, A>.properAncestors
    get() = properAncestors().toList()
val <S: State, A: Action> ArgNode<S, A>.ancestors
    get() = ancestors().toList()
val <S: State, A: Action> ArgNode<S, A>.children
    get() = children().toList()
val <S: State, A: Action> ArgNode<S, A>.properDescendants
    get() = properDescendants().toList()
val <S: State, A: Action> ArgNode<S, A>.descendants
    get() = descendants().toList()
val <S: State, A: Action> ArgNode<S, A>.unexcludedDescendants
    get() = unexcludedDescendants().toList()
