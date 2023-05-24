package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ARG
import hu.bme.mit.theta.analysis.algorithm.ArgNode
import hu.bme.mit.theta.analysis.algorithm.ArgTrace

fun <S: State, A: Action> ARG<S, A>.initNodes(): MutableList<ArgNode<S, A>> = initNodes.toList()
fun <S: State, A: Action> ARG<S, A>.initStates(): MutableList<S> = initStates.toList()
fun <S: State, A: Action> ARG<S, A>.nodes(): MutableList<ArgNode<S, A>> = nodes.toList()
fun <S: State, A: Action> ARG<S, A>.unsafeNodes(): MutableList<ArgNode<S, A>> = unsafeNodes.toList()
fun <S: State, A: Action> ARG<S, A>.incompleteNodes(): MutableList<ArgNode<S, A>> = incompleteNodes.toList()
fun <S: State, A: Action> ARG<S, A>.expandedLeafNodes(): MutableList<ArgNode<S, A>> = expandedLeafNodes.toList()
fun <S: State, A: Action> ARG<S, A>.coveredNodes(): MutableList<ArgNode<S, A>> = coveredNodes.toList()
fun <S: State, A: Action> ARG<S, A>.targetNodes(): MutableList<ArgNode<S, A>> = targetNodes.toList()
fun <S: State, A: Action> ARG<S, A>.cexs(): MutableList<ArgTrace<S, A>> = cexs.toList()
