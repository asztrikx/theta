package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType

fun <S: State, A: Action> AstarNode<S, A>.checkConsistency(child: AstarNode<S, A>) {
    val parent = this
    require(parent.heuristic.isKnown && child.heuristic.isKnown)

    if (parent.heuristic.isInfinite && child.heuristic.isInfinite) {
        return
    }
    check(!parent.heuristic.isInfinite)
    if (child.heuristic.isInfinite) {
        return
    }

    val heuristicDistanceValue = parent.heuristic - child.heuristic
    val edgeWeight = if (parent.argNode.isCovered) 0 else 1
    check(heuristicDistanceValue.value <= edgeWeight)
}

/**
 * Checks property: heuristic <= distance
 */
fun <S: State, A: Action> AstarNode<S, A>.checkAdmissibility() {
    check(!(heuristic.isInfinite && !distance.isInfinite))
    check(distance.isKnown)
    check(heuristic.isKnown)
    if (!heuristic.isInfinite && !distance.isInfinite) {
        check(heuristic.value <= distance.value)
    }
}

// TODO why do astarArg.reachedSet[astarNode] inside the function?
fun <S: State, A: Action, P: Prec> AstarNode<S, A>.close(
    candidates: Collection<AstarNode<S, A>>,
    search: AstarSearch<S, A, P>?
): AstarNode<S, A>? {
    if ((argNode.isExpanded || !argNode.isLeaf) || argNode.isCovered) {
        // isLeaf: After prune node may have children but not fully expanded.
        // isExpanded: If node has no children it still can already be expanded, therefore expanded is already set (should not be covered).
        // isCovered: If node already has covering node, close cloud still choose another one, therefore avoid.
        return null
    }

    var candidates = candidates
    if (argNode.isTarget) {
        // optimization (leq uses smt solver): target node can only be covered with a target node
        candidates = candidates.filter { it.argNode.isTarget }
    }

    for (astarCandidate in candidates) {
        // optimization: Check before calling mayCover which uses Leq
        if (heuristic > astarCandidate.heuristic) {
            continue
        }

        val candidate = astarCandidate.argNode
        if (!candidate.mayCover(argNode)) {
            continue
        }

        argNode.cover(candidate)
        checkConsistency(astarCandidate)
        search ?: return null
        return handleCloseRewire(search)
    }
    return null
}

/**
 * @return The actual AstarNode which is covered after [AstarNode.close].
 * The AstarNode reference should be replaced with the return value.
 */
fun <S: State, A: Action, P: Prec> AstarNode<S, A>.handleCloseRewire(search: AstarSearch<S, A, P>): AstarNode<S, A> {
    // If astarNode's parent is also covered then covering edges have been redirected. (see ArgNode::cover)
    // We have to update parents map according to that.
    //  1) a - - -> b (argNode,coveredAstarNode)
    //  2) a - - -> b - - -> c (coveringNode)
    //  3) a        b - - -> c
    //	   |                 ^
    //     | - - - - - - - - |
    // Heuristic consistency doesn't break as `c` has >= heuristic than `b`
    val parentAstarNode = search.parents[this]
    val parentArgNode = parentAstarNode?.argNode
    val coveredAstarNode = if (parentArgNode != null && parentArgNode.isCovered) {
        check(parentArgNode.coveringNode()!! === argNode.coveringNode()!!)

        // Because argNode is covered it can only reach coveringNode with the same distance as it's new parent
        // therefore we can safely remove it
        search.parents.remove(this)

        parentAstarNode.checkConsistency(astarArg[argNode.coveringNode()!!])

        // Update to new parent if we this node is the current parent
        // as coveringAstarNode may already have a better parent or already in doneSet
        parentAstarNode
    } else {
        this
    }
    return coveredAstarNode
}

/**
 * [astarNode] is the provider node of a different node. This is used to creates children for [astarNode],
 * so that different node's children will have candidate provider nodes.
 *
 * A node's children :=
 * - if it is/can be expanded: its tree children
 * - if it is/can be covered: its coverer node's children (recursive definition)
 *
 * [heuristicFinder] is not called during this.
 */
fun <S: State, A: Action, P: Prec> AstarNode<S, A>.createChildren(prec: P, search: AstarSearch<S, A, P>?, argBuilder: ArgBuilder<S, A, P>) {
    require(DI.heuristicSearchType == HeuristicSearchType.SEMI_ONDEMAND)
    // we could call expand on found target nodes after each search however
    // - the intention would not be as clear as calling it before [createSuccAstarNode]
    // - it could expande more nodes than we would actually need

    var astarNode = this
    var argNode = astarNode.argNode
    val astarArg = astarNode.astarArg
    if (!argNode.isTarget) {
        if (!argNode.isCovered || !argNode.coveringNode()!!.isTarget) {
            // provided AstarNode was in queue =>
            // provided AstarNode has heuristic =>
            // (if not decreasing) [astarNode] has distance &&
            // [astarNode] is not a target =>
            // [astarNode] must have been expanded or if covered then the coverer (if non target) must have been expanded
            check(argNode.isExpanded || argNode.coveringNode()!!.isExpanded)
            return
        }

        // target covering node
        argNode = argNode.coveringNode()!!
        astarNode = astarArg[argNode]
    }
    require(argNode.isTarget)

    if (DI.heuristicSearchType == HeuristicSearchType.FULL) {
        require(argNode.isCovered || argNode.isExpanded)
    }

    if (argNode.isCovered) {
        // [createChildren] is already called (directly or indirectly) on this node
        return
    }

    // [createChildren] can be already called on this node through a different edge
    while(!argNode.isExpanded) {
        check(astarNode.distance.isKnown)
        astarNode.close(astarArg.reachedSet[astarNode], search)?.let {}
        if (argNode.coveringNode() != null) {
            argNode = argNode.coveringNode()!!

            // TODO document: why no 0 distance set

            astarNode = astarArg[argNode]
            check(argNode.isTarget)
            check(!argNode.isCovered)
            continue
        }
        argBuilder.expand(argNode, prec).forEach {
            val succAstarNode = astarArg.createSuccAstarNode(it, argBuilder, prec)
            // optimization
            if (succAstarNode.argNode.isTarget) {
                // Heuristic has to be set (first) otherwise admissibility check fails
                succAstarNode.heuristic = Distance.ZERO
                succAstarNode.distance = Distance.ZERO
            }
        }
    }

    // TODO visualize here
}

val <S: State, A: Action> AstarNode<S, A>.reachesTarget
    // Distance is an exact heuristic => we also can reach a target with this weight
    get() = argNode.isTarget || distance.isFinite
