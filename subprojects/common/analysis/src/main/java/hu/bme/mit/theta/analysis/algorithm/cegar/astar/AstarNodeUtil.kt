package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.State

fun <S: State, A: Action> AstarNode<S, A>.checkConsistency(child: AstarNode<S, A>) {
    val parent = this
    require(!parent.heuristic.isInfinite)
    if (child.heuristic.isInfinite) {
        check(parent.heuristic.isKnown) // why? only exact
        return
    }

    val heuristicDistanceValue = parent.heuristic.value - child.heuristic.value
    val edgeWeight = if (parent.argNode.isCovered) 0 else 1
    check(heuristicDistanceValue <= edgeWeight)
}

fun <S: State, A: Action> AstarNode<S, A>.close(
    candidates: Collection<AstarNode<S, A>>,
    search: AstarSearch<S, A>
): AstarNode<S, A>? {
    // isLeaf: After prune node may have children but not fully expanded.
    // isExpanded: If node has no children it still can already be expanded, therefore expanded is already set (should not be covered).
    // isCovered: If node already has covering node, close cloud still choose another one, therefore avoid.
    if ((argNode.isExpanded || !argNode.isLeaf) || argNode.isCovered) {
        return null
    }

    if (heuristic == Distance.ZERO) {
        // TODO https://photos.app.goo.gl/wguQ7K9opyLqTUPa7
        return null // TODO null or this?
    }

    for (astarCandidate in candidates) {
        val candidate = astarCandidate.argNode
        if (!candidate.mayCover(argNode)) {
            continue
        }
        check(!(argNode.isTarget && !candidate.isTarget))

        check(astarCandidate.heuristic == heuristic) // TODO this is created to be hit, test this, for provider node children exists should have failed earlier

        if (heuristic <= astarCandidate.heuristic) {
            argNode.cover(candidate)
            return handleCloseRewire(search)
        }
    }
    return null
}

/**
 * TODO text
 *
 * @return The actual parent node for covering node after [AstarNode.close].
 * The AstarNode reference should be replaced with the return value.
 */
fun <S: State, A: Action> AstarNode<S, A>.handleCloseRewire(search: AstarSearch<S, A>): AstarNode<S, A> {
    // TODO renaming according to this function context

    // If astarNode's parent is also covered then covering edges have been redirected. (see ArgNode::cover)
    // We have to update parents map according to that.
    //  1) a - - -> b (argNode,coveredAstarNode)
    //  2) a - - -> b - - -> c (coveringNode)
    //  3) a        b - - -> c
    //	   |                 ^
    //     | - - - - - - - - |
    // Heuristic consistency doesn't break as c has >= heuristic than b
    val parentArgNode = search.parents[argNode]
    val parentAstarNode = parentArgNode?.let { astarArg[it] }
    val coveredAstarNode = if (parentArgNode != null && parentArgNode.isCovered && parentArgNode.coveringNode()!! === argNode.coveringNode()!!) {
        // Because argNode is covered it can only reach coveringNode with the same distance as it's new parent
        // therefore we can safely remove it
        search.parents.remove(argNode)

        // Update to new parent if we this node is the current parent
        // as coveringAstarNode may already have a better parent or already in doneSet
        parentAstarNode!!
    } else {
        this
    }
    return coveredAstarNode
}
