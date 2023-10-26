package hu.bme.mit.theta.analysis.algorithm.cegar.astar

import hu.bme.mit.theta.analysis.Action
import hu.bme.mit.theta.analysis.Prec
import hu.bme.mit.theta.analysis.State
import hu.bme.mit.theta.analysis.algorithm.ArgBuilder
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.HeuristicSearchType
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.cegarhistorystorage.CegarHistoryStorage
import hu.bme.mit.theta.analysis.algorithm.cegar.astar.strategy.heuristicFinder.HeuristicFinder

fun <S: State, A: Action> AstarNode<S, A>.checkConsistency(child: AstarNode<S, A>) {
    val parent = this
    require(parent.heuristic.isKnown && child.heuristic.isKnown)
    // Never true "inf - c <= 0..1" // Can't do "inf - inf"
    check(!parent.heuristic.isInfinite)
    // Always holds "c - inf <= 0..1"
    if (child.heuristic.isInfinite) {
        return
    }

    val heuristicDistanceValue = parent.heuristic - child.heuristic
    val edgeWeight = if (parent.argNode.isCovered) 0 else 1
    check(heuristicDistanceValue.value <= edgeWeight)
}

fun <S: State, A: Action, P: Prec> AstarNode<S, A>.close(
    search: AstarSearch<S, A, P>?,
    heuristicFinder: HeuristicFinder<S, A, P>,
    abstractor: AstarAbstractor<S, A, P>,
): AstarNode<S, A>? {
    if ((argNode.isExpanded || !argNode.isLeaf) || argNode.isCovered) {
        // isLeaf: After prune node may have children but not fully expanded.
        // isExpanded: If node has no children it still can already be expanded, therefore expanded is already set (should not be covered).
        // isCovered: If node already has covering node, close cloud still choose another one, therefore avoid.
        return null
    }
    require(heuristic.isKnown)

    val candidates = if (argNode.isTarget) {
        // optimization (leq uses smt solver): target node can only be covered with a target node
        astarArg.reachedSet[this].filter { it.argNode.isTarget }
    } else {
        astarArg.reachedSet[this]
    }

    val weightStopAfter = if (DI.heuristicSearchType == HeuristicSearchType.FULLY_ONDEMAND) {
        heuristic.value - 1L
    } else {
        Long.MAX_VALUE
    }

    for (astarCandidate in candidates) {
        if (astarCandidate.heuristic.isUnknown && DI.disableOptimizations) {
            continue
        }

        // TODO pattern
        if (DI.heuristicSearchType !== HeuristicSearchType.SEMI_ONDEMAND) {
            // optimization: If heuristic is *quickly computable* then check consistency before calling mayCover which uses Leq
            if (astarCandidate.heuristic.isUnknown) {
                heuristicFinder(astarCandidate, abstractor, search, weightStopAfter)
            }
            if (!(astarCandidate.heuristic >= heuristic)) {
                continue
            }
        }

        val candidate = astarCandidate.argNode
        if (!candidate.mayCover(argNode)) {
            continue
        }

        if (astarCandidate.heuristic.isUnknown) {
            // TODO document: leftovers dont have heuristic, but we would want to cover into it, but it can break consistency
            heuristicFinder(astarCandidate, abstractor, search, weightStopAfter)
        }
        if (!(astarCandidate.heuristic >= heuristic)) {
            continue
        }

        // This should hold because of previous check and because of infinite filtering in [AstarSearch]
        checkConsistency(astarCandidate)
        argNode.cover(candidate)

        search ?: return null
        return handleCloseRewire(search)
    }
    return null
}

/**
 * @return The actual AstarNode which is covered after [AstarNode.close].
 * The AstarNode reference should be replaced with the return value so that [AstarSearch] will set that as parent.
 */
fun <S: State, A: Action, P: Prec> AstarNode<S, A>.handleCloseRewire(search: AstarSearch<S, A, P>): AstarNode<S, A> {
    // If astarNode's parent is also covered then covering edges have been redirected. (see ArgNode::cover)
    // We have to update parents map according to that.
    //  1) a - - -> b (argNode)
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

        // Both heuristics known (see caller)
        // Parent can't be infinite (see caller)
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
fun <S: State, A: Action, P: Prec> AstarNode<S, A>.createChildren(
    search: AstarSearch<S, A, P>?,
    argBuilder: ArgBuilder<S, A, P>,
    heuristicFinder: HeuristicFinder<S, A, P>,
    abstractor: AstarAbstractor<S, A, P>,
    cegarHistoryStorage: CegarHistoryStorage<S, A, P>,
) {
    require(DI.heuristicSearchType == HeuristicSearchType.SEMI_ONDEMAND || DI.heuristicSearchType == HeuristicSearchType.FULLY_ONDEMAND || DI.disableOptimizations)
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
    val (_, prec) = cegarHistoryStorage.find(astarArg)
    while(!argNode.isExpanded) {
        astarNode.close(search, heuristicFinder, abstractor)?.let {}
        if (argNode.coveringNode() != null) {
            argNode = argNode.coveringNode()!!
            astarNode = astarArg[argNode]
            check(argNode.isTarget)
            check(!argNode.isCovered)

            // TODO document: why no 0 distance set
            if (DI.disableOptimizations) {
                astarNode.distance = Distance.F0
            }
            continue
        }
        argBuilder.expand(argNode, prec).forEach {
            val newAstarNode = astarArg.createSuccAstarNode(it, argBuilder, cegarHistoryStorage, heuristicFinder, abstractor)
            // optimization
            if (newAstarNode.argNode.isTarget && DI.enableOptimizations) {
                // Heuristic has to be set (first) otherwise admissibility check fails
                newAstarNode.heuristic = Distance.F0
                newAstarNode.distance = Distance.F0
            }
        }
    }

    // TODO visualize here
}

val <S: State, A: Action> AstarNode<S, A>.reachesTarget
    get() = argNode.isTarget || distance.isFinite

fun <S: State, A: Action> AstarNode<S, A>.printSubgraph(
    indent: Int = 0,
    visited: MutableList<AstarNode<S, A>> = mutableListOf(),
    isCovering: Boolean = false
) {
    DI.logger.mainstep(" ".repeat(indent))
    if (isCovering) {
        DI.logger.mainstep("-->")
    }
    DI.logger.mainstep(this.toString())

    if (this in visited) {
        DI.logger.mainstepLine(" LOOP")
        return
    }
    visited += this

    DI.logger.mainstepLine("")

    argNode.succNodes().map{ astarArg[it] }.forEach {
        it.printSubgraph(indent + 1, visited)
    }
    if (argNode.isCovered()) {
        astarArg[argNode.coveringNode()!!].printSubgraph(indent + 1, visited, true)
    }
}
