package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;

import java.util.Comparator;
import java.util.Map;

public class AstarWaitlistComparator<S extends State, A extends Action> implements Comparator<AstarNode<S, A>> {
    private final Map<AstarNode<S, A>, Integer> depths;

    public AstarWaitlistComparator(Map<AstarNode<S, A>, Integer> depths) {
        this.depths = depths;
    }

    /*
        // in astarExtend we do not have INFINITE nodes so this part is only for generality
        final boolean unreachable1 = weight1.getType() == AstarNode.DistanceType.INFINITE;
        final boolean unreachable2 = weight2.getType() == AstarNode.DistanceType.INFINITE;

        if (unreachable1 && unreachable2) {
            return 0;
        } else if (unreachable1) {
            return 1;
        } else if (unreachable2) {
            return -1;
        }
    */

    @Override
    public int compare(AstarNode<S, A> astarNode1, AstarNode<S, A> astarNode2) {
        final AstarNode.Distance weight1 = astarNode1.getWeight(depths);
        final AstarNode.Distance weight2 = astarNode2.getWeight(depths);
        return weight1.getValue() - weight2.getValue();
    }
}
