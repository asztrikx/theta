package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ArgNode;

import javax.annotation.Nullable;
import java.util.Map;
import java.util.Objects;

import static com.google.common.base.Preconditions.checkNotNull;

public final class AstarNode<S extends hu.bme.mit.theta.analysis.State, A extends Action> implements Comparable<AstarNode<S, A>> {
    public final ArgNode<S, A> argNode;
    public final AstarNode<S, A> coveringAstarNode;
    public Distance distance;

    public static final class Distance {
        private final DistanceType type;
        private final int value;

        public Distance(DistanceType type) {
            assert type != DistanceType.EXACT;
            this.type = type;
            this.value = 0;
        }

        public Distance(DistanceType type, int value) {
            this.type = type;
            this.value = value;
        }

        public DistanceType getType() {
            return type;
        }

        public int getValue() {
            if (type == DistanceType.EXACT) {
                return value;
            }
            throw new RuntimeException("Only EXACT Distances have meaningful values.");
        }

        public boolean isKnown() {
            return type == DistanceType.EXACT || type == DistanceType.INFINITE;
        }

        @Override
        public String toString() {
            String result = type.name();
            if (type == DistanceType.EXACT) {
                result += "," + value;
            }
            return result;
        }
    }

    // Java11 does not support inner classes
    public enum DistanceType {
        INFINITE,
        EXACT,
        UNKNOWN,
    }

    // coveringAstarNode: can be null if it is the first arg
    private AstarNode(final ArgNode<S, A> argNode, @Nullable final AstarNode<S, A> coveringAstarNode) {
        this.argNode = checkNotNull(argNode);
        this.coveringAstarNode = coveringAstarNode;
        this.distance = new Distance(DistanceType.UNKNOWN);
    }

    public Distance getHeuristic() {
        if (coveringAstarNode == null) {
            return new Distance(DistanceType.EXACT, 0);
        }
        return coveringAstarNode.distance;
    }

    public Distance getDepth(Map<AstarNode<S, A>, Integer> depths) {
        return new Distance(DistanceType.EXACT, depths.get(this));
    }

    public Distance getWeight(Map<AstarNode<S, A>, Integer> depths) {
        assert getHeuristic().isKnown();
        // TODO maybe we can call findHeuristic here
        Distance f = getHeuristic();
        Distance g = getDepth(depths);
        if (f.type == DistanceType.INFINITE) {
            return f;
        }
        return new Distance(DistanceType.EXACT, f.value + g.value);
    }

    public static <S extends State, A extends Action> AstarNode<S, A> create(
            final ArgNode<S, A> argNode,
            final AstarNode<S, A> ancestor) {
        return new AstarNode<>(argNode, ancestor);
    }

    @Override
    public int compareTo(AstarNode<S, A> astarNode2) {
        final Distance weight1 = getWeight();
        final Distance weight2 = astarNode2.getWeight();

        // in astarExtend we do not have INFINITE nodes so this part is only for generality
        final boolean unreachable1 = weight1.getType() == DistanceType.INFINITE;
        final boolean unreachable2 = weight2.getType() == DistanceType.INFINITE;

        if (unreachable1 && unreachable2) {
            return 0;
        } else if (unreachable1) {
            return 1;
        } else if (unreachable2) {
            return -1;
        }
        return weight1.getValue() - weight2.getValue();
    }

    @Override
    public String toString() {
        return String.format("(%s), %s",
                getHeuristic().toString(),
                AstarVisualizer.getVisualizerState(argNode)
        );
    }
}
