package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import java.util.Comparator;

public final class Distance implements Comparable<Distance> {
	private final Type type;
	private final int value;

	public enum Type {
		INFINITE,
		EXACT,
		UNKNOWN,
	}

	public Distance(Type type) {
		assert type != Type.EXACT;
		this.type = type;
		this.value = 0;
	}

	public Distance(Type type, int value) {
		this.type = type;
		this.value = value;
	}

	public Type getType() {
		return type;
	}

	public int getValue() {
		if (type == Type.EXACT) {
			return value;
		}
		throw new RuntimeException("Only EXACT Distances have meaningful values.");
	}

	public boolean isKnown() {
		return type == Type.EXACT || type == Type.INFINITE;
	}

	// Both distances should be known
	@Override
	public int compareTo(Distance distance) {
		assert isKnown();
		assert distance.isKnown();
		if (getType() == Type.INFINITE) {
			return -1;
		}
		if (distance.getType() == Type.INFINITE) {
			return 1;
		}
		int value1 = getValue();
		int value2 = distance.getValue();
		return value1 - value2;
	}

	@Override
	public String toString() {
		String result = type.name();
		if (type == Type.EXACT) {
			result += "," + value;
		}
		return result;
	}
}