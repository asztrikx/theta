package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

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
		assert type == Type.EXACT;
		return value;
	}

	public boolean isKnown() {
		return type == Type.EXACT || type == Type.INFINITE;
	}

	// Both distances should be known
	@Override
	public int compareTo(Distance distance) {
		assert isKnown();
		assert distance.isKnown();
		if (getType() == Type.INFINITE && distance.getType() == Type.INFINITE) {
			return 0;
		}
		if (getType() == Type.INFINITE) {
			return 1;
		}
		if (distance.getType() == Type.INFINITE) {
			return -1;
		}
		int value1 = getValue();
		int value2 = distance.getValue();
		return value1 - value2;
	}

	@Override
	public boolean equals(Object object) {
		if (object.getClass() != Distance.class) return false;
		return compareTo((Distance) object) == 0;
	}

	@Override
	public String toString() {
		String result = "(";
		if (type == Type.EXACT) {
			result += "E," + value;
		} else if (type == Type.UNKNOWN) {
			result += "U";
		} else if (type == Type.INFINITE) {
			result += "I";
		}
		return result + ")";
	}
}
