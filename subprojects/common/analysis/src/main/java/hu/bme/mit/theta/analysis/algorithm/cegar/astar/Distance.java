package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

public final class Distance {
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

	@Override
	public String toString() {
		String result = type.name();
		if (type == Type.EXACT) {
			result += "," + value;
		}
		return result;
	}
}