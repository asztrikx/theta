package hu.bme.mit.theta.analysis.algorithm.cegar.astar;

import hu.bme.mit.theta.analysis.Action;
import hu.bme.mit.theta.analysis.State;
import hu.bme.mit.theta.analysis.algorithm.ARG;
import hu.bme.mit.theta.analysis.utils.ArgVisualizer;
import hu.bme.mit.theta.common.logging.Logger;

public class ArgFileVisualizer<S extends State, A extends Action> extends FileVisualizer {
	private ARG<S, A> arg;

	public ArgFileVisualizer(Logger logger, ARG<S, A> arg) {
		super(logger);
		this.arg = arg;
	}

	@Override
	public void visualize(String state, int index) {
		String titleText = String.format("%d %s", index, state);
		visualizeBase(state, titleText, () -> ArgVisualizer.getDefault().visualize(arg));
	}
}
