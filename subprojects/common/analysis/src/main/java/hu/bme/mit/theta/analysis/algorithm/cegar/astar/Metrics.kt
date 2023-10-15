package hu.bme.mit.theta.analysis.algorithm.cegar.astar

class Metrics {
	var iteration = 0

	// start
	var leftoverNodes = 0
	var leftoverCoverings = 0

	// searching
	var expand = 0
	var cover = 0
	//var expandingEarlier = 0
	//var decreasingParent = 0
	//var knownDistanceReached = 0

	// end
	lateinit var distance: Distance
	var infiniteDistances = 0
	var finiteDistances = 0
	var targets = 0
	var targetWithKnownDistance = 0
	// Conditionally set distances can be seen from the number of [finiteDistance] difference (!= [finiteDistance] - [distance].value)

	override fun toString(): String {
		return "i$iteration lN$leftoverNodes lC$leftoverCoverings e$expand c$cover d$distance id$infiniteDistances fd$finiteDistances t$targets tk$targetWithKnownDistance"
	}
}