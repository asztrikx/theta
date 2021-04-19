package hu.bme.mit.theta.xcfa.utils;

import hu.bme.mit.theta.core.stmt.SkipStmt;
import hu.bme.mit.theta.core.stmt.Stmt;
import hu.bme.mit.theta.xcfa.model.XCFA;
import hu.bme.mit.theta.xcfa.model.XcfaProcess;
import hu.bme.mit.theta.xcfa.model.XcfaProcedure;
import hu.bme.mit.theta.xcfa.model.XcfaEdge;
import hu.bme.mit.theta.xcfa.model.XcfaLocation;
import hu.bme.mit.theta.xcfa.dsl.CallStmt;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Splits multiple-stmt edges
 * @author laszlo.radnai
 *
 */
public final class XcfaEdgeSplitterTransformation {

	private final List<CallStmtWithOldProcedure> postBuildData = new ArrayList<>();
	private XcfaEdgeSplitterTransformation() {}
	
	public static XCFA transform(XCFA original) {
		return new XcfaEdgeSplitterTransformation().alt_transform(original);
	}
	
	// reuses variables and stmts
	private XCFA alt_transform(XCFA original) {
		XCFA.Builder builder = XCFA.builder();
		Map<XcfaProcedure, XcfaProcedure> origToNewProcedure = new HashMap<>();
		for (XcfaProcess origPs: original.getProcesses()) {
			XcfaProcess.Builder builderPs = XcfaProcess.builder();
			builderPs.setName(origPs.getName());
			for (XcfaProcedure origPc : origPs.getProcedures()) {
				XcfaProcedure.Builder builderPc = XcfaProcedure.builder();
				builderPc.setName(origPc.getName());
				var locMap = addLocations(builderPc, origPc);
				for (var origEdge : origPc.getEdges()) {
					addEdge(locMap, builderPc, origEdge);
				}
				for (var origParam : origPc.getParams()) {
					builderPc.createParam(origParam);
				}
				for (var origVar : origPc.getLocalVars()) {
					builderPc.createVar(origVar, origPc.getInitValue(origVar).isPresent() ? origPc.getInitValue(origVar).get() : null);
				}
				builderPc.setRtype(origPc.getRetType());
				builderPc.setResult(origPc.getResult());
				var pc = builderPc.build();
				builderPs.addProcedure(pc);
				origToNewProcedure.put(origPc, pc);
				if (origPs.getMainProcedure() == origPc) {
					builderPs.setMainProcedure(pc);
				}
			}
			for (var origVar : origPs.getThreadLocalVars()) {
				builderPs.createVar(origVar, origPs.getInitValue(origVar).orElse(null));
			}
			var ps = builderPs.build();
			builder.addProcess(ps);
			if (original.getMainProcess() == origPs) {
				builder.setMainProcess(ps);
			}
		}
		for (var origVar : original.getGlobalVars()) {
			builder.addGlobalVar(origVar, original.getInitValue(origVar).orElse(null));
		}
		for (var q : postBuildData) {
			var pc = origToNewProcedure.get(q.oldPc);
			q.stmt.setProcedure(pc);
		}
		postBuildData.clear();
		return builder.build();
	}

	// TODO use JRE11 feature instead
	private static List<Stmt> listOf(Stmt stmt) {
		var x = new ArrayList<Stmt>();
		x.add(stmt);
		return x;
	}
	
	private Map<XcfaLocation, XcfaLocation> addLocations(XcfaProcedure.Builder builder, XcfaProcedure orig) {
		Map<XcfaLocation, XcfaLocation> locMap = new HashMap<>();
		for (var origLoc : orig.getLocs()) {
			XcfaLocation loc = new XcfaLocation(origLoc.getName(), origLoc.getDictionary());
			locMap.put(origLoc, loc);
			builder.addLoc(loc);
			
			if (orig.getInitLoc() == origLoc) {
				builder.setInitLoc(loc);
			} else if (orig.getErrorLoc() == origLoc) {
				builder.setErrorLoc(loc);
			} else if (orig.getFinalLoc() == origLoc) {
				builder.setFinalLoc(loc);
			}
		}
		return locMap;
	}
	
	private static class CallStmtWithOldProcedure {
		private final CallStmt stmt;
		private final XcfaProcedure oldPc;
		public CallStmtWithOldProcedure(CallStmt stmt, XcfaProcedure oldPc) {
			this.stmt = stmt;
			this.oldPc = oldPc;
		}
	}

	
	/**
	 * Ensures no references of the old XCFA remains in a Stmt.
	 * Might reuse stmt.
	 * 
	 * Variables, stmts are not a problem.
	 * CallStmts must be copied, however.
	 * @return
	 */
	private Stmt copyStmt(Stmt stmt) {
		if (stmt instanceof CallStmt) {
			// We cannot yet fill the *new* procedure, because it might not have been built yet.
			var cc = new CallStmt(((CallStmt) stmt).getResultVar(), null, ((CallStmt) stmt).getParams());
			postBuildData.add(new CallStmtWithOldProcedure(cc, ((CallStmt) stmt).getProcedure()));
			return cc;
		} else {
			return stmt;
		}
	}

	private void addEdge(Map<XcfaLocation, XcfaLocation> locMap, XcfaProcedure.Builder builder, XcfaEdge origEdge) {
		var source = locMap.get(origEdge.getSource());
		var target = locMap.get(origEdge.getTarget());
		var stmts = origEdge.getStmts();
		if (stmts.size() == 0) {
			builder.addEdge(new XcfaEdge(source, target, listOf(SkipStmt.getInstance())));
//		} else if (stmts.size() == 1) {
//			builder.addEdge(new Edge(source, target, stmts));
		} else {
			XcfaLocation lastLoc = source;
			// all but last edge
			for (var i = 0; i < stmts.size()-1; i++) {
				var stmt = stmts.get(i);
				XcfaLocation loc = new XcfaLocation("_" + source.getName() + "_" + target.getName() + "_" + i, source.getDictionary());
				builder.addLoc(loc);
				builder.addEdge(new XcfaEdge(lastLoc, loc, listOf(copyStmt(stmt))));
				lastLoc = loc;
			}
			// last edge
			builder.addEdge(new XcfaEdge(lastLoc, target, listOf(copyStmt(stmts.get(stmts.size()-1)))));
		}
	}
}