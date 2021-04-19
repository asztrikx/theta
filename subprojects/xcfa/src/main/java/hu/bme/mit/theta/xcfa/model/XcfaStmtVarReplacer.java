package hu.bme.mit.theta.xcfa.model;

import hu.bme.mit.theta.core.decl.VarDecl;
import hu.bme.mit.theta.core.stmt.*;
import hu.bme.mit.theta.core.stmt.xcfa.*;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.LitExpr;
import hu.bme.mit.theta.core.type.Type;
import hu.bme.mit.theta.core.type.anytype.RefExpr;
import hu.bme.mit.theta.core.type.booltype.BoolType;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static hu.bme.mit.theta.core.stmt.Stmts.*;
import static hu.bme.mit.theta.core.utils.TypeUtils.cast;

public class XcfaStmtVarReplacer implements XcfaStmtVisitor<Map<VarDecl<?>, VarDecl<?>>, Stmt> {

    public static <T extends Type> Expr<T> replaceVars(Expr<T> expr, Map<VarDecl<?>, VarDecl<?>> varLut) {
        List<? extends Expr<?>> ops = expr.getOps();
        List<Expr<?>> newOps = new ArrayList<>();
        for (Expr<?> op : ops) {
            if(op instanceof LitExpr<?>) newOps.add(op);
            else if(op instanceof RefExpr<?>) {
                if(((RefExpr<?>) op).getDecl() instanceof VarDecl<?>) {
                    newOps.add(varLut.getOrDefault((VarDecl<?>) ((RefExpr<?>) op).getDecl(), (VarDecl<?>) ((RefExpr<?>) op).getDecl()).getRef());
                }
            }
            else newOps.add(replaceVars(op, varLut));
        }
        return expr.withOps(newOps);
    }


    @Override
    public Stmt visit(SkipStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return stmt;
    }

    @Override
    public Stmt visit(AssumeStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return Assume(cast(replaceVars(stmt.getCond(), param), BoolType.getInstance()));
    }

    @Override
    public <DeclType extends Type> Stmt visit(AssignStmt<DeclType> stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return Assign(cast(param.getOrDefault(stmt.getVarDecl(), stmt.getVarDecl()), stmt.getVarDecl().getType()), replaceVars(stmt.getExpr(), param));
    }

    @Override
    public <DeclType extends Type> Stmt visit(HavocStmt<DeclType> stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return Havoc(param.getOrDefault(stmt.getVarDecl(), stmt.getVarDecl()));
    }

    @Override
    public Stmt visit(XcfaStmt xcfaStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return xcfaStmt.accept(this, param);
    }

    @Override
    public Stmt visit(SequenceStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        List<Stmt> stmts = stmt.getStmts();
        List<Stmt> newStmts = new ArrayList<>();
        for (Stmt stmt1 : stmts) {
            newStmts.add(stmt1.accept(this, param));
        }
        return SequenceStmt(newStmts);
    }

    @Override
    public Stmt visit(NonDetStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        List<Stmt> stmts = stmt.getStmts();
        List<Stmt> newStmts = new ArrayList<>();
        for (Stmt stmt1 : stmts) {
            newStmts.add(stmt1.accept(this, param));
        }
        return NonDetStmt(newStmts);
    }

    @Override
    public Stmt visit(OrtStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        List<Stmt> stmts = stmt.getStmts();
        List<Stmt> newStmts = new ArrayList<>();
        for (Stmt stmt1 : stmts) {
            newStmts.add(stmt1.accept(this, param));
        }
        return OrtStmt.of(newStmts);
    }

    @Override
    public Stmt visit(XcfaCallStmt stmt, Map<VarDecl<?>, VarDecl<?>> param) {
        List<Expr<?>> exprs = stmt.getParams();
        List<Expr<?>> newExprs = new ArrayList<>();
        for (Expr<?> expr : exprs) {
            newExprs.add(replaceVars(expr, param));
        }
        return stmt.of(param.getOrDefault(stmt.getResultVar(), stmt.getResultVar()), newExprs);
    }

    @Override
    public Stmt visit(StoreStmt storeStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new StoreStmt(
                param.getOrDefault(storeStmt.getLhs(), storeStmt.getLhs()),
                param.getOrDefault(storeStmt.getRhs(), storeStmt.getRhs()),
                storeStmt.isAtomic(),
                storeStmt.getOrdering()
                );
    }

    @Override
    public Stmt visit(LoadStmt loadStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new LoadStmt(
                param.getOrDefault(loadStmt.getLhs(), loadStmt.getLhs()),
                param.getOrDefault(loadStmt.getRhs(), loadStmt.getRhs()),
                loadStmt.isAtomic(),
                loadStmt.getOrdering()
        );
    }

    @Override
    public Stmt visit(FenceStmt fenceStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return fenceStmt;
    }

    @Override
    public Stmt visit(AtomicBeginStmt atomicBeginStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return atomicBeginStmt;
    }

    @Override
    public Stmt visit(AtomicEndStmt atomicEndStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return atomicEndStmt;
    }

    @Override
    public Stmt visit(NotifyAllStmt notifyAllStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new NotifyAllStmt(param.getOrDefault(notifyAllStmt.getSyncVar(), notifyAllStmt.getSyncVar()));
    }

    @Override
    public Stmt visit(NotifyStmt notifyStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new NotifyStmt(param.getOrDefault(notifyStmt.getSyncVar(), notifyStmt.getSyncVar()));
    }

    @Override
    public Stmt visit(WaitStmt waitStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new WaitStmt(
                param.getOrDefault(waitStmt.getCndSyncVar(), waitStmt.getCndSyncVar()),
                waitStmt.getMtxSyncVar().isPresent() ? param.getOrDefault(waitStmt.getMtxSyncVar().get(), waitStmt.getMtxSyncVar().get()) : null
        );
    }

    @Override
    public Stmt visit(MtxLockStmt lockStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new MtxLockStmt(param.getOrDefault(lockStmt.getSyncVar(), lockStmt.getSyncVar()));
    }

    @Override
    public Stmt visit(MtxUnlockStmt unlockStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new MtxUnlockStmt(param.getOrDefault(unlockStmt.getSyncVar(), unlockStmt.getSyncVar()));
    }

    @Override
    public Stmt visit(ExitWaitStmt exitWaitStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new ExitWaitStmt(param.getOrDefault(exitWaitStmt.getSyncVar(), exitWaitStmt.getSyncVar()));
    }

    @Override
    public Stmt visit(EnterWaitStmt enterWaitStmt, Map<VarDecl<?>, VarDecl<?>> param) {
        return new EnterWaitStmt(param.getOrDefault(enterWaitStmt.getSyncVar(), enterWaitStmt.getSyncVar()));
    }

}