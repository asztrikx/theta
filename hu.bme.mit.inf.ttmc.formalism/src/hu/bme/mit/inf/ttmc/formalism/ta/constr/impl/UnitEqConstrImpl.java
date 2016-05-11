package hu.bme.mit.inf.ttmc.formalism.ta.constr.impl;

import static hu.bme.mit.inf.ttmc.core.expr.impl.Exprs.Eq;
import static hu.bme.mit.inf.ttmc.core.expr.impl.Exprs.Int;

import hu.bme.mit.inf.ttmc.core.expr.EqExpr;
import hu.bme.mit.inf.ttmc.formalism.common.decl.ClockDecl;
import hu.bme.mit.inf.ttmc.formalism.common.expr.ClockRefExpr;
import hu.bme.mit.inf.ttmc.formalism.ta.constr.UnitEqConstr;
import hu.bme.mit.inf.ttmc.formalism.ta.utils.ConstrVisitor;

final class UnitEqConstrImpl extends AbstractUnitConstr implements UnitEqConstr {

	private static final int HASH_SEED = 5987;

	private static final String OPERATOR_LABEL = "=";

	private volatile EqExpr expr = null;

	UnitEqConstrImpl(final ClockDecl clock, final int bound) {
		super(clock, bound);
	}

	@Override
	public EqExpr asExpr() {
		EqExpr result = expr;
		if (result == null) {
			final ClockRefExpr ref = getClock().getRef();
			result = Eq(ref, Int(getBound()));
			expr = result;
		}
		return result;
	}

	@Override
	public <P, R> R accept(final ConstrVisitor<? super P, ? extends R> visitor, final P param) {
		return visitor.visit(this, param);
	}

	@Override
	public boolean equals(final Object obj) {
		if (this == obj) {
			return true;
		} else if (obj instanceof UnitEqConstr) {
			final UnitEqConstr that = (UnitEqConstr) obj;
			return this.getBound() == that.getBound() && this.getClock().equals(that.getClock());
		} else {
			return false;
		}
	}

	@Override
	protected int getHashSeed() {
		return HASH_SEED;
	}

	@Override
	protected String getOperatorLabel() {
		return OPERATOR_LABEL;
	}

}
