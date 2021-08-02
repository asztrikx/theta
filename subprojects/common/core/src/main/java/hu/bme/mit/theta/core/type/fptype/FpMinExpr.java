package hu.bme.mit.theta.core.type.fptype;

import hu.bme.mit.theta.core.model.Valuation;
import hu.bme.mit.theta.core.type.BinaryExpr;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.LitExpr;
import hu.bme.mit.theta.core.utils.FpUtils;

import static com.google.common.base.Preconditions.checkNotNull;
import static hu.bme.mit.theta.core.utils.TypeUtils.castFp;
import static hu.bme.mit.theta.core.utils.TypeUtils.checkAllTypesEqual;

public class FpMinExpr extends BinaryExpr<FpType, FpType> {
	private static final int HASH_SEED = 6667;
	private static final String OPERATOR_LABEL = "fpmin";

	private final FpRoundingMode roundingMode;

	private FpMinExpr(final FpRoundingMode roundingMode, final Expr<FpType> leftOp, final Expr<FpType> rightOp) {
		super(leftOp, rightOp);
		checkAllTypesEqual(leftOp, rightOp);
		checkNotNull(roundingMode);
		this.roundingMode = roundingMode;
	}

	public static FpMinExpr of(final FpRoundingMode roundingMode, final Expr<FpType> leftOp, final Expr<FpType> rightOp) {
		return new FpMinExpr(roundingMode, leftOp, rightOp);
	}

	public static FpMinExpr create(final FpRoundingMode roundingMode, final Expr<FpType> leftOp, final Expr<FpType> rightOp) {
		checkNotNull(leftOp, rightOp);
		final Expr<FpType> newLeftOp = castFp(leftOp);
		final Expr<FpType> newRightOp = castFp(rightOp);
		return FpMinExpr.of(roundingMode, newLeftOp, newRightOp);
	}

	@Override
	public BinaryExpr<FpType, FpType> with(Expr<FpType> leftOp, Expr<FpType> rightOp) {
		if (leftOp == getLeftOp() && rightOp == getRightOp()) {
			return this;
		} else {
			return FpMinExpr.of(roundingMode, leftOp, rightOp);
		}
	}

	@Override
	public BinaryExpr<FpType, FpType> withLeftOp(Expr<FpType> leftOp) {
		return with(leftOp, getRightOp());
	}

	@Override
	public BinaryExpr<FpType, FpType> withRightOp(Expr<FpType> rightOp) {
		return with(getLeftOp(), rightOp);
	}

	@Override
	protected int getHashSeed() {
		return HASH_SEED;
	}

	@Override
	public String getOperatorLabel() {
		return OPERATOR_LABEL;
	}

	@Override
	public FpType getType() {
		return getLeftOp().getType();
	}

	@Override
	public LitExpr<FpType> eval(Valuation val) {
		final FpLitExpr leftOpVal = (FpLitExpr) getLeftOp().eval(val);
		final FpLitExpr rightOpVal = (FpLitExpr) getRightOp().eval(val);
		if (FpUtils.fpLitExprToBigFloat(roundingMode, leftOpVal).lessThan(FpUtils.fpLitExprToBigFloat(roundingMode, rightOpVal))) {
			return leftOpVal;
		} else {
			return rightOpVal;
		}
	}
}
