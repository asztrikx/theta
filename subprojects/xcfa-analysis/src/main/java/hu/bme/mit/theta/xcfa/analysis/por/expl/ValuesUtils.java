/*
 * Copyright 2019 Budapest University of Technology and Economics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package hu.bme.mit.theta.xcfa.analysis.por.expl;

import com.google.common.base.Preconditions;
import hu.bme.mit.theta.core.decl.IndexedConstDecl;
import hu.bme.mit.theta.core.decl.VarDecl;
import hu.bme.mit.theta.core.model.ImmutableValuation;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.LitExpr;
import hu.bme.mit.theta.core.type.Type;
import hu.bme.mit.theta.core.type.inttype.IntExprs;
import hu.bme.mit.theta.core.type.inttype.IntType;
import hu.bme.mit.theta.core.type.xcfa.SyntheticType;
import hu.bme.mit.theta.core.utils.ExprUtils;
import hu.bme.mit.theta.xcfa.XCFA;
import hu.bme.mit.theta.xcfa.type.SyntheticLitExpr;

import java.util.Optional;

/**
 * Contains functions related to valuations.
 * More specifically, evaluating expressions and updating values.
 */
final class ValuesUtils {

    private ValuesUtils() { }

    public static <DeclType extends Type> Optional<LitExpr<DeclType>> eval(ExplState state, Expr<DeclType> expr) {
        Expr<DeclType> simplified = ExprUtils.simplify(state.getIndexing().unfold(expr), state.getValuation());
        if (simplified instanceof LitExpr<?>) {
            return Optional.of((LitExpr<DeclType>)simplified);
        }
        return Optional.empty();
    }
    public static <DeclType extends Type> void putValue(MutableExplState state, VarDecl<DeclType> var, Optional<LitExpr<DeclType>> expr) {
        if (expr.isPresent())
            state.getValuation().put(var.getConstDecl(state.getIndexing().get(var)), expr.get());
        else
            state.getValuation().remove(var.getConstDecl(state.getIndexing().get(var)));
    }

    public static void modifyIndexing(MutableExplState state, XCFA.Process process, XCFA.Process.Procedure procedure, int modifier) {
        Preconditions.checkArgument(modifier == 1 || modifier == -1);
        if (modifier == 1) {
            state.getIndexing().pushProcedure(process, procedure);
        } else {
            state.getIndexing().popProcedure(process, procedure);
        }
    }

    public static void modifyIndexing(MutableExplState state, VarDecl<?> var, int modifier) {
        Preconditions.checkArgument(modifier == 1 || modifier == -1);
        state.getIndexing().inc(var, modifier);
    }

    /**
     * All global integers initialized to 0, all global synthetic vars initialized.
     * @return valuation with integers and synthetic vars initialized
     */
    protected static ImmutableValuation getInitialValuation(XCFA xcfa) {
        var builder = ImmutableValuation.builder();
        for (VarDecl<? extends Type> var: xcfa.getGlobalVars()) {
            IndexedConstDecl<? extends Type> x = var.getConstDecl(0);
            if (x.getType() == IntType.getInstance()) {
                builder.put(x, IntExprs.Int(0));
            } else if (x.getType() == SyntheticType.getInstance()){
                builder.put(x, SyntheticLitExpr.unlocked());
            }
        }
        return builder.build();
    }

    protected static VarDoubleIndexing getInitialIndexing(XCFA xcfa, ProcessStates states) {
        return new VarDoubleIndexing(xcfa, states);
    }
}