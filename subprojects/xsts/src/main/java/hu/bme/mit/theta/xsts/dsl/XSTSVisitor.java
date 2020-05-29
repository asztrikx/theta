package hu.bme.mit.theta.xsts.dsl;

import hu.bme.mit.theta.core.decl.Decls;
import hu.bme.mit.theta.core.decl.VarDecl;
import hu.bme.mit.theta.core.stmt.*;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.Type;
import hu.bme.mit.theta.core.type.booltype.BoolType;
import hu.bme.mit.theta.core.type.inttype.IntType;
import hu.bme.mit.theta.xsts.XSTS;
import hu.bme.mit.theta.xsts.dsl.gen.XstsDslBaseVisitor;
import hu.bme.mit.theta.xsts.dsl.gen.XstsDslParser;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.regex.Pattern;

import static hu.bme.mit.theta.core.type.abstracttype.AbstractExprs.*;
import static hu.bme.mit.theta.core.type.anytype.Exprs.Prime;
import static hu.bme.mit.theta.core.type.booltype.BoolExprs.*;
import static hu.bme.mit.theta.core.type.booltype.BoolExprs.Not;
import static hu.bme.mit.theta.core.type.inttype.IntExprs.Int;
import static hu.bme.mit.theta.core.type.inttype.IntExprs.Mod;

public class XSTSVisitor extends XstsDslBaseVisitor<Expr> {

    XSTS xsts;
    HashMap<String,Integer> literalToIntMap=new HashMap<String,Integer>();

    public HashMap<String, Integer> getLiteralToIntMap() {
        return literalToIntMap;
    }

    HashMap<String,VarDecl> nameToDeclMap=new HashMap<String, VarDecl>();

    public XSTS getXsts(){
        return xsts;
    }

    private HashMap<String,TypeDecl> nameToTypeMap=new HashMap<>();

    private HashSet<Expr<BoolType>> initExprs=new HashSet<Expr<BoolType>>();

    private Pattern tempVarPattern=Pattern.compile("temp([0-9])+");

    @Override
    public Expr visitXsts(XstsDslParser.XstsContext ctx) {

        for(XstsDslParser.TypeDeclarationContext typeDecl: ctx.typeDeclarations){
            visitTypeDeclaration(typeDecl);
        }
        for(TypeDecl decl:nameToTypeMap.values()){
            for(int i=0;i<decl.getLiterals().size();i++) if(!literalToIntMap.containsKey(decl.getLiterals().get(i)))literalToIntMap.put(decl.getLiterals().get(i),i);
        }
        for(XstsDslParser.VariableDeclarationContext varDecl: ctx.variableDeclarations){
            visitVariableDeclaration(varDecl);
        }
        xsts=new XSTS(nameToTypeMap.values(), processNonDet(ctx.initAction.nonDet()), processNonDet(ctx.transitions.nonDet()), processNonDet(ctx.envAction.nonDet()), And(initExprs), visitImplyExpression(ctx.prop));

        return null;
    }

    private void checkIfTempVar(String name){
        if(tempVarPattern.matcher(name).matches()) throw new RuntimeException(name+" is reserved!");
    }

    @Override
    public Expr visitTypeDeclaration(XstsDslParser.TypeDeclarationContext ctx) {
        checkIfTempVar(ctx.name.getText());
        if(nameToTypeMap.containsKey(ctx.name.getText()) || ctx.name.getText().equals("integer") || ctx.name.getText().equals("boolean")) throw new RuntimeException("Type "+ctx.name.getText()+" already exists!"+" On line "+ctx.start.getLine());
        List<String> literals=new ArrayList<String>();
        for(XstsDslParser.TypeLiteralContext literal:ctx.literals){
            checkIfTempVar(literal.name.getText());
            if(literals.contains(literal.name.getText())) throw new RuntimeException("Literal "+literal.name.getText()+" already exists!");
            literals.add(literal.name.getText());
        }
        TypeDecl decl=new TypeDecl(ctx.name.getText(),literals);
        nameToTypeMap.put(decl.getName(),decl);
        return null;
    }

    @Override
    public Expr visitVariableDeclaration(XstsDslParser.VariableDeclarationContext ctx) {
        Type type;
        if(ctx.type.BOOL()!=null) type= BoolType.getInstance();
        else if(ctx.type.INT()!=null) type= IntType.getInstance();
        else if(nameToTypeMap.containsKey(ctx.type.customType().name.getText())) type=IntType.getInstance();
        else throw new RuntimeException("Unknown type "+ctx.type.customType().name.getText()+" on line "+ctx.start.getLine());
        checkIfTempVar(ctx.name.getText());
        VarDecl decl=Decls.Var(ctx.name.getText(),type);
        if(nameToDeclMap.containsKey(ctx.name.getText())){
            throw new RuntimeException("Variable ["+ctx.name.getText()+"] already exists.");
        } else if(literalToIntMap.containsKey(ctx.name.getText())){
            throw new RuntimeException("["+ctx.name.getText()+"] is a type literal, cannot declare variable with this name.");
        } else {
            nameToDeclMap.put(decl.getName(), decl);
            if(ctx.initValue!=null){
                initExprs.add(Eq(decl.getRef(),visitValue(ctx.initValue)));
            }
        }
        return null;
    }

    @Override
    public Expr visitImplyExpression(XstsDslParser.ImplyExpressionContext ctx) {
        if(ctx.ops.size()>1){
            return Imply(visitOrExpr(ctx.ops.get(0)),visitOrExpr(ctx.ops.get(1)));
        }else return visitOrExpr(ctx.ops.get(0));
    }

    @Override
    public Expr visitOrExpr(XstsDslParser.OrExprContext ctx) {
        if(ctx.ops.size()==1) return visitAndExpr(ctx.ops.get(0));
        List<Expr<BoolType>> ops=new ArrayList<Expr<BoolType>>();
        for(XstsDslParser.AndExprContext child: ctx.ops){
            ops.add(visitAndExpr(child));
        }
        return Or(ops);
    }

    @Override
    public Expr<BoolType> visitAndExpr(XstsDslParser.AndExprContext ctx) {
        if(ctx.ops.size()==1) return visitNotExpr(ctx.ops.get(0));
        List<Expr<BoolType>> ops=new ArrayList<Expr<BoolType>>();
        for(XstsDslParser.NotExprContext child: ctx.ops){
            ops.add(visitNotExpr(child));
        }
        return And(ops);
    }

    @Override
    public Expr<BoolType> visitNotExpr(XstsDslParser.NotExprContext ctx) {
        if(ctx.ops.size()>0) return Not(visitNotExpr(ctx.ops.get(0)));
        else return visitEqExpr(ctx.eqExpr());
    }

    @Override
    public Expr visitEqExpr(XstsDslParser.EqExprContext ctx) {
        if(ctx.ops.size()>1){
            if(ctx.oper.EQ()!=null) return Eq(visitRelationExpr(ctx.ops.get(0)),visitRelationExpr(ctx.ops.get(1)));
            else return Neq(visitRelationExpr(ctx.ops.get(0)),visitRelationExpr(ctx.ops.get(1)));
        }else return visitRelationExpr(ctx.ops.get(0));
    }

    @Override
    public Expr visitEqOperator(XstsDslParser.EqOperatorContext ctx) {
        return super.visitEqOperator(ctx);
    }

    @Override
    public Expr visitRelationExpr(XstsDslParser.RelationExprContext ctx) {
        if(ctx.ops.size()>1){
            if(ctx.oper.LEQ()!=null){
                return Leq(visitAdditiveExpr(ctx.ops.get(0)),visitAdditiveExpr(ctx.ops.get(1)));
            }else if(ctx.oper.GEQ()!=null){
                return Geq(visitAdditiveExpr(ctx.ops.get(0)),visitAdditiveExpr(ctx.ops.get(1)));
            }else if(ctx.oper.LT()!=null){
                return Lt(visitAdditiveExpr(ctx.ops.get(0)),visitAdditiveExpr(ctx.ops.get(1)));
            }else if(ctx.oper.GT()!=null){
                return Gt(visitAdditiveExpr(ctx.ops.get(0)),visitAdditiveExpr(ctx.ops.get(1)));
            } else throw new UnsupportedOperationException("Unsupported operation "+ctx.oper.getText()+" on line "+ctx.start.getLine());
        }else return visitAdditiveExpr(ctx.ops.get(0));
    }

    @Override
    public Expr visitRelationOperator(XstsDslParser.RelationOperatorContext ctx) {
        return super.visitRelationOperator(ctx);
    }

    @Override
    public Expr visitAdditiveExpr(XstsDslParser.AdditiveExprContext ctx) {
        Expr res=visitMultiplicativeExpr(ctx.ops.get(0));
        for(int i=1;i<ctx.ops.size();i++){
            if(ctx.opers.get(i-1).PLUS()!=null){
                res=Add(res,visitMultiplicativeExpr(ctx.ops.get(i)));
            }else{
                res=Sub(res,visitMultiplicativeExpr(ctx.ops.get(i)));
            }
        }
        return res;

    }

    @Override
    public Expr visitAdditiveOperator(XstsDslParser.AdditiveOperatorContext ctx) {
        return super.visitAdditiveOperator(ctx);
    }

    @Override
    public Expr visitMultiplicativeExpr(XstsDslParser.MultiplicativeExprContext ctx) {
        Expr res=visitNegExpr(ctx.ops.get(0));
        for(int i=1;i<ctx.ops.size();i++){
            if(ctx.opers.get(i-1).DIV()!=null){
                res=Div(res,visitNegExpr(ctx.ops.get(i)));
            }else if(ctx.opers.get(i-1).MOD()!=null){
                res=Mod(res,visitNegExpr(ctx.ops.get(i)));
            }else if(ctx.opers.get(i-1).MUL()!=null){
                res=Mul(res,visitNegExpr(ctx.ops.get(i)));
            } else{
                throw new UnsupportedOperationException("Unsupported operation "+ctx.opers.get(i-1).getText()+" on line "+ctx.start.getLine());
            }
        }
        return res;
    }

    @Override
    public Expr visitMultiplicativeOperator(XstsDslParser.MultiplicativeOperatorContext ctx) {
        return super.visitMultiplicativeOperator(ctx);
    }

    @Override
    public Expr visitNegExpr(XstsDslParser.NegExprContext ctx) {
        if(ctx.ops.size()>0){
            return Neg(visitNegExpr(ctx.ops.get(0)));
        }else return visitPrimaryExpr(ctx.primaryExpr());
    }

    @Override
    public Expr visitPrimaryExpr(XstsDslParser.PrimaryExprContext ctx) {
        if(ctx.value()!=null) return visitValue(ctx.value());
        else return visitParenExpr(ctx.parenExpr());
    }

    @Override
    public Expr visitParenExpr(XstsDslParser.ParenExprContext ctx) {
        if(ctx.prime()!=null) return visitPrime(ctx.prime());
        else return visitImplyExpression(ctx.ops.get(0));
    }

    @Override
    public Expr visitValue(XstsDslParser.ValueContext ctx) {
        if(ctx.literal()!=null) return visitLiteral(ctx.literal());
        else return visitReference(ctx.reference());
    }

    @Override
    public Expr visitLiteral(XstsDslParser.LiteralContext ctx) {
        if(ctx.BOOLLIT()!=null){
            if(ctx.BOOLLIT().getText().equals("true")) return True(); else return False();
        }else if(ctx.INTLIT()!=null){
            return Int(Integer.parseInt(ctx.INTLIT().getText()));
        }else throw new RuntimeException("Literal "+ctx.getText()+" could not be resolved to integer or boolean type."+" On line "+ctx.start.getLine());
    }

    @Override
    public Expr visitReference(XstsDslParser.ReferenceContext ctx) {
        checkIfTempVar(ctx.name.getText());
        if(literalToIntMap.containsKey(ctx.name.getText())) return Int(literalToIntMap.get(ctx.name.getText()));
        else if(nameToDeclMap.containsKey(ctx.name.getText())) return nameToDeclMap.get(ctx.name.getText()).getRef();
        else throw new RuntimeException("Literal or reference "+ctx.name.getText()+" could not be resolved."+" On line "+ctx.start.getLine());

    }

    @Override
    public Expr visitPrime(XstsDslParser.PrimeContext ctx) {
        if(ctx.reference()!=null) return visitReference(ctx.reference());
        else throw new UnsupportedOperationException("Prime expressions are not supported."+" On line "+ctx.start.getLine());
//            return Prime(visitPrime(ctx.prime()));
    }

    public Stmt processAction(XstsDslParser.ActionContext ctx) {
        if(ctx.assignAction()!=null) return processAssignAction(ctx.assignAction());
        else if(ctx.assumeAction()!=null) return processAssumeAction(ctx.assumeAction());
        else if(ctx.havocAction()!=null) return processHavocAction(ctx.havocAction());
        else if(ctx.nonDetAction()!=null) return processNonDet(ctx.nonDetAction().nonDet());
        else return SkipStmt.getInstance();
    }

    public NonDetStmt processNonDet(XstsDslParser.NonDetContext ctx) {
        List<Stmt> choices=new ArrayList<Stmt>();
        for(XstsDslParser.SequentialActionContext seq:ctx.choices){
            choices.add(processSequentialAction(seq));
        }
        return NonDetStmt.of(choices);
    }

    public SequenceStmt processSequentialAction(XstsDslParser.SequentialActionContext ctx) {
        List<Stmt> stmts=new ArrayList<Stmt>();
        for(XstsDslParser.ActionContext action:ctx.actions){
            stmts.add(processAction(action));
        }
        return SequenceStmt.of(stmts);
    }

    public AssumeStmt processAssumeAction(XstsDslParser.AssumeActionContext ctx) {
        return Stmts.Assume(visitImplyExpression(ctx.cond));
    }

    public AssignStmt processAssignAction(XstsDslParser.AssignActionContext ctx) {
        if(!nameToDeclMap.containsKey(ctx.lhs.getText())) throw new RuntimeException("Could not resolve variable "+ctx.lhs.getText()+" on line "+ctx.start.getLine());
        return Stmts.Assign(nameToDeclMap.get(ctx.lhs.getText()),visitImplyExpression(ctx.rhs));
    }

    public HavocStmt processHavocAction(XstsDslParser.HavocActionContext ctx){
        if(!nameToDeclMap.containsKey(ctx.name.getText())) throw new RuntimeException("Could not resolve variable "+ctx.name.getText()+" on line "+ctx.start.getLine());
        return Stmts.Havoc(nameToDeclMap.get(ctx.name.getText()));
    }
}
