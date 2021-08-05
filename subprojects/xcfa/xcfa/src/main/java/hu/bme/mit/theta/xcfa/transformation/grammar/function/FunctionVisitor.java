package hu.bme.mit.theta.xcfa.transformation.grammar.function;

import hu.bme.mit.theta.common.Tuple2;
import hu.bme.mit.theta.core.decl.VarDecl;
import hu.bme.mit.theta.core.stmt.AssumeStmt;
import hu.bme.mit.theta.core.type.Expr;
import hu.bme.mit.theta.core.type.arraytype.ArrayType;
import hu.bme.mit.theta.xcfa.dsl.gen.CBaseVisitor;
import hu.bme.mit.theta.xcfa.dsl.gen.CParser;
import hu.bme.mit.theta.xcfa.model.XcfaLocation;
import hu.bme.mit.theta.xcfa.model.XcfaMetadata;
import hu.bme.mit.theta.xcfa.transformation.grammar.expression.ExpressionVisitor;
import hu.bme.mit.theta.xcfa.transformation.grammar.preprocess.BitwiseChecker;
import hu.bme.mit.theta.xcfa.transformation.grammar.preprocess.TypedefVisitor;
import hu.bme.mit.theta.xcfa.transformation.grammar.type.DeclarationVisitor;
import hu.bme.mit.theta.xcfa.transformation.grammar.type.TypeVisitor;
import hu.bme.mit.theta.xcfa.transformation.model.declaration.CDeclaration;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CAssignment;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CAssume;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CBreak;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CCase;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CCompound;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CContinue;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CDecls;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CDefault;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CDoWhile;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CExpr;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CFor;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CFunction;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CGoto;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CIf;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CProgram;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CRet;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CStatement;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CSwitch;
import hu.bme.mit.theta.xcfa.transformation.model.statements.CWhile;
import hu.bme.mit.theta.xcfa.transformation.model.types.complex.CComplexType;
import hu.bme.mit.theta.xcfa.transformation.model.types.simple.CSimpleType;
import org.antlr.v4.runtime.ParserRuleContext;
import org.antlr.v4.runtime.Token;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static com.google.common.base.Preconditions.checkState;
import static hu.bme.mit.theta.core.decl.Decls.Var;

/**
 * FunctionVisitor is responsible for the instantiation of high-level model elements, such as Programs, Functions,
 * and Statements. It employs a TypeVisitor instance to provide type information, a DeclarationVisitor instance to
 * provide information on declarations (both global and local, complete with initializations) and an ExpressionVisitor
 * instance to provide information on Expressions in the source code.
 */
public class FunctionVisitor extends CBaseVisitor<CStatement> {
	public static final FunctionVisitor instance = new FunctionVisitor();
	public static final Map<String, XcfaLocation> locLUT = new LinkedHashMap<>();

	private final Deque<Map<String, VarDecl<?>>> variables;
	private final List<VarDecl<?>> flatVariables;
	private final Map<VarDecl<?>, CDeclaration> functions;

	private VarDecl<?> createVar(CDeclaration declaration) {
		String name = declaration.getName();
		Map<String, VarDecl<?>> peek = variables.peek();
		//noinspection ConstantConditions
		checkState(!peek.containsKey(name), "Variable already exists!");
		peek.put(name, Var(name, declaration.getActualType().getSmtType()));
		VarDecl<?> varDecl = peek.get(name);
		XcfaMetadata.create(varDecl.getRef(), "cType", declaration.getActualType());
		flatVariables.add(varDecl);
		declaration.setVarDecl(varDecl);
		return varDecl;
	}

	public FunctionVisitor() {
		variables = new ArrayDeque<>();
		variables.push(new LinkedHashMap<>());
		flatVariables = new ArrayList<>();
		functions = new LinkedHashMap<>();
	}

	@Override
	public CStatement visitCompilationUnit(CParser.CompilationUnitContext ctx) {
		ctx.accept(TypedefVisitor.instance);
		// ExpressionVisitor.setBitwise(ctx.accept(BitwiseChecker.instance));
		BitwiseChecker.instance.checkIfBitwise(ctx);
		CProgram program = new CProgram();
		for (CParser.ExternalDeclarationContext externalDeclarationContext : ctx.translationUnit().externalDeclaration()) {
			CStatement accept = externalDeclarationContext.accept(this);
			if(accept instanceof CFunction) {
				program.getFunctions().add((CFunction) accept);
			}
			else if (accept instanceof CDecls) {
				program.getGlobalDeclarations().addAll(((CDecls) accept).getcDeclarations());
			}
		}
		recordMetadata(ctx, program);
		return program;
	}

	public void recordMetadata(ParserRuleContext ctx, CStatement statement) {
		Token start = ctx.getStart();
		Token stop = ctx.getStop();
		String stopText = stop.getText();
		String[] stopTextLines = stopText.split("\r\n|\r|\n", -1);
		int stopLines = stopTextLines.length - 1;
		int lineNumberStart = start.getLine();
		int colNumberStart = start.getCharPositionInLine();
		int lineNumberStop = stop.getLine() + stopLines;
		int colNumberStop = stopLines == 0 ? stop.getCharPositionInLine() + stopText.length() - 1 : stopTextLines[stopLines].length();
		int offsetStart = start.getStartIndex();
		int offsetEnd = stop.getStopIndex();
		XcfaMetadata.create(statement, "lineNumberStart", lineNumberStart);
		XcfaMetadata.create(statement, "colNumberStart", colNumberStart);
		XcfaMetadata.create(statement, "lineNumberStop", lineNumberStop);
		XcfaMetadata.create(statement, "colNumberStop", colNumberStop);
		XcfaMetadata.create(statement, "offsetStart", offsetStart);
		XcfaMetadata.create(statement, "offsetEnd", offsetEnd);
	}


	@Override
	public CStatement visitGlobalDeclaration(CParser.GlobalDeclarationContext ctx) {
		List<CDeclaration> declarations = DeclarationVisitor.instance.getDeclarations(ctx.declaration().declarationSpecifiers(), ctx.declaration().initDeclaratorList());
		CDecls decls = new CDecls();
		for (CDeclaration declaration : declarations) {
			if(!declaration.isFunc()) // functions should not be interpreted as global variables
				decls.getcDeclarations().add(Tuple2.of(declaration, createVar(declaration)));
			else {
				CSimpleType returnType = declaration.getBaseType();
				declaration.setBaseType(returnType);
				if(!variables.peek().containsKey(declaration.getName())) {
					XcfaMetadata.create(declaration.getName(), "cType", returnType.getActualType());
					VarDecl<?> var = createVar(declaration);
					functions.put(var, declaration);
				}
			}
		}
		recordMetadata(ctx, decls);
		return decls;
	}

	@Override
	public CStatement visitFunctionDefinition(CParser.FunctionDefinitionContext ctx) {
		CSimpleType returnType = ctx.declarationSpecifiers().accept(TypeVisitor.instance);
		CDeclaration funcDecl = ctx.declarator().accept(DeclarationVisitor.instance);
		funcDecl.setBaseType(returnType);
		if(!variables.peek().containsKey(funcDecl.getName())) {
			XcfaMetadata.create(funcDecl.getName(), "cType", returnType.getActualType());
			VarDecl<?> var = createVar(funcDecl);
			functions.put(var, funcDecl);
		}
		variables.push(new LinkedHashMap<>());
		locLUT.clear();
		flatVariables.clear();
		for (CDeclaration functionParam : funcDecl.getFunctionParams()) {
			if(functionParam.getName() != null) createVar(functionParam);
		}
		CParser.BlockItemListContext blockItemListContext = ctx.compoundStatement().blockItemList();
		if(blockItemListContext != null) {
			CStatement accept = blockItemListContext.accept(this);
			variables.pop();
			CFunction cFunction = new CFunction(funcDecl, accept, new ArrayList<>(flatVariables), new LinkedHashMap<>(locLUT));
			recordMetadata(ctx, cFunction);
			return cFunction;
		}
		variables.pop();
		CCompound cCompound = new CCompound();
		CFunction cFunction = new CFunction(funcDecl, cCompound, new ArrayList<>(flatVariables), new LinkedHashMap<>(locLUT));
		recordMetadata(ctx, cCompound);
		recordMetadata(ctx, cFunction);
		return cFunction;
	}

	@Override
	public CStatement visitBlockItemList(CParser.BlockItemListContext ctx) {
		CCompound compound = new CCompound();
		variables.push(new LinkedHashMap<>());
		for (CParser.BlockItemContext blockItemContext : ctx.blockItem()) {
			compound.getcStatementList().add(blockItemContext.accept(this));
		}
		variables.pop();
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitIdentifierStatement(CParser.IdentifierStatementContext ctx) {
		CStatement statement = ctx.statement().accept(this);
		CCompound compound = new CCompound();
		compound.getcStatementList().add(statement);
		compound.setId(ctx.Identifier().getText());
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitCaseStatement(CParser.CaseStatementContext ctx) {
		CExpr cexpr = new CExpr(ctx.constantExpression().accept(ExpressionVisitor.create(variables, functions)));
		CCase cCase = new CCase(
				cexpr,
				ctx.statement().accept(this));
		recordMetadata(ctx, cCase);
		recordMetadata(ctx.constantExpression(), cexpr);
		return cCase;
	}

	@Override
	public CStatement visitDefaultStatement(CParser.DefaultStatementContext ctx) {
		CDefault cDefault = new CDefault(ctx.statement().accept(this));
		recordMetadata(ctx, cDefault);
		return cDefault;
	}

	@Override
	public CStatement visitCompoundStatement(CParser.CompoundStatementContext ctx) {
		if(ctx.blockItemList() != null) {
			return ctx.blockItemList().accept(this);
		}
		CCompound compound = new CCompound();
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitExpressionStatement(CParser.ExpressionStatementContext ctx) {
		CStatement statement = ctx.expression() == null ? new CCompound() : ctx.expression().accept(this);
		recordMetadata(ctx, statement);
		return statement;
	}

	@Override
	public CStatement visitIfStatement(CParser.IfStatementContext ctx) {
		variables.push(new LinkedHashMap<>());
		CIf cIf = new CIf(
				ctx.expression().accept(this),
				ctx.statement(0).accept(this),
				ctx.statement().size() > 1 ? ctx.statement(1).accept(this) : null);
		recordMetadata(ctx, cIf);
		variables.pop();
		return cIf;
	}

	@Override
	public CStatement visitSwitchStatement(CParser.SwitchStatementContext ctx) {
		variables.push(new LinkedHashMap<>());
		CSwitch cSwitch = new CSwitch(
				ctx.expression().accept(this),
				ctx.statement().accept(this));
		recordMetadata(ctx, cSwitch);
		variables.pop();
		return cSwitch;
	}

	@Override
	public CStatement visitWhileStatement(CParser.WhileStatementContext ctx) {
		variables.push(new LinkedHashMap<>());
		CWhile cWhile = new CWhile(
				ctx.statement().accept(this),
				ctx.expression().accept(this));
		recordMetadata(ctx, cWhile);
		variables.pop();
		return cWhile;
	}

	@Override
	public CStatement visitDoWhileStatement(CParser.DoWhileStatementContext ctx) {
		variables.push(new LinkedHashMap<>());
		CDoWhile cDoWhile = new CDoWhile(
				ctx.statement().accept(this),
				ctx.expression().accept(this));
		recordMetadata(ctx, cDoWhile);
		variables.pop();
		return cDoWhile;
	}

	@Override
	public CStatement visitForStatement(CParser.ForStatementContext ctx) {
		variables.push(new LinkedHashMap<>());
		CStatement init = ctx.forCondition().forInit().accept(this);
		CStatement test = ctx.forCondition().forTest().accept(this);
		CStatement incr = ctx.forCondition().forIncr().accept(this);
		CFor cFor = new CFor(
				ctx.statement().accept(this),
				init,
				test,
				incr);
		recordMetadata(ctx, cFor);
		variables.pop();
		return cFor;
	}

	@Override
	public CStatement visitGotoStatement(CParser.GotoStatementContext ctx) {
		CGoto cGoto = new CGoto(ctx.Identifier().getText());
		recordMetadata(ctx, cGoto);
		return cGoto;
	}

	@Override
	public CStatement visitContinueStatement(CParser.ContinueStatementContext ctx) {
		CContinue cContinue = new CContinue();
		recordMetadata(ctx, cContinue);
		return cContinue;
	}

	@Override
	public CStatement visitBreakStatement(CParser.BreakStatementContext ctx) {
		CBreak cBreak = new CBreak();
		recordMetadata(ctx, cBreak);
		return cBreak;
	}

	@Override
	public CStatement visitReturnStatement(CParser.ReturnStatementContext ctx) {
		CRet cRet = new CRet(ctx.expression() == null ? null : ctx.expression().accept(this));
		recordMetadata(ctx, cRet);
		return cRet;
	}

	@Override
	public CStatement visitStatement(CParser.StatementContext ctx) {
		return ctx.children.get(0).accept(this);
	}

	@Override
	public CStatement visitBodyDeclaration(CParser.BodyDeclarationContext ctx) {
		List<CDeclaration> declarations = DeclarationVisitor.instance.getDeclarations(ctx.declaration().declarationSpecifiers(), ctx.declaration().initDeclaratorList());
		CCompound compound = new CCompound();
		for (CDeclaration declaration : declarations) {
			if(declaration.getInitExpr() != null) {
				CAssignment cAssignment = new CAssignment(createVar(declaration).getRef(), declaration.getInitExpr(), "=");
				recordMetadata(ctx, cAssignment);
				compound.getcStatementList().add(cAssignment);
			}
			else {
				VarDecl<?> varDecl = createVar(declaration);
				// if there is no initializer, then we'll add an assumption regarding min and max values
				if (!(varDecl.getType() instanceof ArrayType)) {
					AssumeStmt assumeStmt = CComplexType.getType(varDecl.getRef()).limit(varDecl.getRef());
					CAssume cAssume = new CAssume(assumeStmt);
					compound.getcStatementList().add(cAssume);
				}
			}
		}
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitExpression(CParser.ExpressionContext ctx) {
		CCompound compound = new CCompound();
		for (CParser.AssignmentExpressionContext assignmentExpressionContext : ctx.assignmentExpression()) {
			compound.getcStatementList().add(assignmentExpressionContext.accept(this));
		}
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitAssignmentExpressionAssignmentExpression(CParser.AssignmentExpressionAssignmentExpressionContext ctx) {
		ExpressionVisitor expressionVisitor = ExpressionVisitor.create(variables, functions);
		CCompound compound = new CCompound();
		CCompound preStatements = new CCompound();
		CCompound postStatements = new CCompound();
		Expr<?> ret = ctx.unaryExpression().accept(expressionVisitor);
		CAssignment cAssignment = new CAssignment(ret, ctx.assignmentExpression().accept(this), ctx.assignmentOperator().getText());
		compound.getcStatementList().add(cAssignment);
		preStatements.getcStatementList().addAll(expressionVisitor.getPreStatements());
		compound.setPreStatements(preStatements);
		recordMetadata(ctx, compound);
		postStatements.getcStatementList().addAll(expressionVisitor.getPostStatements());
		compound.setPostStatements(postStatements);
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitAssignmentExpressionConditionalExpression(CParser.AssignmentExpressionConditionalExpressionContext ctx) {
		ExpressionVisitor expressionVisitor = ExpressionVisitor.create(variables, functions);
		CCompound compound = new CCompound();
		CCompound preStatements = new CCompound();
		CCompound postStatements = new CCompound();
		Expr<?> ret = ctx.conditionalExpression().accept(expressionVisitor);
		CExpr cexpr = new CExpr(ret);
		compound.getcStatementList().add(cexpr);
		preStatements.getcStatementList().addAll(expressionVisitor.getPreStatements());
		compound.setPreStatements(preStatements);
		recordMetadata(ctx, compound);
		postStatements.getcStatementList().addAll(expressionVisitor.getPostStatements());
		compound.setPostStatements(postStatements);
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitForDeclaration(CParser.ForDeclarationContext ctx) {
		List<CDeclaration> declarations = DeclarationVisitor.instance.getDeclarations(ctx.declarationSpecifiers(), ctx.initDeclaratorList());
		CCompound compound = new CCompound();
		for (CDeclaration declaration : declarations) {
			CAssignment cAssignment = new CAssignment(createVar(declaration).getRef(), declaration.getInitExpr(), "=");
			recordMetadata(ctx, cAssignment);
			if(declaration.getInitExpr() != null) compound.getcStatementList().add(cAssignment);
		}
		recordMetadata(ctx, compound);
		return compound;
	}

	@Override
	public CStatement visitForExpression(CParser.ForExpressionContext ctx) {
		CCompound compound = new CCompound();
		for (CParser.AssignmentExpressionContext assignmentExpressionContext : ctx.assignmentExpression()) {
			compound.getcStatementList().add(assignmentExpressionContext.accept(this));
		}
		recordMetadata(ctx, compound);
		return compound;
	}
}