package oimo.m;
import haxe.macro.Context;
import haxe.macro.Expr;
import haxe.macro.ExprTools;
import haxe.macro.Type;
import oimo.m.IMat3;
import oimo.m.ITransform;
import oimo.m.IVec3;
using haxe.macro.ComplexTypeTools;
using oimo.m.B;
using oimo.m.U;

/**
 * Build Macro
 */
class B {

#if macro

	public static function build() {
		return filter();
	}

	public static function filter() {
		var fs:Array<Field> = Context.getBuildFields();
		var fs2:Array<Field> = [];
		for (field in fs) {
			switch (field.kind) {
				case FVar(t, e):
					var names:Array<String> = field.name.names(t.toType());
					if (names != null) {
						U.pushVariables(fs2, names, macro:Float, [APublic]);
						field.meta = [{
							name:":extern",
							pos: U.pos()
						}];
						fs2.push(field); // keep it for type inference
					} else {
						fs2.push(field);
					}
				case FFun(f):
					var args:Array<FunctionArg> = [];
					for (arg in f.args) {
						if (arg.type == null) continue;
						var names:Array<String> = arg.name.names(arg.type.toType());
						if (names != null) {
							for (name in names) {
								args.push({
									name: name,
									type: macro:Float
								});
							}
						} else {
							args.push(arg);
						}
					}
					field.kind = FFun({
						args: args,
						expr: filterFuncExpr(f.expr),
						ret: f.ret,
						params: f.params
					});
					fs2.push(field);
				case _:
					fs2.push(field);
			}
		}
		return fs2;
	}

	static function filterFuncExpr(e:Expr):Expr {
		switch(e.expr) {
		case EVars(vars):
			var newVars:Array<Var> = vars.copy();
			for (v in vars) {
				var names:Array<String> = v.name.names(v.type.toType());
				if (names != null) {
					newVars = newVars.concat(names.map(function(name) {
						return {
							name: name,
							type: macro:Float,
							expr: null
						};
					}));
				}
			}
			e.expr = EVars(newVars);
			return e;
		case _:
			return ExprTools.map(e, filterFuncExpr);
		}
	}

#end

}
