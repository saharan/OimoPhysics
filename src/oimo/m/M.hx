package oimo.m;
import haxe.ds.Vector;
import haxe.macro.ComplexTypeTools;
import haxe.macro.Context;
import haxe.macro.Expr;
import haxe.macro.Type;
import oimo.math.Mat3;
import oimo.math.Mat4;
import oimo.math.MathUtil;
import oimo.math.Transform;
import oimo.math.Vec3;
using Lambda;
using oimo.m.M;
using oimo.m.U;
using haxe.macro.ExprTools;
using haxe.macro.TypeTools;

/**
 * Expression Macro
 */
class M {

	// ---------------------------------------------------------------------
	// Util
	// ---------------------------------------------------------------------

	public static function zip<A1, A2, A3>(a1:Array<A1>, a2:Array<A2>, f:A1 -> A2 -> A3):Array<A3> {
		var res:Array<A3> = [];
		assert(a1.length == a2.length);
		var num:Int = a1.length;
		for (i in 0...num) {
			res.push(f(a1[i], a2[i]));
		}
		return res;
	}

	// ---------------------------------------------------------------------
	// Float
	// ---------------------------------------------------------------------

	public static inline var EPS:Float = 1e-5;

	public static macro function toFixed4(x:ExprOf<Float>) {
		return macro $x > 0 ? Std.int($x * 1000 + 0.5) / 1000 : Std.int($x * 1000 - 0.5) / 1000;
	}

	public static macro function round(x:ExprOf<Float>) {
		return macro $x > 0 ? Std.int($x + 0.5) : Std.int($x - 0.5);
	}

	public static macro function eq(x:ExprOf<Float>, y:ExprOf<Float>) {
		return macro $x > $y - M.EPS && $x < $y + M.EPS;
	}

	public static macro function eq0(x:ExprOf<Float>) {
		return macro $x > -M.EPS && $x < M.EPS;
	}

	public static macro function gt0(x:ExprOf<Float>) {
		return macro $x > M.EPS;
	}

	public static macro function lt0(x:ExprOf<Float>) {
		return macro $x < -M.EPS;
	}

	public static macro function min(x:ExprOf<Float>, y:ExprOf<Float>) {
		return macro $x < $y ? $x : $y;
	}

	public static macro function max(x:ExprOf<Float>, y:ExprOf<Float>) {
		return macro $x < $y ? $x : $y;
	}

	// ---------------------------------------------------------------------
	// List
	// ---------------------------------------------------------------------

	public static macro function list_foreach(base:Expr, next:Expr, loop:Expr) {
		var n:String = next.s();
		return macro {
			while ($base != null) {
				var n = $base.$n;
				$loop;
				$base = n;
			}
		}
	}

	public static macro function list_push(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr) {
		var p:String = prev.s();
		var n:String = next.s();
		return macro {
			if ($first == null) {
				$first = $e;
				$last = $e;
			} else {
				$last.$n = $e;
				$e.$p = $last;
				$last = $e;
			}
		}
	}

	public static macro function list_addFirst(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr) {
		var p:String = prev.s();
		var n:String = next.s();
		return macro {
			if ($first == null) {
				$first = $e;
				$last = $e;
			} else {
				$first.$p = $e;
				$e.$n = $first;
				$first = $e;
			}
		}
	}

	public static macro function list_remove(first:Expr, last:Expr, prev:Expr, next:Expr, e:Expr) {
		var p:String = prev.s();
		var n:String = next.s();
		return macro {
			var prev = $e.$p;
			var next = $e.$n;
			if (prev != null) {
				prev.$n = next;
			}
			if (next != null) {
				next.$p = prev;
			}
			if ($e == $first) {
				$first = $first.$n;
			}
			if ($e == $last) {
				$last = $last.$p;
			}
			$e.$n = null;
			$e.$p = null;
		}
	}

	public static macro function singleList_addFirst(first:Expr, next:Expr, e:Expr) {
		var n:String = next.s();
		return macro {
			if ($first == null) {
				$first = $e;
			} else {
				$e.$n = $first;
				$first = $e;
			}
		}
	}

	public static macro function singleList_pick(first:Expr, next:Expr, newInstance:Expr) {
		var n:String = next.s();
		return macro {
			var first = $first;
			if (first != null) {
				$first = first.$n;
				first.$n = null;
			} else {
				first = $newInstance;
			}
			first;
		}
	}

	public static macro function singleList_pool(first:Expr, next:Expr, e:Expr) {
		var n:String = next.s();
		return macro {
			$e.$n = $first;
			$first = $e;
		}
	}

	// ---------------------------------------------------------------------
	// Assertion
	// ---------------------------------------------------------------------

	public static macro function assert(expectedToBeTrue:ExprOf<Bool>) {
		return macro if (!$expectedToBeTrue) throw "assertion error: " + $v{expectedToBeTrue.s()} + " is false";
	}

	// ---------------------------------------------------------------------
	// Vec3
	// ---------------------------------------------------------------------

	public static macro function vec3_fromVec3(dst:Expr, src:ExprOf<Vec3>) {
		return macro {
			var v:oimo.math.Vec3 = cast $src;
			${assignVars(dst.s().vec3Names(), [
				macro v.x,
				macro v.y,
				macro v.z
			])};
		}
	}

	public static macro function vec3_toVec3(dst:ExprOf<Vec3>, src:Expr) {
		return macro {
			var v:oimo.math.Vec3 = cast $dst;
			${assignVars([
				macro v.x,
				macro v.y,
				macro v.z
			], src.s().vec3Names())};
		}
	}

	public static macro function vec3_mulMat3(dst:Expr, src1:Expr, src2:Expr) {
		var v = src1.s().vec3Names();
		var m = src2.s().mat3Names();
		return assignVars(dst.s().vec3Names(), [
			macro ${m[0].f()} * ${v[0].f()} + ${m[1].f()} * ${v[1].f()} + ${m[2].f()} * ${v[2].f()},
			macro ${m[3].f()} * ${v[0].f()} + ${m[4].f()} * ${v[1].f()} + ${m[5].f()} * ${v[2].f()},
			macro ${m[6].f()} * ${v[0].f()} + ${m[7].f()} * ${v[1].f()} + ${m[8].f()} * ${v[2].f()}
		]);
	}

	public static macro function vec3_zero(dst:Expr) {
		return assignVars(dst.s().vec3Names(), [macro 0, macro 0, macro 0]);
	}

	public static macro function vec3_set(dst:Expr, x:ExprOf<Float>, y:ExprOf<Float>, z:ExprOf<Float>) {
		return assignVars(dst.s().vec3Names(), [x, y, z]);
	}

	public static macro function vec3_assign(dst:Expr, src:Expr) {
		return assignVars(dst.s().vec3Names(), src.s().vec3Names());
	}

	public static macro function vec3_add(dst:Expr, src1:Expr, src2:Expr) {
		return binOpVars(dst.s().vec3Names(), src1.s().vec3Names(), src2.s().vec3Names(), OpAdd);
	}

	public static macro function vec3_addRhsScaled(dst:Expr, src1:Expr, src2:Expr, src3:ExprOf<Float>) {
		return binOp2RLVars(dst.s().vec3Names(), src1.s().vec3Names(), src2.s().vec3Names(), [src3, src3, src3], OpAdd, OpMult);
	}

	public static macro function vec3_sub(dst:Expr, src1:Expr, src2:Expr) {
		return binOpVars(dst.s().vec3Names(), src1.s().vec3Names(), src2.s().vec3Names(), OpSub);
	}

	public static macro function vec3_scale(dst:Expr, src1:Expr, src2:ExprOf<Float>) {
		return binOpVars(dst.s().vec3Names(), src1.s().vec3Names(), [src2, src2, src2], OpMult);
	}

	public static macro function vec3_abs(dst:Expr, src:Expr) {
		return assignVars(dst.s().vec3Names(), src.s().vec3Names().map(f).map(function(e1) {
			return macro $e1 < 0 ? -$e1 : $e1;
		}));
	}

	public static macro function vec3_min(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().vec3Names().map(f);
		var bs = src2.s().vec3Names().map(f);
		return assignVars(dst.s().vec3Names(), zip(as, bs, function(e1, e2) {
			return macro $e1 < $e2 ? $e1 : $e2;
		}));
	}

	public static macro function vec3_max(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().vec3Names().map(f);
		var bs = src2.s().vec3Names().map(f);
		return assignVars(dst.s().vec3Names(), zip(as, bs, function(e1, e2) {
			return macro $e1 > $e2 ? $e1 : $e2;
		}));
	}

	public static macro function vec3_dot(src1:Expr, src2:Expr) {
		var as = src1.s().vec3Names();
		var bs = src2.s().vec3Names();
		return macro ${as[0].f()} * ${bs[0].f()} + ${as[1].f()} * ${bs[1].f()} + ${as[2].f()} * ${bs[2].f()};
	}

	public static macro function vec3_addHorizontal(src:Expr) {
		var as = src.s().vec3Names();
		return macro ${as[0].f()} + ${as[1].f()} + ${as[2].f()};
	}

	public static macro function vec3_mulHorizontal(src:Expr) {
		var as = src.s().vec3Names();
		return macro ${as[0].f()} * ${as[1].f()} * ${as[2].f()};
	}

	public static macro function vec3_get(src:Expr, index:ExprOf<Int>) {
		var as = src.s().vec3Names();
		var i = 0;
		switch (index.expr) {
		case EConst(CInt(v)):
			i = Std.parseInt(v);
		case _:
			Context.error("invalid index: " + index.expr, U.pos());
		}
		return macro ${as[i].f()};
	}

	public static macro function vec3_length(src:Expr) {
		return macro oimo.math.MathUtil.sqrt(M.vec3_dot($src, $src));
	}

	public static macro function vec3_cross(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().vec3Names();
		var bs = src2.s().vec3Names();
		return assignVars(dst.s().vec3Names(), [
			macro ${as[1].f()} * ${bs[2].f()} - ${as[2].f()} * ${bs[1].f()},
			macro ${as[2].f()} * ${bs[0].f()} - ${as[0].f()} * ${bs[2].f()},
			macro ${as[0].f()} * ${bs[1].f()} - ${as[1].f()} * ${bs[0].f()}
		]);
	}

	// ---------------------------------------------------------------------
	// Mat3
	// ---------------------------------------------------------------------

	public static macro function mat3_fromMat3(dst:Expr, src:ExprOf<Mat3>) {
		return macro {
			var m:oimo.math.Mat3 = cast $src;
			${assignVars(dst.s().mat3Names(), [
				macro m.e00,
				macro m.e01,
				macro m.e02,
				macro m.e10,
				macro m.e11,
				macro m.e12,
				macro m.e20,
				macro m.e21,
				macro m.e22
			])};
		}
	}

	public static macro function mat3_fromQuat(dst:Expr, src:Expr) {
		var as = src.s().quatNames();
		return macro {
			var x:Float = ${as[0].f()};
			var y:Float = ${as[1].f()};
			var z:Float = ${as[2].f()};
			var w:Float = ${as[3].f()};
			var x2:Float = 2 * x;
			var y2:Float = 2 * y;
			var z2:Float = 2 * z;
			var xx:Float = x * x2;
			var yy:Float = y * y2;
			var zz:Float = z * z2;
			var xy:Float = x * y2;
			var yz:Float = y * z2;
			var xz:Float = x * z2;
			var wx:Float = w * x2;
			var wy:Float = w * y2;
			var wz:Float = w * z2;
			${assignVars(dst.s().mat3Names(), [
				macro 1 - yy - zz,
				macro xy - wz,
				macro xz + wy,
				macro xy + wz,
				macro 1 - xx - zz,
				macro yz - wx,
				macro xz - wy,
				macro yz + wx,
				macro 1 - xx - yy
			])};
		}
	}

	public static macro function mat3_id(dst:Expr) {
		return assignVars(dst.s().mat3Names(), [
			macro 1, macro 0, macro 0,
			macro 0, macro 1, macro 0,
			macro 0, macro 0, macro 1
		]);
	}

	public static macro function mat3_zero(dst:Expr) {
		return assignVars(dst.s().mat3Names(), [
			macro 0, macro 0, macro 0,
			macro 0, macro 0, macro 0,
			macro 0, macro 0, macro 0
		]);
	}

	public static macro function mat3_diagonal(dst:Expr, x:ExprOf<Float>, y:ExprOf<Float>, z:ExprOf<Float>) {
		return assignVars(dst.s().mat3Names(), [
			x, macro 0, macro 0,
			macro 0, y, macro 0,
			macro 0, macro 0, z
		]);
	}

	public static macro function mat3_assign(dst:Expr, src:Expr) {
		return assignVars(dst.s().mat3Names(), src.s().mat3Names());
	}

	public static macro function mat3_add(dst:Expr, src1:Expr, src2:Expr) {
		return binOpVars(dst.s().mat3Names(), src1.s().mat3Names(), src2.s().mat3Names(), OpAdd);
	}

	public static macro function mat3_addDiag(dst:Expr, src:Expr, x:ExprOf<Float>, y:ExprOf<Float>, z:ExprOf<Float>) {
		return binOpVars(dst.s().mat3NamesDiag(), src.s().mat3NamesDiag(), [x, y, z], OpAdd);
	}

	public static macro function mat3_addRhsScaled(dst:Expr, src1:Expr, src2:Expr, src3:ExprOf<Float>) {
		return binOp2RLVars(dst.s().mat3Names(), src1.s().mat3Names(), src2.s().mat3Names(), [
			src3, src3, src3,
			src3, src3, src3,
			src3, src3, src3
		], OpAdd, OpMult);
	}

	public static macro function mat3_addAll(dst:Expr, src1:Expr, src2:Expr,
		e00:ExprOf<Float>, e01:ExprOf<Float>, e02:ExprOf<Float>,
		e10:ExprOf<Float>, e11:ExprOf<Float>, e12:ExprOf<Float>,
		e20:ExprOf<Float>, e21:ExprOf<Float>, e22:ExprOf<Float>
	) {
		return binOpVars(dst.s().mat3Names(), src1.s().mat3Names(), [
			e00, e01, e02,
			e10, e11, e12,
			e20, e21, e22
		], OpAdd);
	}

	public static macro function mat3_sub(dst:Expr, src1:Expr, src2:Expr) {
		return binOpVars(dst.s().mat3Names(), src1.s().mat3Names(), src2.s().mat3Names(), OpSub);
	}

	public static macro function mat3_scale(dst:Expr, src1:Expr, src2:ExprOf<Float>) {
		return binOpVars(dst.s().mat3Names(), src1.s().mat3Names(), [
			src2, src2, src2,
			src2, src2, src2,
			src2, src2, src2
		], OpMult);
	}

	public static macro function mat3_scaleDiag(dst:Expr, src:Expr, x:ExprOf<Float>, y:ExprOf<Float>, z:ExprOf<Float>) {
		return binOpVars(dst.s().mat3NamesDiag(), src.s().mat3NamesDiag(), [x, y, z], OpMult);
	}

	public static macro function mat3_scaleRows(dst:Expr, src:Expr, x:ExprOf<Float>, y:ExprOf<Float>, z:ExprOf<Float>) {
		return binOpVars(dst.s().mat3Names(), src.s().mat3Names(), [
			x, x, x,
			y, y, y,
			z, z, z
		], OpMult);
	}

	public static macro function mat3_scaleRowsVec3(dst:Expr, src1:Expr, src2:Expr) {
		var v:Array<String> = src2.s().vec3Names();
		return binOpVars(dst.s().mat3Names(), src1.s().mat3Names(), [
			v[0], v[0], v[0],
			v[1], v[1], v[1],
			v[2], v[2], v[2]
		], OpMult);
	}

	public static macro function mat3_mul(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().mat3Names();
		var bs = src2.s().mat3Names();
		return assignVars(dst.s().mat3Names(), [
			macro ${as[0].f()} * ${bs[0].f()} + ${as[1].f()} * ${bs[3].f()} + ${as[2].f()} * ${bs[6].f()},
			macro ${as[0].f()} * ${bs[1].f()} + ${as[1].f()} * ${bs[4].f()} + ${as[2].f()} * ${bs[7].f()},
			macro ${as[0].f()} * ${bs[2].f()} + ${as[1].f()} * ${bs[5].f()} + ${as[2].f()} * ${bs[8].f()},
			macro ${as[3].f()} * ${bs[0].f()} + ${as[4].f()} * ${bs[3].f()} + ${as[5].f()} * ${bs[6].f()},
			macro ${as[3].f()} * ${bs[1].f()} + ${as[4].f()} * ${bs[4].f()} + ${as[5].f()} * ${bs[7].f()},
			macro ${as[3].f()} * ${bs[2].f()} + ${as[4].f()} * ${bs[5].f()} + ${as[5].f()} * ${bs[8].f()},
			macro ${as[6].f()} * ${bs[0].f()} + ${as[7].f()} * ${bs[3].f()} + ${as[8].f()} * ${bs[6].f()},
			macro ${as[6].f()} * ${bs[1].f()} + ${as[7].f()} * ${bs[4].f()} + ${as[8].f()} * ${bs[7].f()},
			macro ${as[6].f()} * ${bs[2].f()} + ${as[7].f()} * ${bs[5].f()} + ${as[8].f()} * ${bs[8].f()}
		]);
	}

	public static macro function mat3_mulLhsTransposed(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().mat3Names();
		var bs = src2.s().mat3Names();
		return assignVars(dst.s().mat3Names(), [
			macro ${as[0].f()} * ${bs[0].f()} + ${as[3].f()} * ${bs[3].f()} + ${as[6].f()} * ${bs[6].f()},
			macro ${as[0].f()} * ${bs[1].f()} + ${as[3].f()} * ${bs[4].f()} + ${as[6].f()} * ${bs[7].f()},
			macro ${as[0].f()} * ${bs[2].f()} + ${as[3].f()} * ${bs[5].f()} + ${as[6].f()} * ${bs[8].f()},
			macro ${as[1].f()} * ${bs[0].f()} + ${as[4].f()} * ${bs[3].f()} + ${as[7].f()} * ${bs[6].f()},
			macro ${as[1].f()} * ${bs[1].f()} + ${as[4].f()} * ${bs[4].f()} + ${as[7].f()} * ${bs[7].f()},
			macro ${as[1].f()} * ${bs[2].f()} + ${as[4].f()} * ${bs[5].f()} + ${as[7].f()} * ${bs[8].f()},
			macro ${as[2].f()} * ${bs[0].f()} + ${as[5].f()} * ${bs[3].f()} + ${as[8].f()} * ${bs[6].f()},
			macro ${as[2].f()} * ${bs[1].f()} + ${as[5].f()} * ${bs[4].f()} + ${as[8].f()} * ${bs[7].f()},
			macro ${as[2].f()} * ${bs[2].f()} + ${as[5].f()} * ${bs[5].f()} + ${as[8].f()} * ${bs[8].f()}
		]);
	}

	public static macro function mat3_mulRhsTransposed(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().mat3Names();
		var bs = src2.s().mat3Names();
		return assignVars(dst.s().mat3Names(), [
			macro ${as[0].f()} * ${bs[0].f()} + ${as[1].f()} * ${bs[1].f()} + ${as[2].f()} * ${bs[2].f()},
			macro ${as[0].f()} * ${bs[3].f()} + ${as[1].f()} * ${bs[4].f()} + ${as[2].f()} * ${bs[5].f()},
			macro ${as[0].f()} * ${bs[6].f()} + ${as[1].f()} * ${bs[7].f()} + ${as[2].f()} * ${bs[8].f()},
			macro ${as[3].f()} * ${bs[0].f()} + ${as[4].f()} * ${bs[1].f()} + ${as[5].f()} * ${bs[2].f()},
			macro ${as[3].f()} * ${bs[3].f()} + ${as[4].f()} * ${bs[4].f()} + ${as[5].f()} * ${bs[5].f()},
			macro ${as[3].f()} * ${bs[6].f()} + ${as[4].f()} * ${bs[7].f()} + ${as[5].f()} * ${bs[8].f()},
			macro ${as[6].f()} * ${bs[0].f()} + ${as[7].f()} * ${bs[1].f()} + ${as[8].f()} * ${bs[2].f()},
			macro ${as[6].f()} * ${bs[3].f()} + ${as[7].f()} * ${bs[4].f()} + ${as[8].f()} * ${bs[5].f()},
			macro ${as[6].f()} * ${bs[6].f()} + ${as[7].f()} * ${bs[7].f()} + ${as[8].f()} * ${bs[8].f()}
		]);
	}

	public static macro function mat3_mulResultTransposed(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().mat3Names();
		var bs = src2.s().mat3Names();
		return assignVars(dst.s().mat3Names(), [
			macro ${as[0].f()} * ${bs[0].f()} + ${as[1].f()} * ${bs[3].f()} + ${as[2].f()} * ${bs[6].f()},
			macro ${as[3].f()} * ${bs[0].f()} + ${as[4].f()} * ${bs[3].f()} + ${as[5].f()} * ${bs[6].f()},
			macro ${as[6].f()} * ${bs[0].f()} + ${as[7].f()} * ${bs[3].f()} + ${as[8].f()} * ${bs[6].f()},
			macro ${as[0].f()} * ${bs[1].f()} + ${as[1].f()} * ${bs[4].f()} + ${as[2].f()} * ${bs[7].f()},
			macro ${as[3].f()} * ${bs[1].f()} + ${as[4].f()} * ${bs[4].f()} + ${as[5].f()} * ${bs[7].f()},
			macro ${as[6].f()} * ${bs[1].f()} + ${as[7].f()} * ${bs[4].f()} + ${as[8].f()} * ${bs[7].f()},
			macro ${as[0].f()} * ${bs[2].f()} + ${as[1].f()} * ${bs[5].f()} + ${as[2].f()} * ${bs[8].f()},
			macro ${as[3].f()} * ${bs[2].f()} + ${as[4].f()} * ${bs[5].f()} + ${as[5].f()} * ${bs[8].f()},
			macro ${as[6].f()} * ${bs[2].f()} + ${as[7].f()} * ${bs[5].f()} + ${as[8].f()} * ${bs[8].f()}
		]);
	}

	public static macro function mat3_inertiaFromCOG(dst:Expr, src:Expr) {
		var as = src.s().vec3Names();
		return macro {
			var xx:Float = ${as[0].f()} * ${as[0].f()};
			var yy:Float = ${as[1].f()} * ${as[1].f()};
			var zz:Float = ${as[2].f()} * ${as[2].f()};
			var xy:Float = -${as[0].f()} * ${as[1].f()};
			var yz:Float = -${as[1].f()} * ${as[2].f()};
			var zx:Float = -${as[2].f()} * ${as[0].f()};
			${assignVars(dst.s().mat3Names(), [
				macro yy + zz, macro xy, macro zx,
				macro xy, macro xx + zz, macro yz,
				macro zx, macro yz, macro xx + yy
			])};
		}
	}

	public static macro function mat3_transformInertia(dst:Expr, inertia:Expr, transform:Expr) {
		return macro {
			M.mat3_mul($dst, $transform, $inertia);
			M.mat3_mulRhsTransposed($dst, $dst, $transform);
		}
	}

	public static macro function mat3_inv(dst:Expr, src:Expr) {
		var as = src.s().mat3Names();
		return macro {
			var d00:Float = ${as[4].f()} * ${as[8].f()} - ${as[5].f()} * ${as[7].f()};
			var d01:Float = ${as[3].f()} * ${as[8].f()} - ${as[5].f()} * ${as[6].f()};
			var d02:Float = ${as[3].f()} * ${as[7].f()} - ${as[4].f()} * ${as[6].f()};
			var d10:Float = ${as[1].f()} * ${as[8].f()} - ${as[2].f()} * ${as[7].f()};
			var d11:Float = ${as[0].f()} * ${as[8].f()} - ${as[2].f()} * ${as[6].f()};
			var d12:Float = ${as[0].f()} * ${as[7].f()} - ${as[1].f()} * ${as[6].f()};
			var d20:Float = ${as[1].f()} * ${as[5].f()} - ${as[2].f()} * ${as[4].f()};
			var d21:Float = ${as[0].f()} * ${as[5].f()} - ${as[2].f()} * ${as[3].f()};
			var d22:Float = ${as[0].f()} * ${as[4].f()} - ${as[1].f()} * ${as[3].f()};
			var d:Float = ${as[0].f()} * d00 - ${as[1].f()} * d01 + ${as[2].f()} * d02;
			if (!M.eq0(d)) d = 1 / d;
			${assignVars(dst.s().mat3Names(), [
				macro d00 * d,
				macro -d10 * d,
				macro d20 * d,
				macro -d01 * d,
				macro d11 * d,
				macro -d21 * d,
				macro d02 * d,
				macro -d12 * d,
				macro d22 * d
			])};
		}
	}

	// ---------------------------------------------------------------------
	// Transform
	// ---------------------------------------------------------------------

	public static macro function transform_fromTransform(dst:Expr, src:ExprOf<Transform>) {
		return macro {
			var t:oimo.math.Transform = cast $src;
			var v:oimo.math.Vec3 = t.origin;
			var m:oimo.math.Mat3 = t.rotation;
			${assignVars(dst.s().transformNames(), [
				macro v.x,
				macro v.y,
				macro v.z,
				macro m.e00,
				macro m.e01,
				macro m.e02,
				macro m.e10,
				macro m.e11,
				macro m.e12,
				macro m.e20,
				macro m.e21,
				macro m.e22
			])};
		}
	}

	public static macro function transform_assign(dst:Expr, src:Expr) {
		return assignVars(dst.s().transformNames(), src.s().transformNames());
	}

	public static macro function transform_id(dst:Expr) {
		return assignVars(dst.s().transformNames(), [
			macro 0, macro 0, macro 0,
			macro 1, macro 0, macro 0,
			macro 0, macro 1, macro 0,
			macro 0, macro 0, macro 1
		]);
	}

	public static macro function transform_toMat4(dst:Expr, src:Expr) {
		return macro {
			var m:oimo.math.Mat4 = cast $dst;
			${assignVars([
				macro m.e03, macro m.e13, macro m.e23,
				macro m.e00, macro m.e01, macro m.e02,
				macro m.e10, macro m.e11, macro m.e12,
				macro m.e20, macro m.e21, macro m.e22
			], src.s().transformNames())};
			${assignVars([macro m.e30, macro m.e31, macro m.e32, macro m.e33], [macro 0, macro 0, macro 0, macro 1])};
		}
	}

	public static macro function transform_mul(dst:Expr, src1:Expr, src2:Expr) {
		return macro {
			${macro M.mat3_mul($dst, $src1, $src2)};
			${macro M.vec3_mulMat3($dst, $src1, $src2)};
			${macro M.vec3_add($dst, $dst, $src2)};
		}
	}

	// ---------------------------------------------------------------------
	// Quat
	// ---------------------------------------------------------------------

	public static macro function quat_id(dst:Expr) {
		return assignVars(dst.s().quatNames(), [macro 0, macro 0, macro 0, macro 1]);
	}

	public static macro function quat_fromVec3AndFloat(dst:Expr, src1:Expr, src2:ExprOf<Float>) {
		return assignVars(dst.s().quatNames(), src1.s().vec3Names().map(f).concat([src2]));
	}

	public static macro function quat_fromMat3(dst:Expr, src:Expr) {
		var ds = dst.s().quatNames();
		var as = src.s().mat3Names();
		return macro {
			var e00:Float = ${as[0].f()};
			var e11:Float = ${as[4].f()};
			var e22:Float = ${as[8].f()};
			var t:Float = e00 + e11 + e22;
			var s:Float;
			if (t > 0) {
				s = oimo.math.MathUtil.sqrt(t + 1);
				${ds[3].f()} = 0.5 * s;
				s = 0.5 / s;
				${ds[0].f()} = (${as[7].f()} - ${as[5].f()}) * s;
				${ds[1].f()} = (${as[2].f()} - ${as[6].f()}) * s;
				${ds[2].f()} = (${as[3].f()} - ${as[1].f()}) * s;
			} else if (e00 > e11 && e00 > e22) { // e00 is the largest
				s = oimo.math.MathUtil.sqrt(e00 - e11 - e22 + 1);
				${ds[0].f()} = 0.5 * s;
				s = 0.5 / s;
				${ds[1].f()} = (${as[1].f()} + ${as[3].f()}) * s;
				${ds[2].f()} = (${as[2].f()} + ${as[6].f()}) * s;
				${ds[3].f()} = (${as[7].f()} - ${as[5].f()}) * s;
			} else if (e11 > e22) { // e11 is the largest
				s = oimo.math.MathUtil.sqrt(e11 - e22 - e00 + 1);
				${ds[1].f()} = 0.5 * s;
				s = 0.5 / s;
				${ds[0].f()} = (${as[1].f()} + ${as[3].f()}) * s;
				${ds[2].f()} = (${as[5].f()} + ${as[7].f()}) * s;
				${ds[3].f()} = (${as[2].f()} - ${as[6].f()}) * s;
			} else { // e22 is the largest
				s = oimo.math.MathUtil.sqrt(e22 - e00 - e11 + 1);
				${ds[2].f()} = 0.5 * s;
				s = 0.5 / s;
				${ds[0].f()} = (${as[2].f()} + ${as[6].f()}) * s;
				${ds[1].f()} = (${as[5].f()} + ${as[7].f()}) * s;
				${ds[3].f()} = (${as[3].f()} - ${as[1].f()}) * s;
			}
		}
	}

	public static macro function quat_scale(dst:Expr, src1:Expr, src2:ExprOf<Float>) {
		return binOpVars(dst.s().quatNames(), src1.s().quatNames(), [src2, src2, src2, src2], OpMult);
	}

	public static macro function quat_lengthSq(src:Expr) {
		var as = src.s().quatNames();
		return macro ${as[0].f()} * ${as[0].f()} + ${as[1].f()} * ${as[1].f()} + ${as[2].f()} * ${as[2].f()} + ${as[3].f()} * ${as[3].f()};
	}

	public static macro function quat_normalize(dst:Expr, src:Expr) {
		var as = src.s().quatNames();
		return macro {
			var l:Float = M.quat_lengthSq($src);
			if (!M.eq0(l)) l = 1 / oimo.math.MathUtil.sqrt(l);
			M.quat_scale($dst, $src, l);
		}
	}

	public static macro function quat_mul(dst:Expr, src1:Expr, src2:Expr) {
		var as = src1.s().quatNames();
		var bs = src2.s().quatNames();
		return assignVars(dst.s().quatNames(), [
			macro ${as[3].f()} * ${bs[0].f()} + ${as[0].f()} * ${bs[3].f()} + ${as[1].f()} * ${bs[2].f()} - ${as[2].f()} * ${bs[1].f()},
			macro ${as[3].f()} * ${bs[1].f()} - ${as[0].f()} * ${bs[2].f()} + ${as[1].f()} * ${bs[3].f()} + ${as[2].f()} * ${bs[0].f()},
			macro ${as[3].f()} * ${bs[2].f()} + ${as[0].f()} * ${bs[1].f()} - ${as[1].f()} * ${bs[0].f()} + ${as[2].f()} * ${bs[3].f()},
			macro ${as[3].f()} * ${bs[3].f()} - ${as[0].f()} * ${bs[0].f()} - ${as[1].f()} * ${bs[1].f()} - ${as[2].f()} * ${bs[2].f()}
		]);
	}

	// ---------------------------------------------------------------------
	// AABB
	// ---------------------------------------------------------------------

	public static macro function aabb_isOverlapped(min1:Expr, max1:Expr, min2:Expr, max2:Expr) {
		var mi1:Array<String> = min1.s().vec3Names();
		var ma1:Array<String> = max1.s().vec3Names();
		var mi2:Array<String> = min2.s().vec3Names();
		var ma2:Array<String> = max2.s().vec3Names();
		return macro
			${mi1[0].f()} < ${ma2[0].f()} && ${ma1[0].f()} > ${mi2[0].f()} &&
			${mi1[1].f()} < ${ma2[1].f()} && ${ma1[1].f()} > ${mi2[1].f()} &&
			${mi1[2].f()} < ${ma2[2].f()} && ${ma1[2].f()} > ${mi2[2].f()}
		;
	}

	// ---------------------------------------------------------------------
	// Call
	// ---------------------------------------------------------------------

	public static macro function call(e:Expr) {
		switch(e.expr) {
		case ECall(func, params):
			var newParams:Array<Expr> = [];
			for (param in params) {
				var names:Array<String> = U.namesE(param);
				if (names != null) {
					for (name in names) {
						newParams.push(name.e());
					}
				} else {
					newParams.push(param);
				}
			}
			e.expr = ECall(func, newParams);
			return macro $e;
		case _:
			Context.error("invalid call", U.pos());
			return macro $e;
		}
	}

#if macro

	// ---------------------------------------------------------------------
	// local
	// ---------------------------------------------------------------------

	static function t(e:Expr):Type {
		return Context.typeof(e);
	}

	static function s(e:Expr):String {
		return e.toString();
	}

	static inline function f(s:String):Expr {
		return macro $p{s.split(".")};
	}

	static inline function lf(f:Float):Expr {
		return macro $v{f};
	}

	static function defineVars(names:Array<String>, type:ComplexType) {
		var es:Array<Expr> = [];
		for (name in names) {
			es.push(macro var $name);
		}
		var vs:Array<Var> = [];
		for (e in es) {
			switch(e.expr) {
			case EVars(vars):
				vs = vs.concat(vars.map(function(v) { v.type = type; return v; }));
			case _:
			}
		}
		return EVars(vs).toExpr();
	}

	static function assignVars(lhs:Array<Dynamic>, rhs:Array<Dynamic>) {
		return assignVarsExpr(lhs.toExprArray(), rhs.toExprArray());
	}

	/**
	 * lhs = rhs
	 */
	static function assignVarsExpr(lhs:Array<Expr>, rhs:Array<Expr>) {
		var es:Array<Expr> = [];
		var num:Int = lhs.length;
		for (i in 0...num) {
			es.push(macro ${lhs[i]} = ${rhs[i]});
		}
		return macro $b{es};
	}

	static function binOpVars(dst:Array<Dynamic>, src1:Array<Dynamic>, src2:Array<Dynamic>, op:Binop) {
		return binOpVarsExpr(dst.toExprArray(), src1.toExprArray(), src2.toExprArray(), op);
	}

	/**
	 * dst = src1 op src2
	 */
	static function binOpVarsExpr(dst:Array<Expr>, src1:Array<Expr>, src2:Array<Expr>, op:Binop) {
		var es:Array<Expr> = [];
		var num:Int = dst.length;
		for (i in 0...num) {
			es.push(macro ${dst[i]} = ${EBinop(op, src1[i], src2[i]).toExpr()});
		}
		return macro $b{es};
	}

	static function binOp2RLVars(dst:Array<Dynamic>, src1:Array<Dynamic>, src2:Array<Dynamic>, src3:Array<Dynamic>, op:Binop, op2:Binop) {
		return binOp2VarsRLExpr(dst.toExprArray(), src1.toExprArray(), src2.toExprArray(), src3.toExprArray(), op, op2);
	}

	/**
	 * dst = src1 op (src2 op2 src3)
	 */
	static function binOp2VarsRLExpr(dst:Array<Expr>, src1:Array<Expr>, src2:Array<Expr>, src3:Array<Expr>, op:Binop, op2:Binop) {
		var es:Array<Expr> = [];
		var num:Int = dst.length;
		for (i in 0...num) {
			es.push(macro ${dst[i]} = ${EBinop(op, src1[i], EBinop(op2, src2[i], src3[i]).toExpr()).toExpr()});
		}
		return macro $b{es};
	}

	static function toExprArray(a:Array<Dynamic>):Array<Expr> {
		return cast (Std.is(a[0], String) ? a.map(f) : a);
	}

	static function toExpr(def:ExprDef):Expr {
		return {
			expr: def,
			pos: U.pos()
		}
	}

#end

}
