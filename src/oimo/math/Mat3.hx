package oimo.math;
import oimo.m.M;
import oimo.math.MathUtil;
// using M; // mixing it in causes errors :(

/**
 * 3x3 Matrix class.
 *
 * Note that columns and rows are 0-indexed.
 */
@:expose("OIMO.Mat3")
class Mat3 {
	/**
	 * The element at row 0 column 0.
	 */
	public var e00:Float;

	/**
	 * The element at row 0 column 1.
	 */
	public var e01:Float;

	/**
	 * The element at row 0 column 2.
	 */
	public var e02:Float;

	/**
	 * The element at row 1 column 0.
	 */
	public var e10:Float;

	/**
	 * The element at row 1 column 1.
	 */
	public var e11:Float;

	/**
	 * The element at row 1 column 2.
	 */
	public var e12:Float;

	/**
	 * The element at row 2 column 0.
	 */
	public var e20:Float;

	/**
	 * The element at row 2 column 1.
	 */
	public var e21:Float;

	/**
	 * The element at row 2 column 2.
	 */
	public var e22:Float;

	/**
	 * Creates a new matrix. The matrix is identity by default.
	 */
	public inline function new(
		e00:Float = 1, e01:Float = 0, e02:Float = 0,
		e10:Float = 0, e11:Float = 1, e12:Float = 0,
		e20:Float = 0, e21:Float = 0, e22:Float = 1
	) {
		this.e00 = e00;
		this.e01 = e01;
		this.e02 = e02;
		this.e10 = e10;
		this.e11 = e11;
		this.e12 = e12;
		this.e20 = e20;
		this.e21 = e21;
		this.e22 = e22;
	}

	/**
	 * Sets all elements at once and returns `this`.
	 */
	public inline function init(
		e00:Float, e01:Float, e02:Float,
		e10:Float, e11:Float, e12:Float,
		e20:Float, e21:Float, e22:Float
	):Mat3 {
		this.e00 = e00;
		this.e01 = e01;
		this.e02 = e02;
		this.e10 = e10;
		this.e11 = e11;
		this.e12 = e12;
		this.e20 = e20;
		this.e21 = e21;
		this.e22 = e22;
		return this;
	}

	@:extern
	inline function initi(
		e00:Float, e01:Float, e02:Float,
		e10:Float, e11:Float, e12:Float,
		e20:Float, e21:Float, e22:Float
	):Mat3 {
		var t00:Float = e00;
		var t01:Float = e01;
		var t02:Float = e02;
		var t10:Float = e10;
		var t11:Float = e11;
		var t12:Float = e12;
		var t20:Float = e20;
		var t21:Float = e21;
		var t22:Float = e22;
		this.e00 = t00;
		this.e01 = t01;
		this.e02 = t02;
		this.e10 = t10;
		this.e11 = t11;
		this.e12 = t12;
		this.e20 = t20;
		this.e21 = t21;
		this.e22 = t22;
		return this;
	}

	/**
	 * Sets this matrix to identity matrix and returns `this`.
	 */
	public inline function identity():Mat3 {
		initi(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		);
		return this;
	}

	/**
	 * Returns `this` + `m`
	 */
	public inline function add(m:Mat3):Mat3 {
		return new Mat3(
			e00 + m.e00, e01 + m.e01, e02 + m.e02,
			e10 + m.e10, e11 + m.e11, e12 + m.e12,
			e20 + m.e20, e21 + m.e21, e22 + m.e22
		);
	}

	/**
	 * Returns `this` - `m`
	 */
	public inline function sub(m:Mat3):Mat3 {
		return new Mat3(
			e00 - m.e00, e01 - m.e01, e02 - m.e02,
			e10 - m.e10, e11 - m.e11, e12 - m.e12,
			e20 - m.e20, e21 - m.e21, e22 - m.e22
		);
	}

	/**
	 * Returns `this` * `s`
	 */
	public inline function scale(s:Float):Mat3 {
		return new Mat3(
			e00 * s, e01 * s, e02 * s,
			e10 * s, e11 * s, e12 * s,
			e20 * s, e21 * s, e22 * s
		);
	}

	/**
	 * Returns `this` * `m`
	 */
	public inline function mul(m:Mat3):Mat3 {
		return new Mat3(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22
		);
	}

	/**
	 * Sets this matrix to `this` + `m` and returns `this`.
	 */
	public inline function addEqual(m:Mat3):Mat3 {
		return initi(
			e00 + m.e00, e01 + m.e01, e02 + m.e02,
			e10 + m.e10, e11 + m.e11, e12 + m.e12,
			e20 + m.e20, e21 + m.e21, e22 + m.e22
		);
	}

	/**
	 * Sets this matrix to `this` - `m` and returns `this`.
	 */
	public inline function subEqual(m:Mat3):Mat3 {
		return initi(
			e00 - m.e00, e01 - m.e01, e02 - m.e02,
			e10 - m.e10, e11 - m.e11, e12 - m.e12,
			e20 - m.e20, e21 - m.e21, e22 - m.e22
		);
	}

	/**
	 * Sets this matrix to `this` * `s` and returns `this`.
	 */
	public inline function scaleEqual(s:Float):Mat3 {
		return initi(
			e00 * s, e01 * s, e02 * s,
			e10 * s, e11 * s, e12 * s,
			e20 * s, e21 * s, e22 * s
		);
	}

	/**
	 * Sets this matrix to `this` * `m` and returns `this`.
	 */
	public inline function mulEqual(m:Mat3):Mat3 {
		return initi(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22
		);
	}

	/**
	 * Returns *scaling matrix* * `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public inline function prependScale(sx:Float, sy:Float, sz:Float):Mat3 {
		return new Mat3(
			e00 * sx, e01 * sx, e02 * sx,
			e10 * sy, e11 * sy, e12 * sy,
			e20 * sz, e21 * sz, e22 * sz
		);
	}

	/**
	 * Returns `this` * *scaling matrix*.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public inline function appendScale(sx:Float, sy:Float, sz:Float):Mat3 {
		return new Mat3(
			e00 * sx, e01 * sy, e02 * sz,
			e10 * sx, e11 * sy, e12 * sz,
			e20 * sx, e21 * sy, e22 * sz
		);
	}

	/**
	 * Returns *rotation matrix* * `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public inline function prependRotation(rad:Float, axisX:Float, axisY:Float, axisZ:Float):Mat3 {
		var s:Float = MathUtil.sin(rad);
		var c:Float = MathUtil.cos(rad);
		var c1:Float = 1 - c;
		var r00:Float = axisX * axisX * c1 + c;
		var r01:Float = axisX * axisY * c1 - axisZ * s;
		var r02:Float = axisX * axisZ * c1 + axisY * s;
		var r10:Float = axisY * axisX * c1 + axisZ * s;
		var r11:Float = axisY * axisY * c1 + c;
		var r12:Float = axisY * axisZ * c1 - axisX * s;
		var r20:Float = axisZ * axisX * c1 - axisY * s;
		var r21:Float = axisZ * axisY * c1 + axisX * s;
		var r22:Float = axisZ * axisZ * c1 + c;
		return new Mat3(
			r00 * e00 + r01 * e10 + r02 * e20,
			r00 * e01 + r01 * e11 + r02 * e21,
			r00 * e02 + r01 * e12 + r02 * e22,
			r10 * e00 + r11 * e10 + r12 * e20,
			r10 * e01 + r11 * e11 + r12 * e21,
			r10 * e02 + r11 * e12 + r12 * e22,
			r20 * e00 + r21 * e10 + r22 * e20,
			r20 * e01 + r21 * e11 + r22 * e21,
			r20 * e02 + r21 * e12 + r22 * e22
		);
	}

	/**
	 * Returns `this` * *rotation matrix*.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public inline function appendRotation(rad:Float, axisX:Float, axisY:Float, axisZ:Float):Mat3 {
		var s:Float = MathUtil.sin(rad);
		var c:Float = MathUtil.cos(rad);
		var c1:Float = 1 - c;
		var r00:Float = axisX * axisX * c1 + c;
		var r01:Float = axisX * axisY * c1 - axisZ * s;
		var r02:Float = axisX * axisZ * c1 + axisY * s;
		var r10:Float = axisY * axisX * c1 + axisZ * s;
		var r11:Float = axisY * axisY * c1 + c;
		var r12:Float = axisY * axisZ * c1 - axisX * s;
		var r20:Float = axisZ * axisX * c1 - axisY * s;
		var r21:Float = axisZ * axisY * c1 + axisX * s;
		var r22:Float = axisZ * axisZ * c1 + c;
		return new Mat3(
			e00 * r00 + e01 * r10 + e02 * r20,
			e00 * r01 + e01 * r11 + e02 * r21,
			e00 * r02 + e01 * r12 + e02 * r22,
			e10 * r00 + e11 * r10 + e12 * r20,
			e10 * r01 + e11 * r11 + e12 * r21,
			e10 * r02 + e11 * r12 + e12 * r22,
			e20 * r00 + e21 * r10 + e22 * r20,
			e20 * r01 + e21 * r11 + e22 * r21,
			e20 * r02 + e21 * r12 + e22 * r22
		);
	}

	/**
	 * Sets this matrix to *scaling matrix* * `this`, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public inline function prependScaleEqual(sx:Float, sy:Float, sz:Float):Mat3 {
		return initi(
			e00 * sx, e01 * sx, e02 * sx,
			e10 * sy, e11 * sy, e12 * sy,
			e20 * sz, e21 * sz, e22 * sz
		);
	}

	/**
	 * Sets this matrix to `this` * *scaling matrix*, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public inline function appendScaleEqual(sx:Float, sy:Float, sz:Float):Mat3 {
		return initi(
			e00 * sx, e01 * sy, e02 * sz,
			e10 * sx, e11 * sy, e12 * sz,
			e20 * sx, e21 * sy, e22 * sz
		);
	}

	/**
	 * Sets this matrix to *rotation matrix* * `this`, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public inline function prependRotationEqual(rad:Float, axisX:Float, axisY:Float, axisZ:Float):Mat3 {
		var s:Float = MathUtil.sin(rad);
		var c:Float = MathUtil.cos(rad);
		var c1:Float = 1 - c;
		var r00:Float = axisX * axisX * c1 + c;
		var r01:Float = axisX * axisY * c1 - axisZ * s;
		var r02:Float = axisX * axisZ * c1 + axisY * s;
		var r10:Float = axisY * axisX * c1 + axisZ * s;
		var r11:Float = axisY * axisY * c1 + c;
		var r12:Float = axisY * axisZ * c1 - axisX * s;
		var r20:Float = axisZ * axisX * c1 - axisY * s;
		var r21:Float = axisZ * axisY * c1 + axisX * s;
		var r22:Float = axisZ * axisZ * c1 + c;
		return initi(
			r00 * e00 + r01 * e10 + r02 * e20,
			r00 * e01 + r01 * e11 + r02 * e21,
			r00 * e02 + r01 * e12 + r02 * e22,
			r10 * e00 + r11 * e10 + r12 * e20,
			r10 * e01 + r11 * e11 + r12 * e21,
			r10 * e02 + r11 * e12 + r12 * e22,
			r20 * e00 + r21 * e10 + r22 * e20,
			r20 * e01 + r21 * e11 + r22 * e21,
			r20 * e02 + r21 * e12 + r22 * e22
		);
	}

	/**
	 * Sets this matrix to `this` * *rotation matrix*, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public inline function appendRotationEqual(rad:Float, axisX:Float, axisY:Float, axisZ:Float):Mat3 {
		var s:Float = MathUtil.sin(rad);
		var c:Float = MathUtil.cos(rad);
		var c1:Float = 1 - c;
		var r00:Float = axisX * axisX * c1 + c;
		var r01:Float = axisX * axisY * c1 - axisZ * s;
		var r02:Float = axisX * axisZ * c1 + axisY * s;
		var r10:Float = axisY * axisX * c1 + axisZ * s;
		var r11:Float = axisY * axisY * c1 + c;
		var r12:Float = axisY * axisZ * c1 - axisX * s;
		var r20:Float = axisZ * axisX * c1 - axisY * s;
		var r21:Float = axisZ * axisY * c1 + axisX * s;
		var r22:Float = axisZ * axisZ * c1 + c;
		return initi(
			e00 * r00 + e01 * r10 + e02 * r20,
			e00 * r01 + e01 * r11 + e02 * r21,
			e00 * r02 + e01 * r12 + e02 * r22,
			e10 * r00 + e11 * r10 + e12 * r20,
			e10 * r01 + e11 * r11 + e12 * r21,
			e10 * r02 + e11 * r12 + e12 * r22,
			e20 * r00 + e21 * r10 + e22 * r20,
			e20 * r01 + e21 * r11 + e22 * r21,
			e20 * r02 + e21 * r12 + e22 * r22
		);
	}

	/**
	 * Returns the transposed matrix.
	 */
	public inline function transpose():Mat3 {
		return new Mat3(
			e00, e10, e20,
			e01, e11, e21,
			e02, e12, e22
		);
	}

	/**
	 * Sets this matrix to the transposed matrix and returns `this`.
	 */
	public inline function transposeEqual():Mat3 {
		return initi(
			e00, e10, e20,
			e01, e11, e21,
			e02, e12, e22
		);
	}

	/**
	 * Returns the determinant.
	 */
	public inline function determinant():Float {
		return e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
	}

	/**
	 * Returns the trace.
	 */
	public inline function trace():Float {
		return e00 + e11 + e22;
	}

	/**
	 * Returns the inverse matrix.
	 *
	 * If the determinant is zero, zero matrix is returned.
	 */
	public inline function invert():Mat3 {
		var d00:Float = e11 * e22 - e12 * e21;
		var d01:Float = e10 * e22 - e12 * e20;
		var d02:Float = e10 * e21 - e11 * e20;
		var d10:Float = e01 * e22 - e02 * e21;
		var d11:Float = e00 * e22 - e02 * e20;
		var d12:Float = e00 * e21 - e01 * e20;
		var d20:Float = e01 * e12 - e02 * e11;
		var d21:Float = e00 * e12 - e02 * e10;
		var d22:Float = e00 * e11 - e01 * e10;
		var invDet:Float = e00 * d00 - e01 * d01 + e02 * d02;
		if (!M.eq0(invDet)) invDet = 1 / invDet;
		return new Mat3(
			d00 * invDet, -d10 * invDet, d20 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet
		);
	}

	/**
	 * Sets this matrix to the inverse matrix and returns `this`.
	 *
	 * If the determinant is zero, this matrix is set to zero matrix.
	 */
	public inline function invertEqual():Mat3 {
		var d00:Float = e11 * e22 - e12 * e21;
		var d01:Float = e10 * e22 - e12 * e20;
		var d02:Float = e10 * e21 - e11 * e20;
		var d10:Float = e01 * e22 - e02 * e21;
		var d11:Float = e00 * e22 - e02 * e20;
		var d12:Float = e00 * e21 - e01 * e20;
		var d20:Float = e01 * e12 - e02 * e11;
		var d21:Float = e00 * e12 - e02 * e10;
		var d22:Float = e00 * e11 - e01 * e10;
		var invDet:Float = e00 * d00 - e01 * d01 + e02 * d02;
		if (!M.eq0(invDet)) invDet = 1 / invDet;
		return initi(
			d00 * invDet, -d10 * invDet, d20 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet
		);
	}

	/**
	 * Returns an array of the elements of this matrix.
	 *
	 * If `columnMajor` is true, the array is arranged in column-major order.
	 * Otherwise, the array is arranged in row-major order.
	 */
	public inline function toArray(columnMajor:Bool = false):Array<Float> {
		if (columnMajor) {
			return [
				e00, e10, e20,
				e01, e11, e21,
				e02, e12, e22
			];
		} else {
			return [
				e00, e01, e02,
				e10, e11, e12,
				e20, e21, e22
			];
		}
	}

	/**
	 * Copies values from `m` and returns `this`.
	 */
	public inline function copyFrom(m:Mat3):Mat3 {
		e00 = m.e00;
		e01 = m.e01;
		e02 = m.e02;
		e10 = m.e10;
		e11 = m.e11;
		e12 = m.e12;
		e20 = m.e20;
		e21 = m.e21;
		e22 = m.e22;
		return this;
	}

	/**
	 * Returns a clone of the matrix.
	 */
	public inline function clone():Mat3 {
		return new Mat3(
			e00, e01, e02,
			e10, e11, e12,
			e20, e21, e22
		);
	}

	/**
	 * Sets this matrix to the representation of the quaternion `q`, and returns `this`.
	 */
	public inline function fromQuaternion(q:Quat):Mat3 {
		var x:Float = q.x;
		var y:Float = q.y;
		var z:Float = q.z;
		var w:Float = q.w;
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
		e00 = 1 - yy - zz;
		e01 = xy - wz;
		e02 = xz + wy;
		e10 = xy + wz;
		e11 = 1 - xx - zz;
		e12 = yz - wx;
		e20 = xz - wy;
		e21 = yz + wx;
		e22 = 1 - xx - yy;
		return this;
	}

	/**
	 * Returns a quaternion which represents this matrix.
	 *
	 * This matrix must be a rotation matrix, that is, must be orthogonalized and have determinant 1.
	 */
	public inline function toQuaternion():Quat {
		return new Quat().fromMatrix(this);
	}

	public function toString():String {
		return
			"[" + M.toFixed4(e00) + ", " + M.toFixed4(e01) + ", " + M.toFixed4(e02) + ";\n" +
			" " + M.toFixed4(e10) + ", " + M.toFixed4(e11) + ", " + M.toFixed4(e12) + ";\n" +
			" " + M.toFixed4(e20) + ", " + M.toFixed4(e22) + ", " + M.toFixed4(e22) + "]"
		;
	}

}
