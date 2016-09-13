package oimo.math;
import oimo.m.M;
import oimo.math.MathUtil;
// using M; // mixing it in causes errors :(

/**
 * 3D vector class.
 */
@:expose("OIMO.Vec3")
class Vec3 {
	/**
	 * The x-value of the vector.
	 */
	public var x:Float;

	/**
	 * The y-value of the vector.
	 */
	public var y:Float;

	/**
	 * The z-value of the vector.
	 */
	public var z:Float;

	/**
	 * Creates a new vector. The vector is zero vector by default.
	 */
	public inline function new(x:Float = 0, y:Float = 0, z:Float = 0) {
		this.x = x;
		this.y = y;
		this.z = z;
	}

	/**
	 * Sets all values at once and returns `this`.
	 */
	public function init(x:Float, y:Float, z:Float):Vec3 {
		this.x = x;
		this.y = y;
		this.z = z;
		return this;
	}

	@:extern
	inline function initi(x:Float, y:Float, z:Float):Vec3 {
		var tx:Float = x;
		var ty:Float = y;
		var tz:Float = z;
		this.x = tx;
		this.y = ty;
		this.z = tz;
		return this;
	}

	/**
	 * Returns `this` + `v`.
	 */
	public inline function add(v:Vec3):Vec3 {
		return new Vec3(x + v.x, y + v.y, z + v.z);
	}

	/**
	 * Returns `this` - `v`.
	 */
	public inline function sub(v:Vec3):Vec3 {
		return new Vec3(x - v.x, y - v.y, z - v.z);
	}

	/**
	 * Returns `this` * `s`.
	 */
	public inline function scale(s:Float):Vec3 {
		return new Vec3(x * s, y * s, z * s);
	}

	/**
	 * Returns the dot product of `this` and `v`.
	 */
	public inline function dot(v:Vec3):Float {
		return x * v.x + y * v.y + z * v.z;
	}

	/**
	 * Returns the cross product of `this` and `v`.
	 */
	public inline function cross(v:Vec3):Vec3 {
		return new Vec3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}

	/**
	 * Sets this vector to `this` + `v` and returns `this`.
	 */
	public inline function addEqual(v:Vec3):Vec3 {
		return initi(x + v.x, y + v.y, z + v.z);
	}

	/**
	 * Sets this vector to `this` - `v` and returns `this`.
	 */
	public inline function subEqual(v:Vec3):Vec3 {
		return initi(x - v.x, y - v.y, z - v.z);
	}

	/**
	 * Sets this vector to `this` * `s` and returns `this`.
	 */
	public inline function scaleEqual(s:Float):Vec3 {
		return initi(x * s, y * s, z * s);
	}

	/**
	 * Sets this vector to the cross product of `this` and `s`, and returns `this`.
	 */
	public inline function crossEqual(v:Vec3):Vec3 {
		return initi(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}

	/**
	 * Returns the length of the vector.
	 */
	public inline function length():Float {
		return MathUtil.sqrt(x * x + y * y + z * z);
	}

	/**
	 * Returns the squared length of the vector.
	 */
	public inline function lengthSq():Float {
		return x * x + y * y + z * z;
	}

	/**
	 * Returns the normalized vector.
	 *
	 * If the length is zero, zero vector is returned.
	 */
	public inline function normalize():Vec3 {
		var invLen:Float = length();
		if (M.gt0(invLen)) invLen = 1 / invLen;
		return new Vec3(x * invLen, y * invLen, z * invLen);
	}

	/**
	 * Sets this vector to the normalized vector and returns `this`.
	 *
	 * If the length is zero, this vector is set to zero vector.
	 */
	public inline function normalizeEqual():Vec3 {
		var invLen:Float = length();
		if (M.gt0(invLen)) invLen = 1 / invLen;
		return initi(x * invLen, y * invLen, z * invLen);
	}

	/**
	 * Copies values from `v` and returns `this`.
	 */
	public inline function copyFrom(v:Vec3):Vec3 {
		x = v.x;
		y = v.y;
		z = v.z;
		return this;
	}

	/**
	 * Returns a clone of the vector.
	 */
	public inline function clone():Vec3 {
		return new Vec3(x, y, z);
	}

	public function toString():String {
		return
			"[" + M.toFixed4(x) + ";\n" +
			" " + M.toFixed4(y) + ";\n" +
			" " + M.toFixed4(z) + "]"
		;
	}

}