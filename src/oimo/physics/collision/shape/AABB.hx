package oimo.physics.collision.shape;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Vec3;

/**
 * Axis-Aligned Bounding Box.
 */
@:expose("OIMO.AABB")
@:build(oimo.m.B.bu())
class AABB {
	public var _min:IVec3;
	public var _max:IVec3;

	public function new() {
		M.vec3_zero(_min);
		M.vec3_zero(_max);
	}

	/**
	 * Returns the minimum point of the axis-aligned bounding box.
	 */
	public function getMin():Vec3 {
		var min:Vec3 = new Vec3();
		M.vec3_toVec3(min, _min);
		return min;
	}

	/**
	 * Sets the minimum point of the axis-aligned bounding box to `min`.
	 */
	public function setMin(min:Vec3):Void {
		M.vec3_fromVec3(_min, min);
	}

	/**
	 * Returns the maximum point of the axis-aligned bounding box.
	 */
	public function getMax():Vec3 {
		var max:Vec3 = new Vec3();
		M.vec3_toVec3(max, _max);
		return max;
	}

	/**
	 * Sets the maximum point of the axis-aligned bounding box to `max`.
	 */
	public function setMax(max:Vec3):Void {
		M.vec3_fromVec3(_max, max);
	}

}
