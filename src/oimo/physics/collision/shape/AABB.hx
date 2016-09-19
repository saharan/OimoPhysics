package oimo.physics.collision.shape;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Vec3;

/**
 * Axis-Aligned Bounding Box.
 */
@:expose("OIMO.AABB")
@:build(oimo.m.B.build())
class AABB {
	public var _min:IVec3;
	public var _max:IVec3;

	public function new() {
		M.vec3_zero(_min);
		M.vec3_zero(_max);
	}

}
