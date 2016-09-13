package oimo.physics.collision.broadphase;
import oimo.math.Vec3;

/**
 * Axis-Aligned Bounding Box.
 */
class AABB {
	public var min:Vec3;
	public var max:Vec3;

	public function new() {
		min = new Vec3();
		max = new Vec3();
	}

}