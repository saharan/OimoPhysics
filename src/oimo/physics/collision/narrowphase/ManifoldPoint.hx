package oimo.physics.collision.narrowphase;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * point of a contact manifold
 */
@:expose("OIMO.ManifoldPoint")
@:build(oimo.m.B.bu())
class ManifoldPoint {
	public var _relPos1:IVec3;
	public var _relPos2:IVec3;
	public var _penetration:Float;

	public function new() {
		M.vec3_zero(_relPos1);
		M.vec3_zero(_relPos2);
		_penetration = 0;
	}
}
