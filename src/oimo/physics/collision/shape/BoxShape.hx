package oimo.physics.collision.shape;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.math.Vec3;
import oimo.physics.Settings;

/**
 * Box shape
 */
@:expose("OIMO.BoxShape")
@:build(oimo.m.B.build())
class BoxShape extends Shape {
	public var _halfExtents:IVec3;

	public function new(halfExtents:Vec3) {
		super(Box);
		M.vec3_fromVec3(_halfExtents, halfExtents);
	}

	override public function _updateMass():Void {
		_volume = 8 * M.vec3_mulHorizontal(_halfExtents);
		var sq:IVec3;
		M.vec3_dot(_halfExtents, _halfExtents);
		M.mat3_diagonal(_inertiaCoeff,
			1 / 3 * (M.vec3_get(sq, 1) + M.vec3_get(sq, 2)),
			1 / 3 * (M.vec3_get(sq, 2) + M.vec3_get(sq, 0)),
			1 / 3 * (M.vec3_get(sq, 0) + M.vec3_get(sq, 1))
		);
	}

}