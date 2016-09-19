package oimo.physics.collision.shape;
import oimo.m.ITransform;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.math.Vec3;
import oimo.physics.Settings;
import oimo.physics.collision.broadphase.Proxy;

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

	override public function _computeAABB(aabb:AABB, tf:ITransform):Void {
		var x:IVec3;
		var y:IVec3;
		var z:IVec3;
		M.vec3_set(x, M.vec3_get(_halfExtents, 0), 0, 0);
		M.vec3_set(y, 0, M.vec3_get(_halfExtents, 1), 0);
		M.vec3_set(z, 0, 0, M.vec3_get(_halfExtents, 2));

		var tfx:IVec3;
		var tfy:IVec3;
		var tfz:IVec3;
		M.vec3_mulMat3(tfx, x, tf);
		M.vec3_mulMat3(tfy, y, tf);
		M.vec3_mulMat3(tfz, z, tf);
		M.vec3_abs(tfx, tfx);
		M.vec3_abs(tfy, tfy);
		M.vec3_abs(tfz, tfz);
		var tfs:IVec3;
		M.vec3_add(tfs, tfx, tfy);
		M.vec3_add(tfs, tfs, tfz);

		M.vec3_sub(aabb._min, tf, tfs);
		M.vec3_add(aabb._max, tf, tfs);
	}

}
