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

	override public function _computeAABB(aabb:AABB, tf1:ITransform, tf2:ITransform):Void {
		var x:IVec3;
		var y:IVec3;
		var z:IVec3;
		M.vec3_set(x, M.vec3_get(_halfExtents, 0), 0, 0);
		M.vec3_set(y, 0, M.vec3_get(_halfExtents, 1), 0);
		M.vec3_set(z, 0, 0, M.vec3_get(_halfExtents, 2));

		var tf1x:IVec3;
		var tf1y:IVec3;
		var tf1z:IVec3;
		M.vec3_mulMat3(tf1x, x, tf1);
		M.vec3_mulMat3(tf1y, y, tf1);
		M.vec3_mulMat3(tf1z, z, tf1);
		M.vec3_abs(tf1x, tf1x);
		M.vec3_abs(tf1y, tf1y);
		M.vec3_abs(tf1z, tf1z);
		var tf1s:IVec3;
		M.vec3_add(tf1s, tf1x, tf1y);
		M.vec3_add(tf1s, tf1s, tf1z);

		var tf2x:IVec3;
		var tf2y:IVec3;
		var tf2z:IVec3;
		M.vec3_mulMat3(tf2x, x, tf2);
		M.vec3_mulMat3(tf2y, y, tf2);
		M.vec3_mulMat3(tf2z, z, tf2);
		M.vec3_abs(tf2x, tf2x);
		M.vec3_abs(tf2y, tf2y);
		M.vec3_abs(tf2z, tf2z);
		var tf2s:IVec3;
		M.vec3_add(tf2s, tf2x, tf2y);
		M.vec3_add(tf2s, tf2s, tf2z);

		var max:IVec3;
		var min:IVec3;

		var min1:IVec3;
		var min2:IVec3;
		var max1:IVec3;
		var max2:IVec3;
		M.vec3_sub(min1, tf1, tf1s);
		M.vec3_add(max1, tf1, tf1s);
		M.vec3_sub(min2, tf2, tf2s);
		M.vec3_add(max2, tf2, tf2s);
		M.vec3_min(aabb._min, min1, min2);
		M.vec3_max(aabb._max, max1, max2);
	}

}
