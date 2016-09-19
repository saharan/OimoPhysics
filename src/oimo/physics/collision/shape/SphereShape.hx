package oimo.physics.collision.shape;
import haxe.macro.Expr;
import oimo.m.ITransform;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.physics.Settings;
import oimo.physics.collision.broadphase.Proxy;

/**
 * Sphere shape
 */
@:expose("OIMO.SphereShape")
@:build(oimo.m.B.build())
class SphereShape extends Shape {
	public var _radius:Float;

	public function new(radius:Float) {
		super(Sphere);
		_radius = radius;
	}

	override public function _updateMass():Void {
		_volume = 4 / 3 * MathUtil.PI * _radius * _radius * _radius;
		M.mat3_diagonal(_inertiaCoeff,
			2 / 5 * _radius * _radius,
			2 / 5 * _radius * _radius,
			2 / 5 * _radius * _radius
		);
	}

	override public function _computeAABB(aabb:AABB, tf1:ITransform, tf2:ITransform):Void {
		var min:IVec3;
		var max:IVec3;
		var radVec:IVec3;
		M.vec3_set(radVec, _radius, _radius, _radius);
		M.vec3_min(min, tf1, tf2);
		M.vec3_max(max, tf1, tf2);
		M.vec3_sub(aabb._min, min, radVec);
		M.vec3_add(aabb._max, max, radVec);
	}

}