package oimo.physics.collision.shape;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;

/**
 * Sphere shape
 */
@:expose("OIMO.SphereShape")
@:build(oimo.m.B.bu())
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

	override public function _computeAABB(aabb:AABB, tf:Transform):Void {
		var radVec:IVec3;
		M.vec3_set(radVec, _radius, _radius, _radius);
		M.vec3_sub(aabb._min, tf._origin, radVec);
		M.vec3_add(aabb._max, tf._origin, radVec);
	}

}
