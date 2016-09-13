package oimo.physics.collision.shape;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.physics.Settings;

/**
 * Sphere shape
 */
@:expose("OIMO.SphereShape")
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

}