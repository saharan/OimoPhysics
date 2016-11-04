package oimo.physics.dynamics.rigidbody;
import oimo.math.Mat3;
import oimo.math.Vec3;
import oimo.physics.collision.shape.Transform;
import oimo.physics.Settings;
import oimo.physics.collision.shape.Shape;

/**
 * Used for construction of Component.
 */
@:expose("OIMO.ComponentConfig")
class ComponentConfig {
	public var position:Vec3;
	public var rotation:Mat3;
	public var friction:Float;
	public var restitution:Float;
	public var density:Float;
	public var shape:Shape;

	public function new() {
		position = new Vec3();
		rotation = new Mat3();
		friction = Settings.defaultFriction;
		restitution = Settings.defaultRestitution;
		density = Settings.defaultDensity;
		shape = null;
	}
}
