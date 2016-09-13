package oimo.physics.dynamics;
import oimo.math.Transform;
import oimo.physics.Settings;
import oimo.physics.collision.shape.Shape;

/**
 * Used for construction of Component.
 */
@:expose("OIMO.ComponentConfig")
class ComponentConfig {
	public var transform:Transform;
	public var friction:Float;
	public var restitution:Float;
	public var density:Float;
	public var shape:Shape;

	public function new() {
		transform = new Transform();
		friction = Settings.defaultFriction;
		restitution = Settings.defaultRestitution;
		density = Settings.defaultDensity;
		shape = null;
	}
}
