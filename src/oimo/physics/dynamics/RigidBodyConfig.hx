package oimo.physics.dynamics;
import oimo.math.Transform;
import oimo.math.Vec3;
import oimo.physics.Settings;
import oimo.physics.collision.shape.Shape;

/**
 * Used for construction of Component.
 */
@:expose("OIMO.RigidBodyConfig")
class RigidBodyConfig {
	public var transform:Transform;
	public var linearVelocity:Vec3;
	public var angularVelocity:Vec3;

	public function new() {
		transform = new Transform();
		linearVelocity = new Vec3();
		angularVelocity = new Vec3();
	}
}
