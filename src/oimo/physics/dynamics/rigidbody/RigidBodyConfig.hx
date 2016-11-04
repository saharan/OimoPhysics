package oimo.physics.dynamics.rigidbody;
import oimo.math.Mat3;
import oimo.physics.collision.shape.Transform;
import oimo.math.Vec3;

/**
 * Used for construction of Component.
 */
@:expose("OIMO.RigidBodyConfig")
class RigidBodyConfig {
	public var position:Vec3;
	public var rotation:Mat3;
	public var linearVelocity:Vec3;
	public var angularVelocity:Vec3;
	public var type:RigidBodyType;


	public function new() {
		position = new Vec3();
		rotation = new Mat3();
		linearVelocity = new Vec3();
		angularVelocity = new Vec3();
		type = Dynamic;
	}
}
