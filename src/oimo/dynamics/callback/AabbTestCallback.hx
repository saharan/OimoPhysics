package oimo.dynamics.callback;
import oimo.dynamics.rigidbody.Shape;

/**
 * A callback class for aabb tests in a world.
 */
class AabbTestCallback {

	/**
	 * Default constructor.
	 */
	public function new() {
	}

	/**
	 * This is called every time the world detects a shape `shape` that
	 * the query aabb intersects.
	 */
	public function process(shape:Shape):Void {
	}

}
