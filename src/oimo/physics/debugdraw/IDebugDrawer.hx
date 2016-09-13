package oimo.physics.debugdraw;
import oimo.math.Mat4;
import oimo.math.Vec3;

/**
 * The interface of simple debug drawer used for drawing a physics world.
 */
@:expose("OIMO.IDebugDrawer")
interface IDebugDrawer {
	/**
	 * Begins drawing the physics world and clear the background to (`r`, `g`, `b`).
	 */
	public function begin(r:Float, g:Float, b:Float):Void;

	/**
	 * Ends drawing the physics world.
	 */
	public function end():Void;

	/**
	 * Draws a sphere of radius `radius` transformed by the matrix `transform`.
	 */
	public function drawSphere(transform:Mat4, radius:Float):Void;

	/**
	 * Draws a box of half-extents `halfExtents` transformed by the matrix `transform`.
	 */
	public function drawBox(transform:Mat4, halfExtents:Vec3):Void;

	/**
	 * Sets the color of objects drawn after to (`r`, `g`, `b`).
	 */
	public function color(r:Float, g:Float, b:Float):Void;
}