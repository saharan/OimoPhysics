package oimo.physics.debugdraw;
import oimo.math.Mat4;
import oimo.math.Vec3;

/**
 * The interface of simple debug graphics for drawing a physics world.
 */
@:expose("OIMO.IDebugGraphics")
interface IDebugGraphics {
	/**
	 * Begins drawing the physics world and clear the background to `color`.
	 */
	public function begin(color:Vec3):Void;

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
	 * Draws a wireframed box of half-extents `halfExtents` transformed by the matrix `transform`.
	 */
	public function drawWireframeBox(transform:Mat4, halfExtents:Vec3):Void;

	/**
	 * Draws a line from `from` to `to`.
	 */
	public function drawLine(from:Vec3, to:Vec3):Void;

	/**
	 * Sets the color of objects drawn after to `color`.
	 */
	public function color(color:Vec3):Void;

	/**
	 * Sets the view transformation matrix to `matrix`.
	 */
	public function setViewMatrix(matrix:Mat4):Void;

	/**
	 * Sets the projection transformation matrix to `matrix`.
	 */
	public function setProjectionMatrix(matrix:Mat4):Void;
}
