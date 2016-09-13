package oimo.physics.dynamics;
import oimo.m.ITransform;
import oimo.m.M;
import oimo.math.Mat4;
import oimo.math.Vec3;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.debugdraw.IDebugDrawer;

/**
 * Physics world.
 */
@:expose("OIMO.World")
class World {
	public var _rigidBodyList:RigidBody;
	public var _rigidBodyListLast:RigidBody;

	var tmpMat4:Mat4;
	var tmpVec3:Vec3;

	public function new() {
		_rigidBodyList = null;
		_rigidBodyListLast = null;

		tmpMat4 = new Mat4();
		tmpVec3 = new Vec3();
	}

	inline function _updateContacts():Void {
		// TODO: saharan
	}

	inline function _solveIslands():Void {
		// TODO: saharan
	}

	/**
	 * @deprecated
	 */
	inline function _integrate(timeStep:Float):Void {
		var r:RigidBody = _rigidBodyList;
		M.list_foreach(r, _next, {
			r._integrate(timeStep);
			r._syncComponents();
		});
	}

	@:extern
	inline function _drawRigidBodies(debugDrawer:IDebugDrawer):Void {
		var r:RigidBody = _rigidBodyList;
		M.list_foreach(r, _next, {
			var color:Vec3;
			var isDynamic:Bool = r._type == Dynamic;
			if (!isDynamic) {
				color = Settings.debugDrawStaticShapeColor;
				debugDrawer.color(color.x, color.y, color.z);
			}
			var c:Component = r._componentList;
			M.list_foreach(c, _next, {
				M.transform_toMat4(tmpMat4, c._transform);
				if (isDynamic) {
					color = (c._id & 1) == 0 ? Settings.debugDrawShapeColor1 : Settings.debugDrawShapeColor2;
					debugDrawer.color(color.x, color.y, color.z);
				}
				_drawShape(c._shape, debugDrawer);
			});
		});
	}

	@:extern
	inline function _drawShape(shape:Shape, debugDrawer:IDebugDrawer):Void {
		switch(shape._type) {
		case Sphere:
			_drawSphere(cast shape, debugDrawer);
		case Box:
			_drawBox(cast shape, debugDrawer);
		}
	}

	@:extern
	inline function _drawSphere(sphere:SphereShape, debugDrawer:IDebugDrawer):Void {
		debugDrawer.drawSphere(tmpMat4, sphere._radius);
	}

	@:extern
	inline function _drawBox(box:BoxShape, debugDrawer:IDebugDrawer):Void {
		M.vec3_toVec3(tmpVec3, box._halfExtents);
		debugDrawer.drawBox(tmpMat4, tmpVec3);
	}

	// --- public ---

	public function step(timeStep:Float):Void {
		//_updateContacts();
		//_solveIslands(timeStep);
		_integrate(timeStep);
	}

	public function debugDraw(debugDrawer:IDebugDrawer):Void {
		_drawRigidBodies(debugDrawer);
	}

	public function addRigidBody(rigidBody:RigidBody):Void {
		M.list_push(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
	}

	public function removeRigidBody(rigidBody:RigidBody):Void {
		M.list_remove(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
	}

}