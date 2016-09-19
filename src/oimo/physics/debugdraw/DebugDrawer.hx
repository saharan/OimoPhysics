package oimo.physics.debugdraw;
import oimo.m.ITransform;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Mat4;
import oimo.math.Vec3;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.dynamics.Component;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.World;

/**
 * Debug drawer.
 */
@:expose("OIMO.DebugDrawer")
@:build(oimo.m.B.build())
class DebugDrawer {
	public var drawAABB:Bool;

	var debugGraphics:IDebugGraphics;
	var world:World;

	var componentTransform:Mat4;
	var aabbTransform:Mat4;
	var tmpVec3:Vec3;

	var viewMat:Mat4;
	var projMat:Mat4;

	public function new(world:World, debugGraphics:IDebugGraphics) {
		this.world = world;
		this.debugGraphics = debugGraphics;

		componentTransform = new Mat4();
		aabbTransform = new Mat4();
		tmpVec3 = new Vec3();

		viewMat = new Mat4();
		projMat = new Mat4();

		drawAABB = true;
	}

	public function draw():Void {
		debugGraphics.begin(Settings.debugDrawBackgroundColor);
		debugGraphics.setViewMatrix(viewMat);
		debugGraphics.setProjectionMatrix(projMat);
		_drawRigidBodies(debugGraphics);
		debugGraphics.end();
	}

	public function camera(eye:Vec3, at:Vec3, up:Vec3):Void {
		viewMat.lookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
	}

	public function perspective(fovY:Float, aspect:Float):Void {
		projMat.perspective(fovY, aspect, 0.1, 1000);
	}

	@:extern
	inline function _drawRigidBodies(debugDrawer:IDebugGraphics):Void {
		var r:RigidBody = world._rigidBodyList;
		M.list_foreach(r, _next, {
			var shapeColor:Vec3;
			var isDynamic:Bool = r._type == Dynamic;
			if (!isDynamic) {
				shapeColor = Settings.debugDrawStaticShapeColor;
			}
			var c:Component = r._componentList;
			M.list_foreach(c, _next, {
				M.transform_toMat4(componentTransform, c._transform);
				if (isDynamic) {
					shapeColor = (c._id & 1) == 0 ? Settings.debugDrawShapeColor1 : Settings.debugDrawShapeColor2;
				}
				debugDrawer.color(shapeColor);
				_drawShape(c._shape, debugDrawer);
				debugDrawer.color(Settings.debugDrawAABBColor);
				_drawAABB(c._aabb, debugDrawer);
			});
		});
	}

	@:extern
	inline function _drawShape(shape:Shape, debugDrawer:IDebugGraphics):Void {
		switch(shape._type) {
		case Sphere:
			_drawSphere(cast shape, debugDrawer);
		case Box:
			_drawBox(cast shape, debugDrawer);
		}
	}

	@:extern
	inline function _drawSphere(sphere:SphereShape, debugDrawer:IDebugGraphics):Void {
		debugDrawer.drawSphere(componentTransform, sphere._radius);
	}

	@:extern
	inline function _drawBox(box:BoxShape, debugDrawer:IDebugGraphics):Void {
		M.vec3_toVec3(tmpVec3, box._halfExtents);
		debugDrawer.drawBox(componentTransform, tmpVec3);
	}

	@:extern
	inline function _drawAABB(aabb:AABB, debugDrawer:IDebugGraphics):Void {
		var min:IVec3;
		var max:IVec3;
		var halfExtents:IVec3;
		var center:IVec3;
		var tf:ITransform;
		M.vec3_assign(min, aabb._min);
		M.vec3_assign(max, aabb._max);
		M.vec3_sub(halfExtents, max, min);
		M.vec3_scale(halfExtents, halfExtents, 0.5);
		M.vec3_toVec3(tmpVec3, halfExtents);
		M.vec3_add(center, max, min);
		M.vec3_scale(center, center, 0.5);
		M.transform_id(tf);
		M.vec3_assign(tf, center);
		M.transform_toMat4(aabbTransform, tf);
		debugDrawer.drawWireframeBox(aabbTransform, tmpVec3);
	}

}
