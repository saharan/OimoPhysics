package oimo.physics.debugdraw;
import oimo.m.ITransform;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Mat4;
import oimo.math.Vec3;
import oimo.physics.collision.Pair;
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
	public var drawShapes:Bool;
	public var drawAABBs:Bool;
	public var drawPairs:Bool;

	var g:IDebugGraphics;
	var world:World;

	var componentTransform:Mat4;
	var aabbTransform:Mat4;
	var tmpVec1:Vec3;
	var tmpVec2:Vec3;

	var viewMat:Mat4;
	var projMat:Mat4;

	public function new(world:World, debugGraphics:IDebugGraphics) {
		this.world = world;
		this.g = debugGraphics;

		componentTransform = new Mat4();
		aabbTransform = new Mat4();
		tmpVec1 = new Vec3();
		tmpVec2 = new Vec3();

		viewMat = new Mat4();
		projMat = new Mat4();

		drawShapes = true;
		drawAABBs = false;
		drawPairs = false;
	}

	public function draw():Void {
		g.begin(Settings.debugDrawBackgroundColor);
		g.setViewMatrix(viewMat);
		g.setProjectionMatrix(projMat);
		_drawRigidBodies();
		g.end();
	}

	public function camera(eye:Vec3, at:Vec3, up:Vec3):Void {
		viewMat.lookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
	}

	public function perspective(fovY:Float, aspect:Float):Void {
		projMat.perspective(fovY, aspect, 0.1, 1000);
	}

	@:extern
	inline function _drawRigidBodies():Void {
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
				if (drawShapes) {
					g.color(shapeColor);
					_drawShape(c._shape);
				}
				if (drawAABBs) {
					g.color(Settings.debugDrawAABBColor);
					_drawAABB(c._aabb);
				}
			});
		});
		if (drawPairs) {
			var p:Pair = world._pairManager._pairList;
			g.color(Settings.debugDrawPairColor);
			M.list_foreach(p, _next, {
				_drawPair(p);
			});
		}
	}

	@:extern
	inline function _drawShape(shape:Shape):Void {
		switch(shape._type) {
		case Sphere:
			_drawSphere(cast shape);
		case Box:
			_drawBox(cast shape);
		}
	}

	@:extern
	inline function _drawSphere(sphere:SphereShape):Void {
		g.drawSphere(componentTransform, sphere._radius);
	}

	@:extern
	inline function _drawBox(box:BoxShape):Void {
		M.vec3_toVec3(tmpVec1, box._halfExtents);
		g.drawBox(componentTransform, tmpVec1);
	}

	@:extern
	inline function _drawAABB(aabb:AABB):Void {
		var min:IVec3;
		var max:IVec3;
		var halfExtents:IVec3;
		var center:IVec3;
		var tf:ITransform;
		M.vec3_assign(min, aabb._min);
		M.vec3_assign(max, aabb._max);
		M.vec3_sub(halfExtents, max, min);
		M.vec3_scale(halfExtents, halfExtents, 0.5);
		M.vec3_toVec3(tmpVec1, halfExtents);
		M.vec3_add(center, max, min);
		M.vec3_scale(center, center, 0.5);
		M.transform_id(tf);
		M.vec3_assign(tf, center);
		M.transform_toMat4(aabbTransform, tf);
		g.drawWireframeBox(aabbTransform, tmpVec1);
	}

	@:extern
	inline function _drawPair(pair:Pair):Void {
		var c1:Component = pair._c1;
		var c2:Component = pair._c2;
		M.vec3_toVec3(tmpVec1, c1._transform);
		M.vec3_toVec3(tmpVec2, c2._transform);
		g.drawLine(tmpVec1, tmpVec2);
	}

}
