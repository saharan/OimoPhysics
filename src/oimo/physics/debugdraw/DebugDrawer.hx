package oimo.physics.debugdraw;
import oimo.m.IMat3;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Mat4;
import oimo.math.Vec3;
import oimo.physics.collision.Pair;
import oimo.physics.collision.broadphase.bvh.BVHBroadPhase;
import oimo.physics.collision.broadphase.bvh.BVHNode;
import oimo.physics.collision.broadphase.bvh.BVHTree;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.collision.shape.Transform;
import oimo.physics.dynamics.World;
import oimo.physics.dynamics.constraint.contact.ContactConstraint;
import oimo.physics.dynamics.constraint.contact.ContactPoint;
import oimo.physics.dynamics.rigidbody.Component;
import oimo.physics.dynamics.rigidbody.RigidBody;

/**
 * Debug drawer.
 */
@:expose("OIMO.DebugDrawer")
@:build(oimo.m.B.bu())
class DebugDrawer {
	public var wireframe:Bool;

	public var drawShapes:Bool;
	public var drawBVH:Bool;
	public var drawBVHMinLevel:Int;
	public var drawBVHMaxLevel:Int;
	public var drawAABBs:Bool;
	public var drawPairs:Bool;
	public var drawContacts:Bool;

	var g:IDebugGraphics;
	var w:World;

	var componentTransform:Mat4;
	var aabbTransform:Mat4;
	var tmpVec1:Vec3;
	var tmpVec2:Vec3;
	var tmpVec3:Vec3;
	var tmpMat:Mat4;

	var viewMat:Mat4;
	var projMat:Mat4;

	public function new(world:World, debugGraphics:IDebugGraphics) {
		w = world;
		g = debugGraphics;

		componentTransform = new Mat4();
		aabbTransform = new Mat4();
		tmpVec1 = new Vec3();
		tmpVec2 = new Vec3();
		tmpVec3 = new Vec3();
		tmpMat = new Mat4();

		viewMat = new Mat4();
		projMat = new Mat4();

		wireframe = false;
		drawShapes = true;
		drawBVH = false;
		drawAABBs = false;
		drawPairs = false;
		drawContacts = false;
		drawBVHMinLevel = 0;
		drawBVHMaxLevel = 65536;
	}

	public function setWorld(world:World):Void {
		w = world;
	}

	public function setDebugGraphics(debugGraphics:IDebugGraphics):Void {
		g = debugGraphics;
	}

	public function draw():Void {
		g.begin(Settings.debugDrawBackgroundColor);
		g.setViewMatrix(viewMat);
		g.setProjectionMatrix(projMat);

		if (drawBVH && Std.is(w._broadPhase, BVHBroadPhase)) {
			drawBVHTree(cast(w._broadPhase, BVHBroadPhase)._tree);
		}

		drawRigidBodies();
		g.end();
	}

	public function camera(eye:Vec3, at:Vec3, up:Vec3):Void {
		viewMat.lookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
	}

	public function perspective(fovY:Float, aspect:Float):Void {
		projMat.perspective(fovY, aspect, 0.1, 1000);
	}

	@:extern
	inline function drawBVHTree(tree:BVHTree):Void {
		g.color(Settings.Settings.debugDrawBVHNodeColor);
		drawBVHNode(tree._root, 0);
	}

	function drawBVHNode(node:BVHNode, level:Int):Void {
		if (node == null) return;
		if (level >= drawBVHMinLevel && level <= drawBVHMaxLevel) {
			M.call(drawAABBMinMax(node._aabbMin, node._aabbMax));
		}
		drawBVHNode(node._children[0], level + 1);
		drawBVHNode(node._children[1], level + 1);
	}

	@:extern
	inline function drawRigidBodies():Void {
		var r:RigidBody = w._rigidBodyList;
		M.list_foreach(r, _next, {
			var shapeColor:Vec3 = null;
			var isDynamic:Bool = r._type == Dynamic;
			if (!isDynamic) {
				shapeColor = Settings.debugDrawStaticShapeColor;
			}
			var c:Component = r._componentList;
			g.disableLighting();
			M.list_foreach(c, _next, {
				M.transform_toMat4(componentTransform, c._transform._origin, c._transform._rotation);
				if (isDynamic) {
					shapeColor = (c._id & 1) == 0 ? Settings.debugDrawShapeColor1 : Settings.debugDrawShapeColor2;
				}
				if (drawShapes) {
					g.color(shapeColor);
					if (!wireframe) g.enableLighting();
					drawShape(c._shape);
				}
				if (drawAABBs) {
					g.color(Settings.debugDrawAABBColor);
					if (!wireframe) g.disableLighting();
					drawAABB(c._aabb);
				}
			});
		});
		g.disableLighting();
		if (drawPairs || drawContacts) {
			var p:Pair = w._pairManager._pairList;
			M.list_foreach(p, _next, {
				if (drawPairs) {
					g.color(Settings.debugDrawPairColor);
					drawPair(p);
				}
				if (drawContacts) {
					for (i in 0...p._contactConstraint._numPoints) {
						var c:ContactConstraint = p._contactConstraint;
						g.color(Settings.debugDrawContactColor);
						M.call(drawContactPoint(p._contactConstraint._points[i], p._c1._transform, p._c2._transform, c._normal));
						M.call(drawContactBasis(p._contactConstraint._points[i], p._c1._transform, p._c2._transform, c._normal, c._tangent, c._binormal));
					}
				}
			});
		}
	}

	@:extern
	inline function drawShape(shape:Shape):Void {
		switch(shape._type) {
		case Sphere:
			drawSphere(cast shape);
		case Box:
			drawBox(cast shape);
		}
	}

	@:extern
	inline function drawContactPoint(contact:ContactPoint, tf1:Transform, tf2:Transform, normal:IVec3):Void {
		var t1:IVec3;
		var t2:IVec3;
		M.vec3_add(t1, tf1._origin, contact._relPos1);
		M.vec3_add(t2, tf2._origin, contact._relPos2);
		M.vec3_toVec3(tmpVec1, t1);
		M.vec3_toVec3(tmpVec2, t2);

		tmpMat.identity();
		tmpMat.appendTranslationEqual(tmpVec1.x, tmpVec1.y, tmpVec1.z);
		g.drawSphere(tmpMat, 0.1);

		tmpMat.identity();
		tmpMat.appendTranslationEqual(tmpVec2.x, tmpVec2.y, tmpVec2.z);
		g.drawSphere(tmpMat, 0.1);

		g.drawLine(tmpVec1, tmpVec2);
	}

	@:extern
	inline function drawContactBasis(contact:ContactPoint, tf1:Transform, tf2:Transform, normal:IVec3, tangent:IVec3, binormal:IVec3):Void {
		var t2:IVec3;
		M.vec3_add(t2, tf2._origin, contact._relPos2);
		M.vec3_toVec3(tmpVec2, t2);

		g.color(Settings.debugDrawContactNormalColor);
		M.vec3_toVec3(tmpVec1, normal);
		tmpVec1.scaleEqual(Settings.debugDrawContactNormalLength);
		tmpVec3.copyFrom(tmpVec2);
		tmpVec3.addEqual(tmpVec1);
		g.drawLine(tmpVec2, tmpVec3);

		g.color(Settings.debugDrawContactTangentColor);
		M.vec3_toVec3(tmpVec1, tangent);
		tmpVec1.scaleEqual(Settings.debugDrawContactTangentLength);
		tmpVec3.copyFrom(tmpVec2);
		tmpVec3.addEqual(tmpVec1);
		g.drawLine(tmpVec2, tmpVec3);

		g.color(Settings.debugDrawContactBinormalColor);
		M.vec3_toVec3(tmpVec1, binormal);
		tmpVec1.scaleEqual(Settings.debugDrawContactBinormalLength);
		tmpVec3.copyFrom(tmpVec2);
		tmpVec3.addEqual(tmpVec1);
		g.drawLine(tmpVec2, tmpVec3);
	}

	@:extern
	inline function drawSphere(sphere:SphereShape):Void {
		if (wireframe) {
			g.drawWireframeSphere(componentTransform, sphere._radius);
		} else {
			g.drawSphere(componentTransform, sphere._radius);
		}
	}

	@:extern
	inline function drawBox(box:BoxShape):Void {
		M.vec3_toVec3(tmpVec1, box._halfExtents);
		if (wireframe) {
			g.drawWireframeBox(componentTransform, tmpVec1);
		} else {
			g.drawBox(componentTransform, tmpVec1);
		}
	}

	@:extern
	inline function drawAABB(aabb:AABB):Void {
		M.call(drawAABBMinMax(aabb._min, aabb._max));
	}

	@:extern
	inline function drawAABBMinMax(min:IVec3, max:IVec3):Void {
		var halfExtents:IVec3;
		var center:IVec3;
		var tfOrigin:IVec3;
		var tfRotation:IMat3;
		M.vec3_sub(halfExtents, max, min);
		M.vec3_scale(halfExtents, halfExtents, 0.5);
		M.vec3_toVec3(tmpVec1, halfExtents);
		M.vec3_add(center, max, min);
		M.vec3_scale(center, center, 0.5);

		M.mat3_id(tfRotation);
		M.vec3_zero(tfOrigin);

		M.vec3_assign(tfOrigin, center);
		M.transform_toMat4(aabbTransform, tfOrigin, tfRotation);
		g.drawWireframeBox(aabbTransform, tmpVec1);
	}

	@:extern
	inline function drawPair(pair:Pair):Void {
		var c1:Component = pair._c1;
		var c2:Component = pair._c2;
		M.vec3_toVec3(tmpVec1, c1._transform._origin);
		M.vec3_toVec3(tmpVec2, c2._transform._origin);
		g.drawLine(tmpVec1, tmpVec2);
	}

}
