package oimo.physics.collision.broadphase.bvh;
import haxe.ds.Vector;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * BVH Node
 */
@:build(oimo.m.B.bu())
class BVHNode {
	// for object pool
	public var _next:BVHNode;

	public var _children:Vector<BVHNode>;
	public var _childIndex:Int;
	public var _parent:BVHNode;
	public var _height:Int;
	public var _proxy:BVHProxy;
	public var _aabbMin:IVec3;
	public var _aabbMax:IVec3;

	public function new() {
		_next = null;
		_children = new Vector<BVHNode>(2);
		_childIndex = 0;
		_parent = null;
		_height = 0;
		_proxy = null;
		M.vec3_zero(_aabbMin);
		M.vec3_zero(_aabbMax);
	}

	@:extern
	public inline function _setChild(index:Int, child:BVHNode):Void {
		_children[index] = child;
		child._parent = this;
		child._childIndex = index;
	}

	@:extern
	public inline function _removeReferences():Void {
		_next = null;
		_childIndex = 0;
		_children[0] = null;
		_children[1] = null;
		_childIndex = 0;
		_parent = null;
		_height = 0;
		_proxy = null;
	}

	@:extern
	public inline function _computeAABB():Void {
		var c1:BVHNode = _children[0];
		var c2:BVHNode = _children[1];
		M.vec3_min(_aabbMin, c1._aabbMin, c2._aabbMin);
		M.vec3_max(_aabbMax, c1._aabbMax, c2._aabbMax);
	}

	@:extern
	public inline function _computeHeight():Void {
		var h1:Int = _children[0]._height;
		var h2:Int = _children[1]._height;
		_height = M.max(h1, h2) + 1;
	}

	@:extern
	public inline function _perimeter():Float {
		var size:IVec3;
		M.vec3_sub(size, _aabbMax, _aabbMin);
		var x:Float = M.vec3_get(size, 0);
		var y:Float = M.vec3_get(size, 1);
		var z:Float = M.vec3_get(size, 2);
		return x * (y + z) + y * z;
	}

}
