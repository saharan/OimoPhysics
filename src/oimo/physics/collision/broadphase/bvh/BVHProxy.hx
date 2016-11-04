package oimo.physics.collision.broadphase.bvh;

import oimo.physics.collision.broadphase.Proxy;

/**
 * BVH Proxy
 */
class BVHProxy extends Proxy {
	public var _leaf:BVHNode;
	public var _moved:Bool;

	public function new(userData:Dynamic, id:Int) {
		super(userData, id);
		_leaf = null;
		_moved = false;
	}

}
