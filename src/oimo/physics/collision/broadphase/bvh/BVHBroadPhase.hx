package oimo.physics.collision.broadphase.bvh;
import haxe.ds.Vector;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.broadphase.bvh.BVHProxy;
import oimo.physics.collision.shape.AABB;


/**
 * The broad-phase collision detection algorithm based on bounding
 * volume hierarchy (BVH).
 */
@:build(oimo.m.B.bu())
class BVHBroadPhase extends BroadPhase {
	public var _tree:BVHTree;
	var movedProxies:Vector<BVHProxy>;
	var numMovedProxies:Int;

	public function new() {
		super();
		_incremental = true;
		_tree = new BVHTree();
		movedProxies = new Vector<BVHProxy>(1024);
		numMovedProxies = 0;
	}

	override public function createProxy(userData:Dynamic, aabb:AABB):Proxy {
		var p:BVHProxy = new BVHProxy(userData, _idCount++);
		addProxy(p);

		updateProxy(p, aabb, null);
		_tree.insertProxy(p);
		addToMovedProxy(p);

		return p;
	}

	override public function destroyProxy(proxy:Proxy):Void {
		removeProxy(proxy);

		var bvhProxy:BVHProxy = cast proxy;
		_tree.deleteProxy(bvhProxy);
		bvhProxy._userData = null;
		bvhProxy._next = null;
		bvhProxy._prev = null;

		if (bvhProxy._moved) {
			bvhProxy._moved = false;
		}
	}

	override public function moveProxy(proxy:Proxy, aabb:AABB, displacement:Vec3):Void {
		var bvhProxy:BVHProxy = cast proxy;
		if (M.aabb_contains(proxy._aabbMin, proxy._aabbMax, aabb._min, aabb._max)) {
			// need not move proxy
			return;
		}

		updateProxy(bvhProxy, aabb, displacement);
		addToMovedProxy(bvhProxy);
	}

	@:extern
	inline function addToMovedProxy(bvhProxy:BVHProxy):Void {
		// add to the buffer
		if (bvhProxy._moved) return;
		bvhProxy._moved = true;

		// expand buffer
		if (movedProxies.length == numMovedProxies) {
			M.array_expand(movedProxies, numMovedProxies);
		}

		movedProxies[numMovedProxies++] = bvhProxy;
	}

	@:extern
	inline function updateProxy(p:BVHProxy, aabb:AABB, displacement:Vec3):Void {
		// set tight AABB
		p._setAABB(aabb);

		// fatten the AABB
		var padding:Float = Settings.bvhProxyPadding;
		var paddingVec:IVec3;
		M.vec3_set(paddingVec, padding, padding, padding);
		M.vec3_sub(p._aabbMin, p._aabbMin, paddingVec);
		M.vec3_add(p._aabbMax, p._aabbMax, paddingVec);

		if (displacement != null) {
			// predict movement
			var d:IVec3;
			var zero:IVec3;
			var addToMin:IVec3;
			var addToMax:IVec3;
			M.vec3_zero(zero);
			M.vec3_fromVec3(d, displacement);
			M.vec3_min(addToMin, zero, d);
			M.vec3_max(addToMax, zero, d);
			M.vec3_add(p._aabbMin, p._aabbMin, addToMin);
			M.vec3_add(p._aabbMax, p._aabbMax, addToMax);
		}
	}

	override public function collectPairs():Void {
		poolProxyPairs();
		_testCount = 0;
		if (_numProxies < 2) return;

		var incrementalCollision:Bool = numMovedProxies / _numProxies < Settings.bvhIncrementalCollisionThreshold;

		// incremental modification
		for (i in 0...numMovedProxies) {
			var p:BVHProxy = movedProxies[i];
			if (p._moved) {
				_tree.deleteProxy(p);
				_tree.insertProxy(p);
				if (incrementalCollision) {
					collide(_tree._root, p._leaf);
				}
				p._moved = false;
			}
			movedProxies[i] = null;
		}
		if (!incrementalCollision) {
			collide(_tree._root, _tree._root);
		}

		numMovedProxies = 0;
	}

	function collide(n1:BVHNode, n2:BVHNode):Void {
		_testCount++;
		var l1:Bool = n1._height == 0;
		var l2:Bool = n2._height == 0;
		if (n1 == n2) {
			if (l1) return;
			collide(n1._children[0], n2);
			collide(n1._children[1], n2);
			return;
		}
		if (!M.aabb_intersects(n1._aabbMin, n1._aabbMax, n2._aabbMin, n2._aabbMax)) {
			return;
		}
		if (l1 && l2) {
			pickAndPushProxyPair(n1._proxy, n2._proxy);
			return;
		}
		if (l2 || n1._height > n2._height) {
			// descend node 1
			collide(n1._children[0], n2);
			collide(n1._children[1], n2);
		} else {
			// descend node 2
			collide(n2._children[0], n1);
			collide(n2._children[1], n1);
		}
	}
}
