package oimo.physics.collision.broadphase.bvh;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Mat4;
import oimo.math.Vec3;
import oimo.physics.debugdraw.IDebugGraphics;

/**
 * BVH Tree
 */
@:build(oimo.m.B.bu())
class BVHTree {
	public var _root:BVHNode;

	var _nodePool:BVHNode;

	public function new() {
	}

	public function print(root:BVHNode, indent:String = ""):Void {
		if (root == null) return;
		if (root._height == 0) {
			trace(indent + root._proxy._id);
		} else {
			print(root._children[0], indent + "  ");
			trace(indent + "#" + root._height + ", " + M.toFixed4(root._perimeter()));
			print(root._children[1], indent + "  ");
		}
	}

	@:extern
	public inline function insertProxy(proxy:BVHProxy):Void {
		var leaf:BVHNode = pick();
		leaf._proxy = proxy;
		proxy._leaf = leaf;

		M.vec3_assign(leaf._aabbMin, proxy._aabbMin);
		M.vec3_assign(leaf._aabbMax, proxy._aabbMax);

		insertLeaf(leaf);
	}

	@:extern
	public inline function deleteProxy(proxy:BVHProxy):Void {
		var leaf:BVHNode = proxy._leaf;

		deleteLeaf(leaf);

		proxy._leaf = null;
		pool(leaf);
	}

	@:extern
	public inline function optimize(count:Int):Void {
		if (_root == null) return;
		for (i in 0...count) {
			var leaf:BVHNode = _root;
			while (leaf._height > 0) {
				var h1:Int = leaf._children[0]._height;
				var h2:Int = leaf._children[1]._height;
				leaf = leaf._children[Math.random() > (h1 / (h1 + h2)) ? 1 : 0];
			}
			deleteLeaf(leaf);
			insertLeaf(leaf);
		}
	}

	public function getBalance() {
		return getBalanceRecursive(_root);
	}

	function getBalanceRecursive(root:BVHNode) {
		if (root == null || root._height == 0) return 0;
		var balance:Int = root._children[0]._height - root._children[1]._height;
		if (balance < 0) balance = -balance;
		return balance + getBalanceRecursive(root._children[0]) + getBalanceRecursive(root._children[1]);
	}

	@:extern
	public inline function clear():Void {
		if (_root == null) return;
		deleteRecursive(_root);
		_root = null;
	}

	function deleteRecursive(root:BVHNode):Void {
		if (root._height == 0) {
			root._proxy._leaf = null;
			pool(root);
			return;
		}
		deleteRecursive(root._children[0]);
		deleteRecursive(root._children[1]);
		pool(root);
	}

	@:extern
	inline function insertLeaf(leaf:BVHNode):Void {
		assertBeLeaf(leaf);
		if (_root == null) { // the tree is empty
			_root = leaf;
			return;
		}
		// search for the position to insert
		var sibling:BVHNode = _root;

		while (sibling._height > 0) {
			var nextStep:Int = decideNextStep(sibling, leaf);

			if (nextStep == -1) {
				// insert to current position
				break;
			} else {
				sibling = sibling._children[nextStep];
			}
		}

		var parent:BVHNode = sibling._parent;

		// new common parent with the sibling
		var node:BVHNode = pick();

		if (parent == null) {
			// replace the root node
			_root = node;
		} else {
			// connect to the old parent
			parent._setChild(sibling._childIndex, node);
		}
		node._setChild(sibling._childIndex, sibling);
		node._setChild(sibling._childIndex ^ 1, leaf);

		// fix data
		while (node != null) {
			//node = balance(node);
			node._computeHeight();
			node._computeAABB();
			node = node._parent;
		}
	}

	@:extern
	inline function deleteLeaf(leaf:BVHNode):Void {
		assertBeLeaf(leaf);
		if (_root == leaf) { // the tree has only the leaf
			_root = null;
			return;
		}
		var parent:BVHNode = leaf._parent;
		var sibling:BVHNode = parent._children[leaf._childIndex ^ 1];
		var grandParent:BVHNode = parent._parent;
		if (grandParent == null) {
			sibling._parent = null;
			sibling._childIndex = 0;
			_root = sibling;
			pool(parent);
			return;
		}
		sibling._parent = grandParent;
		grandParent._setChild(parent._childIndex, sibling);
		pool(parent);

		// fix data
		var node:BVHNode = grandParent;
		while (node != null) {
			//node = balance(node);
			node._computeHeight();
			node._computeAABB();
			node = node._parent;
		}
	}

	/**
	 * 0 or 1 -> descend to corresponding child
	 * -1 -> insert to current position
	 */
	@:extern
	inline function decideNextStep(node:BVHNode, leaf:BVHNode):Int {
		var simple:Bool = false;
		if (simple) {
			var center:IVec3;
			M.vec3_add(center, leaf._aabbMin, leaf._aabbMax);

			var c1:BVHNode = node._children[0];
			var c2:BVHNode = node._children[1];
			var diff1:IVec3;
			var diff2:IVec3;
			M.vec3_add(diff1, c1._aabbMin, c1._aabbMax);
			M.vec3_add(diff2, c2._aabbMin, c2._aabbMax);
			M.vec3_sub(diff1, diff1, center);
			M.vec3_sub(diff2, diff2, center);
			var dist1:Float = M.vec3_dot(diff1, diff1);
			var dist2:Float = M.vec3_dot(diff2, diff2);

			return dist1 < dist2 ? 0 : 1;
		} else {
			var c1:BVHNode = node._children[0];
			var c2:BVHNode = node._children[1];

			var oldArea:Float = M.aabb_surfaceArea(node._aabbMin, node._aabbMax);

			var combinedMin:IVec3;
			var combinedMax:IVec3;
			M.aabb_combine(combinedMin, combinedMax, node._aabbMin, node._aabbMax, leaf._aabbMin, leaf._aabbMax);

			var newArea:Float = M.aabb_surfaceArea(combinedMin, combinedMax);

			// cost of creating a new pair with the node
			var creatingCost:Float = newArea * 2;
			var incrementalCost:Float = (newArea - oldArea) * 2;

			var descendingCost1:Float = incrementalCost;
			M.aabb_combine(combinedMin, combinedMax, c1._aabbMin, c1._aabbMax, leaf._aabbMin, leaf._aabbMax);
			if (c1._height == 0) {
				// leaf cost = area(combined aabb)
				descendingCost1 += M.aabb_surfaceArea(combinedMin, combinedMax);
			} else {
				// node cost = area(combined aabb) - area(old aabb)
				descendingCost1 += M.aabb_surfaceArea(combinedMin, combinedMax) - M.aabb_surfaceArea(c1._aabbMin, c1._aabbMax);
			}

			var descendingCost2:Float = incrementalCost;
			M.aabb_combine(combinedMin, combinedMax, c2._aabbMin, c2._aabbMax, leaf._aabbMin, leaf._aabbMax);
			if (c2._height == 0) {
				// leaf cost = area(combined aabb)
				descendingCost2 += M.aabb_surfaceArea(combinedMin, combinedMax);
			} else {
				// node cost = area(combined aabb) - area(old aabb)
				descendingCost2 += M.aabb_surfaceArea(combinedMin, combinedMax) - M.aabb_surfaceArea(c2._aabbMin, c2._aabbMax);
			}

			M.compare3min(creatingCost, descendingCost1, descendingCost2, {
				return -1;
			},{
				return 0;
			},{
				return 1;
			});
		}
	}

	/**
	 * Balances and returns the node at the same position of `node`.
	 */
	@:extern
	inline function balance(node:BVHNode):BVHNode {
		var nh:Int = node._height;
		if (nh < 2) {
			return node;
		}
		var p:BVHNode = node._parent;
		var l:BVHNode = node._children[0];
		var r:BVHNode = node._children[1];
		var lh:Int = l._height;
		var rh:Int = r._height;
		var balance:Int = lh - rh;
		var nodeIndex:Int = node._childIndex;

		//          [ N ]
		//         /     \
		//    [ L ]       [ R ]
		//     / \         / \
		// [L-L] [L-R] [R-L] [R-R]

		// is the tree balanced?
		if (balance > 1) {
			var ll:BVHNode = l._children[0];
			var lr:BVHNode = l._children[1];
			var llh:Int = ll._height;
			var lrh:Int = lr._height;

			// is L-L higher than L-R?
			if (llh > lrh) {
				// set N to L-R
				l._setChild(1, node);

				//          [ L ]
				//         /     \
				//    [L-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [ R ]

				// set L-R
				node._setChild(0, lr);

				//          [ L ]
				//         /     \
				//    [L-L]       [ N ]
				//     / \         / \
				// [...] [...] [L-R] [ R ]

				// fix bounds and heights
				l._computeAABB();
				l._computeHeight();
				node._computeAABB();
				node._computeHeight();
			} else {
				// set N to L-L
				l._setChild(0, node);

				//          [ L ]
				//         /     \
				//    [ N ]       [L-R]
				//     / \         / \
				// [ L ] [ R ] [...] [...]

				// set L-L
				node._setChild(0, ll);

				//          [ L ]
				//         /     \
				//    [ N ]       [L-R]
				//     / \         / \
				// [L-L] [ R ] [...] [...]

				// fix bounds and heights
				l._computeAABB();
				l._computeHeight();
				node._computeAABB();
				node._computeHeight();
			}
			// set new parent of L
			if (p != null) {
				p._setChild(nodeIndex, l);
			} else {
				_root = l;
				l._parent = null;
			}
			return l;
		}
		if (balance < -1) {
			var rl:BVHNode = r._children[0];
			var rr:BVHNode = r._children[1];
			var rlh:Int = rl._height;
			var rrh:Int = rr._height;

			// is R-L higher than R-R?
			if (rlh > rrh) {
				// set N to R-R
				r._setChild(1, node);

				//          [ R ]
				//         /     \
				//    [R-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [ R ]

				// set R-R
				node._setChild(1, rr);

				//          [ R ]
				//         /     \
				//    [R-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [R-R]

				// fix bounds and heights
				r._computeAABB();
				r._computeHeight();
				node._computeAABB();
				node._computeHeight();
			} else {
				// set N to R-L
				r._setChild(0, node);

				//          [ R ]
				//         /     \
				//    [ N ]       [R-R]
				//     / \         / \
				// [ L ] [ R ] [...] [...]

				// set R-L
				node._setChild(1, rl);

				//          [ R ]
				//         /     \
				//    [ N ]       [R-R]
				//     / \         / \
				// [ L ] [R-L] [...] [...]

				// fix bounds and heights
				r._computeAABB();
				r._computeHeight();
				node._computeAABB();
				node._computeHeight();
			}
			// set new parent of R
			if (p != null) {
				p._setChild(nodeIndex, r);
			} else {
				_root = r;
				r._parent = null;
			}
			return r;
		}
		return node;
	}

	@:extern
	inline function assertBeLeaf(leaf:BVHNode):Void {
		M.assert(leaf._proxy != null && leaf._proxy._leaf == leaf && leaf._children[0] == null && leaf._children[1] == null && leaf._height == 0);
	}

	@:extern
	inline function pool(node:BVHNode):Void {
		M.assert(node._proxy == null || node._proxy._leaf == null);
		node._removeReferences();
		M.singleList_pool(_nodePool, _next, node);
	}

	@:extern
	inline function pick():BVHNode {
		return M.singleList_pick(_nodePool, _next, new BVHNode());
	}

}
