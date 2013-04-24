package com.element.oimo.physics.collision.broadphase {
	import com.element.oimo.physics.collision.shape.Shape;
	/**
	 * A broad-phase algorithm using Dynamic Bounding Volume Hierarchy.
	 * This shows better performance than other broad-phase algorithm in ray-casting.
	 * @author saharan
	 */
	public class DynamicBVTreeBroadPhase extends BroadPhase {
		public var tree:DynamicBVTree;
		private var stack1:Vector.<DynamicBVTreeNode>;
		private var stack2:Vector.<DynamicBVTreeNode>;
		private var leaves:Vector.<DynamicBVTreeNode>;
		private var numLeaves:int;
		
		public function DynamicBVTreeBroadPhase() {
			tree = new DynamicBVTree();
			stack1 = new Vector.<DynamicBVTreeNode>(65536, true);
			stack2 = new Vector.<DynamicBVTreeNode>(65536, true);
			leaves = new Vector.<DynamicBVTreeNode>(256, true);
		}
		
		override public function addProxy(proxy:Proxy):void {
			tree.insertLeaf(proxy.leaf);
			if (numLeaves == leaves.length) {
				var newLeaves:Vector.<DynamicBVTreeNode> = new Vector.<DynamicBVTreeNode>(numLeaves << 1, true);
				for (var i:int = 0; i < numLeaves; i++) {
					newLeaves[i] = leaves[i];
				}
				leaves = newLeaves;
			}
			leaves[numLeaves++] = proxy.leaf;
		}
		
		override public function removeProxy(proxy:Proxy):void {
			tree.deleteLeaf(proxy.leaf);
			for (var i:int = 0; i < numLeaves; i++) {
				if (leaves[i] == proxy.leaf) {
					leaves[i] = leaves[--numLeaves];
					leaves[numLeaves] = null;
					return;
				}
			}
		}
		
		override protected function collectPairs():void {
			numPairChecks = 0;
			if (numLeaves < 2) return;
			for (var i:int = 0; i < numLeaves; i++) {
				var leaf:DynamicBVTreeNode = leaves[i];
				var trueB:Proxy = leaf.proxy;
				var leafB:AABB = leaf.aabb;
				tree.deleteLeaf(leaf);
				leafB.minX = trueB.minX;
				leafB.maxX = trueB.maxX;
				leafB.minY = trueB.minY;
				leafB.maxY = trueB.maxY;
				leafB.minZ = trueB.minZ;
				leafB.maxZ = trueB.maxZ;
				tree.insertLeaf(leaf);
				//collide(tree.root, leaf, true);
			}
			collide(tree.root.child1, tree.root);
			collide(tree.root.child2, tree.root);
			trace(numPairChecks);
		}
		
		private function collide(node1:DynamicBVTreeNode, node2:DynamicBVTreeNode):void {
			var numStacks:int = 1;
			stack1[0] = node1;
			stack2[0] = node2;
			while (numStacks > 0) {
				var n1:DynamicBVTreeNode = stack1[--numStacks];
				var n2:DynamicBVTreeNode = stack2[numStacks];
				var b1:AABB = n1.aabb;
				var b2:AABB = n2.aabb;
				numPairChecks++;
				if (
					b1.maxX < b2.minX || b1.minX > b2.maxX ||
					b1.maxY < b2.minY || b1.minY > b2.maxY ||
					b1.maxZ < b2.minZ || b1.minZ > b2.maxZ
				) {
					continue;
				}
				var l1:Boolean = n1.proxy != null;
				var l2:Boolean = n2.proxy != null;
				if (l1 && l2) {
					var s1:Shape = n1.proxy.parent;
					var s2:Shape = n2.proxy.parent;
					if (s1 != s2 && isAvailablePair(s1, s2)) {
						addPair(s1, s2);
					}
					continue;
				}
				if (l2 || !l1 && (n1.aabb.surfaceArea() > n2.aabb.surfaceArea())) {
					stack1[numStacks] = n1.child1;
					stack2[numStacks++] = n2;
					stack1[numStacks] = n1.child2;
					stack2[numStacks++] = n2;
				} else {
					stack1[numStacks] = n1;
					stack2[numStacks++] = n2.child1;
					stack1[numStacks] = n1;
					stack2[numStacks++] = n2.child2;
				}
			}
		}
		
	}

}