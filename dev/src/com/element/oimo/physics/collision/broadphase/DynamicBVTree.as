package com.element.oimo.physics.collision.broadphase {
	/**
	 * A Dynamic Bounding Volume Tree for broad-phase algorithm.
	 * @author saharan
	 */
	public class DynamicBVTree {
		/**
		 * The root of this tree.
		 */
		public var root:DynamicBVTreeNode;
		
		public function DynamicBVTree() {
		}
		
		/**
		 * Insert the leaf to this tree.
		 * @param	node
		 */
		public function insertLeaf(leaf:DynamicBVTreeNode):void {
			if (root == null) {
				root = leaf;
				return;
			}
			var pair:DynamicBVTreeNode = root;
			while (pair.proxy == null) { // descend the node to search the best pair
				var cost:Number = cost(pair); // cost of making a pair with the node
				var down1Cost:Number = down1Cost(pair); // cost of descending into first child
				var down2Cost:Number = down2Cost(pair); // cost of descending into second child
				if (down1Cost < down2Cost) {
					if (cost < down1Cost) {
						break; // stop descending
					} else {
						pair = pair.child1; // descend into first child
					}
				} else {
					if (cost < down2Cost) {
						break; // stop descending
					} else {
						pair = pair.child2; // descend into second child
					}
				}
			}
			var oldParent:DynamicBVTreeNode = pair.parent;
			var newParent:DynamicBVTreeNode = new DynamicBVTreeNode();
			newParent.parent = oldParent;
			newParent.child1 = leaf;
			newParent.child2 = pair;
			pair.parent = newParent;
			leaf.parent = newParent;
			if (pair == root) {
				// replace root
				root = newParent;
			} else {
				// replace child
				if (oldParent.child1 == pair) {
					oldParent.child1 = newParent;
				} else {
					oldParent.child2 = newParent;
				}
			}
		}
		
		public function print(node:DynamicBVTreeNode, indent:int, text:String):String {
			var hasChild:Boolean = node.proxy == null;
			if (hasChild) text = print(node.child1, indent + 1, text);
			for (var i:int = indent * 2; i >= 0; i--) {
				text += " ";
			}
			text += (hasChild ? "*" : node.proxy.minX) + "\n";
			if (hasChild) text = print(node.child2, indent + 1, text);
			return text;
		}
		
		private function cost(node:DynamicBVTreeNode):Number {
			return Math.random();
		}
		
		private function down1Cost(node:DynamicBVTreeNode):Number {
			return Math.random();
		}
		
		private function down2Cost(node:DynamicBVTreeNode):Number {
			return Math.random();
		}
		
		/**
		 * Remove the node to this tree.
		 * @param	node
		 */
		public function removeNode(node:DynamicBVTreeNode):void {
			
		}
		
		/**
		 * Right rotation.
		 * @param	node
		 * @return
		 */
		public function rotateRight(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var tmp:DynamicBVTreeNode = node.child1;
			node.child1 = tmp.child2;
			tmp.child2 = node;
			return tmp;
		}
		
		/**
		 * Left rotation.
		 * @param	node
		 * @return
		 */
		public function rotateLeft(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var tmp:DynamicBVTreeNode = node.child2;
			node.child2 = tmp.child1;
			tmp.child1 = node;
			return tmp;
		}
		
	}

}