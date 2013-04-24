package com.element.oimo.physics.collision.broadphase {
	/**
	 * A dynamic bounding volume tree for the broad-phase algorithm.
	 * @author saharan
	 */
	public class DynamicBVTree {
		/**
		 * The root of this tree.
		 */
		public var root:DynamicBVTreeNode;
		
		private var freeNodes:Vector.<DynamicBVTreeNode>;
		private var numFreeNodes:int;
		private var aabb:AABB;
		
		public function DynamicBVTree() {
			freeNodes = new Vector.<DynamicBVTreeNode>(16384, true);
			numFreeNodes = 0;
			aabb = new AABB();
		}
		
		/**
		 * Move the leaf.
		 * @param	leaf
		 */
		public function moveLeaf(leaf:DynamicBVTreeNode):void {
			deleteLeaf(leaf);
			insertLeaf(leaf);
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
			var lb:AABB = leaf.aabb;
			var sibling:DynamicBVTreeNode = root;
			var oldArea:Number;
			var newArea:Number;
			while (sibling.proxy == null) { // descend the node to search the best pair
				var c1:DynamicBVTreeNode = sibling.child1;
				var c2:DynamicBVTreeNode = sibling.child2;
				var b:AABB = sibling.aabb;
				var c1b:AABB = c1.aabb;
				var c2b:AABB = c2.aabb;
				
				oldArea = b.surfaceArea();
				aabb.combine(lb, b);
				newArea = aabb.surfaceArea();
				var creatingCost:Number = newArea * 2; // cost of creating a new pair with the node
				var incrementalCost:Number = (newArea - oldArea) * 2;
				
				var discendingCost1:Number = incrementalCost;
				aabb.combine(lb, c1b);
				if (c1.proxy != null) {
					// leaf cost = area(combined aabb)
					discendingCost1 += aabb.surfaceArea();
				} else {
					// node cost = area(combined aabb) - area(old aabb)
					discendingCost1 += aabb.surfaceArea() - c1b.surfaceArea();
				}
				
				var discendingCost2:Number = incrementalCost;
				aabb.combine(lb, c2b);
				if (c2.proxy != null) {
					// leaf cost = area(combined aabb)
					discendingCost2 += aabb.surfaceArea();
				} else {
					// node cost = area(combined aabb) - area(old aabb)
					discendingCost2 += aabb.surfaceArea() - c2b.surfaceArea();
				}
				
				if (discendingCost1 < discendingCost2) {
					if (creatingCost < discendingCost1) {
						break; // stop descending
					} else {
						sibling = c1; // descend into first child
					}
				} else {
					if (creatingCost < discendingCost2) {
						break; // stop descending
					} else {
						sibling = c2; // descend into second child
					}
				}
			}
			var oldParent:DynamicBVTreeNode = sibling.parent;
			var newParent:DynamicBVTreeNode;
			if (numFreeNodes > 0) {
				newParent = freeNodes[--numFreeNodes];
			} else {
				newParent = new DynamicBVTreeNode();
			}
			newParent.parent = oldParent;
			newParent.child1 = leaf;
			newParent.child2 = sibling;
			newParent.aabb.combine(leaf.aabb, sibling.aabb);
			newParent.height = sibling.height + 1;
			sibling.parent = newParent;
			leaf.parent = newParent;
			if (sibling == root) {
				// replace root
				root = newParent;
			} else {
				// replace child
				if (oldParent.child1 == sibling) {
					oldParent.child1 = newParent;
				} else {
					oldParent.child2 = newParent;
				}
			}
			// update whole tree
			do {
				newParent = balance(newParent);
				fix(newParent);
				newParent = newParent.parent;
			} while (newParent != null);
		}
		
		public function getBalance(node:DynamicBVTreeNode):int {
			if (node.proxy != null) return 0;
			return node.child1.height - node.child2.height;
		}
		
		public function print(node:DynamicBVTreeNode, indent:int, text:String):String {
			var hasChild:Boolean = node.proxy == null;
			if (hasChild) text = print(node.child1, indent + 1, text);
			for (var i:int = indent * 2; i >= 0; i--) {
				text += " ";
			}
			text += (hasChild ? getBalance(node) : "[" + node.proxy.minX + "]") + "\n";
			if (hasChild) text = print(node.child2, indent + 1, text);
			return text;
		}
		
		/**
		 * Delete the leaf from this tree.
		 * @param	node
		 */
		public function deleteLeaf(leaf:DynamicBVTreeNode):void {
			if (leaf == root) {
				root = null;
				return;
			}
			var parent:DynamicBVTreeNode = leaf.parent;
			var sibling:DynamicBVTreeNode;
			if (parent.child1 == leaf) {
				sibling = parent.child2;
			} else {
				sibling = parent.child1;
			}
			if (parent == root) {
				root = sibling;
				sibling.parent = null;
				return;
			}
			var grandParent:DynamicBVTreeNode = parent.parent;
			sibling.parent = grandParent;
			if (grandParent.child1 == parent) {
				grandParent.child1 = sibling;
			} else {
				grandParent.child2 = sibling;
			}
			if (numFreeNodes < 16384) {
				freeNodes[numFreeNodes++] = parent;
			}
			do {
				grandParent = balance(grandParent);
				fix(grandParent);
				grandParent = grandParent.parent;
			} while (grandParent != null);
		}
		
		private function balance(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var nh:int = node.height;
			if (nh < 2) {
				return node;
			}
			var p:DynamicBVTreeNode = node.parent;
			var l:DynamicBVTreeNode = node.child1;
			var r:DynamicBVTreeNode = node.child2;
			var lh:int = l.height;
			var rh:int = r.height;
			var balance:int = lh - rh;
			var t:int; // for bit operation
		
			//          [ N ]
			//         /     \
			//    [ L ]       [ R ]
			//     / \         / \
			// [L-L] [L-R] [R-L] [R-R]
			
			// Is this tree balanced?
			if (balance > 1) {
				var ll:DynamicBVTreeNode = l.child1;
				var lr:DynamicBVTreeNode = l.child2;
				var llh:int = ll.height;
				var lrh:int = lr.height;
				
				// Is L-L higher than L-R?
				if (llh > lrh) {
					
					// set N to L-R
					l.child2 = node;
					node.parent = l;
					
					//          [ L ]
					//         /     \
					//    [L-L]       [ N ]
					//     / \         / \
					// [...] [...] [ L ] [ R ]
					
					// set L-R
					node.child1 = lr;
					lr.parent = node;
					
					//          [ L ]
					//         /     \
					//    [L-L]       [ N ]
					//     / \         / \
					// [...] [...] [L-R] [ R ]
					
					
					// fix bounds and heights
					node.aabb.combine(lr.aabb, r.aabb);
					t = lrh - rh;
					node.height = lrh - (t & t >> 31) + 1;
					
					l.aabb.combine(ll.aabb, node.aabb);
					t = llh - nh;
					l.height = llh - (t & t >> 31) + 1;
					
				} else {
					
					// set N to L-L
					l.child1 = node;
					node.parent = l;
					
					//          [ L ]
					//         /     \
					//    [ N ]       [L-R]
					//     / \         / \
					// [ L ] [ R ] [...] [...]
					
					// set L-L
					node.child1 = ll;
					ll.parent = node;
					
					//          [ L ]
					//         /     \
					//    [ N ]       [L-R]
					//     / \         / \
					// [L-L] [ R ] [...] [...]
					
					// fix bounds and heights
					node.aabb.combine(ll.aabb, r.aabb);
					t = llh - rh;
					node.height = llh - (t & t >> 31) + 1;
					
					l.aabb.combine(node.aabb, lr.aabb);
					t = nh - lrh;
					l.height = nh - (t & t >> 31) + 1;
					
				}
				
				// set new parent of L
				if (p != null) {
					if (p.child1 == node) {
						p.child1 = l;
					} else {
						p.child2 = l;
					}
				} else {
					root = l;
				}
				l.parent = p;
				
				return l;
			} else if (balance < -1) {
				var rl:DynamicBVTreeNode = r.child1;
				var rr:DynamicBVTreeNode = r.child2;
				var rlh:int = rl.height;
				var rrh:int = rr.height;
				
				// Is R-L higher than R-R?
				if (rlh > rrh) {
					
					// set N to R-R
					r.child2 = node;
					node.parent = r;
					
					//          [ R ]
					//         /     \
					//    [R-L]       [ N ]
					//     / \         / \
					// [...] [...] [ L ] [ R ]
					
					// set R-R
					node.child2 = rr;
					rr.parent = node;
					
					//          [ R ]
					//         /     \
					//    [R-L]       [ N ]
					//     / \         / \
					// [...] [...] [ L ] [R-R]
					
					
					// fix bounds and heights
					node.aabb.combine(l.aabb, rr.aabb);
					t = lh - rrh;
					node.height = lh - (t & t >> 31) + 1;
					
					r.aabb.combine(rl.aabb, node.aabb);
					t = rlh - nh;
					r.height = rlh - (t & t >> 31) + 1;
					
				} else {
					
					// set N to R-L
					r.child1 = node;
					node.parent = r;
					
					//          [ R ]
					//         /     \
					//    [ N ]       [R-R]
					//     / \         / \
					// [ L ] [ R ] [...] [...]
					
					// set R-L
					node.child2 = rl;
					rl.parent = node;
					
					//          [ R ]
					//         /     \
					//    [ N ]       [R-R]
					//     / \         / \
					// [ L ] [R-L] [...] [...]
					
					
					// fix bounds and heights
					node.aabb.combine(l.aabb, rl.aabb);
					t = lh - rlh;
					node.height = lh - (t & t >> 31) + 1;
					
					r.aabb.combine(node.aabb, rr.aabb);
					t = nh - rrh;
					r.height = nh - (t & t >> 31) + 1;
					
				}
				
				// set new parent of R
				if (p != null) {
					if (p.child1 == node) {
						p.child1 = r;
					} else {
						p.child2 = r;
					}
				} else {
					root = r;
				}
				r.parent = p;
				
				return r;
				
			}
			return node;
		}
		
		private function balance_old(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var balance:int = getBalance(node);
			if (balance > 1) {
				if (getBalance(node.child1) < 0) {
					node.child1 = rotateLeft(node.child1);
				}
				return rotateRight(node);
			} else if (balance < -1) {
				if (getBalance(node.child2) > 0) {
					node.child2 = rotateRight(node.child2);
				}
				return rotateLeft(node);
			}
			return node;
		}
		
		private function fix(node:DynamicBVTreeNode):void {
			var c1:DynamicBVTreeNode = node.child1;
			var c2:DynamicBVTreeNode = node.child2;
			node.aabb.combine(c1.aabb, c2.aabb);
			var h1:int = c1.height;
			var h2:int = c2.height;
			if (h1 < h2) {
				node.height = h2 + 1;
			} else {
				node.height = h1 + 1;
			}
		}
		
		private function rotateRight(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var p:DynamicBVTreeNode = node.parent;
			var l:DynamicBVTreeNode = node.child1;
			var lr:DynamicBVTreeNode = l.child2;
			(node.child1 = lr).parent = node;
			fix(node);
			((l.child2 = node).parent = l).parent = p;
			fix(l);
			if (p != null) {
				if (p.child2 == node) {
					p.child2 = l;
				} else {
					p.child1 = l;
				}
				fix(p);
			} else root = l;
			return l;
		}
		
		private function rotateLeft(node:DynamicBVTreeNode):DynamicBVTreeNode {
			var p:DynamicBVTreeNode = node.parent;
			var r:DynamicBVTreeNode = node.child2;
			var rl:DynamicBVTreeNode = r.child1;
			(node.child2 = rl).parent = node;
			fix(node);
			((r.child1 = node).parent = r).parent = p;
			fix(r);
			if (p != null) {
				if (p.child1 == node) {
					p.child1 = r;
				} else {
					p.child2 = r;
				}
				fix(p);
			} else root = r;
			return r;
		}
		
	}

}