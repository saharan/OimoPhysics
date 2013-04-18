package com.element.oimo.physics.collision.broadphase {
	/**
	 * An node of the Dynamic Bounding Volume Tree.
	 * @author saharan
	 */
	public class DynamicBVTreeNode {
		/**
		 * The first child of this node.
		 */
		public var child1:DynamicBVTreeNode;
		
		/**
		 * The second child of this node.
		 */
		public var child2:DynamicBVTreeNode;
		
		/**
		 * The parent of this tree.
		 */
		public var parent:DynamicBVTreeNode;
		
		/**
		 * The proxy of this node. This has no value if this node is not leaf.
		 */
		public var proxy:Proxy;
		
		/**
		 * The balance of this node.
		 */
		public var balance:int;
		
		public function DynamicBVTreeNode() {
		}
		
	}

}