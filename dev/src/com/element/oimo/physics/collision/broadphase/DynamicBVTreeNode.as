package com.element.oimo.physics.collision.broadphase {
	/**
	 * A node of the dynamic bounding volume tree.
	 * @author saharan
	 */
	public class DynamicBVTreeNode {
		/**
		 * The first child node of this node.
		 */
		public var child1:DynamicBVTreeNode;
		
		/**
		 * The second child node of this node.
		 */
		public var child2:DynamicBVTreeNode;
		
		/**
		 * The parent node of this tree.
		 */
		public var parent:DynamicBVTreeNode;
		
		/**
		 * The proxy of this node. This has no value if this node is not leaf.
		 */
		public var proxy:Proxy;
		
		/**
		 * The maximum distance from leaf nodes.
		 */
		public var height:int;
		
		/**
		 * The AABB of this node.
		 */
		public var aabb:AABB;
		
		public function DynamicBVTreeNode() {
			aabb = new AABB();
		}
		
	}

}