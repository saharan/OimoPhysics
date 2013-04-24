package com.element.oimo.physics.collision.broadphase {
	/**
	 * An axis-aligned bounding box.
	 * @author saharan
	 */
	public class AABB {
		public var minX:Number;
		public var maxX:Number;
		public var minY:Number;
		public var maxY:Number;
		public var minZ:Number;
		public var maxZ:Number;
		
		public function AABB(
			minX:Number = 0, maxX:Number = 0,
			minY:Number = 0, maxY:Number = 0,
			minZ:Number = 0, maxZ:Number = 0
		) {
			this.minX = minX;
			this.maxX = maxX;
			this.minY = minY;
			this.maxY = maxY;
			this.minZ = minZ;
			this.maxZ = maxZ;
		}
		
		/**
		 * Combine two AABBs.
		 * @param	aabb1
		 * @param	aabb2
		 */
		public function combine(aabb1:AABB, aabb2:AABB):void {
			if (aabb1.minX < aabb2.minX) {
				minX = aabb1.minX;
			} else {
				minX = aabb2.minX;
			}
			if (aabb1.maxX > aabb2.maxX) {
				maxX = aabb1.maxX;
			} else {
				maxX = aabb2.maxX;
			}
			if (aabb1.minY < aabb2.minY) {
				minY = aabb1.minY;
			} else {
				minY = aabb2.minY;
			}
			if (aabb1.maxY > aabb2.maxY) {
				maxY = aabb1.maxY;
			} else {
				maxY = aabb2.maxY;
			}
			if (aabb1.minZ < aabb2.minZ) {
				minZ = aabb1.minZ;
			} else {
				minZ = aabb2.minZ;
			}
			if (aabb1.maxZ > aabb2.maxZ) {
				maxZ = aabb1.maxZ;
			} else {
				maxZ = aabb2.maxZ;
			}
			var margin:Number = 0;
			minX -= margin;
			minY -= margin;
			minZ -= margin;
			maxX += margin;
			maxY += margin;
			maxZ += margin;
		}
		
		/**
		 * Get the surface area of this AABB.
		 * @return
		 */
		public function surfaceArea():Number {
			var h:Number = maxY - minY;
			var d:Number = maxZ - minZ;
			return 2 * ((maxX - minX) * (h + d) + h * d);
		}
		
		/**
		 * Returns whether this AABB intersects with the point or not.
		 * @param	x
		 * @param	y
		 * @param	z
		 * @return
		 */
		public function intersectsWithPoint(x:Number, y:Number, z:Number):Boolean {
			return x >= minX && x <= maxX && y >= minY && y <= maxY && z >= minZ && z <= maxZ;
		}
		
	}

}