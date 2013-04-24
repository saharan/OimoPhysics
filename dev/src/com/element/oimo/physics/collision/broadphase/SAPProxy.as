package com.element.oimo.physics.collision.broadphase {
	import com.element.oimo.physics.collision.shape.Shape;
	/**
	 * A broad-phase proxy for sweep and prune algorithm.
	 * @author saharan
	 */
	public class SAPProxy {
		/**
		 * The minimum value of x
		 */
		public var minX:SAPValue;
		
		/**
		 * The maximum value of x
		 */
		public var maxX:SAPValue;
		
		/**
		 * The minimum value of y
		 */
		public var minY:SAPValue;
		
		/**
		 * The maximum value of y
		 */
		public var maxY:SAPValue;
		
		/**
		 * The minimum value of z
		 */
		public var minZ:SAPValue;
		
		/**
		 * The maximum value of z
		 */
		public var maxZ:SAPValue;
		
		public var parent:Proxy;
		
		public var next:SAPProxy;
		
		public function SAPProxy(parent:Proxy) {
			this.parent = parent;
			minX = new SAPValue(parent);
			maxX = new SAPValue(parent);
			minY = new SAPValue(parent);
			maxY = new SAPValue(parent);
			minZ = new SAPValue(parent);
			maxZ = new SAPValue(parent);
			minX.pair = maxX;
			maxX.pair = minX;
			minY.pair = maxY;
			maxY.pair = minY;
			minZ.pair = maxZ;
			maxZ.pair = minZ;
		}
		
		public function update():void {
			minX.value = parent.minX;
			maxX.value = parent.maxX;
			minY.value = parent.minY;
			maxY.value = parent.maxY;
			minZ.value = parent.minZ;
			maxZ.value = parent.maxZ;
			
			minX.value2 = floor(minX.value * 10000 + 0x800000);
			maxX.value2 = ceil(maxX.value * 10000 + 0x800000) | 0x40000000;
			minY.value2 = floor(minY.value * 10000 + 0x800000);
			maxY.value2 = ceil(maxY.value * 10000 + 0x800000) | 0x40000000;
			minZ.value2 = floor(minZ.value * 10000 + 0x800000);
			maxZ.value2 = ceil(maxZ.value * 10000 + 0x800000) | 0x40000000;
		}
		
		private function floor(v:Number):int {
			return v > 0 ? int(v) : int(v + 0.999999999);
		}
		
		private function ceil(v:Number):int {
			return v < 0 ? int(v) : int(v + 0.999999999);
		}
		
	}

}