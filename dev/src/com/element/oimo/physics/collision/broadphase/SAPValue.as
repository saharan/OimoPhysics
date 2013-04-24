package com.element.oimo.physics.collision.broadphase {
	/**
	 * ...
	 * @author saharan
	 */
	public class SAPValue {
		public var value:Number;
		public var value2:int;
		public var value3:int;
		public var aabb:Proxy;
		public var pair:SAPValue;
		
		public function SAPValue(aabb:Proxy) {
			this.aabb = aabb;
			value = 0;
		}
		
	}

}