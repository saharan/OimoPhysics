package com.element.oimo.physics.constraint.contact {
	import com.element.oimo.math.Vec3;
	/**
	 * The class holds details of the contact point.
	 * @author saharan
	 */
	public class ManifoldPoint {
		/**
		 * The position of this manifold point.
		 */
		public var position:Vec3;
		
		/**
		 * The relative position to the first shape.
		 */
		public var localRelativePosition1:Vec3;
		
		/**
		 * The relative position to the second shape.
		 */
		public var localRelativePosition2:Vec3;
		
		/**
		 * The normal vector of this manifold point.
		 */
		public var normal:Vec3;
		
		/**
		 * The tangent vector of this manifold point.
		 */
		public var tangent:Vec3;
		
		/**
		 * The binormal vector of this manifold point.
		 */
		public var binormal:Vec3;
		
		/**
		 * The impulse in normal direction.
		 */
		public var normalImpulse:Number;
		
		/**
		 * The impulse in tangent direction.
		 */
		public var tangentImpulse:Number;
		
		/**
		 * The impulse in binormal direction.
		 */
		public var binormalImpulse:Number;
		
		/**
		 * The denominator in normal direction.
		 */
		public var normalDenominator:Number;
		
		/**
		 * The denominator in tangent direction.
		 */
		public var tangentDenominator:Number;
		
		/**
		 * The denominator in binormal direction.
		 */
		public var binormalDenominator:Number;
		
		/**
		 * Whether this manifold point is persisting.
		 */
		public var warmStarted:Boolean;
		
		/**
		 * The depth of penetration.
		 */
		public var penetration:Number;
		
		public function ManifoldPoint() {
			position = new Vec3();
			localRelativePosition1 = new Vec3();
			localRelativePosition2 = new Vec3();
			normal = new Vec3();
			tangent = new Vec3();
			binormal = new Vec3();
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
			normalDenominator = 0;
			tangentDenominator = 0;
			binormalDenominator = 0;
			penetration = 0;
		}
		
	}

}