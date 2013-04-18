package com.element.oimo.physics.constraint.contact {
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * ...
	 * @author saharan
	 */
	public class ContactLink {
		public var prev:ContactLink;
		public var next:ContactLink;
		
		public var shape:Shape;
		public var body:RigidBody;
		
		public var parent:Contact;
		
		public function ContactLink(parent:Contact) {
			this.parent = parent;
		}
		
	}

}