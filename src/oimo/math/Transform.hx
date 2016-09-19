package oimo.math;

/**
 * Transform class provides a set of translation and rotation.
 */
@:expose("OIMO.Transform")
class Transform {
	public var origin:Vec3;
	public var rotation:Mat3;

	public function new(origin:Vec3 = null, rotation:Mat3 = null) {
		if (origin != null) {
			this.origin = origin.clone();
		} else {
			this.origin = new Vec3();
		}
		if (rotation != null) {
			this.rotation = rotation.clone();
		} else {
			this.rotation = new Mat3();
		}
	}

	/**
	 * Sets the origin and the rotation matrix at once, and returns `this`.
	 */
	public function init(origin:Vec3, rotation:Mat3):Transform {
		this.origin.copyFrom(origin);
		this.rotation.copyFrom(rotation);
		return this;
	}

	/**
	 * Sets this transformation to identity transformation and returns `this`.
	 */
	public function identity():Transform {
		this.origin.zero();
		this.rotation.identity();
		return this;
	}

	/**
	 * Copies values from `t` and returns `this`.
	 */
	public inline function copyFrom(t:Transform):Transform {
		origin.copyFrom(t.origin);
		rotation.copyFrom(t.rotation);
		return this;
	}

	/**
	 * Returns a clone of the transform.
	 */
	public inline function clone():Transform {
		return new Transform(origin, rotation);
	}

}
