package oimo.physics.collision.shape;
import oimo.m.IMat3;
import oimo.m.ITransform;

/**
 * Collision shape.
 */
@:expose("OIMO.Shape")
@:build(oimo.m.B.build())
class Shape {
	public var _type:ShapeType;
	public var _volume:Float;
	public var _inertiaCoeff:IMat3; // I / mass

	function new(type:ShapeType) {
		_type = type;
	}

	public function _updateMass():Void {
	}

	public function _computeAABB(aabb:AABB, tf:ITransform):Void {
	}

}
