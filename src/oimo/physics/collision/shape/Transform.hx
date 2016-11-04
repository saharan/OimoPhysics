package oimo.physics.collision.shape;
import oimo.m.IMat3;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Mat3;
import oimo.math.Vec3;

/**
 * Transform class provides a set of translation and rotation.
 */
@:expose("OIMO.Transform")
#if !macro
@:build(oimo.m.B.bu())
#end
class Transform {
	public var _origin:IVec3;
	public var _rotation:IMat3;

	public function new() {
		M.vec3_zero(_origin);
		M.mat3_id(_rotation);
	}

	public function identity():Void {
		M.vec3_zero(_origin);
		M.mat3_id(_rotation);
	}

	public function getOrigin():Vec3 {
		var origin:Vec3 = new Vec3();
		M.vec3_toVec3(origin, _origin);
		return origin;
	}

	public function setOrigin(origin:Vec3):Void {
		M.vec3_fromVec3(_origin, origin);
	}

	public function getRotation():Mat3 {
		var rotation:Mat3 = new Mat3();
		M.mat3_toMat3(rotation, _rotation);
		return rotation;
	}

	public function setRotation(rotation:Mat3):Void {
		M.mat3_fromMat3(_rotation, rotation);
	}

}
