package oimo.physics.dynamics.constraint.contact;
import oimo.m.IMat3;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.physics.collision.narrowphase.ManifoldPoint;

/**
 * ...
 */
@:expose("OIMO.ContactPoint")
@:build(oimo.m.B.bu())
class ContactPoint {
	public var _relPos1:IVec3;
	public var _relPos2:IVec3;
	public var _penetration:Float;

	// impulse along the normal -> velocity difference
	public var _invMassLinN1:IVec3;
	public var _invMassLinN2:IVec3;
	public var _invMassAngN1:IVec3;
	public var _invMassAngN2:IVec3;

	// impulse along the tangent -> velocity difference
	public var _invMassLinT1:IVec3;
	public var _invMassLinT2:IVec3;
	public var _invMassAngT1:IVec3;
	public var _invMassAngT2:IVec3;

	// impulse along the binormal -> velocity difference
	public var _invMassLinB1:IVec3;
	public var _invMassLinB2:IVec3;
	public var _invMassAngB1:IVec3;
	public var _invMassAngB2:IVec3;

	// cross(relative position, normal)
	public var _torqueN1:IVec3;
	public var _torqueN2:IVec3;

	// cross(relative position, tangent)
	public var _torqueT1:IVec3;
	public var _torqueT2:IVec3;

	// cross(relative position, binormal)
	public var _torqueB1:IVec3;
	public var _torqueB2:IVec3;

	// inverse normal-normal effective mass
	public var _invEffectiveMassN:Float;
	// inverse tangent-tangent effective mass
	public var _invEffectiveMassTT:Float;
	// inverse tangent-binormal (= binormal-tangent) effective mass
	public var _invEffectiveMassTB:Float;
	// inverse binormal-binormal effective mass
	public var _invEffectiveMassBB:Float;

	public var _normalBias:Float;

	public var _normalImpulse:Float;
	public var _tangentImpulse:Float;
	public var _binormalImpulse:Float;

	public function new() {
		M.vec3_zero(_relPos1);
		M.vec3_zero(_relPos2);
		M.vec3_zero(_invMassLinN1);
		M.vec3_zero(_invMassLinN2);
		M.vec3_zero(_invMassAngN1);
		M.vec3_zero(_invMassAngN2);
		M.vec3_zero(_invMassLinT1);
		M.vec3_zero(_invMassLinT2);
		M.vec3_zero(_invMassAngT1);
		M.vec3_zero(_invMassAngT2);
		M.vec3_zero(_invMassLinB1);
		M.vec3_zero(_invMassLinB2);
		M.vec3_zero(_invMassAngB1);
		M.vec3_zero(_invMassAngB2);
		M.vec3_zero(_torqueN1);
		M.vec3_zero(_torqueN2);
		M.vec3_zero(_torqueT1);
		M.vec3_zero(_torqueT2);
		M.vec3_zero(_torqueB1);
		M.vec3_zero(_torqueB2);
		_penetration = 0;
		_normalBias = 0;
		_normalImpulse = 0;
		_tangentImpulse = 0;
		_binormalImpulse = 0;
	}

	@:extern
	public inline function _clear():Void {
		M.vec3_zero(_relPos1);
		M.vec3_zero(_relPos2);
		_penetration = 0;
		_normalImpulse = 0;
		_tangentImpulse = 0;
		_binormalImpulse = 0;
	}

	@:extern
	public inline function _initialize(info:ManifoldPoint):Void {
		M.vec3_assign(_relPos1, info._relPos1);
		M.vec3_assign(_relPos2, info._relPos2);
		_penetration = info._penetration;
		_normalImpulse = 0;
		_tangentImpulse = 0;
		_binormalImpulse = 0;
	}

}
