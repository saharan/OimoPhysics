package oimo.physics.dynamics.constraint.contact;
import haxe.ds.Vector;
import oimo.m.IMat3;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.math.Vec3;
import oimo.physics.collision.narrowphase.Manifold;
import oimo.physics.collision.narrowphase.ManifoldPoint;
import oimo.physics.dynamics.constraint.Constraint;
import oimo.physics.dynamics.rigidbody.Component;
import oimo.physics.dynamics.rigidbody.RigidBody;

/**
 * Contact Constraint
 */
@:expose("OIMO.ContactConstraint")
@:build(oimo.m.B.bu())
class ContactConstraint extends Constraint {
	public var _numPoints:Int;
	public var _normal:IVec3;
	public var _tangent:IVec3;
	public var _binormal:IVec3;
	public var _points:Vector<ContactPoint>;

	public var _c1:Component;
	public var _c2:Component;
	public var _b1:RigidBody;
	public var _b2:RigidBody;
	public var _invM1:Float;
	public var _invM2:Float;
	public var _friction:Float;
	public var _restitution:Float;

	public var _invI1:IMat3;
	public var _invI2:IMat3;

	public function new() {
		super();

		// initialize contact points
		_numPoints = 0;
		M.vec3_zero(_normal);
		_points = new Vector<ContactPoint>(Settings.maxManifoldPoints);
		for (i in 0...Settings.maxManifoldPoints) {
			_points[i] = new ContactPoint();
		}
	}

	@:extern
	public inline function _clear():Void {
		_numPoints = 0;
	}

	@:extern
	public inline function _update(manifold:Manifold):Void {
		// TODO: warm starting
		if (manifold._incremental) {
			_incrementalUpdate(manifold);
		} else {
			_totalUpdate(manifold);
		}

		// compute tangent and binormal
		var nx:Float = M.vec3_get(_normal, 0);
		var ny:Float = M.vec3_get(_normal, 1);
		var nz:Float = M.vec3_get(_normal, 2);
		var nx2:Float = nx * nx;
		var ny2:Float = ny * ny;
		var nz2:Float = nz * nz;
		var tx:Float;
		var ty:Float;
		var tz:Float;
		var bx:Float;
		var by:Float;
		var bz:Float;
		M.compare3min(nx2, ny2, nz2, {
			var invL:Float = 1 / MathUtil.sqrt(ny2 + nz2);
			tx = 0;
			ty = -nz * invL;
			tz = ny * invL;
			bx = ny * tz - nz * ty;
			by = -nx * tz;
			bz = nx * ty;
		}, {
			var invL:Float = 1 / MathUtil.sqrt(nx2 + nz2);
			tx = nz * invL;
			ty = 0;
			tz = -nx * invL;
			bx = ny * tz;
			by = nz * tx - nx * tz;
			bz = -ny * tx;
		}, {
			var invL:Float = 1 / MathUtil.sqrt(nx2 + ny2);
			tx = -ny * invL;
			ty = nx * invL;
			tz = 0;
			bx = -nz * ty;
			by = nz * tx;
			bz = nx * ty - ny * tx;
		});
		M.vec3_set(_tangent, tx, ty, tz);
		M.vec3_set(_binormal, bx, by, bz);
	}

	@:extern
	inline function _incrementalUpdate(manifold:Manifold):Void {
		// TODO: incremental update
	}

	@:extern
	inline function _totalUpdate(info:Manifold):Void {
		var numPrev:Int = _numPoints;
		var num:Int = info._numPoints;
		_numPoints = num;

		M.vec3_assign(_normal, info._normal);
		for (i in 0...num) {
			var cp:ContactPoint = _points[i];
			var mp:ManifoldPoint = info._points[i];
			cp._initialize(mp);
		}
	}

	@:extern
	public inline function _attach(c1:Component, c2:Component):Void {
		_c1 = c1;
		_c2 = c2;
		_b1 = _c1._rigidBody;
		_b2 = _c2._rigidBody;
	}

	@:extern
	public inline function _detach():Void {
		_c1 = null;
		_c2 = null;
		_b1 = null;
		_b2 = null;
	}

	override public function preSolve():Void {
		// calculate mass data
		_invM1 = _b1._invMass;
		_invM2 = _b2._invMass;
		M.mat3_assign(_invI1, _b1._invInertia);
		M.mat3_assign(_invI2, _b2._invInertia);
		_friction = MathUtil.sqrt(_c1._friction * _c2._friction);
		_restitution = MathUtil.sqrt(_c1._restitution * _c2._restitution);

		computeData();
	}

	override public function solveVelocity():Void {
		// solve friction
		for (i in 0..._numPoints) {
			var cp:ContactPoint = _points[i];
			var rvt:Float = 0;
			rvt += M.vec3_dot(_b1._linearVel, _tangent);
			rvt -= M.vec3_dot(_b2._linearVel, _tangent);
			rvt += M.vec3_dot(_b1._angularVel, cp._torqueT1);
			rvt -= M.vec3_dot(_b2._angularVel, cp._torqueT2);
			var rvb:Float = 0;
			rvb += M.vec3_dot(_b1._linearVel, _binormal);
			rvb -= M.vec3_dot(_b2._linearVel, _binormal);
			rvb += M.vec3_dot(_b1._angularVel, cp._torqueB1);
			rvb -= M.vec3_dot(_b2._angularVel, cp._torqueB2);

			var tangentImpulse:Float = -(rvt * cp._invEffectiveMassTT + rvb * cp._invEffectiveMassTB);
			var binormalImpulse:Float = -(rvt * cp._invEffectiveMassTB + rvb * cp._invEffectiveMassBB);

			var oldTangentImpulse:Float = cp._tangentImpulse;
			var oldBinormalImpulse:Float = cp._binormalImpulse;
			cp._tangentImpulse += tangentImpulse;
			cp._binormalImpulse += binormalImpulse;

			// cone friction
			var maxImpulse:Float = _friction * cp._normalImpulse;
			var impulseLengthSq:Float = cp._tangentImpulse * cp._tangentImpulse + cp._binormalImpulse * cp._binormalImpulse;
			if (M.gt0(impulseLengthSq) && impulseLengthSq > maxImpulse * maxImpulse) {
				var invL:Float = maxImpulse / MathUtil.sqrt(impulseLengthSq);
				cp._tangentImpulse *= invL;
				cp._binormalImpulse *= invL;
			}

			var deltaTangentImpulse:Float = cp._tangentImpulse - oldTangentImpulse;
			var deltaBinormalImpulse:Float = cp._binormalImpulse - oldBinormalImpulse;

			M.vec3_addRhsScaled(_b1._linearVel, _b1._linearVel, cp._invMassLinT1, deltaTangentImpulse);
			M.vec3_addRhsScaled(_b2._linearVel, _b2._linearVel, cp._invMassLinT2, -deltaTangentImpulse);
			M.vec3_addRhsScaled(_b1._angularVel, _b1._angularVel, cp._invMassAngT1, deltaTangentImpulse);
			M.vec3_addRhsScaled(_b2._angularVel, _b2._angularVel, cp._invMassAngT2, -deltaTangentImpulse);
			M.vec3_addRhsScaled(_b1._linearVel, _b1._linearVel, cp._invMassLinB1, deltaBinormalImpulse);
			M.vec3_addRhsScaled(_b2._linearVel, _b2._linearVel, cp._invMassLinB2, -deltaBinormalImpulse);
			M.vec3_addRhsScaled(_b1._angularVel, _b1._angularVel, cp._invMassAngB1, deltaBinormalImpulse);
			M.vec3_addRhsScaled(_b2._angularVel, _b2._angularVel, cp._invMassAngB2, -deltaBinormalImpulse);
		}
		// solve normal
		for (i in 0..._numPoints) {
			var cp:ContactPoint = _points[i];
			var rvn:Float = 0;
			rvn += M.vec3_dot(_b1._linearVel, _normal);
			rvn -= M.vec3_dot(_b2._linearVel, _normal);
			rvn += M.vec3_dot(_b1._angularVel, cp._torqueN1);
			rvn -= M.vec3_dot(_b2._angularVel, cp._torqueN2);
			var normalImpulse:Float = (cp._normalBias - rvn) * cp._invEffectiveMassN;

			var oldNormalImpulse:Float = cp._normalImpulse;
			cp._normalImpulse += normalImpulse;
			if (cp._normalImpulse < 0) cp._normalImpulse = 0;
			var deltaNormalImpulse:Float = cp._normalImpulse - oldNormalImpulse;

			M.vec3_addRhsScaled(_b1._linearVel, _b1._linearVel, cp._invMassLinN1, deltaNormalImpulse);
			M.vec3_addRhsScaled(_b2._linearVel, _b2._linearVel, cp._invMassLinN2, -deltaNormalImpulse);
			M.vec3_addRhsScaled(_b1._angularVel, _b1._angularVel, cp._invMassAngN1, deltaNormalImpulse);
			M.vec3_addRhsScaled(_b2._angularVel, _b2._angularVel, cp._invMassAngN2, -deltaNormalImpulse);
		}
	}

	@:extern
	inline function computeData():Void {
		var lv1:IVec3;
		var lv2:IVec3;
		var av1:IVec3;
		var av2:IVec3;
		M.vec3_assign(lv1, _b1._linearVel);
		M.vec3_assign(lv2, _b2._linearVel);
		M.vec3_assign(av1, _b1._angularVel);
		M.vec3_assign(av2, _b2._angularVel);

		for (i in 0..._numPoints) {
			var cp:ContactPoint = _points[i];
			var relPos1:IVec3;
			var relPos2:IVec3;
			M.vec3_assign(relPos1, cp._relPos1);
			M.vec3_assign(relPos2, cp._relPos2);

			// velocities at manifold point
			var v1:IVec3;
			var v2:IVec3;

			// vel = cross(angularVel, relPos) + linearVel
			M.vec3_cross(v1, av1, relPos1);
			M.vec3_cross(v2, av2, relPos2);
			M.vec3_add(v1, v1, lv1);
			M.vec3_add(v2, v2, lv2);

			var relVel:IVec3;
			M.vec3_sub(relVel, v1, v2);

			var rvn:Float;
			rvn = M.vec3_dot(_normal, relVel);

			if (rvn < -Settings.contactBounceThreshold) {
				cp._normalBias = -rvn * _restitution;
			} else {
				cp._normalBias = 0;
			}

			M.vec3_scale(cp._invMassLinN1, _normal, _invM1);
			M.vec3_scale(cp._invMassLinN2, _normal, _invM2);
			M.vec3_scale(cp._invMassLinT1, _tangent, _invM1);
			M.vec3_scale(cp._invMassLinT2, _tangent, _invM2);
			M.vec3_scale(cp._invMassLinB1, _binormal, _invM1);
			M.vec3_scale(cp._invMassLinB2, _binormal, _invM2);

			M.vec3_cross(cp._torqueN1, relPos1, _normal);
			M.vec3_cross(cp._torqueN2, relPos2, _normal);
			M.vec3_mulMat3(cp._invMassAngN1, cp._torqueN1, _invI1);
			M.vec3_mulMat3(cp._invMassAngN2, cp._torqueN2, _invI2);

			M.vec3_cross(cp._torqueT1, relPos1, _tangent);
			M.vec3_cross(cp._torqueT2, relPos2, _tangent);
			M.vec3_mulMat3(cp._invMassAngT1, cp._torqueT1, _invI1);
			M.vec3_mulMat3(cp._invMassAngT2, cp._torqueT2, _invI2);

			M.vec3_cross(cp._torqueB1, relPos1, _binormal);
			M.vec3_cross(cp._torqueB2, relPos2, _binormal);
			M.vec3_mulMat3(cp._invMassAngB1, cp._torqueB1, _invI1);
			M.vec3_mulMat3(cp._invMassAngB2, cp._torqueB2, _invI2);

			cp._invEffectiveMassN = 1 / (_invM1 + _invM2 + M.vec3_dot(cp._invMassAngN1, cp._torqueN1) + M.vec3_dot(cp._invMassAngN2, cp._torqueN2));

			// compute effective mass matrix for friction
			var effectiveMassTT:Float = _invM1 + _invM2 + M.vec3_dot(cp._invMassAngT1, cp._torqueT1) + M.vec3_dot(cp._invMassAngT2, cp._torqueT2);
			var effectiveMassTB:Float = M.vec3_dot(cp._invMassAngT1, cp._torqueB1) + M.vec3_dot(cp._invMassAngT2, cp._torqueB2);
			var effectiveMassBB:Float = _invM1 + _invM2 + M.vec3_dot(cp._invMassAngB1, cp._torqueB1) + M.vec3_dot(cp._invMassAngB2, cp._torqueB2);

			var invDet:Float = 1 / (effectiveMassTT * effectiveMassBB - effectiveMassTB * effectiveMassTB);

			cp._invEffectiveMassTT = effectiveMassBB * invDet;
			cp._invEffectiveMassTB = -effectiveMassTB * invDet;
			cp._invEffectiveMassBB = effectiveMassTT * invDet;

			// TODO: warm starting
			cp._normalImpulse = 0;
			cp._tangentImpulse = 0;
			cp._binormalImpulse = 0;
		}
	}
}
