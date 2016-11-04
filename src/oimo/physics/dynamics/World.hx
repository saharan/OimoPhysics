package oimo.physics.dynamics;
import haxe.ds.Vector;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.Pair;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.PairManager;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.BroadPhaseType;
import oimo.physics.collision.broadphase.bruteforce.BruteForceBroadPhase;
import oimo.physics.collision.broadphase.bvh.BVHBroadPhase;
import oimo.physics.dynamics.constraint.Constraint;
import oimo.physics.dynamics.rigidbody.Component;
import oimo.physics.dynamics.rigidbody.RigidBody;

/**
 * Physics world.
 */
@:expose("OIMO.World")
@:build(oimo.m.B.bu())
class World {
	public var _numRigidBodies:Int;
	public var _rigidBodyList:RigidBody;
	public var _rigidBodyListLast:RigidBody;

	public var _numComponents:Int;

	public var _pairManager:PairManager;

	public var _broadPhase:BroadPhase;

	public var _numVelocityIterations:Int;

	public var _gravity:IVec3;

	var _constraints:Vector<Constraint>;

	var _componentIdCount:Int;

	public function new(?broadPhaseType:BroadPhaseType, gravity:Vec3 = null) {
		if (broadPhaseType == null) broadPhaseType = BVH;
		switch(broadPhaseType) {
		case BruteForce:
			_broadPhase = new BruteForceBroadPhase();
		case BVH:
			_broadPhase = new BVHBroadPhase();

		}
		_pairManager = new PairManager(_broadPhase);

		if (gravity == null) gravity = new Vec3(0, -9.80665, 0);
		M.vec3_fromVec3(_gravity, gravity);

		_numRigidBodies = 0;
		_rigidBodyList = null;
		_rigidBodyListLast = null;

		_constraints = new Vector<Constraint>(1024);
		_numVelocityIterations = 10;

		_numComponents = 0;

		_componentIdCount = 0;
	}

	@:extern
	inline function _updateContacts():Void {
		// update pairs (broad phase)
		Profile.broadPhaseTime = M.profile({
			_pairManager._updatePairs();
		});
		// update contacts (narrow phase)
		Profile.narrowPhaseTime = M.profile({
			var p:Pair = _pairManager._pairList;
			M.list_foreach(p, _next, {
				p._updateManifold();
			});
		});
	}

	inline function _solveIslands():Void {
		// TODO: saharan
	}

	@:extern
	public inline function _addComponent(component:Component):Void {
		component._proxy = _broadPhase.createProxy(component, component._aabb);
		component._id = _componentIdCount++;

		_numComponents++;
	}

	@:extern
	public inline function _removeComponent(component:Component):Void {
		_broadPhase.destroyProxy(component._proxy);
		component._proxy = null;
		component._id = -1;

		// destroy linked pairs
		var pl:PairLink = component._pairLink;
		M.list_foreach(pl, _next, {
			_pairManager._destroyPair(pl._pair);
		});
		M.assert(component._numPairs == 0);
		M.assert(component._pairLink == null);

		_numComponents--;
	}

	/**
	 * @deprecated
	 */
	inline function _integrate(timeStep:Float):Void {
		Profile.integratingTime = M.profile({
			var r:RigidBody = _rigidBodyList;
			M.list_foreach(r, _next, {
				r._integrate(timeStep);
				r._syncComponents();
				if (r._type == Dynamic) {
					M.vec3_addRhsScaled(r._linearVel, r._linearVel, _gravity, timeStep);
				}
			});
		});
	}

	/**
	 * @deprecated
	 */
	@:extern
	inline function _solveAll():Void {
		var maxConstraints:Int = _pairManager._numPairs;
		while (_constraints.length < maxConstraints) {
			_constraints = new Vector<Constraint>(_constraints.length * 2);
		}

		// collect constraints
		var numConstraints:Int = 0;
		var p:Pair = _pairManager._pairList;
		M.list_foreach(p, _next, {
			if (p._touching) {
				_constraints[numConstraints] = p._contactConstraint;
				numConstraints++;
			}
		});

		// solve
		for (i in 0...numConstraints) {
			_constraints[i].preSolve();
		}
		for (t in 0..._numVelocityIterations) {
			for (i in 0...numConstraints) {
				_constraints[i].solveVelocity();
			}
		}
		for (i in 0...numConstraints) {
			_constraints[i].postSolveVelocity();
		}

		// TODO: position iteration

		while (numConstraints > 0) {
			_constraints[--numConstraints] = null;
		}
	}

	// --- public ---

	public function step(timeStep:Float):Void {
		Profile.totalTime = M.profile({
			_updateContacts();
			_solveAll();
			//_solveIslands(timeStep);
			_integrate(timeStep);
		});
	}

	public function addRigidBody(rigidBody:RigidBody):Void {
		// first, add the rigid body to the world
		M.list_push(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		rigidBody._world = this;

		// then add the components to the world
		var c:Component = rigidBody._componentList;
		M.list_foreach(c, _next, {
			_addComponent(c);
		});

		_numRigidBodies++;
	}

	public function removeRigidBody(rigidBody:RigidBody):Void {
		// first, remove the rigid body from the world
		M.list_remove(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		rigidBody._world = null;

		// then remove the components from the world
		var c:Component = rigidBody._componentList;
		M.list_foreach(c, _next, {
			_removeComponent(c);
		});

		_numRigidBodies--;
	}

}
