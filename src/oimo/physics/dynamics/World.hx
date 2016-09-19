package oimo.physics.dynamics;
import oimo.m.M;
import oimo.physics.collision.Pair;
import oimo.physics.collision.PairManager;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.BroadPhaseType;
import oimo.physics.collision.broadphase.bruteforce.BruteForceBroadPhase;

/**
 * Physics world.
 */
@:expose("OIMO.World")
@:build(oimo.m.B.build())
class World {
	public var _rigidBodyList:RigidBody;
	public var _rigidBodyListLast:RigidBody;

	public var _pairManager:PairManager;

	public var _broadPhase:BroadPhase;

	var _componentIdCount:Int;

	public function new(?broadPhaseType:BroadPhaseType) {
		if (broadPhaseType == null) broadPhaseType = BruteForce;
		switch(broadPhaseType) {
		case BruteForce:
			_broadPhase = new BruteForceBroadPhase();
		}
		_pairManager = new PairManager(_broadPhase);
		_rigidBodyList = null;
		_rigidBodyListLast = null;

		_componentIdCount = 0;
	}

	inline function _updateContacts():Void {
		// update pairs
		_pairManager.updatePairs();
		var p:Pair = _pairManager._pairList;
		M.list_foreach(p, _next, {
			// TODO: narrowphase
		});
	}

	inline function _solveIslands():Void {
		// TODO: saharan
	}

	public inline function _addComponent(component:Component):Void {
		component._id = _componentIdCount++;
		_broadPhase._addComponent(component);
	}

	public inline function _removeComponent(component:Component):Void {
		_broadPhase._removeComponent(component);
		component._id = -1;
	}

	/**
	 * @deprecated
	 */
	inline function _integrate(timeStep:Float):Void {
		var r:RigidBody = _rigidBodyList;
		M.list_foreach(r, _next, {
			r._integrate(timeStep);
			r._syncComponents();
		});
	}

	// --- public ---

	public function step(timeStep:Float):Void {
		_updateContacts();
		//_solveIslands(timeStep);
		_integrate(timeStep);
	}

	public function addRigidBody(rigidBody:RigidBody):Void {
		M.list_push(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		var c:Component = rigidBody._componentList;
		M.list_foreach(c, _next, {
			_addComponent(c);
		});
	}

	public function removeRigidBody(rigidBody:RigidBody):Void {
		M.list_remove(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		var c:Component = rigidBody._componentList;
		M.list_foreach(c, _next, {
			_removeComponent(c);
		});
	}

}
