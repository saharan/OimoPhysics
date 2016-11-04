package oimo.physics.collision;
import oimo.m.M;
import oimo.physics.collision.Pair;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.ProxyPair;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.narrowphase.NarrowPhase;
import oimo.physics.collision.shape.AABB;
import oimo.physics.dynamics.rigidbody.Component;
import oimo.physics.dynamics.rigidbody.RigidBody;

/**
 * A manager of the overlapping component pairs.
 */
@:expose("OIMO.PairManager")
class PairManager {
	public var _numPairs:Int;
	public var _pairList:Pair;
	public var _pairListLast:Pair;
	public var _pairPool:Pair;

	public var _broadPhase:BroadPhase;
	public var _narrowPhase:NarrowPhase;

	public function new(broadPhase:BroadPhase) {
		_broadPhase = broadPhase;
		_narrowPhase = new NarrowPhase();
		_numPairs = 0;
	}

	public function _updatePairs():Void {
		// whether the broadphase　to return only new overlapping pairs
		var incremental:Bool = _broadPhase._incremental;

		// collect new pairs
		_broadPhase.collectPairs();

		// create pairs
		var pp:ProxyPair = _broadPhase._proxyPairList;
		M.list_foreach(pp, _next, {
			do {
				var c1:Component;
				var c2:Component;
				M.assert(pp._p1._id != pp._p2._id);
				if (pp._p1._id < pp._p2._id) {
					c1 = cast pp._p1._userData;
					c2 = cast pp._p2._userData;
				} else {
					c1 = cast pp._p2._userData;
					c2 = cast pp._p1._userData;
				}
				// collision filtering
				if (!shouldCollide(c1, c2)) {
					break;
				}

				// search for the same pair
				var n1:Int = c1._numPairs;
				var n2:Int = c2._numPairs;
				var l:PairLink;
				// select shorter linked list
				if (n1 < n2) {
					l = c1._pairLink;
				} else {
					l = c2._pairLink;
				}
				var id1:Int = c1._id;
				var id2:Int = c2._id;
				var found:Bool = false;
				M.list_foreach(l, _next, {
					var p:Pair = l._pair;
					if (p._c1._id == id1 && p._c2._id == id2) {
						// the same pair found
						p._latest = true;
						found = true;
						break;
					}
				});

				// if not found, create a new pair
				if (!found) {
					// trying to pick an object up from the pool
					var p:Pair = M.singleList_pick(_pairPool, _next, new Pair());
					M.list_push(_pairList, _pairListLast, _prev, _next, p);
					p._latest = true;
					p._attach(c1, c2, _narrowPhase.getDetector(c1._shape._type, c2._shape._type));
					_numPairs++;
				}
			} while (false);
		});

		// destroy separeted pairs
		var p:Pair = _pairList;
		M.list_foreach(p, _next, {
			do {
				if (p._latest) {
					// the pair is overlapping, make it old for the next step
					p._latest = false;
					p._shouldBeSkipped = false;
					break;
				}
				if (!incremental) {
					// the pair is separated, because the broad-phase algorithm collects
					// all the overlapping pairs and they are marked as latest
					_destroyPair(p);
					break;
				}

				var c1:Component = p._c1;
				var c2:Component = p._c2;
				var r1:RigidBody = c1._rigidBody;
				var r2:RigidBody = c2._rigidBody;
				var active1:Bool = !r1._sleeping && r1._type != Static;
				var active2:Bool = !r2._sleeping && r2._type != Static;
				if (!active1 && !active2) {
					// skip the pair if both rigid bodies are inactive
					p._shouldBeSkipped = true;
					break;
				}
				var aabb1:AABB = c1._aabb;
				var aabb2:AABB = c2._aabb;
				if (!_broadPhase.isOverlapping(c1._proxy, c2._proxy)) {
					// the proxy pair is separated
					_destroyPair(p);
					break;
				}
				// the proxies are overlapping, but AABBs might be separate
				var aabbOverlapping:Bool = M.aabb_intersects(aabb1._min, aabb1._max, aabb2._min, aabb2._max);
				// needs narrow-phase collision detection if AABBs are overlapping
				p._shouldBeSkipped = !aabbOverlapping;
			} while (false);
		});
	}

	@:extern
	public inline function _updateManifolds():Void {
		var p:Pair = _pairList;
		M.list_foreach(p, _next, {
			if (!p._shouldBeSkipped) {
				p._updateManifold();
			}
		});
	}

	@:extern
	inline function shouldCollide(c1:Component, c2:Component):Bool {
		var r1:RigidBody = c1._rigidBody;
		var r2:RigidBody = c2._rigidBody;
		// TODO: collision filtering
		return
			r1 != r2 &&
			(r1._type == Dynamic || r2._type == Dynamic)
		;
	}

	@:extern
	public inline function _destroyPair(pair:Pair):Void {
		M.list_remove(_pairList, _pairListLast, _prev, _next, pair);
		pair._detach();
		// put it into the pool
		M.singleList_pool(_pairPool, _next, pair);

		_numPairs--;
	}

}
