package oimo.physics.collision;
import oimo.m.M;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.ProxyPair;
import oimo.physics.collision.PairLink;
import oimo.physics.dynamics.Component;

/**
 * A manager of the overlapping component pairs.
 */
@:expose("OIMO.PairManager")
class PairManager {
	public var _pairList:Pair;
	public var _pairListLast:Pair;
	public var _pairPool:Pair;

	public var _broadPhase:BroadPhase;

	public function new(broadPhase:BroadPhase) {
		_broadPhase = broadPhase;
	}

	public function updatePairs():Void {
		_broadPhase._collectPairs();
		// whether the broadphase　to return only the new overlapping pairs
		var incremental:Bool = _broadPhase._incremental;
		var pp:ProxyPair = _broadPhase._proxyPairList;
		M.list_foreach(pp, _next, {
			var c1:Component = pp._p1._component;
			var c2:Component = pp._p2._component;
			var n1:Int = c1._numPairs;
			var n2:Int = c2._numPairs;
			var l:PairLink;
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
					trace("pair updated");
					p._persistent = true;
					found = true;
					break;
				}
			});
			// add the new pair
			if (!found) {
				trace("pair created");
				// trying to pick an object up from the pool
				var p:Pair = M.singleList_pick(_pairPool, _next, { trace("new pair created!"); new Pair(); } );
				M.list_push(_pairList, _pairListLast, _prev, _next, p);
				p._persistent = true;
				p._setFromProxyPair(pp);
			}
		});
		var p:Pair = _pairList;
		M.list_foreach(p, _next, {
			// check if the pair is not updated
			if (
				!p._persistent && (
				// the broadphase ensures it returns the whole existing pairs,
				// so the pairs that have not been updated should be deleted
				!incremental ||

				// the broadphase may not return overlapping pairs,
				// so we should check if they are still overlapping
				!M.aabb_isOverlapped(p._c1._aabb._min, p._c1._aabb._max, p._c2._aabb._min, p._c2._aabb._max)
			)) {
				trace("pair destroyed");
				M.list_remove(_pairList, _pairListLast, _prev, _next, p);
				p._clear();
				// put the object into the pool
				M.singleList_pool(_pairPool, _next, p);
			} else {
				// TODO: popopo
			}
			p._persistent = false;
		});
	}

}
