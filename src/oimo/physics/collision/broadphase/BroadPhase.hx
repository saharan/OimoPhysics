package oimo.physics.collision.broadphase;
import js.Lib;
import oimo.m.M;
import oimo.physics.collision.shape.AABB;
import oimo.physics.dynamics.Component;

/**
 * The abstract class of broadphase algorithms.
 */
@:expose("OIMO.BroadPhase")
class BroadPhase {
	public var _proxyPairList:ProxyPair;
	public var _incremental:Bool;

	var _proxyPairPool:ProxyPair;
	var _idCount:Int;

	public function new() {
		_idCount = 0;
	}

	@:extern
	inline function pickAndPushProxyPair(p1:Proxy, p2:Proxy):Void {
		var pp:ProxyPair = M.singleList_pick(_proxyPairPool, _next, new ProxyPair());
		M.singleList_addFirst(_proxyPairList, _next, pp);
		pp._p1 = p1;
		pp._p2 = p2;
	}

	@:extern
	inline function poolProxyPairs():Void {
		var p:ProxyPair = _proxyPairList;
		if (p != null) {
			do {
				p._p1 = null;
				p._p2 = null;
				p = p._next;
			} while (p != null);
			_proxyPairList._next = _proxyPairPool;
			_proxyPairPool = _proxyPairList;
			_proxyPairList = null;
		}
	}

	// --- public ---

	/**
	 * Returns a new proxy connected with the user data `userData` in the axis-aligned bounding
	 * box `aabb`, and adds the proxy into the broad-phase algorithm.
	 */
	public function createProxy(userData:Dynamic, aabb:AABB):Proxy {
		return null;
	}

	/**
	 * Removes the proxy `proxy` from the broad-phase algorithm.
	 */
	public function destroyProxy(proxy:Proxy):Void {
	}

	/**
	 * Moves the proxy `proxy` to the axis-aligned bounding box `aabb`.
	 */
	public function moveProxy(proxy:Proxy, aabb:AABB):Void {
	}

	/**
	 * Collects overlapping pairs of the proxies and put them into a linked list. The linked list
	 * can be get through `getProxyPairList` function.
	 *
	 * Note that in order to collect pairs, the broad-phase algorithm requires to be informed of
	 * movements of proxies through `moveProxy` function.
	 */
	public function collectPairs():Void {
	}

	/**
	 * Returns the linked list of collected pairs of proxies.
	 */
	public inline function getProxyPairList():ProxyPair {
		return _proxyPairList;
	}

	/**
	 * Returns whether to collect only pairs created in the last step. If this returns
	 * true, the pairs that are not collected might still be overlapping. Otherwise, such
	 * pairs are guaranteed to be separated.
	 */
	public inline function isIncremental():Bool {
		return _incremental;
	}
}
