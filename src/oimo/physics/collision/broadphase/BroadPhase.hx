package oimo.physics.collision.broadphase;
import js.Lib;
import oimo.m.M;
import oimo.physics.dynamics.Component;

/**
 * The abstract class of broadphase algorithms.
 */
@:expose("OIMO.BroadPhase")
class BroadPhase {
	public var _proxyPairList:ProxyPair;

	/**
	 * Whether to collect only pairs created in the last step. If `_incremental` is true,
	 * the pairs that are not collected might still be overlapping. Otherwise, such pairs
	 * are guaranteed to be separated.
	 */
	public var _incremental:Bool;

	var _proxyPairPool:ProxyPair;
	var _idCount:Int;

	public function new() {
		_idCount = 0;
	}

	public function _addComponent(component:Component):Void {
		component._proxy = _createProxy(component);
	}

	public function _removeComponent(component:Component):Void {
		_destroyProxy(component._proxy);
		component._proxy = null;
	}

	public function _collectPairs():Void {
	}

	function _createProxy(component:Component):Proxy {
		return null;
	}

	function _destroyProxy(proxy:Proxy):Void {
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
}
