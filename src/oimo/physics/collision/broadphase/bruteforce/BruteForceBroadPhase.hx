package oimo.physics.collision.broadphase.bruteforce;
import oimo.m.M;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.ProxyPair;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.dynamics.Component;

/**
 * Brute force. O(n^2)
 */
@:expose("OIMO.BruteForceBroadPhase")
class BruteForceBroadPhase extends BroadPhase {
	public var _proxyList:Proxy;
	public var _proxyListLast:Proxy;

	public function new() {
		super();
	}

	override function _createProxy(component:Component):Proxy {
		var proxy:Proxy = new Proxy(component, _idCount++);
		M.list_push(_proxyList, _proxyListLast, _prev, _next, proxy);
		return proxy;
	}

	override function _destroyProxy(proxy:Proxy):Void {
		M.list_remove(_proxyList, _proxyListLast, _prev, _next, proxy);
	}

	override public function _collectPairs():Void {
		poolProxyPairs();
		_incremental = false;
		var p1:Proxy = _proxyList;
		M.list_foreach(p1, _next, {
			var p2:Proxy = p1._next;
			M.list_foreach(p2, _next, {
				if (M.aabb_isOverlapped(p1._aabb._min, p1._aabb._max, p2._aabb._min, p2._aabb._max)) {
					pickAndPushProxyPair(p1, p2);
				}
			});
		});
	}

}
