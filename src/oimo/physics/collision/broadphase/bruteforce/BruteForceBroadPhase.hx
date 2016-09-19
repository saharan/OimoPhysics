package oimo.physics.collision.broadphase.bruteforce;
import oimo.m.M;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.ProxyPair;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.shape.AABB;
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

	override public function createProxy(userData:Dynamic, aabb:AABB):Proxy {
		var proxy:Proxy = new Proxy(userData, _idCount++);
		proxy._setAABB(aabb);
		M.list_push(_proxyList, _proxyListLast, _prev, _next, proxy);
		return proxy;
	}

	override public function destroyProxy(proxy:Proxy):Void {
		M.list_remove(_proxyList, _proxyListLast, _prev, _next, proxy);
		proxy._userData = null;
	}

	override public function moveProxy(proxy:Proxy, aabb:AABB):Void {
		proxy._setAABB(aabb);
	}

	override public function collectPairs():Void {
		poolProxyPairs();
		_incremental = false;
		var p1:Proxy = _proxyList;
		M.list_foreach(p1, _next, {
			var p2:Proxy = p1._next;
			M.list_foreach(p2, _next, {
				if (M.aabb_isOverlapped(p1._aabbMin, p1._aabbMax, p2._aabbMin, p2._aabbMax)) {
					pickAndPushProxyPair(p1, p2);
				}
			});
		});
	}

}
