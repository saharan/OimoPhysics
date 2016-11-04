package oimo.physics.collision.broadphase.bruteforce;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.shape.AABB;

/**
 * Brute force. O(n^2)
 */
@:expose("OIMO.BruteForceBroadPhase")
class BruteForceBroadPhase extends BroadPhase {

	public function new() {
		super();
		_incremental = false;
	}

	override public function createProxy(userData:Dynamic, aabb:AABB):Proxy {
		var proxy:Proxy = new Proxy(userData, _idCount++);
		addProxy(proxy);

		proxy._setAABB(aabb);
		return proxy;
	}

	override public function destroyProxy(proxy:Proxy):Void {
		removeProxy(proxy);

		proxy._userData = null;
	}

	override public function moveProxy(proxy:Proxy, aabb:AABB, dislacement:Vec3):Void {
		proxy._setAABB(aabb);
	}

	override public function collectPairs():Void {
		poolProxyPairs();
		_testCount = 0;
		var p1:Proxy = _proxyList;
		M.list_foreach(p1, _next, {
			var p2:Proxy = p1._next;
			M.list_foreach(p2, _next, {
				_testCount++;
				if (overlap(p1, p2)) {
					pickAndPushProxyPair(p1, p2);
				}
			});
		});
	}

	@:extern
	inline function overlap(p1:Proxy, p2:Proxy):Bool {
		return M.aabb_intersects(p1._aabbMin, p1._aabbMax, p2._aabbMin, p2._aabbMax);
	}

}
