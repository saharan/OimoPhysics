package oimo.physics.collision.broadphase;
import oimo.physics.dynamics.Component;

/**
 * A pair between two proxies. Broad-phase collision algorithms collect pairs of proxies
 * as linked list of ProxyPair.
 */
@:expose("OIMO.ProxyPair")
class ProxyPair {
	public var _next:ProxyPair;

	public var _p1:Proxy;
	public var _p2:Proxy;

	public function new() {
		_p1 = null;
		_p2 = null;
	}

}