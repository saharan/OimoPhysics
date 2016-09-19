package oimo.physics.collision.broadphase;

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

	/**
	 * Returns the first proxy of the pair.
	 */
	public function getProxy1():Proxy {
		return _p1;
	}

	/**
	 * Returns the second proxy of the pair.
	 */
	public function getProxy2():Proxy {
		return _p2;
	}

	/**
	 * Returns the next pair.
	 */
	public function getNext():ProxyPair {
		return _next;
	}

}
