package oimo.physics.collision.broadphase;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.physics.collision.shape.AABB;

/**
 * Broad-phase proxy.
 */
@:expose("OIMO.Proxy")
@:build(oimo.m.B.bu())
class Proxy {
	public var _prev:Proxy;
	public var _next:Proxy;

	public var _userData:Dynamic;

	public var _aabbMin:IVec3;
	public var _aabbMax:IVec3;

	public var _lastCenter:IVec3;

	public var _id:Int;

	public function new(userData:Dynamic, id:Int) {
		_userData = userData;
		_id = id;
		_prev = null;
		_next = null;
		M.vec3_zero(_aabbMin);
		M.vec3_zero(_aabbMax);
		M.vec3_zero(_lastCenter);
	}

	@:extern
	public inline function _setAABB(aabb:AABB):Void {
		M.vec3_assign(_aabbMin, aabb._min);
		M.vec3_assign(_aabbMax, aabb._max);
	}

	/**
	 * Returns the user data of the proxy.
	 */
	public function getUserData():Dynamic {
		return _userData;
	}

	/**
	 * Sets user data of the proxy to `userData`.
	 */
	public function setUserData(userData:Dynamic):Void {
		_userData = userData;
	}

	/**
	 * Returns the unique id of the proxy.
	 */
	public function getId():Int {
		return _id;
	}
}
