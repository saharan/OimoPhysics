package oimo.physics.collision.broadphase;
import oimo.m.IAABB;
import oimo.m.M;
import oimo.physics.collision.shape.AABB;
import oimo.physics.dynamics.Component;

/**
 * Proxy
 */
@:expose("OIMO.Proxy")
class Proxy {
	public var _prev:Proxy;
	public var _next:Proxy;

	public var _component:Component;
	public var _aabb:AABB;

	public var _id:Int;

	public function new(component:Component, id:Int) {
		_component = component;
		_aabb = component._aabb;
		_id = id;
	}
}