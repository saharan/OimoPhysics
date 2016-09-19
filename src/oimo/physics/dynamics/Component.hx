package oimo.physics.dynamics;
import oimo.m.B;
import oimo.m.IAABB;
import oimo.m.IMat3;
import oimo.m.ITransform;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.Shape;

/**
 * Component of rigid bodies.
 */
@:expose("OIMO.Component")
@:build(oimo.m.B.build())
class Component {
	public var _id:Int;

	public var _prev:Component;
	public var _next:Component;
	public var _body:RigidBody;
	public var _shape:Shape;

	public var _localTransform:ITransform;
	public var _ptransform:ITransform;
	public var _transform:ITransform;

	public var _restitution:Float;
	public var _friction:Float;
	public var _density:Float;

	public var _pairLink:PairLink;
	public var _pairLinkLast:PairLink;
	public var _numPairs:Int;

	public var _aabb:AABB;

	public var _proxy:Proxy;

	public function new(config:ComponentConfig) {
		_id = -1;

		M.transform_fromTransform(_localTransform, config.transform);
		M.transform_assign(_ptransform, _localTransform);
		M.transform_assign(_transform, _localTransform);
		_restitution = config.restitution;
		_friction = config.friction;
		_density = config.density;
		_shape = config.shape;

		_pairLink = null;
		_pairLinkLast = null;
		_numPairs = 0;

		_aabb = new AABB();

		_proxy = null;
	}

}
