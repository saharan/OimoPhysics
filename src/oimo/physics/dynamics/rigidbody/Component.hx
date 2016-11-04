package oimo.physics.dynamics.rigidbody;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.Transform;

/**
 * Component of rigid bodies.
 */
@:expose("OIMO.Component")
@:build(oimo.m.B.bu())
class Component {
	public var _id:Int;

	public var _prev:Component;
	public var _next:Component;
	public var _rigidBody:RigidBody;
	public var _shape:Shape;

	public var _localTransform:Transform;
	public var _ptransform:Transform;
	public var _transform:Transform;

	public var _restitution:Float;
	public var _friction:Float;
	public var _density:Float;

	public var _pairLink:PairLink;
	public var _pairLinkLast:PairLink;
	public var _numPairs:Int;

	public var _aabb:AABB;

	public var _proxy:Proxy;
	var displacement:Vec3;

	public function new(config:ComponentConfig) {
		_id = -1;

		_localTransform = new Transform();
		_ptransform = new Transform();
		_transform = new Transform();

		M.vec3_fromVec3(_localTransform._origin, config.position);
		M.mat3_fromMat3(_localTransform._rotation, config.rotation);
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
		displacement = new Vec3();
	}

	@:extern
	public inline function _sync(tf1:Transform, tf2:Transform):Void {
		M.transform_mul(_ptransform, _localTransform, tf1);
		M.transform_mul(_transform, _localTransform, tf2);

		var min:IVec3;
		var max:IVec3;

		M.call(_shape._computeAABB(_aabb, _ptransform));
		M.vec3_assign(min, _aabb._min);
		M.vec3_assign(max, _aabb._max);

		M.call(_shape._computeAABB(_aabb, _transform));
		M.vec3_min(_aabb._min, min, _aabb._min);
		M.vec3_max(_aabb._max, max, _aabb._max);

		if (_proxy != null) {
			var d:IVec3;
			M.vec3_sub(d, _transform._origin, _ptransform._origin);
			M.vec3_toVec3(displacement, d);
			_rigidBody._world._broadPhase.moveProxy(_proxy, _aabb, displacement);
		}
	}

}
