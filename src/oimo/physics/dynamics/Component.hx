package oimo.physics.dynamics;
import oimo.m.B;
import oimo.m.IMat3;
import oimo.m.ITransform;
import oimo.m.M;
import oimo.math.Vec3;
import oimo.physics.collision.shape.Shape;

/**
 * Component of rigid bodies.
 */
@:expose("OIMO.Component")
@:build(oimo.m.B.build())
class Component {
	public static var _idCount:Int = 0;

	public var _prev:Component;
	public var _next:Component;
	public var _body:RigidBody;
	public var _shape:Shape;

	public var _localTransform:ITransform;
	public var _transform:ITransform;

	public var _restitution:Float;
	public var _friction:Float;
	public var _density:Float;

	public var _id:Int;

	public function new(config:ComponentConfig) {
		M.transform_fromTransform(_localTransform, config.transform);
		M.transform_assign(_transform, _localTransform);
		_restitution = config.restitution;
		_friction = config.friction;
		_density = config.density;
		_shape = config.shape;

		_id = _idCount++;
	}

}
