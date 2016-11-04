package oimo.physics;
import oimo.math.MathUtil;
import oimo.math.Vec3;
import oimo.physics.Settings;
import oimo.physics.collision.Pair;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.PairManager;
import oimo.physics.collision.broadphase.BroadPhase;
import oimo.physics.collision.broadphase.BroadPhaseType;
import oimo.physics.collision.broadphase.Proxy;
import oimo.physics.collision.broadphase.ProxyPair;
import oimo.physics.collision.broadphase.bruteforce.BruteForceBroadPhase;
import oimo.physics.collision.narrowphase.Manifold;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.ShapeType;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.debugdraw.DebugDrawer;
import oimo.physics.debugdraw.IDebugGraphics;
import oimo.physics.dynamics.World;
import oimo.physics.dynamics.constraint.Constraint;
import oimo.physics.dynamics.constraint.contact.ContactConstraint;
import oimo.physics.dynamics.constraint.contact.ContactPoint;
import oimo.physics.dynamics.rigidbody.Component;
import oimo.physics.dynamics.rigidbody.ComponentConfig;
import oimo.physics.dynamics.rigidbody.RigidBody;
import oimo.physics.dynamics.rigidbody.RigidBodyConfig;
import oimo.physics.dynamics.rigidbody.RigidBodyType;

/**
 * Settings class has parameters used by the physics simulation.
 */
@:expose("OIMO.Settings")
class Settings {
	public static var defaultFriction:Float = 0.2;
	public static var defaultRestitution:Float = 0.2;
	public static var defaultDensity:Float = 1;

	public static var maxTranslationPerStep:Float = 5;
	public static var maxRotationPerStep:Float = 0.5 * MathUtil.PI;
	public static var maxTranslationPerStepSq(get, null):Float;
	public static var maxRotationPerStepSq(get, null):Float;

	public static var bvhProxyPadding:Float = 0.1;
	public static var bvhIncrementalCollisionThreshold:Float = 0.45;

	public static var maxManifoldPoints:Int = 4;

	public static var contactBounceThreshold:Float = 0.5;

	public static var debugDrawBackgroundColor:Vec3 = new Vec3(0.1, 0.1, 0.1);

	public static var debugDrawShapeColor1:Vec3 = new Vec3(0.7, 0.2, 0.4);
	public static var debugDrawShapeColor2:Vec3 = new Vec3(1.0, 0.8, 0.1);
	public static var debugDrawStaticShapeColor:Vec3 = new Vec3(0.7, 0.7, 0.7);
	public static var debugDrawAABBColor:Vec3 = new Vec3(1.0, 0.1, 0.1);
	public static var debugDrawBVHNodeColor:Vec3 = new Vec3(0.4, 0.4, 0.4);
	public static var debugDrawPairColor:Vec3 = new Vec3(1.0, 1.0, 0.1);
	public static var debugDrawContactColor:Vec3 = new Vec3(1.0, 0.1, 0.1);
	public static var debugDrawContactNormalColor:Vec3 = new Vec3(1.0, 0.1, 0.1);
	public static var debugDrawContactTangentColor:Vec3 = new Vec3(0.1, 0.8, 0.1);
	public static var debugDrawContactBinormalColor:Vec3 = new Vec3(0.2, 0.2, 1.0);
	public static var debugDrawContactNormalLength:Float = 0.5;
	public static var debugDrawContactTangentLength:Float = 0.5;
	public static var debugDrawContactBinormalLength:Float = 0.5;

	public static var allClasses(default, never):Array<Dynamic> = [
		// physics.collision.broadphase.bruteforce
		BruteForceBroadPhase,

		// physics.collision.broadphase
		BroadPhase,
		BroadPhaseType,
		Proxy,
		ProxyPair,

		// physics.collision.narrowphase
		Manifold,

		// physics.collision.shape
		AABB,
		BoxShape,
		Shape,
		ShapeType,
		SphereShape,

		// physics.collision
		Pair,
		PairLink,
		PairManager,

		// physics.debugdraw
		DebugDrawer,
		IDebugGraphics,

		// physics.dynamics.constraint
		Constraint,

		// physics.dynamics.constraint.contact
		ContactConstraint,
		ContactPoint,

		// physics.dynamics
		World,

		// physics.dynamics.rigidbody
		Component,
		ComponentConfig,
		RigidBody,
		RigidBodyConfig,
		RigidBodyType,

		// physics
		Profile,
		Settings
	];

	@:extern
	static inline function get_maxTranslationPerStepSq():Float {
		return maxTranslationPerStep * maxTranslationPerStep;
	}

	@:extern
	static inline function get_maxRotationPerStepSq():Float {
		return maxRotationPerStep * maxRotationPerStep;
	}
}
