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
import oimo.physics.collision.narrowphase.ManifoldInfo;
import oimo.physics.collision.shape.AABB;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.ShapeType;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.debugdraw.DebugDrawer;
import oimo.physics.debugdraw.IDebugGraphics;
import oimo.physics.dynamics.Component;
import oimo.physics.dynamics.ComponentConfig;
import oimo.physics.dynamics.RigidBody;
import oimo.physics.dynamics.RigidBodyConfig;
import oimo.physics.dynamics.RigidBodyType;
import oimo.physics.dynamics.World;
import oimo.physics.dynamics.contact.Manifold;

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

	public static var debugDrawBackgroundColor:Vec3 = new Vec3(0.1, 0.1, 0.1);

	public static var debugDrawShapeColor1:Vec3 = new Vec3(0.7, 0.2, 0.4);
	public static var debugDrawShapeColor2:Vec3 = new Vec3(1.0, 0.8, 0.1);
	public static var debugDrawStaticShapeColor:Vec3 = new Vec3(0.7, 0.7, 0.7);

	public static var debugDrawAABBColor:Vec3 = new Vec3(1.0, 0.1, 0.1);

	public static var allClasses(default, never):Array<Dynamic> = [
		// physics.collision.broadphase.bruteforce
		BruteForceBroadPhase,

		// physics.collision.broadphase
		BroadPhase,
		BroadPhaseType,
		Proxy,
		ProxyPair,

		// physics.collision.narrowphase
		ManifoldInfo,

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

		// physics.dynamics.contact
		Manifold,

		// physics.dynamics
		Component,
		ComponentConfig,
		RigidBody,
		RigidBodyConfig,
		RigidBodyType,
		World,

		// physics
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
