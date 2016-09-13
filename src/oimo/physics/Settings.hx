package oimo.physics;
import oimo.math.MathUtil;
import oimo.math.Vec3;

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

	public static var debugDrawShapeColor1:Vec3 = new Vec3(0.7, 0.2, 0.4);
	public static var debugDrawShapeColor2:Vec3 = new Vec3(1.0, 0.8, 0.1);

	public static var debugDrawStaticShapeColor:Vec3 = new Vec3(0.5, 0.5, 0.5);

	@:extern
	static inline function get_maxTranslationPerStepSq():Float {
		return maxTranslationPerStep * maxTranslationPerStep;
	}

	@:extern
	static inline function get_maxRotationPerStepSq():Float {
		return maxRotationPerStep * maxRotationPerStep;
	}
}
