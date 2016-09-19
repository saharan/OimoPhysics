package oimo.math;

/**
 * Math util
 */
class MathUtil {
	public static inline var PI:Float = 3.14159265358979;
	public static inline var TO_RADIANS:Float = PI / 180;
	public static inline var TO_DEGREES:Float = 180 / PI;
	public static inline var EPS:Float = 1e-5;

	public static inline function sin(x:Float):Float {
		return Math.sin(x);
	}

	public static inline function cos(x:Float):Float {
		return Math.cos(x);
	}

	public static inline function tan(x:Float):Float {
		return Math.tan(x);
	}

	public static inline function asin(x:Float):Float {
		return Math.asin(x);
	}

	public static inline function acos(x:Float):Float {
		return Math.acos(x);
	}

	public static inline function atan(x:Float):Float {
		return Math.atan(x);
	}

	public static inline function atan2(y:Float, x:Float):Float {
		return Math.atan2(y, x);
	}

	public static inline function sqrt(x:Float):Float {
		return Math.sqrt(x);
	}

	public static inline function rand():Float {
		return Math.random();
	}

	public static inline function randIn(min:Float, max:Float):Float {
		return min + rand() * (max - min);
	}
}
