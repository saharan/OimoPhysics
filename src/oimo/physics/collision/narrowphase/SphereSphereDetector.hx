package oimo.physics.collision.narrowphase;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.physics.collision.narrowphase.CachedDetectorData;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.collision.shape.Transform;
import oimo.physics.collision.narrowphase.Manifold;

/**
 * Sphere vs Sphere detector
 */
@:build(oimo.m.B.bu())
class SphereSphereDetector extends Detector {
	public function new() {
		super(false);
	}

	override function detectImpl(output:Manifold, shape1:Shape, shape2:Shape, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var s1:SphereShape = cast shape1;
		var s2:SphereShape = cast shape2;
		var d:IVec3;
		M.vec3_sub(d, tf1._origin, tf2._origin);
		var r1:Float = s1._radius;
		var r2:Float = s2._radius;
		var len2:Float = M.vec3_dot(d, d);
		if (len2 >= (r1 + r2) * (r1 + r2)) return;
		var len:Float = MathUtil.sqrt(len2);
		var n:IVec3;
		if (M.gt0(len)) {
			M.vec3_scale(n, d, 1 / len);
		} else {
			M.vec3_set(n, 1, 0, 0);
		}
		M.call(output.setNormal(n));
		var rp1:IVec3;
		var rp2:IVec3;
		M.vec3_scale(rp1, n, -r1);
		M.vec3_scale(rp2, n, r2);
		M.call(output.addPoint(rp1, rp2, r1 + r2 - len));
	}
}
