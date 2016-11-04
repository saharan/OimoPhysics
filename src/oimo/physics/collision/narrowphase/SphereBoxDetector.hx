package oimo.physics.collision.narrowphase;
import oimo.m.IMat3;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.math.MathUtil;
import oimo.physics.collision.narrowphase.CachedDetectorData;
import oimo.physics.collision.shape.BoxShape;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.SphereShape;
import oimo.physics.collision.shape.Transform;
import oimo.physics.collision.narrowphase.Manifold;

/**
 * Sphere vs Box detector
 */
@:build(oimo.m.B.bu())
class SphereBoxDetector extends Detector {
	public function new(swapped:Bool) {
		super(swapped);
	}

	override function detectImpl(output:Manifold, shape1:Shape, shape2:Shape, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var s:SphereShape = cast shape1;
		var b:BoxShape = cast shape2;

		var halfExt:IVec3;
		var negHalfExt:IVec3;
		M.vec3_assign(halfExt, b._halfExtents);
		M.vec3_negate(negHalfExt, halfExt);

		var r:Float = s._radius;

		var boxToSphere:IVec3;
		M.vec3_sub(boxToSphere, tf1._origin, tf2._origin);

		var boxToSphereInBox:IVec3;
		M.vec3_mulMat3Transposed(boxToSphereInBox, boxToSphere, tf2._rotation);

		// is the center of the sphere inside the box?
		var insideBox:Bool = M.aabb_intersects(negHalfExt, halfExt, boxToSphereInBox, boxToSphereInBox);
		if (insideBox) {
			// compute minimum distance between the center of the sphere and the box's surfaces that are perpendicular to each axis
			var sphereToBoxSurface:IVec3;
			M.vec3_abs(sphereToBoxSurface, boxToSphereInBox);
			M.vec3_sub(sphereToBoxSurface, halfExt, sphereToBoxSurface);

			var normalInBox:IVec3;

			// ... and select the smallest one, setting normal
			var distX:Float = M.vec3_get(sphereToBoxSurface, 0);
			var distY:Float = M.vec3_get(sphereToBoxSurface, 1);
			var distZ:Float = M.vec3_get(sphereToBoxSurface, 2);
			var penetration:Float;
			var projectionMask:IVec3;
			M.compare3min(
				distX, distY, distZ, {
					if (M.vec3_get(boxToSphereInBox, 0) > 0) {
						M.vec3_set(normalInBox, 1, 0, 0);
					} else {
						M.vec3_set(normalInBox, -1, 0, 0);
					}
					M.vec3_set(projectionMask, 0, 1, 1);
					penetration = -distX;
				}, {
					if (M.vec3_get(boxToSphereInBox, 1) > 0) {
						M.vec3_set(normalInBox, 0, 1, 0);
					} else {
						M.vec3_set(normalInBox, 0, -1, 0);
					}
					M.vec3_set(projectionMask, 1, 0, 1);
					penetration = -distY;
				}, {
					if (M.vec3_get(boxToSphereInBox, 2) > 0) {
						M.vec3_set(normalInBox, 0, 0, 1);
					} else {
						M.vec3_set(normalInBox, 0, 0, -1);
					}
					M.vec3_set(projectionMask, 1, 1, 0);
					penetration = -distZ;
				}
			);

			// compute the closest point
			var base:IVec3;
			M.vec3_compWiseMul(base, projectionMask, boxToSphereInBox);
			var boxToClosestPointInBox:IVec3;
			M.vec3_compWiseMul(boxToClosestPointInBox, normalInBox, halfExt);
			M.vec3_add(boxToClosestPointInBox, boxToClosestPointInBox, base);

			// bring them back to the world coordinate system
			var boxToClosestPoint:IVec3;
			var normal:IVec3;
			M.vec3_mulMat3(boxToClosestPoint, boxToClosestPointInBox, tf2._rotation);
			M.vec3_mulMat3(normal, normalInBox, tf2._rotation);
			M.call(output.setNormal(normal));

			var rp1:IVec3;
			var rp2:IVec3;
			M.vec3_scale(rp1, normal, -r);
			M.vec3_assign(rp2, boxToClosestPoint);
			M.call(output.addPoint(rp1, rp2, penetration));
			return;
		}

		// compute the closest point to the center of the sphere; just clamp the coordinate
		var boxToClosestPointInBox:IVec3;
		M.vec3_clamp(boxToClosestPointInBox, boxToSphereInBox, negHalfExt, halfExt);

		var closestPointToSphereInBox:IVec3;
		M.vec3_sub(closestPointToSphereInBox, boxToSphereInBox, boxToClosestPointInBox);

		var dist:Float = M.vec3_dot(closestPointToSphereInBox, closestPointToSphereInBox);
		if (dist >= r * r) {
			// does not collide
			return;
		}
		dist = MathUtil.sqrt(dist);

		// bring them back to the world coordinate system
		var boxToClosestPoint:IVec3;
		var closestPointToSphere:IVec3;
		M.vec3_mulMat3(boxToClosestPoint, boxToClosestPointInBox, tf2._rotation);
		M.vec3_mulMat3(closestPointToSphere, closestPointToSphereInBox, tf2._rotation);

		var normal:IVec3;
		M.vec3_normalize(normal, closestPointToSphere);
		M.call(output.setNormal(normal));

		var rp1:IVec3;
		var rp2:IVec3;
		M.vec3_scale(rp1, normal, -r);
		M.vec3_assign(rp2, boxToClosestPoint);
		M.call(output.addPoint(rp1, rp2, r - dist));
	}
}
