package oimo.physics.collision.narrowphase;
import oimo.physics.collision.shape.Shape;
import oimo.physics.collision.shape.Transform;

/**
 * Interface of a collision detector for narrow-phase collision detection.
 */
class Detector {
	var swapped:Bool;

	public function new(swapped:Bool) {
		this.swapped = swapped;
	}

	public function detect(output:Manifold, shape1:Shape, shape2:Shape, transform1:Transform, transform2:Transform, cachedData:CachedDetectorData):Void {
		output.clear();
		if (swapped) {
			output._swapped = true;
			detectImpl(output, shape2, shape1, transform2, transform1, cachedData);
		} else {
			output._swapped = false;
			detectImpl(output, shape1, shape2, transform1, transform2, cachedData);
		}
	}

	function detectImpl(output:Manifold, shape1:Shape, shape2:Shape, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
	}
}
