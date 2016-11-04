package oimo.physics.collision.narrowphase;
import haxe.ds.Vector;
import oimo.physics.collision.shape.ShapeType;

/**
 * NarrowPhase chass provides corresponding collision detector for pairs of shapes.
 */
@:expose("OIMO.NarrowPhase")
class NarrowPhase {
	private var detectors:Vector<Vector<Detector>>;

	public function new() {
		detectors = new Vector<Vector<Detector>>(4);
		for (i in 0...4) {
			detectors[i] = new Vector<Detector>(4);
		}

		var sphereIndex:Int = toShapeTypeIndex(Sphere);
		var boxIndex:Int = toShapeTypeIndex(Box);
		detectors[sphereIndex][sphereIndex] = new SphereSphereDetector();
		detectors[sphereIndex][boxIndex] = new SphereBoxDetector(false);
		detectors[boxIndex][sphereIndex] = new SphereBoxDetector(true);
	}

	public inline function getDetector(type1:ShapeType, type2:ShapeType):Detector {
		return detectors[toShapeTypeIndex(type1)][toShapeTypeIndex(type2)];
	}

	@:extern
	inline function toShapeTypeIndex(type:ShapeType):Int {
		var index:Int;
		switch (type) {
		case Sphere:
			index = 0;
		case Box:
			index = 1;
		}
		return index;
	}
}
