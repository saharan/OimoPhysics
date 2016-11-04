package oimo.physics.collision.narrowphase;
import haxe.ds.Vector;
import oimo.m.IVec3;
import oimo.m.M;
import oimo.physics.Settings;
import oimo.physics.collision.shape.Transform;

/**
 * Manifold information.
 */
@:expose("OIMO.Manifold")
@:build(oimo.m.B.bu())
class Manifold {
	public var _numPoints:Int;
	public var _normal:IVec3;
	public var _points:Vector<ManifoldPoint>;
	public var _swapped:Bool;
	public var _incremental:Bool; // for GJK/EPA detector

	public function new() {
		_numPoints = 0;
		M.vec3_zero(_normal);
		_points = new Vector<ManifoldPoint>(Settings.maxManifoldPoints);
		_swapped = false;
		_incremental = false;

		for (i in 0...Settings.maxManifoldPoints) {
			_points[i] = new ManifoldPoint();
		}
	}

	@:extern
	public inline function setNormal(n:IVec3):Void {
		if (_swapped) {
			M.vec3_negate(_normal, n);
		} else {
			M.vec3_assign(_normal, n);
		}
	}

	@:extern
	public inline function clear():Void {
		_numPoints = 0;
	}

	@:extern
	public inline function addPoint(rp1:IVec3, rp2:IVec3, penetration:Float):Void {
		M.assert(_numPoints < Settings.maxManifoldPoints);
		var p = _points[_numPoints++];
		p._penetration = penetration;
		if (_swapped) {
			M.vec3_assign(p._relPos1, rp2);
			M.vec3_assign(p._relPos2, rp1);
		} else {
			M.vec3_assign(p._relPos1, rp1);
			M.vec3_assign(p._relPos2, rp2);
		}
	}
}
