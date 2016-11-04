package oimo.physics.collision;
import oimo.m.M;
import oimo.physics.collision.PairLink;
import oimo.physics.collision.narrowphase.CachedDetectorData;
import oimo.physics.collision.narrowphase.Detector;
import oimo.physics.collision.narrowphase.Manifold;
import oimo.physics.dynamics.constraint.contact.ContactConstraint;
import oimo.physics.dynamics.rigidbody.Component;

/**
 * Cached overlapping component pairs.
 */
@:expose("OIMO.Pair")
class Pair {
	public var _next:Pair;
	public var _prev:Pair;
	public var _c1:Component;
	public var _c2:Component;
	public var _link1:PairLink;
	public var _link2:PairLink;
	public var _manifoldInfo:Manifold;
	public var _contactConstraint:ContactConstraint;
	public var _touching:Bool;

	public var _detector:Detector;
	public var _cachedDetectorData:CachedDetectorData;

	public var _latest:Bool;
	public var _shouldBeSkipped:Bool;

	public function new() {
		_link1 = new PairLink();
		_link2 = new PairLink();
		_manifoldInfo = new Manifold();
		_contactConstraint = new ContactConstraint();

		_detector = null;
		_cachedDetectorData = new CachedDetectorData();

		_latest = false;
		_shouldBeSkipped = false;
	}

	@:extern
	public inline function _attach(c1:Component, c2:Component, detector:Detector):Void {
		_c1 = c1;
		_c2 = c2;
		attachLinks();

		_detector = detector;

		_contactConstraint._attach(c1, c2);
	}

	@:extern
	public inline function _detach():Void {
		detachLinks();
		_c1 = null;
		_c2 = null;

		_detector = null;
		_cachedDetectorData.clear();

		_contactConstraint._clear();
		_contactConstraint._detach();
	}

	@:extern
	public inline function _updateManifold():Void {
		if (_detector == null) return;
		_detector.detect(_manifoldInfo, _c1._shape, _c2._shape, _c1._transform, _c2._transform, _cachedDetectorData);
		_touching = _manifoldInfo._numPoints > 0;
		if (_touching) {
			_contactConstraint._update(_manifoldInfo);
		} else {
			_contactConstraint._clear();
		}
	}

	@:extern
	inline function attachLinks():Void {
		M.list_push(_c1._pairLink, _c1._pairLinkLast, _prev, _next, _link1);
		M.list_push(_c2._pairLink, _c2._pairLinkLast, _prev, _next, _link2);
		_c1._numPairs++;
		_c2._numPairs++;
		_link1._theOther = _c2;
		_link2._theOther = _c1;
		_link1._pair = this;
		_link2._pair = this;
	}

	@:extern
	inline function detachLinks():Void {
		M.list_remove(_c1._pairLink, _c1._pairLinkLast, _prev, _next, _link1);
		M.list_remove(_c2._pairLink, _c2._pairLinkLast, _prev, _next, _link2);
		_c1._numPairs--;
		_c2._numPairs--;
		_link1._theOther = null;
		_link2._theOther = null;
		_link1._pair = null;
		_link2._pair = null;
	}

}
