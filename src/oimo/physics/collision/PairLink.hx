package oimo.physics.collision;
import oimo.physics.collision.Pair;
import oimo.physics.dynamics.Component;

/**
 * Pair link.
 */
@:expose("OIMO.PairLink")
class PairLink {
	public var _prev:PairLink;
	public var _next:PairLink;
	public var _pair:Pair;
	public var _theOther:Component;

	public function new() {
		_prev = null;
		_next = null;
		_pair = null;
		_theOther = null;
	}
}