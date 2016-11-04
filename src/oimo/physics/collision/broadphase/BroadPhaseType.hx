package oimo.physics.collision.broadphase;

/**
 * Types of broad-phase algorithms.
 */
@:expose("OIMO.BroadPhaseType")
enum BroadPhaseType {
	BruteForce;
	BVH;
}
