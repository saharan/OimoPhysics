package demo.core;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;
import oimo.physics.*;

/**
 * ...
 */
class BasicDemo extends DemoBase {
	public function new() {
		super("Basic Demo");
	}

	override public function init(world:World, renderer:DemoRenderer, input:UserInput, viewInfo:ViewInfo):Void {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		var thickness:Float = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		var w:Int = 2;
		var h:Int = 2;
		var sp:Float = 0.61;
		var n:Int = 5;
		var size:Float = 0.3;
		var centerX:Float = 0;
		var centerZ:Float = 0;
		for (i in 0...n) {
			for (j in -w...w + 1) {
				for (k in -h...h + 1) {
					var pos:Vec3 = new Vec3(centerX + j * sp + MathUtil.randIn(-0.05, 0.05), size + i * size * 3.0, centerZ + k * sp + MathUtil.randIn(-0.05, 0.05));
					OimoUtil.addBox(world, pos, new Vec3(size, size, size), false).setAngularVelocity(MathUtil.randVec3In(-0.05, 0.05));
				}
			}
		}
	}

	override public function update():Void {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}
