/* Copyright (c) 2012 EL-EMENT saharan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation  * files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy,  * modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to
 * whom the Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package com.element.oimo.physics.test {
	import com.element.oimo.physics.collision.shape.BoxShape;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.ShapeConfig;
	import com.element.oimo.physics.collision.shape.SphereShape;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Quat;
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.OimoPhysics;
	import com.element.oimo.physics.util.DebugDraw;
	import flash.display.Sprite;
	import flash.display.Stage3D;
	import flash.events.Event;
	import flash.text.TextField;
	import flash.text.TextFormat;
	import net.hires.debug.Stats;
	/**
	 * 動作テスト
	 * @author saharan
	 */
	[SWF(width = "640", height = "480", frameRate = "60")]
	public class SphereStackTest extends Sprite {
		private var s3d:Stage3D;
		private var world:World;
		private var renderer:DebugDraw;
		private var rigid:RigidBody;
		private var count:uint;
		private var tf:TextField;
		private var fps:Number;
		
		public function SphereStackTest() {
			if (stage) init();
			else addEventListener(Event.ADDED_TO_STAGE, init);
		}
		
		private function init(e:Event = null):void {
			removeEventListener(Event.ADDED_TO_STAGE, init);
			
			var debug:Stats = new Stats();
			debug.x = 570;
			addChild(debug);
			tf = new TextField();
			tf.selectable = false;
			tf.defaultTextFormat = new TextFormat("_monospace", 12, 0xffffff);
			tf.x = 0;
			tf.y = 0;
			tf.width = 400;
			tf.height = 400;
			addChild(tf);
			trace(OimoPhysics.DESCRIPTION);
			world = new World();
			renderer = new DebugDraw(640, 480);
			renderer.setWorld(world);
			var r:RigidBody;
			var s:Shape;
			var c:ShapeConfig = new ShapeConfig();
			c.position.init(0, 0, -6);
			s = new BoxShape(24, 1, 15, c);
			r = new RigidBody(35 * Math.PI / 180, 1, 0, 0);
			r.addShape(s);
			r.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(r);
			c.position.init(0, -10, 6);
			s = new BoxShape(24, 1, 15, c);
			r = new RigidBody(-35 * Math.PI / 180, 1, 0, 0);
			r.addShape(s);
			r.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(r);
			c.position.init(-12, -5, 0);
			s = new BoxShape(1, 20, 25, c);
			r = new RigidBody();
			r.addShape(s);
			r.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(r);
			c.position.init(12, -5, 0);
			s = new BoxShape(1, 20, 25, c);
			r = new RigidBody();
			r.addShape(s);
			r.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(r);
			for (var k:int = 0; k < 512; k++) {
				makeRigid(Math.random() * 8 - 4, 2 + k * 0.5, Math.random() * 8 - 4);
			}
			fps = 0;
			
			s3d = stage.stage3Ds[0];
			s3d.addEventListener(Event.CONTEXT3D_CREATE, onContext3DCreated);
			s3d.requestContext3D();
			addEventListener(Event.ENTER_FRAME, frame);
		}
		
		private function makeRigid(x:Number, y:Number, z:Number):void {
			var r1:Number = 0.3 + Math.random() * 0.5;
			var r2:Number = 0.3 + Math.random() * 0.5;
			var cfg:ShapeConfig = new ShapeConfig();
			cfg.position.x = x - r1;
			cfg.position.y = y;
			cfg.position.z = z;
			var shape1:Shape = new SphereShape(r1, cfg);
			cfg.position.x = x + r2;
			var shape2:Shape = new SphereShape(r2, cfg);
			rigid = new RigidBody();
			rigid.angularVelocity.x = Math.random() * 2 - 1;
			rigid.angularVelocity.y = Math.random() * 2 - 1;
			rigid.angularVelocity.z = Math.random() * 2 - 1;
			rigid.addShape(shape1);
			rigid.addShape(shape2);
			rigid.setupMass();
			world.addRigidBody(rigid);
		}
		
		private function onContext3DCreated(e:Event = null):void {
			renderer.setContext3D(s3d.context3D);
			renderer.camera(0, 0, 10);
		}
		
		private function frame(e:Event = null):void {
			count++;
			renderer.camera(
				Math.cos((320 - mouseX) * 0.01) * 18,
				(240 - mouseY) * 0.2,
				Math.sin((320 - mouseX) * 0.01) * 18
			);
			world.step();
			fps += (1000 / world.performance.totalTime - fps) * 0.5;
			if (fps > 1000 || fps != fps) {
				fps = 1000;
			}
			tf.text =
				"Rigid Body Count: " + world.numRigidBodies + "\n" +
				"Shape Count: " + world.numShapes + "\n" +
				"Contact Count: " + world.numContacts + "\n" +
				"Island Count: " + world.numIslands + "\n\n" +
				"Broad Phase Time: " + world.performance.broadPhaseTime + "ms\n" +
				"Narrow Phase Time: " + world.performance.narrowPhaseTime + "ms\n" +
				"Solving Time: " + world.performance.solvingTime + "ms\n" +
				"Updating Time: " + world.performance.updatingTime + "ms\n" +
				"Total Time: " + world.performance.totalTime + "ms\n" +
				"Physics FPS: " + fps.toFixed(2) + "\n"
			;
			renderer.render();
			var len:uint = world.numRigidBodies;
			var rbs:Vector.<RigidBody> = world.rigidBodies;
			for (var i:int = 0; i < len; i++) {
				var r:RigidBody = rbs[i];
				if (r.position.y < -24) {
					r.position.init(Math.random() * 8 - 4, Math.random() * 8 + 8, Math.random() * 8 - 4);
				}
			}
		}
		
	}

}