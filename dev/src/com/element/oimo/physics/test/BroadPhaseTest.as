/* Copyright (c) 2012-2013 EL-EMENT saharan
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
	import com.element.oimo.physics.constraint.joint.DistanceJoint;
	import com.element.oimo.physics.constraint.joint.Joint;
	import com.element.oimo.physics.constraint.joint.JointConfig;
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
	import flash.events.KeyboardEvent;
	import flash.text.TextField;
	import flash.text.TextFormat;
	import flash.ui.Keyboard;
	import net.hires.debug.Stats;
	/**
	 * 動作テスト
	 * @author saharan
	 */
	[SWF(width = "640", height = "480", frameRate = "60")]
	public class BroadPhaseTest extends Sprite {
		private var s3d:Stage3D;
		private var world:World;
		private var renderer:DebugDraw;
		private var rigid:RigidBody;
		private var count:uint;
		private var tf:TextField;
		private var fps:Number;
		private var l:Boolean;
		private var r:Boolean;
		private var u:Boolean;
		private var d:Boolean;
		private var ctr:RigidBody;
		BoxTest
		JointTest
		PyramidTest
		SphereStackTest
		
		public function BroadPhaseTest() {
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
			tf.defaultTextFormat = new TextFormat("courier new", 12, 0xffffff);
			tf.x = 4;
			tf.y = 4;
			tf.width = 400;
			tf.height = 400;
			addChild(tf);
			initWorld();
			fps = 0;
			
			s3d = stage.stage3Ds[0];
			s3d.addEventListener(Event.CONTEXT3D_CREATE, onContext3DCreated);
			s3d.requestContext3D();
			stage.addEventListener(KeyboardEvent.KEY_DOWN, function(e:KeyboardEvent):void {
				var code:uint = e.keyCode;
				if (code == Keyboard.W) {
					u = true;
				}
				if (code == Keyboard.S) {
					d = true;
				}
				if (code == Keyboard.A) {
					l = true;
				}
				if (code == Keyboard.D) {
					r = true;
				}
				if (code == Keyboard.SPACE) {
					initWorld();
				}
			});
			stage.addEventListener(KeyboardEvent.KEY_UP, function(e:KeyboardEvent):void {
				var code:uint = e.keyCode;
				if (code == Keyboard.W) {
					u = false;
				}
				if (code == Keyboard.S) {
					d = false;
				}
				if (code == Keyboard.A) {
					l = false;
				}
				if (code == Keyboard.D) {
					r = false;
				}
			});
			addEventListener(Event.ENTER_FRAME, frame);
		}
		
		private function initWorld():void {
			if (world) world.clear();
			else world = new World();
			if (!renderer) renderer = new DebugDraw(640, 480);
			renderer.setWorld(world);
			var rb:RigidBody;
			var s:Shape;
			var c:ShapeConfig = new ShapeConfig();
			// c.friction = 0;
			// c.restitution = 0;
			rb = new RigidBody();
			c.position.init(0, -0.5, 0);
			c.rotation.init();
			s = new BoxShape(32, 1, 32, c);
			rb.addShape(s);
			rb.setupMass(RigidBody.BODY_STATIC);
			//world.addRigidBody(rb);
			c.rotation.init();
			var num:uint = 1000;
			for (var i:int = 0; i < num; i++) {
				rb = new RigidBody();
				rb.allowSleep = false;
				c.position.init(Math.random() * 32 - 16, Math.random() * 32 - 16, Math.random() * 32 - 16);
				s = new BoxShape(Math.random() * 0.4 + 0.2, Math.random() * 0.4 + 0.2, Math.random() * 0.4 + 0.2, c);
				rb.addShape(s);
				rb.setupMass(RigidBody.BODY_DYNAMIC);
				world.addRigidBody(rb);
			}
			
			c.friction = 2;
			c.position.init(0, 1, 6);
			c.density = 10;
			c.rotation.init();
			s = new BoxShape(2, 2, 2, c);
			ctr = new RigidBody();
			ctr.addShape(s);
			ctr.setupMass(RigidBody.BODY_DYNAMIC);
			world.addRigidBody(ctr);
		}
		
		private function onContext3DCreated(e:Event = null):void {
			renderer.setContext3D(s3d.context3D);
			renderer.camera(0, 2, 4);
		}
		
		private function frame(e:Event = null):void {
			count++;
			var ang:Number = (320 - mouseX) * 0.01 + Math.PI * 0.5;
			renderer.camera(
				ctr.position.x + Math.cos(ang) * 6,
				ctr.position.y + (240 - mouseY) * 0.1,
				ctr.position.z + Math.sin(ang) * 6,
				ctr.position.x - Math.cos(ang),
				ctr.position.y,
				ctr.position.z - Math.sin(ang)
			);
			if (l) {
				ctr.linearVelocity.x -= Math.cos(ang - Math.PI * 0.5) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang - Math.PI * 0.5) * 0.8;
			}
			if (r) {
				ctr.linearVelocity.x -= Math.cos(ang + Math.PI * 0.5) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang + Math.PI * 0.5) * 0.8;
			}
			if (u) {
				ctr.linearVelocity.x -= Math.cos(ang) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang) * 0.8;
			}
			if (d) {
				ctr.linearVelocity.x += Math.cos(ang) * 0.8;
				ctr.linearVelocity.z += Math.sin(ang) * 0.8;
			}
			world.step();
			fps += (1000 / world.performance.totalTime - fps) * 0.5;
			if (fps > 1000 || fps != fps) {
				fps = 1000;
			}
			tf.text =
				"Rigid Body Count: " + world.numRigidBodies + "\n" +
				"Contact Count: " + world.numContacts + "\n" +
				"Pair Check Count: " + world.broadPhase.numPairChecks + "\n" +
				"Contact Point Count: " + world.numContactPoints + "\n" +
				"Island Count: " + world.numIslands + "\n\n" +
				"Broad Phase Time: " + world.performance.broadPhaseTime + "ms\n" +
				"Narrow Phase Time: " + world.performance.narrowPhaseTime + "ms\n" +
				"Solving Time: " + world.performance.solvingTime + "ms\n" +
				"Updating Time: " + world.performance.updatingTime + "ms\n" +
				"Total Time: " + world.performance.totalTime + "ms\n" +
				"Physics FPS: " + fps.toFixed(2) + "\n"
			;
			//var tree:DynamicBVTree = DynamicBVTreeBroadPhase(world.broadPhase).tree;
			//if (tree.root != null) {
				//tf.appendText(tree.print(tree.root, 0, ""));
			//}
			renderer.render();
			var body:RigidBody = world.rigidBodies;
			while (body != null) {
				if (body.position.y < -16) {
					body.position.init(Math.random() * 32 - 16, Math.random() * 32 - 16, Math.random() * 32 - 16);
					body.linearVelocity.x = Math.random() * 64 - 32;
					body.linearVelocity.y *= 0.8;
					body.linearVelocity.z = Math.random() * 64 - 32;
					body.angularVelocity.x = Math.random() * 16 - 8;
					body.angularVelocity.y = Math.random() * 16 - 8;
					body.angularVelocity.z = Math.random() * 16 - 8;
				}
				body = body.next;
			}
		}
		
	}

}