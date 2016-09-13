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
	import com.element.oimo.physics.constraint.contact.Contact;
	import com.element.oimo.physics.constraint.joint.BallJoint;
	import com.element.oimo.physics.constraint.joint.DistanceJoint;
	import com.element.oimo.physics.constraint.joint.Hinge2Joint;
	import com.element.oimo.physics.constraint.joint.HingeJoint;
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
	import flash.geom.Matrix3D;
	import flash.text.TextField;
	import flash.text.TextFormat;
	import flash.ui.Keyboard;
	import net.hires.debug.Stats;
	/**
	 * 動作テスト
	 * @author saharan
	 */
	[SWF(width = "640", height = "480", frameRate = "60")]
	public class JointTest extends Sprite {
		private var s3d:Stage3D;
		private var world:World;
		private var renderer:DebugDraw;
		private var rigid:RigidBody;
		private var count:uint;
		private var tf:TextField;
		private var fps:Number;
		private var kw:Boolean;
		private var ka:Boolean;
		private var ks:Boolean;
		private var kd:Boolean;
		private var l:Boolean;
		private var r:Boolean;
		private var u:Boolean;
		private var d:Boolean;
		private var ctr:RigidBody;
		
		public function JointTest() {
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
			initWorld();
			fps = 0;
			
			s3d = stage.stage3Ds[0];
			s3d.addEventListener(Event.CONTEXT3D_CREATE, onContext3DCreated);
			s3d.requestContext3D();
			stage.addEventListener(KeyboardEvent.KEY_DOWN, function(e:KeyboardEvent):void {
				var code:uint = e.keyCode;
				if (code == Keyboard.W) {
					kw = true;
				}
				if (code == Keyboard.S) {
					ks = true;
				}
				if (code == Keyboard.A) {
					ka = true;
				}
				if (code == Keyboard.D) {
					kd = true;
				}
				if (code == Keyboard.UP) {
					u = true;
				}
				if (code == Keyboard.DOWN) {
					d = true;
				}
				if (code == Keyboard.LEFT) {
					l = true;
				}
				if (code == Keyboard.RIGHT) {
					r = true;
				}
				if (code == Keyboard.SPACE) {
					initWorld();
				}
			});
			stage.addEventListener(KeyboardEvent.KEY_UP, function(e:KeyboardEvent):void {
				var code:uint = e.keyCode;
				if (code == Keyboard.W) {
					kw = false;
				}
				if (code == Keyboard.S) {
					ks = false;
				}
				if (code == Keyboard.A) {
					ka = false;
				}
				if (code == Keyboard.D) {
					kd = false;
				}
				if (code == Keyboard.UP) {
					u = false;
				}
				if (code == Keyboard.DOWN) {
					d = false;
				}
				if (code == Keyboard.LEFT) {
					l = false;
				}
				if (code == Keyboard.RIGHT) {
					r = false;
				}
			});
			addEventListener(Event.ENTER_FRAME, frame);
		}
		
		private function initWorld():void {
			world = new World();
			if (!renderer) renderer = new DebugDraw(640, 480);
			renderer.setWorld(world);
			var rb:RigidBody;
			var s:Shape;
			var c:ShapeConfig = new ShapeConfig();
			// c.restitution = 0;
			// c.friction = 0;
			rb = new RigidBody();
			c.position.init(0, -0.5, 0);
			s = new BoxShape(48, 1, 48, c);
			rb.addShape(s);
			c.position.init(24, 8, 0);
			s = new BoxShape(1, 16, 48, c);
			rb.addShape(s);
			renderer.addIgnore(s);
			c.position.init(-24, 8, 0);
			s = new BoxShape(1, 16, 48, c);
			rb.addShape(s);
			renderer.addIgnore(s);
			c.position.init(0, 8, 24);
			s = new BoxShape(48, 16, 1, c);
			rb.addShape(s);
			renderer.addIgnore(s);
			c.position.init(0, 8, -24);
			s = new BoxShape(48, 16, 1, c);
			rb.addShape(s);
			renderer.addIgnore(s);
			rb.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(rb);
			
			c.position.init(0, 0.5, 12);
			c.density = 6;
			c.rotation.init();
			s = new SphereShape(0.75, c);
			ctr = new RigidBody();
			ctr.addShape(s);
			ctr.setupMass(RigidBody.BODY_DYNAMIC);
			world.addRigidBody(ctr);
			
			var jc:JointConfig = new JointConfig();
			jc.allowCollision = false;
			jc.localRelativeAnchorPosition1.y += 0.75;
			jc.localRelativeAnchorPosition2.y -= 0.3;
			var jo:Joint;
			var prv:RigidBody = ctr;
			c.position.y += 0.5;
			for (var m:int = 0; m < 8; m++) {
				c.position.y += 0.6;
				s = (m & 1) ? new BoxShape(0.6, 0.6, 0.6, c) : new SphereShape(0.3, c);
				rb = new RigidBody();
				rb.addShape(s);
				rb.setupMass(RigidBody.BODY_DYNAMIC);
				world.addRigidBody(rb);
				jo = new BallJoint(prv, rb, jc);
				world.addJoint(jo);
				prv = rb;
				if (m == 0) jc.localRelativeAnchorPosition1.y = 0.3;
			}
			c.density = 1;
			c.friction = 2;
			// create carts
			for (var n:int = 0; n < 8; n++) {
				var body:RigidBody;
				var wheel:RigidBody;
				body = new RigidBody();
				c.position.x = 7;
				c.position.y = 2 + n * 1.5;
				c.position.z = 0;
				s = new BoxShape(2, 0.5, 2, c);
				body.addShape(s);
				body.setupMass();
				world.addRigidBody(body);
				
				wheel = new RigidBody();
				c.position.x = 6;
				c.position.y = 2 + n * 1.5;
				c.position.z = 1;
				s = new SphereShape(0.5, c);
				wheel.addShape(s);
				wheel.setupMass();
				world.addRigidBody(wheel);
				Shape.nextID++;
				jc.localRelativeAnchorPosition1.init(-1, 0, 1);
				jc.localRelativeAnchorPosition2.init();
				jo = new BallJoint(body, wheel, jc);
				world.addJoint(jo);
				
				wheel = new RigidBody();
				c.position.x = 6;
				c.position.y = 2 + n * 1.5;
				c.position.z = -1;
				s = new SphereShape(0.5, c);
				wheel.addShape(s);
				wheel.setupMass();
				world.addRigidBody(wheel);
				Shape.nextID++;
				jc.localRelativeAnchorPosition1.init(-1, 0, -1);
				jc.localRelativeAnchorPosition2.init();
				jo = new BallJoint(body, wheel, jc);
				world.addJoint(jo);
				
				wheel = new RigidBody();
				c.position.x = 8;
				c.position.y = 2 + n * 1.5;
				c.position.z = 1;
				s = new SphereShape(0.5, c);
				wheel.addShape(s);
				wheel.setupMass();
				world.addRigidBody(wheel);
				Shape.nextID++;
				jc.localRelativeAnchorPosition1.init(1, 0, 1);
				jc.localRelativeAnchorPosition2.init();
				jo = new BallJoint(body, wheel, jc);
				world.addJoint(jo);
				
				wheel = new RigidBody();
				c.position.x = 8;
				c.position.y = 2 + n * 1.5;
				c.position.z = -1;
				s = new SphereShape(0.5, c);
				wheel.addShape(s);
				wheel.setupMass();
				world.addRigidBody(wheel);
				Shape.nextID++;
				jc.localRelativeAnchorPosition1.init(1, 0, -1);
				jc.localRelativeAnchorPosition2.init();
				jo = new BallJoint(body, wheel, jc);
				world.addJoint(jo);
			}
			c.friction = 0.5;
			
			jc.localAxis1.init(1, 0, 0);
			jc.localAxis2.init(1, 0, 0);
			jc.allowCollision = false;
			for (var o:int = 0; o < 3; o++) {
				jc.localRelativeAnchorPosition1.init(0, 0.3, 0);
				jc.localRelativeAnchorPosition2.init(0, -0.3, 0);
				c.position.init(-5 - o * 2, 0, 0);
				for (m = 0; m < 12; m++) {
					c.position.y += 0.4;
					s = m == 11 ? new SphereShape(0.2, c) : new BoxShape(1, 0.6, 0.5, c);
					rb = new RigidBody();
					rb.addShape(s);
					rb.setupMass(m == 11 ? RigidBody.BODY_STATIC : RigidBody.BODY_DYNAMIC);
					world.addRigidBody(rb);
					if (m > 0) {
						if (m == 11) {
							renderer.addIgnore(s);
							rb.position.y += 10;
							jc.localRelativeAnchorPosition2.y = -10;
						}
						jo = new HingeJoint(prv, rb, jc);
						if (m != 11) {
							(jo as HingeJoint).enableLimits = true;
							(jo as HingeJoint).lowerLimit = -Math.PI * 0.2 * (o + 1);
							(jo as HingeJoint).upperLimit = Math.PI * 0.2 * (o + 1);
						}
						world.addJoint(jo);
					}
					prv = rb;
				}
			}
			
			jc.localAxis1.init(0, 1, 0);
			jc.localAxis2.init(0, 1, 0);
			jc.localRelativeAnchorPosition1.init(0, 0, 0);
			jc.localRelativeAnchorPosition2.init(0, -1, 0);
			
			c.position.init(0, 1.1, 0);
			s = new BoxShape(5, 2, 0.4, c);
			rb = new RigidBody();
			rb.addShape(s);
			rb.setupMass();
			world.addRigidBody(rb);
			c.position.init(0, 2.1, 0);
			s = new SphereShape(0.25, c);
			prv = new RigidBody();
			prv.addShape(s);
			prv.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(prv);
			jo = new HingeJoint(rb, prv, jc);
			(jo as HingeJoint).enableMotor = true;
			(jo as HingeJoint).motorSpeed = Math.PI * 2 * 3;
			(jo as HingeJoint).maxMotorTorque = 300;
			world.addJoint(jo);
			
			c.position.init(0, 1.1, -7);
			s = new BoxShape(5, 2, 0.4, c);
			rb = new RigidBody();
			rb.addShape(s);
			rb.setupMass();
			world.addRigidBody(rb);
			c.position.init(0, 2.1, -7);
			s = new SphereShape(0.25, c);
			prv = new RigidBody();
			prv.addShape(s);
			prv.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(prv);
			jo = new HingeJoint(rb, prv, jc);
			(jo as HingeJoint).enableMotor = true;
			(jo as HingeJoint).motorSpeed = Math.PI * 2 * 8.33333;
			(jo as HingeJoint).maxMotorTorque = 50;
			world.addJoint(jo);
			
			jc.localAxis1.init(0, 0, 1);
			jc.localRelativeAnchorPosition2.init(0, 0, 0);
			jc.localRelativeAnchorPosition1.init(0, 0, -1.5);
			c.position.init(0, 1.5, 7);
			s = new BoxShape(2.4, 0.6, 2.5, c);
			rb = new RigidBody();
			rb.addShape(s);
			rb.setupMass();
			world.addRigidBody(rb);
			c.position.init(0, 2, 7);
			s = new SphereShape(0.25, c);
			prv = new RigidBody();
			prv.addShape(s);
			prv.setupMass(RigidBody.BODY_STATIC);
			world.addRigidBody(prv);
			jo = new Hinge2Joint(rb, prv, jc);
			(jo as Hinge2Joint).enableMotor1 = true;
			(jo as Hinge2Joint).motorSpeed1 = Math.PI * 2;
			(jo as Hinge2Joint).maxMotorTorque1 = 50;
			(jo as Hinge2Joint).enableMotor2 = true;
			(jo as Hinge2Joint).motorSpeed2 = Math.PI * 2;
			(jo as Hinge2Joint).maxMotorTorque2 = 50;
			world.addJoint(jo);
			
			c.position.init(0, 5, 0);
			jc.localAxis1.init(0, 1, 0);
			jc.localAxis2.init(0, 0, 1);
			jc.localRelativeAnchorPosition1.init(0, 1, 0);
			jc.localRelativeAnchorPosition2.init(0, 0, 1);
			for (var i:int = 0; i < 4; i++) {
				s = new BoxShape(1, 0.6, 1, c);
				rb = new RigidBody();
				rb.addShape(s);
				rb.setupMass();
				world.addRigidBody(rb);
				prv = rb;
				rb = new RigidBody();
				c.position.y += 1;
				s = new BoxShape(1, 1, 0.6, c);
				rb.addShape(s);
				rb.setupMass();
				world.addRigidBody(rb);
				jo = new Hinge2Joint(prv, rb, jc);
				world.addJoint(jo);
				c.position.y += 2;
			}
		}
		
		private function onContext3DCreated(e:Event = null):void {
			renderer.setContext3D(s3d.context3D);
			renderer.camera(0, 2, 4);
		}
		
		private function frame(e:Event = null):void {
			count++;
			var ang:Number = (320 - mouseX) * 0.01 + Math.PI * 0.5;
			renderer.camera(
				ctr.position.x + Math.cos(ang) * 8,
				ctr.position.y + (320 - mouseY) * 0.1,
				ctr.position.z + Math.sin(ang) * 8,
				ctr.position.x - Math.cos(ang) * 2,
				ctr.position.y,
				ctr.position.z - Math.sin(ang) * 2
			);
			if (l || ka) {
				ctr.linearVelocity.x -= Math.cos(ang - Math.PI * 0.5) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang - Math.PI * 0.5) * 0.8;
			}
			if (r || kd) {
				ctr.linearVelocity.x -= Math.cos(ang + Math.PI * 0.5) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang + Math.PI * 0.5) * 0.8;
			}
			if (u || kw) {
				ctr.linearVelocity.x -= Math.cos(ang) * 0.8;
				ctr.linearVelocity.z -= Math.sin(ang) * 0.8;
			}
			if (d || ks) {
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
				if (r.position.y < -12) {
					r.position.init(Math.random() * 4 - 2, Math.random() * 1 + 8, Math.random() * 4 - 2);
					r.linearVelocity.init();
				}
			}
		}
		
	}

}