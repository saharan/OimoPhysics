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
	import com.element.oimo.physics.collision.shape.CylinderShape;
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
	import flash.events.MouseEvent;
	import flash.text.TextField;
	import flash.text.TextFormat;
	import flash.ui.Keyboard;
	import flash.utils.getTimer;
	import net.hires.debug.Stats;
	/**
	 * 負荷テスト
	 * @author saharan
	 */
	[SWF(width = "640", height = "480", frameRate = "60")]
	public class StressTest extends Sprite {
		private var count:uint;
		private var tf:TextField;
		
		public function StressTest() {
			if (stage) init();
			else addEventListener(Event.ADDED_TO_STAGE, init);
		}
		
		private function init(e:Event = null):void {
			removeEventListener(Event.ADDED_TO_STAGE, init);
			
			tf = new TextField();
			tf.selectable = false;
			tf.defaultTextFormat = new TextFormat("Courier New", 12, 0);
			tf.x = 4;
			tf.y = 4;
			tf.width = 400;
			tf.height = 400;
			addChild(tf);
			tf.text = "Stress test: click to run\n\n";
			stage.addEventListener(MouseEvent.MOUSE_DOWN, test);
		}
		
		private var array:Vector.<int>;
		private var cls:RigidBody;
		private var data:int;
		private function test(e:Event = null):void {
			var st:int;
			var en:int;
			var idx:int;
			var i:int;
			array = new Vector.<int>(4, true);
			array[0] = 1;
			array[1] = 2;
			array[2] = 3;
			array[3] = 0;
			
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = array[idx];
			}
			en = getTimer();
			tf.appendText("array:  " + (en - st) + "ms\n");
			
			cls = new RigidBody();
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = cls.type;
			}
			en = getTimer();
			tf.appendText("class:  " + (en - st) + "ms\n");
			
			data = 0;
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = data;
			}
			en = getTimer();
			tf.appendText("field:  " + (en - st) + "ms\n");
			
			var array2:Vector.<int> = array;
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = array2[idx];
			}
			en = getTimer();
			tf.appendText("array2: " + (en - st) + "ms\n");
			
			var cls2:RigidBody = cls;
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = cls2.type;
			}
			en = getTimer();
			tf.appendText("class2: " + (en - st) + "ms\n");
			
			var data2:int = data;
			st = getTimer();
			idx = 0;
			for (i = 0; i < 100000000; i++) {
				idx = data2;
			}
			en = getTimer();
			tf.appendText("field2: " + (en - st) + "ms\n\n");
			
			
		}
		
	}

}