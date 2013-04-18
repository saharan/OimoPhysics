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
	import com.element.oimo.physics.collision.broadphase.DynamicBVTree;
	import com.element.oimo.physics.collision.broadphase.DynamicBVTreeNode;
	import com.element.oimo.physics.collision.broadphase.Proxy;
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
	 * DBVTテスト
	 * @author saharan
	 */
	[SWF(width = "640", height = "480", frameRate = "60")]
	public class TreeTest extends Sprite {
		private var count:uint;
		private var tf:TextField;
		
		public function TreeTest() {
			if (stage) init();
			else addEventListener(Event.ADDED_TO_STAGE, init);
		}
		
		private function init(e:Event = null):void {
			removeEventListener(Event.ADDED_TO_STAGE, init);
			tf = new TextField();
			tf.selectable = false;
			tf.defaultTextFormat = new TextFormat("Courier New", 12, null, null, null, null, null, null, null, null, null, null, -6);
			tf.x = 4;
			tf.y = 4;
			tf.width = 640 - 8;
			tf.height = 480 - 8;
			addChild(tf);
			test();
		}
		
		private function test():void {
			var tree:DynamicBVTree = new DynamicBVTree();
			for (var i:int = 0; i < 20; i++) {
				var leaf:DynamicBVTreeNode = new DynamicBVTreeNode();
				leaf.proxy = new Proxy(i);
				tree.insertLeaf(leaf);
			}
			tf.text = tree.print(tree.root, 0, "");
		}
		
	}

}