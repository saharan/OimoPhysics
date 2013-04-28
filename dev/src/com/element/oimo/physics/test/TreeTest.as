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
	import com.element.oimo.physics.collision.broadphase.AABB;
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
	import flash.utils.Dictionary;
	import flash.utils.getTimer;
	import flash.utils.setTimeout;
	import net.hires.debug.Stats;
	/**
	 * DBVT
	 * @author saharan
	 */
	[SWF(width = "640", height = "640", frameRate = "60")]
	public class TreeTest extends Sprite {
		private var count:uint;
		private var tf:TextField;
		private var ts:Dictionary;
		private var tree:DynamicBVTree;
		private var px:Number;
		private var py:Number;
		private var press:Boolean;
		
		public function TreeTest() {
			if (stage) init();
			else addEventListener(Event.ADDED_TO_STAGE, init);
		}
		
		private function init(e:Event = null):void {
			removeEventListener(Event.ADDED_TO_STAGE, init);
			tf = new TextField();
			tf.selectable = false;
			tf.defaultTextFormat = new TextFormat("Courier New", 10, 0xffffff, null, null, null, null, null, null, null, null, null, -6);
			tf.alpha = 0.4;
			tf.x = 0;
			tf.y = 0;
			tf.width = 640;
			tf.height = 640;
			addChild(tf);
			ts = new Dictionary(true);
			tree = new DynamicBVTree();
			var id:int = 0;
			stage.addEventListener(Event.ENTER_FRAME, frame);
			stage.addEventListener(MouseEvent.MOUSE_DOWN, function(e:Event = null):void {
				if (tree.root != null && deleteNode(tree.root)) return;
				px = mouseX;
				py = mouseY;
				press = true;
			});
			stage.addEventListener(MouseEvent.MOUSE_UP, function(e:Event = null):void {
				if (!press) return;
				press = false;
				var cx:Number = px + mouseX >> 1;
				var cy:Number = py + mouseY >> 1;
				var w:Number = px - mouseX >> 1;
				var h:Number = py - mouseY >> 1;
				if (w < 0) w = -w;
				if (h < 0) h = -h;
				if (w < 4 || h < 4) return;
				addText(insert(tree, ++id, new AABB(cx - w, cx + w, cy - h, cy + h)));
			});
		}
		
		private function addText(node:DynamicBVTreeNode):void {
			var txt:TextField = new TextField();
			txt.alpha = 0.6;
			txt.selectable = false;
			txt.defaultTextFormat = new TextFormat("Courier New", 12, 0xffffff);
			txt.width = 100;
			txt.height = 32;
			txt.text = "[" + node.proxy.minX + "]";
			txt.x = node.aabb.minX;
			txt.y = node.aabb.minY;
			ts[node] = txt;
			addChild(txt);
		}
		
		private function deleteNode(node:DynamicBVTreeNode):Boolean {
			if (!node.aabb.intersectsWithPoint(mouseX, mouseY, 0)) return false;
			if (node.proxy == null) {
				var c1:DynamicBVTreeNode = node.child1;
				var c2:DynamicBVTreeNode = node.child2;
				return ((deleteNode(c1) ? 1 : 0) | (deleteNode(c2) ? 1 : 0)) != 0;
			} else {
				tree.deleteLeaf(node);
				removeChild(ts[node]);
				return true;
			}
		}
		
		private function frame(e:Event):void {
			graphics.clear();
			graphics.beginFill(0x101010);
			graphics.drawRect(0, 0, 640, 640);
			graphics.endFill();
			if (tree.root != null) {
				render(tree.root, 0, tree.root.height);
				tf.text = tree.print(tree.root, 0, "");
			}
			if (press) {
				graphics.lineStyle(1, 0xffff00);
				graphics.drawRect(px, py, mouseX - px, mouseY - py);
			}
		}
		
		private function insert(tree:DynamicBVTree, id:int, aabb:AABB):DynamicBVTreeNode {
			var leaf:DynamicBVTreeNode = new DynamicBVTreeNode();
			leaf.proxy = new Proxy(id);
			leaf.aabb.minX = aabb.minX;
			leaf.aabb.maxX = aabb.maxX;
			leaf.aabb.minY = aabb.minY;
			leaf.aabb.maxY = aabb.maxY;
			leaf.aabb.minZ = aabb.minZ;
			leaf.aabb.maxZ = aabb.maxZ;
			tree.insertLeaf(leaf);
			return leaf;
		}
		
		private function render(node:DynamicBVTreeNode, depth:int, maxDepth:int, hit:Boolean = true):void {
			if (hit == true && !node.aabb.intersectsWithPoint(mouseX, mouseY, 0)) {
				hit = false;
			}
			if (node.proxy == null) {
				var c1:DynamicBVTreeNode = node.child1;
				var c2:DynamicBVTreeNode = node.child2;
				render(c1, depth + 1, maxDepth, hit);
				render(c2, depth + 1, maxDepth, hit);
				if (!hit) return;
				graphics.lineStyle(1, 0xff0000, (maxDepth - depth) / maxDepth);
			} else {
				graphics.lineStyle(1, hit ? 0x00ff00 : 0x0000ff);
				if (hit) {
					var dx:int = Math.random() * 4 - 2; // wriggle around
					var dy:int = Math.random() * 4 - 2;
					node.aabb.minX += dx;
					node.aabb.maxX += dx;
					node.aabb.minY += dy;
					node.aabb.maxY += dy;
					ts[node].x += dx;
					ts[node].y += dy;
					tree.deleteLeaf(node); // re-insert the node
					tree.insertLeaf(node);
				}
			}
			var minX:Number = node.aabb.minX;
			var minY:Number = node.aabb.minY;
			graphics.drawRect(minX, minY, node.aabb.maxX - minX, node.aabb.maxY - minY);
		}
		
	}

}