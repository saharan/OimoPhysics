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
package com.element.oimo.physics.util {
	import com.element.oimo.glmini.OimoGLMini;
	import com.element.oimo.math.Mat44;
	import com.element.oimo.physics.collision.shape.BoxShape;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.SphereShape;
	import com.element.oimo.physics.constraint.contact.Contact;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	import flash.display3D.*;
	
	/**
	 * 簡易的なワールドの3D表示機能を持ったクラスです。
	 * @author saharan
	 */
	public class DebugDraw {
		private var w:uint;
		private var h:uint;
		private var wld:World;
		private var c3d:Context3D;
		private var gl:OimoGLMini;
		private var m44:Mat44;
		
		/**
		 * 新しい DebugDraw オブジェクトを生成します。
		 * @param	width 画面の幅
		 * @param	height 画面の高さ
		 */
		public function DebugDraw(width:uint, height:uint) {
			w = width;
			h = height;
			m44 = new Mat44();
		}
		
		/**
		 * Context3D の初期化時に呼び出します。
		 * @param	context3D 使用する Context3D
		 */
		public function setContext3D(context3D:Context3D):void {
			c3d = context3D;
			gl = new OimoGLMini(c3d, w, h);
			gl.material(1, 1, 0, 0.4, 16);
			gl.registerSphere(0, 1, 10, 5);
			gl.registerBox(1, 1, 1, 1);
			gl.camera(0, 5, 10, 0, 0, 0, 0, 1, 0);
		}
		
		/**
		 * 描画対象のワールドを設定します。
		 * @param	world
		 */
		public function setWorld(world:World):void {
			wld = world;
		}
		
		/**
		 * カメラ位置などを設定します。
		 * 平行光源の向きもカメラの位置に合わせて調整されます。
		 * @param	camX カメラの x 座標
		 * @param	camY カメラの y 座標
		 * @param	camZ カメラの z 座標
		 * @param	targetX 注視点の x 座標
		 * @param	targetY 注視点の y 座標
		 * @param	targetZ 注視点の z 座標
		 * @param	upX 上方向のベクトルの x 成分
		 * @param	upY 上方向のベクトルの y 成分
		 * @param	upZ 上方向のベクトルの z 成分
		 */
		public function camera(
			camX:Number, camY:Number, camZ:Number,
			targetX:Number = 0, targetY:Number = 0, targetZ:Number = 0,
			upX:Number = 0, upY:Number = 1, upZ:Number = 0
		):void {
			gl.camera(camX, camY, camZ, targetX, targetY, targetZ, upX, upY, upZ);
			var dx:Number = targetX - camX;
			var dy:Number = targetY - camY;
			var dz:Number = targetZ - camZ;
			var len:Number = Math.sqrt(dx * dx + dy * dy + dz * dz);
			if (len > 0) len = 1 / len;
			gl.directionalLightDirection(dx * len, dy * len, dz * len);
		}
		
		/**
		 * ワールドのレンダリングを行います。
		 */
		public function render():void {
			if (!c3d) {
				return;
			}
			gl.beginScene(0.1, 0.1, 0.1);
			var cs:Vector.<Contact> = wld.contacts;
			var num:uint = wld.numContacts;
			/*gl.color(1, 0, 0);
			for (var j:int = 0; j < num; j++) {
				var c:Contact = cs[j];
				gl.push();
				gl.translate(c.position.x, c.position.y, c.position.z);
				gl.scale(0.1, 0.1, 0.1);
				gl.drawTriangles(0);
				gl.pop();
			}*/
			var ss:Vector.<Shape> = wld.shapes;
			num = wld.numShapes;
			for (var i:int = 0; i < num; i++) {
				var s:Shape = ss[i];
				gl.push();
				m44.copyMat33(s.rotation);
				m44.e03 = s.position.x;
				m44.e13 = s.position.y;
				m44.e23 = s.position.z;
				gl.transform(m44);
				switch(s.parent.type) {
				case RigidBody.BODY_DYNAMIC:
					gl.color(1, 0.8, 0.4/*, 0.75*/);
					break;
				case RigidBody.BODY_STATIC:
					gl.color(0.6, 1, 0.4/*, 0.75*/);
					break;
				}
				switch(s.type) {
				case Shape.SHAPE_SPHERE:
					var sph:SphereShape = s as SphereShape;
					gl.scale(sph.radius, sph.radius, sph.radius);
					gl.drawTriangles(0);
					break;
				case Shape.SHAPE_BOX:
					var box:BoxShape = s as BoxShape;
					gl.scale(box.width, box.height, box.depth);
					gl.drawTriangles(1);
					break;
				}
				gl.pop();
			}
			gl.endScene();
		}
	}

}