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
	import com.element.oimo.physics.collision.shape.CylinderShape;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.SphereShape;
	import com.element.oimo.physics.constraint.contact.Contact;
	import com.element.oimo.physics.constraint.joint.Joint;
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
		private var ignores:Vector.<Shape>;
		private var numIgnores:uint;
		
		/**
		 * 新しい DebugDraw オブジェクトを生成します。
		 * @param	width 画面の幅
		 * @param	height 画面の高さ
		 */
		public function DebugDraw(width:uint, height:uint) {
			w = width;
			h = height;
			m44 = new Mat44();
			ignores = new Vector.<Shape>(1024, true);
			numIgnores = 0;
		}
		
		/**
		 * Context3D の初期化時に呼び出します。
		 * @param	context3D 使用する Context3D
		 */
		public function setContext3D(context3D:Context3D):void {
			c3d = context3D;
			gl = new OimoGLMini(c3d, w, h);
			gl.material(1, 1, 0, 0.6, 32);
			gl.registerSphere(0, 1, 8, 4);
			gl.registerBox(1, 1, 1, 1);
			gl.registerCylinder(2, 1, 1, 8);
			gl.camera(0, 5, 10, 0, 0, 0, 0, 1, 0);
		}
		
		/**
		 * 描画対象のワールドを設定します。
		 * @param	world
		 */
		public function setWorld(world:World):void {
			clearIgnores();
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
		 * 描画から除外する形状を追加します。
		 * @param	shape 描画から除外する形状
		 */
		public function addIgnore(shape:Shape):void {
			ignores[numIgnores++] = shape;
		}
		
		/**
		 * 描画から除外した形状を全て削除します。
		 */
		public function clearIgnores():void {
			while (numIgnores > 0) {
				ignores[--numIgnores] = null;
			}
		}
		
		/**
		 * ワールドのレンダリングを行います。
		 */
		public function render():void {
			if (!c3d) {
				return;
			}
			gl.beginScene(0.1, 0.1, 0.1);
			var alpha:Number = 1;
			var drawContacts:Boolean = false;
			var drawNormals:Boolean = true;
			var drawForces:Boolean = false;
			var drawJoints:Boolean = false;
			var cs:Vector.<Contact> = wld.contacts;
			var num:uint = wld.numContacts;
			if (drawContacts) {
				for (var j:int = 0; j < num; j++) {
					var c:Contact = cs[j];
					gl.push();
					gl.translate(c.position.x, c.position.y, c.position.z);
					if (drawNormals) gl.push();
					if (c.warmStarted) {
						gl.scale(0.1, 0.1, 0.1);
						gl.color(0.5, 0.5, 0.5);
					} else {
						gl.scale(0.15, 0.15, 0.15);
						gl.color(1, 1, 0);
					}
					gl.drawTriangles(0);
					if (drawNormals) {
						gl.pop();
						gl.push();
						if (drawForces) gl.translate(c.normal.x * -c.normalImpulse * 0.3, c.normal.y * -c.normalImpulse * 0.3, c.normal.z * -c.normalImpulse * 0.3);
						else gl.translate(c.normal.x * 0.5, c.normal.y * 0.5, c.normal.z * 0.5);
						var size:Number = 0.075 + Math.sqrt(-c.normalImpulse / c.normalDenominator) * 0.1;
						gl.scale(size, size, size);
						gl.color(1, 0.2, 0.2);
						gl.drawTriangles(0);
						gl.pop();
						if (!drawForces) {
							gl.push();
							gl.translate(c.tangent.x * 0.2, c.tangent.y * 0.2, c.tangent.z * 0.2);
							gl.scale(0.075, 0.075, 0.075);
							gl.color(0.2, 0.6, 0.2);
							gl.drawTriangles(0);
							gl.pop();
							gl.push();
							gl.translate(c.binormal.x * 0.2, c.binormal.y * 0.2, c.binormal.z * 0.2);
							gl.scale(0.075, 0.075, 0.075);
							gl.color(0.2, 0.2, 1);
							gl.drawTriangles(0);
							gl.pop();
						}
						if (drawForces) {
							gl.push();
							gl.translate(
								(c.tangent.x * c.tangentImpulse + c.binormal.x * c.binormalImpulse) * 0.3,
								(c.tangent.y * c.tangentImpulse + c.binormal.y * c.binormalImpulse) * 0.3,
								(c.tangent.z * c.tangentImpulse + c.binormal.z * c.binormalImpulse) * 0.3
							);
							size = 0.075 +
								Math.sqrt(
									(c.tangentImpulse > 0 ? c.tangentImpulse : -c.tangentImpulse)
									/ c.tangentDenominator +
									(c.binormalImpulse > 0 ? c.binormalImpulse : -c.binormalImpulse)
									/ c.binormalDenominator
								) * 0.1
							;
							gl.scale(size, size, size);
							gl.color(0.2, 1, 1);
							gl.drawTriangles(0);
							gl.pop();
						}
					}
					gl.pop();
				}
			}
			var js:Vector.<Joint> = wld.joints;
			num = wld.numJoints;
			if (drawJoints) {
				for (var k:int = 0; k < num; k++) {
					var jo:Joint = js[k];
					gl.push();
					gl.translate(jo.anchorPosition1.x, jo.anchorPosition1.y, jo.anchorPosition1.z);
					gl.scale(0.1, 0.1, 0.1);
					gl.color(0.2, 0.2, 0.8);
					gl.drawTriangles(0);
					gl.pop();
					gl.push();
					gl.translate(jo.anchorPosition2.x, jo.anchorPosition2.y, jo.anchorPosition2.z);
					gl.scale(0.1, 0.1, 0.1);
					gl.color(0.2, 0.2, 0.8);
					gl.drawTriangles(0);
					gl.pop();
				}
			}
			var ss:Vector.<Shape> = wld.shapes;
			num = wld.numShapes;
			shapeLoop: for (var i:int = 0; i < num; i++) {
				var s:Shape = ss[i];
				for (var l:int = 0; l < numIgnores; l++) {
					if (s == ignores[l]) continue shapeLoop;
				}
				gl.push();
				m44.copyMat33(s.rotation);
				m44.e03 = s.position.x;
				m44.e13 = s.position.y;
				m44.e23 = s.position.z;
				gl.transform(m44);
				switch(s.parent.type) {
				case RigidBody.BODY_DYNAMIC:
					if (s.id & 1) {
						if (s.parent.sleeping) gl.color(0.2, 0.8, 0.4, alpha);
						else if (s.parent.sleepTime > 0.5) gl.color(0.6, 0.7, 0.1, alpha);
						else gl.color(1, 0.6, 0.2, alpha);
					} else {
						if (s.parent.sleeping) gl.color(0.2, 0.4, 0.8, alpha);
						else if (s.parent.sleepTime > 0.5) gl.color(0.4, 0.3, 0.9, alpha);
						else gl.color(0.6, 0.2, 1, alpha);
					}
					break;
				case RigidBody.BODY_STATIC:
					gl.color(0.5, 0.5, 0.5, alpha);
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
				case Shape.SHAPE_CYLINDER:
					var cyl:CylinderShape = s as CylinderShape;
					gl.scale(cyl.radius, cyl.height, cyl.radius);
					gl.drawTriangles(2);
					break;
				}
				gl.pop();
			}
			gl.endScene();
		}
	}

}