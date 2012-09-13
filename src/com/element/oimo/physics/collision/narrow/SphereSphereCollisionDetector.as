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
package com.element.oimo.physics.collision.narrow {
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.SphereShape;
	/**
	 * 球体同士の詳細な衝突検出を行います。
	 * @author saharan
	 */
	public class SphereSphereCollisionDetector extends NarrowPhase {
		
		/**
		 * 新しく SphereSphereCollisionDetector オブジェクトを作成します。
		 */
		public function SphereSphereCollisionDetector() {
		}
		
		/**
		 * 球体同士の詳細な衝突検出を行います。
		 * 形状の種類はどちらも球体である必要があります。
		 * @param	shape1 球体1
		 * @param	shape2 球体2
		 * @param	contactInfos 接触点情報を格納する配列
		 * @param	numContactInfos 現在の接触点情報の数
		 * @return 新しい接触点情報の数
		 */
		override public function collisionDetection(shape1:Shape, shape2:Shape, contactInfos:Vector.<ContactInfo>, numContactInfos:uint):uint {
			var s1:SphereShape = shape1 as SphereShape;
			var s2:SphereShape = shape2 as SphereShape;
			var p1:Vec3 = s1.position;
			var p2:Vec3 = s2.position;
			var dx:Number = p2.x - p1.x;
			var dy:Number = p2.y - p1.y;
			var dz:Number = p2.z - p1.z;
			var len:Number = dx * dx + dy * dy + dz * dz;
			var rad:Number = s1.radius + s2.radius;
			if (len > 0 && len < rad * rad && contactInfos.length > numContactInfos) {
				len = Math.sqrt(len);
				var invLen:Number = 1 / len;
				dx *= invLen;
				dy *= invLen;
				dz *= invLen;
				if (!contactInfos[numContactInfos]) {
					contactInfos[numContactInfos] = new ContactInfo();
				}
				var c:ContactInfo = contactInfos[numContactInfos++];
				c.normal.x = dx;
				c.normal.y = dy;
				c.normal.z = dz;
				c.position.x = p1.x + dx * s1.radius;
				c.position.y = p1.y + dy * s1.radius;
				c.position.z = p1.z + dz * s1.radius;
				c.overlap = len - rad;
				c.shape1 = s1;
				c.shape2 = s2;
				c.id.data1 = 0;
				c.id.data2 = 0;
				c.id.flip = false;
			}
			return numContactInfos;
		}
		
	}

}