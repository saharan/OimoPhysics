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
	import com.element.oimo.physics.collision.shape.CylinderShape;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.SphereShape;
	/**
	 * 球体と円柱との詳細な衝突判定を行います。
	 * detectCollision 関数の引数に指定する形状は、
	 * コンストラクタで flip を true にしていない場合、
	 * 一つ目が球体、二つ目が円柱である必要があります。
	 * @author saharan
	 */
	public class SphereCylinderCollisionDetector extends CollisionDetector {
		
		/**
		 * 新しく SphereCylinderCollisionDetector オブジェクトを作成します。
		 * @param	flip detectCollision 関数の引数に指定する形状の順序を、反転して受け取る場合は true
		 */
		public function SphereCylinderCollisionDetector(flip:Boolean) {
			this.flip = flip;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function detectCollision(shape1:Shape, shape2:Shape, result:CollisionResult):void {
			var s:SphereShape;
			var c:CylinderShape;
			if (flip) {
				s = shape2 as SphereShape;
				c = shape1 as CylinderShape;
			} else {
				s = shape1 as SphereShape;
				c = shape2 as CylinderShape;
			}
			var ps:Vec3 = s.position;
			var psx:Number = ps.x;
			var psy:Number = ps.y;
			var psz:Number = ps.z;
			var pc:Vec3 = c.position;
			var pcx:Number = pc.x;
			var pcy:Number = pc.y;
			var pcz:Number = pc.z;
			var dirx:Number = c.normalDirection.x;
			var diry:Number = c.normalDirection.y;
			var dirz:Number = c.normalDirection.z;
			var rads:Number = s.radius;
			var radc:Number = c.radius;
			var rad2:Number = rads + radc;
			var halfh:Number = c.halfHeight;
			var dx:Number = psx - pcx;
			var dy:Number = psy - pcy;
			var dz:Number = psz - pcz;
			var dot:Number = dx * dirx + dy * diry + dz * dirz;
			if (dot < -halfh - rads || dot > halfh + rads) return;
			var cx:Number = pcx + dot * dirx;
			var cy:Number = pcy + dot * diry;
			var cz:Number = pcz + dot * dirz;
			var d2x:Number = psx - cx;
			var d2y:Number = psy - cy;
			var d2z:Number = psz - cz;
			var len:Number = d2x * d2x + d2y * d2y + d2z * d2z;
			if (len > rad2 * rad2) return;
			if (len > radc * radc) {
				len = radc / Math.sqrt(len);
				d2x *= len;
				d2y *= len;
				d2z *= len;
			}
			if (dot < -halfh) dot = -halfh;
			else if (dot > halfh) dot = halfh;
			cx = pcx + dot * dirx + d2x;
			cy = pcy + dot * diry + d2y;
			cz = pcz + dot * dirz + d2z;
			dx = cx - psx;
			dy = cy - psy;
			dz = cz - psz;
			len = dx * dx + dy * dy + dz * dz;
			var invLen:Number;
			if (len > 0 && len < rads * rads) {
				len = Math.sqrt(len);
				invLen = 1 / len;
				dx *= invLen;
				dy *= invLen;
				dz *= invLen;
				result.addContactInfo(psx + dx * rads, psy + dy * rads, psz + dz * rads, dx, dy, dz, len - rads, s, c, 0, 0, false);
			}
		}
		
	}

}