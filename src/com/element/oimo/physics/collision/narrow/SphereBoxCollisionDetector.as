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
	import com.element.oimo.physics.collision.shape.BoxShape;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.collision.shape.SphereShape;
	/**
	 * 球体と箱との詳細な衝突判定を行います。
	 * detectCollision 関数の引数に指定する形状は、
	 * コンストラクタで flip を true にしていない場合、
	 * 一つ目が球体、二つ目が箱である必要があります。
	 * @author saharan
	 */
	public class SphereBoxCollisionDetector extends CollisionDetector {
		
		/**
		 * 新しく SphereBoxCollisionDetector オブジェクトを作成します。
		 * @param	flip detectCollision 関数の引数に指定する形状の順序を、反転して受け取る場合は true
		 */
		public function SphereBoxCollisionDetector(flip:Boolean) {
			this.flip = flip;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function detectCollision(shape1:Shape, shape2:Shape, result:CollisionResult):void {
			var s:SphereShape;
			var b:BoxShape;
			if (flip) {
				s = shape2 as SphereShape;
				b = shape1 as BoxShape;
			} else {
				s = shape1 as SphereShape;
				b = shape2 as BoxShape;
			}
			var ps:Vec3 = s.position;
			var psx:Number = ps.x;
			var psy:Number = ps.y;
			var psz:Number = ps.z;
			var pb:Vec3 = b.position;
			var pbx:Number = pb.x;
			var pby:Number = pb.y;
			var pbz:Number = pb.z;
			var rad:Number = s.radius;
			// normal
			var nw:Vec3 = b.normalDirectionWidth;
			var nh:Vec3 = b.normalDirectionHeight;
			var nd:Vec3 = b.normalDirectionDepth;
			// half
			var hw:Number = b.halfWidth;
			var hh:Number = b.halfHeight;
			var hd:Number = b.halfDepth;
			// diff
			var dx:Number = psx - pbx;
			var dy:Number = psy - pby;
			var dz:Number = psz - pbz;
			// shadow
			var sx:Number = nw.x * dx + nw.y * dy + nw.z * dz;
			var sy:Number = nh.x * dx + nh.y * dy + nh.z * dz;
			var sz:Number = nd.x * dx + nd.y * dy + nd.z * dz;
			// closest
			var cx:Number;
			var cy:Number;
			var cz:Number;
			var len:Number;
			var invLen:Number;
			var c:ContactInfo;
			var overlap:uint = 0;
			if (sx > hw) {
				sx = hw;
			} else if (sx < -hw) {
				sx = -hw;
			} else {
				overlap = 1;
			}
			if (sy > hh) {
				sy = hh;
			} else if (sy < -hh) {
				sy = -hh;
			} else {
				overlap |= 2;
			}
			if (sz > hd) {
				sz = hd;
			} else if (sz < -hd) {
				sz = -hd;
			} else {
				overlap |= 4;
			}
			if (overlap == 7) {
				// center of sphere is in the box
				if (sx < 0) {
					dx = hw + sx;
				} else {
					dx = hw - sx;
				}
				if (sy < 0) {
					dy = hh + sy;
				} else {
					dy = hh - sy;
				}
				if (sz < 0) {
					dz = hd + sz;
				} else {
					dz = hd - sz;
				}
				if (dx < dy) {
					if (dx < dz) {
						// x
						len = dx - hw;
						if (sx < 0) {
							sx = -hw;
							dx = nw.x;
							dy = nw.y;
							dz = nw.z;
						} else {
							sx = hw;
							dx = -nw.x;
							dy = -nw.y;
							dz = -nw.z;
						}
					} else {
						// z
						len = dz - hd;
						if (sz < 0) {
							sz = -hd;
							dx = nd.x;
							dy = nd.y;
							dz = nd.z;
						} else {
							sz = hd;
							dx = -nd.x;
							dy = -nd.y;
							dz = -nd.z;
						}
					}
				} else {
					if (dy < dz) {
						// y
						len = dy - hh;
						if (sy < 0) {
							sy = -hh;
							dx = nh.x;
							dy = nh.y;
							dz = nh.z;
						} else {
							sy = hh;
							dx = -nh.x;
							dy = -nh.y;
							dz = -nh.z;
						}
					} else {
						// z
						len = dz - hd;
						if (sz < 0) {
							sz = -hd;
							dx = nd.x;
							dy = nd.y;
							dz = nd.z;
						} else {
							sz = hd;
							dx = -nd.x;
							dy = -nd.y;
							dz = -nd.z;
						}
					}
				}
				cx = pbx + sx * nw.x + sy * nh.x + sz * nd.x;
				cy = pby + sx * nw.y + sy * nh.y + sz * nd.y;
				cz = pbz + sx * nw.z + sy * nh.z + sz * nd.z;
				result.addContactInfo(psx + rad * dx, psy + rad * dy, psz + rad * dz, dx, dy, dz, len, s, b, 0, 0, false);
			} else {
				// closest
				cx = pbx + sx * nw.x + sy * nh.x + sz * nd.x;
				cy = pby + sx * nw.y + sy * nh.y + sz * nd.y;
				cz = pbz + sx * nw.z + sy * nh.z + sz * nd.z;
				dx = cx - ps.x;
				dy = cy - ps.y;
				dz = cz - ps.z;
				len = dx * dx + dy * dy + dz * dz;
				if (len > 0 && len < rad * rad) {
					len = Math.sqrt(len);
					invLen = 1 / len;
					dx *= invLen;
					dy *= invLen;
					dz *= invLen;
					result.addContactInfo(psx + rad * dx, psy + rad * dy, psz + rad * dz, dx, dy, dz, len, s, b, 0, 0, false);
				}
			}
		}
		
	}

}