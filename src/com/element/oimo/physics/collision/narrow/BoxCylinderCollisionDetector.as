package com.element.oimo.physics.collision.narrow {
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.collision.shape.BoxShape;
	import com.element.oimo.physics.collision.shape.CylinderShape;
	import com.element.oimo.physics.collision.shape.Shape;
	/**
	 * Minkowski portal refinement
	 * @author saharan
	 */
	public class BoxCylinderCollisionDetector extends CollisionDetector {
		
		private function getSep(c1:BoxShape, c2:CylinderShape, sep:Vec3, pos:Vec3, dep:Vec3):Boolean {
			var t1x:Number;
			var t1y:Number;
			var t1z:Number;
			var t2x:Number;
			var t2y:Number;
			var t2z:Number;
			var sup:Vec3 = new Vec3();
			var len:Number;
			var p1x:Number;
			var p1y:Number;
			var p1z:Number;
			var p2x:Number;
			var p2y:Number;
			var p2z:Number;
			var v01x:Number = c1.position.x;
			var v01y:Number = c1.position.y;
			var v01z:Number = c1.position.z;
			var v02x:Number = c2.position.x;
			var v02y:Number = c2.position.y;
			var v02z:Number = c2.position.z;
			var v0x:Number = v02x - v01x;
			var v0y:Number = v02y - v01y;
			var v0z:Number = v02z - v01z;
			if (v0x * v0x + v0y * v0y + v0z * v0z == 0) v0y = 0.001;
			var nx:Number = -v0x;
			var ny:Number = -v0y;
			var nz:Number = -v0z;
			supportPointB(c1, -nx, -ny, -nz, sup);
			var v11x:Number = sup.x;
			var v11y:Number = sup.y;
			var v11z:Number = sup.z;
			supportPointC(c2, nx, ny, nz, sup);
			var v12x:Number = sup.x;
			var v12y:Number = sup.y;
			var v12z:Number = sup.z;
			var v1x:Number = v12x - v11x;
			var v1y:Number = v12y - v11y;
			var v1z:Number = v12z - v11z;
			if (v1x * nx + v1y * ny + v1z * nz <= 0) {
				return false; // No collision
			}
			nx = v1y * v0z - v1z * v0y;
			ny = v1z * v0x - v1x * v0z;
			nz = v1x * v0y - v1y * v0x;
			if (nx * nx + ny * ny + nz * nz == 0) {
				sep.init(v1x - v0x, v1y - v0y, v1z - v0z);
				sep.normalize(sep);
				pos.init((v11x + v12x) * 0.5, (v11y + v12y) * 0.5, (v11z + v12z) * 0.5);
				return true;
			}
			supportPointB(c1, -nx, -ny, -nz, sup);
			var v21x:Number = sup.x;
			var v21y:Number = sup.y;
			var v21z:Number = sup.z;
			supportPointC(c2, nx, ny, nz, sup);
			var v22x:Number = sup.x;
			var v22y:Number = sup.y;
			var v22z:Number = sup.z;
			var v2x:Number = v22x - v21x;
			var v2y:Number = v22y - v21y;
			var v2z:Number = v22z - v21z;
			if (v2x * nx + v2y * ny + v2z * nz <= 0) {
				return false; // No collision
			}
			t1x = v1x - v0x;
			t1y = v1y - v0y;
			t1z = v1z - v0z;
			t2x = v2x - v0x;
			t2y = v2y - v0y;
			t2z = v2z - v0z;
			nx = t1y * t2z - t1z * t2y;
			ny = t1z * t2x - t1x * t2z;
			nz = t1x * t2y - t1y * t2x;
			if (nx * v0x + ny * v0y + nz * v0z > 0) { // flip v1 and v2
				t1x = v1x;
				t1y = v1y;
				t1z = v1z;
				v1x = v2x;
				v1y = v2y;
				v1z = v2z;
				v2x = t1x;
				v2y = t1y;
				v2z = t1z;
				t1x = v11x;
				t1y = v11y;
				t1z = v11z;
				v11x = v21x;
				v11y = v21y;
				v11z = v21z;
				v21x = t1x;
				v21y = t1y;
				v21z = t1z;
				t1x = v12x;
				t1y = v12y;
				t1z = v12z;
				v12x = v22x;
				v12y = v22y;
				v12z = v22z;
				v22x = t1x;
				v22y = t1y;
				v22z = t1z;
				nx = -nx;
				ny = -ny;
				nz = -nz;
			}
			var iterations:uint = 0;
			while (true) {
				if (++iterations > 100) {
					trace("!!!!!!!!!!!!!!!FAILED!!!!!!!!!!!!!!!");
					return false;
				}
				
				supportPointB(c1, -nx, -ny, -nz, sup);
				var v31x:Number = sup.x;
				var v31y:Number = sup.y;
				var v31z:Number = sup.z;
				supportPointC(c2, nx, ny, nz, sup);
				var v32x:Number = sup.x;
				var v32y:Number = sup.y;
				var v32z:Number = sup.z;
				var v3x:Number = v32x - v31x;
				var v3y:Number = v32y - v31y;
				var v3z:Number = v32z - v31z;
				if (v3x * nx + v3y * ny + v3z * nz <= 0) {
					return false; // No collision
				}
				if ((v1y * v3z - v1z * v3y) * v0x + (v1z * v3x - v1x * v3z) * v0y + (v1x * v3y - v1y * v3x) * v0z < 0) { // remove v2
					v2x = v3x;
					v2y = v3y;
					v2z = v3z;
					v21x = v31x;
					v21y = v31y;
					v21z = v31z;
					v22x = v32x;
					v22y = v32y;
					v22z = v32z;
					t1x = v1x - v0x;
					t1y = v1y - v0y;
					t1z = v1z - v0z;
					t2x = v3x - v0x;
					t2y = v3y - v0y;
					t2z = v3z - v0z;
					nx = t1y * t2z - t1z * t2y;
					ny = t1z * t2x - t1x * t2z;
					nz = t1x * t2y - t1y * t2x;
					continue;
				}
				if ((v3y * v2z - v3z * v2y) * v0x + (v3z * v2x - v3x * v2z) * v0y + (v3x * v2y - v3y * v2x) * v0z < 0) { // remove v1
					v1x = v3x;
					v1y = v3y;
					v1z = v3z;
					v11x = v31x;
					v11y = v31y;
					v11z = v31z;
					v12x = v32x;
					v12y = v32y;
					v12z = v32z;
					t1x = v3x - v0x;
					t1y = v3y - v0y;
					t1z = v3z - v0z;
					t2x = v2x - v0x;
					t2y = v2y - v0y;
					t2z = v2z - v0z;
					nx = t1y * t2z - t1z * t2y;
					ny = t1z * t2x - t1x * t2z;
					nz = t1x * t2y - t1y * t2x;
					continue;
				}
				var hit:Boolean = false;
				while (true) { // refinement phase
					t1x = v2x - v1x;
					t1y = v2y - v1y;
					t1z = v2z - v1z;
					t2x = v3x - v1x;
					t2y = v3y - v1y;
					t2z = v3z - v1z;
					nx = t1y * t2z - t1z * t2y;
					ny = t1z * t2x - t1x * t2z;
					nz = t1x * t2y - t1y * t2x;
					len = 1 / Math.sqrt(nx * nx + ny * ny + nz * nz);
					nx *= len;
					ny *= len;
					nz *= len;
					if (nx * v1x + ny * v1y + nz * v1z >= 0 && !hit) { // hit
						var b0:Number = (v1y * v2z - v1z * v2y) * v3x + (v1z * v2x - v1x * v2z) * v3y + (v1x * v2y - v1y * v2x) * v3z;
						var b1:Number = (v3y * v2z - v3z * v2y) * v0x + (v3z * v2x - v3x * v2z) * v0y + (v3x * v2y - v3y * v2x) * v0z;
						var b2:Number = (v0y * v1z - v0z * v1y) * v3x + (v0z * v1x - v0x * v1z) * v3y + (v0x * v1y - v0y * v1x) * v3z;
						var b3:Number = (v2y * v1z - v2z * v1y) * v0x + (v2z * v1x - v2x * v1z) * v0y + (v2x * v1y - v2y * v1x) * v0z;
						var sum:Number = b0 + b1 + b2 + b3;
						if (sum <= 0) {
							b0 = 0;
							b1 = (v2y * v3z - v2z * v3y) * nx + (v2z * v3x - v2x * v3z) * ny + (v2x * v3y - v2y * v3x) * nz;
							b2 = (v3y * v2z - v3z * v2y) * nx + (v3z * v2x - v3x * v2z) * ny + (v3x * v2y - v3y * v2x) * nz;
							b3 = (v1y * v2z - v1z * v2y) * nx + (v1z * v2x - v1x * v2z) * ny + (v1x * v2y - v1y * v2x) * nz;
							sum = b1 + b2 + b3;
						}
						var inv:Number = 1 / sum;
						p1x = (v01x * b0 + v11x * b1 + v21x * b2 + v31x * b3) * inv;
						p1y = (v01y * b0 + v11y * b1 + v21y * b2 + v31y * b3) * inv;
						p1z = (v01z * b0 + v11z * b1 + v21z * b2 + v31z * b3) * inv;
						p2x = (v02x * b0 + v12x * b1 + v22x * b2 + v32x * b3) * inv;
						p2y = (v02y * b0 + v12y * b1 + v22y * b2 + v32y * b3) * inv;
						p2z = (v02z * b0 + v12z * b1 + v22z * b2 + v32z * b3) * inv;
						hit = true;
					}
					supportPointB(c1, -nx, -ny, -nz, sup);
					var v41x:Number = sup.x;
					var v41y:Number = sup.y;
					var v41z:Number = sup.z;
					supportPointC(c2, nx, ny, nz, sup);
					var v42x:Number = sup.x;
					var v42y:Number = sup.y;
					var v42z:Number = sup.z;
					var v4x:Number = v42x - v41x;
					var v4y:Number = v42y - v41y;
					var v4z:Number = v42z - v41z;
					var separation:Number = -(v4x * nx + v4y * ny + v4z * nz);
					if ((v4x - v3x) * nx + (v4y - v3y) * ny + (v4z - v3z) * nz <= 0.01 || separation >= 0) {
						if (hit) {
							sep.init(-nx, -ny, -nz);
							pos.init((p1x + p2x) * 0.5, (p1y + p2y) * 0.5, (p1z + p2z) * 0.5);
							dep.x = separation;
							return true;
						}
						return false; // no hit
					}
					if (
						(v4y * v1z - v4z * v1y) * v0x +
						(v4z * v1x - v4x * v1z) * v0y +
						(v4x * v1y - v4y * v1x) * v0z < 0
					) {
						if (
							(v4y * v2z - v4z * v2y) * v0x +
							(v4z * v2x - v4x * v2z) * v0y +
							(v4x * v2y - v4y * v2x) * v0z < 0
						) { // remove v1
							v1x = v4x;
							v1y = v4y;
							v1z = v4z;
							v11x = v41x;
							v11y = v41y;
							v11z = v41z;
							v12x = v42x;
							v12y = v42y;
							v12z = v42z;
						} else { // remove v3
							v3x = v4x;
							v3y = v4y;
							v3z = v4z;
							v31x = v41x;
							v31y = v41y;
							v31z = v41z;
							v32x = v42x;
							v32y = v42y;
							v32z = v42z;
						}
					} else {
						if (
							(v4y * v3z - v4z * v3y) * v0x +
							(v4z * v3x - v4x * v3z) * v0y +
							(v4x * v3y - v4y * v3x) * v0z < 0
						) { // remove v2
							v2x = v4x;
							v2y = v4y;
							v2z = v4z;
							v21x = v41x;
							v21y = v41y;
							v21z = v41z;
							v22x = v42x;
							v22y = v42y;
							v22z = v42z;
						} else { // remove v1
							v1x = v4x;
							v1y = v4y;
							v1z = v4z;
							v11x = v41x;
							v11y = v41y;
							v11z = v41z;
							v12x = v42x;
							v12y = v42y;
							v12z = v42z;
						}
					}
				}
			}
			return false;
		}
		
		private function supportPointB(c:BoxShape, dx:Number, dy:Number, dz:Number, out:Vec3):void {
			var rot:Mat33 = c.rotation;
			var ldx:Number = rot.e00 * dx + rot.e10 * dy + rot.e20 * dz;
			var ldy:Number = rot.e01 * dx + rot.e11 * dy + rot.e21 * dz;
			var ldz:Number = rot.e02 * dx + rot.e12 * dy + rot.e22 * dz;
			var w:Number = c.halfWidth;
			var h:Number = c.halfHeight;
			var d:Number = c.halfDepth;
			var ox:Number;
			var oy:Number;
			var oz:Number;
			if (ldx < 0) ox = -w;
			else ox = w;
			if (ldy < 0) oy = -h;
			else oy = h;
			if (ldz < 0) oz = -d;
			else oz = d;
			ldx = rot.e00 * ox + rot.e01 * oy + rot.e02 * oz + c.position.x;
			ldy = rot.e10 * ox + rot.e11 * oy + rot.e12 * oz + c.position.y;
			ldz = rot.e20 * ox + rot.e21 * oy + rot.e22 * oz + c.position.z;
			out.init(ldx, ldy, ldz);
		}
		
		private function supportPointC(c:CylinderShape, dx:Number, dy:Number, dz:Number, out:Vec3):void {
			var rot:Mat33 = c.rotation;
			var ldx:Number = rot.e00 * dx + rot.e10 * dy + rot.e20 * dz;
			var ldy:Number = rot.e01 * dx + rot.e11 * dy + rot.e21 * dz;
			var ldz:Number = rot.e02 * dx + rot.e12 * dy + rot.e22 * dz;
			var radx:Number = ldx;
			var radz:Number = ldz;
			var len:Number = radx * radx + radz * radz;
			var rad:Number = c.radius;
			var hh:Number = c.halfHeight;
			var ox:Number;
			var oy:Number;
			var oz:Number;
			if (len == 0) {
				if (ldy < 0) {
					ox = rad;
					oy = -hh;
					oz = 0;
				} else {
					ox = rad;
					oy = hh;
					oz = 0;
				}
			} else {
				len = c.radius / Math.sqrt(len);
				if (ldy < 0) {
					ox = radx * len;
					oy = -hh;
					oz = radz * len;
				} else {
					ox = radx * len;
					oy = hh;
					oz = radz * len;
				}
			}
			ldx = rot.e00 * ox + rot.e01 * oy + rot.e02 * oz + c.position.x;
			ldy = rot.e10 * ox + rot.e11 * oy + rot.e12 * oz + c.position.y;
			ldz = rot.e20 * ox + rot.e21 * oy + rot.e22 * oz + c.position.z;
			out.init(ldx, ldy, ldz);
		}
		
		/**
		 * 新しく BoxCylinderCollisionDetector オブジェクトを作成します。
		 * @param	flip detectCollision 関数の引数に指定する形状の順序を、反転して受け取る場合は true
		 */
		public function BoxCylinderCollisionDetector(flip:Boolean) {
			this.flip = flip;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function detectCollision(shape1:Shape, shape2:Shape, result:CollisionResult):void {
			var b:BoxShape;
			var c:CylinderShape;
			if (flip) {
				b = shape2 as BoxShape;
				c = shape1 as CylinderShape;
			} else {
				b = shape1 as BoxShape;
				c = shape2 as CylinderShape;
			}
			var sep:Vec3 = new Vec3();
			var pos:Vec3 = new Vec3();
			var dep:Vec3 = new Vec3();
			var co:ContactInfo;
			if (!getSep(b, c, sep, pos, dep)) return;
			var pbx:Number = b.position.x;
			var pby:Number = b.position.y;
			var pbz:Number = b.position.z;
			var pcx:Number = c.position.x;
			var pcy:Number = c.position.y;
			var pcz:Number = c.position.z;
			var bw:Number = b.halfWidth;
			var bh:Number = b.halfHeight;
			var bd:Number = b.halfDepth;
			var ch:Number = c.halfHeight;
			var r:Number = c.radius;
			var nwx:Number = b.normalDirectionWidth.x;
			var nwy:Number = b.normalDirectionWidth.y;
			var nwz:Number = b.normalDirectionWidth.z;
			var nhx:Number = b.normalDirectionHeight.x;
			var nhy:Number = b.normalDirectionHeight.y;
			var nhz:Number = b.normalDirectionHeight.z;
			var ndx:Number = b.normalDirectionDepth.x;
			var ndy:Number = b.normalDirectionDepth.y;
			var ndz:Number = b.normalDirectionDepth.z;
			var dwx:Number = b.halfDirectionWidth.x;
			var dwy:Number = b.halfDirectionWidth.y;
			var dwz:Number = b.halfDirectionWidth.z;
			var dhx:Number = b.halfDirectionHeight.x;
			var dhy:Number = b.halfDirectionHeight.y;
			var dhz:Number = b.halfDirectionHeight.z;
			var ddx:Number = b.halfDirectionDepth.x;
			var ddy:Number = b.halfDirectionDepth.y;
			var ddz:Number = b.halfDirectionDepth.z;
			var ncx:Number = c.normalDirection.x;
			var ncy:Number = c.normalDirection.y;
			var ncz:Number = c.normalDirection.z;
			var dcx:Number = c.halfDirection.x;
			var dcy:Number = c.halfDirection.y;
			var dcz:Number = c.halfDirection.z;
			var nx:Number = sep.x;
			var ny:Number = sep.y;
			var nz:Number = sep.z;
			var dotw:Number = nx * nwx + ny * nwy + nz * nwz;
			var doth:Number = nx * nhx + ny * nhy + nz * nhz;
			var dotd:Number = nx * ndx + ny * ndy + nz * ndz;
			var dotc:Number = nx * ncx + ny * ncy + nz * ncz;
			var right1:Boolean = dotw > 0;
			var right2:Boolean = doth > 0;
			var right3:Boolean = dotd > 0;
			var right4:Boolean = dotc > 0;
			if (!right1) dotw = -dotw;
			if (!right2) doth = -doth;
			if (!right3) dotd = -dotd;
			if (!right4) dotc = -dotc;
			var state:uint = 0;
			if (dotc > 0.999) {
				if (dotw > 0.999) {
					if (dotw > dotc) state = 1;
					else state = 4;
				} else if (doth > 0.999) {
					if (doth > dotc) state = 2;
					else state = 4;
				} else if (dotd > 0.999) {
					if (dotd > dotc) state = 3;
					else state = 4;
				} else state = 4;
			} else {
				if (dotw > 0.999) state = 1;
				else if (doth > 0.999) state = 2;
				else if (dotd > 0.999) state = 3;
			}
			var cbx:Number;
			var cby:Number;
			var cbz:Number;
			var ccx:Number;
			var ccy:Number;
			var ccz:Number;
			var r00:Number;
			var r01:Number;
			var r02:Number;
			var r10:Number;
			var r11:Number;
			var r12:Number;
			var r20:Number;
			var r21:Number;
			var r22:Number;
			var px:Number;
			var py:Number;
			var pz:Number;
			var pd:Number;
			var dot:Number;
			var len:Number;
			var tx:Number;
			var ty:Number;
			var tz:Number;
			var td:Number;
			var dx:Number;
			var dy:Number;
			var dz:Number;
			var d1x:Number;
			var d1y:Number;
			var d1z:Number;
			var d2x:Number;
			var d2y:Number;
			var d2z:Number;
			var sx:Number;
			var sy:Number;
			var sz:Number;
			var sd:Number;
			var ex:Number;
			var ey:Number;
			var ez:Number;
			var ed:Number;
			var dot1:Number;
			var dot2:Number;
			var t1:Number;
			var t2:Number;
			
			var dir1x:Number;
			var dir1y:Number;
			var dir1z:Number;
			var dir2x:Number;
			var dir2y:Number;
			var dir2z:Number;
			var dir1l:Number;
			var dir2l:Number;
			if (state == 0) {
				result.addContactInfo(pos.x, pos.y, pos.z, nx, ny, nz, dep.x, b, c, 0, 0, false);
			} else if (state == 4) {
				if (right4) {
					ccx = pcx - dcx;
					ccy = pcy - dcy;
					ccz = pcz - dcz;
					nx = -ncx;
					ny = -ncy;
					nz = -ncz;
				} else {
					ccx = pcx + dcx;
					ccy = pcy + dcy;
					ccz = pcz + dcz;
					nx = ncx;
					ny = ncy;
					nz = ncz;
				}
				var v1x:Number;
				var v1y:Number;
				var v1z:Number;
				var v2x:Number;
				var v2y:Number;
				var v2z:Number;
				var v3x:Number;
				var v3y:Number;
				var v3z:Number;
				var v4x:Number;
				var v4y:Number;
				var v4z:Number;
				var v:Vec3;
				dot = 1;
				state = 0;
				dot1 = nwx * nx + nwy * ny + nwz * nz;
				if (dot1 < dot) {
					dot = dot1;
					state = 0;
				}
				if (-dot1 < dot) {
					dot = -dot1;
					state = 1;
				}
				dot1 = nhx * nx + nhy * ny + nhz * nz;
				if (dot1 < dot) {
					dot = dot1;
					state = 2;
				}
				if (-dot1 < dot) {
					dot = -dot1;
					state = 3;
				}
				dot1 = ndx * nx + ndy * ny + ndz * nz;
				if (dot1 < dot) {
					dot = dot1;
					state = 4;
				}
				if (-dot1 < dot) {
					dot = -dot1;
					state = 5;
				}
				switch(state) {
				case 0: // x+ face
					v = b.vertex1;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex3;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex4;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex2;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				case 1: // x- face
					v = b.vertex6;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex8;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex7;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex5;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				case 2: // y+ face
					v = b.vertex5;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex1;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex2;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex6;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				case 3: // y- face
					v = b.vertex8;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex4;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex3;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex7;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				case 4: // z+ face
					v = b.vertex5;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex7;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex3;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex1;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				case 5: // z- face
					v = b.vertex2;
					v1x = v.x;
					v1y = v.y;
					v1z = v.z;
					v = b.vertex4;
					v2x = v.x;
					v2y = v.y;
					v2z = v.z;
					v = b.vertex8;
					v3x = v.x;
					v3y = v.y;
					v3z = v.z;
					v = b.vertex6;
					v4x = v.x;
					v4y = v.y;
					v4z = v.z;
					break;
				}
				pd = nx * (v1x - ccx) + ny * (v1y - ccy) + nz * (v1z - ccz);
				if (pd <= 0) result.addContactInfo(v1x, v1y, v1z, -nx, -ny, -nz, pd, b, c, 5, 0, false);
				pd = nx * (v2x - ccx) + ny * (v2y - ccy) + nz * (v2z - ccz);
				if (pd <= 0) result.addContactInfo(v2x, v2y, v2z, -nx, -ny, -nz, pd, b, c, 6, 0, false);
				pd = nx * (v3x - ccx) + ny * (v3y - ccy) + nz * (v3z - ccz);
				if (pd <= 0) result.addContactInfo(v3x, v3y, v3z, -nx, -ny, -nz, pd, b, c, 7, 0, false);
				pd = nx * (v4x - ccx) + ny * (v4y - ccy) + nz * (v4z - ccz);
				if (pd <= 0) result.addContactInfo(v4x, v4y, v4z, -nx, -ny, -nz, pd, b, c, 8, 0, false);
			} else {
				switch(state) {
				case 1:
					if (right1) {
						cbx = pbx + dwx;
						cby = pby + dwy;
						cbz = pbz + dwz;
						nx = nwx;
						ny = nwy;
						nz = nwz;
					} else {
						cbx = pbx - dwx;
						cby = pby - dwy;
						cbz = pbz - dwz;
						nx = -nwx;
						ny = -nwy;
						nz = -nwz;
					}
					dir1x = nhx;
					dir1y = nhy;
					dir1z = nhz;
					dir1l = bh;
					dir2x = ndx;
					dir2y = ndy;
					dir2z = ndz;
					dir2l = bd;
					break;
				case 2:
					if (right2) {
						cbx = pbx + dhx;
						cby = pby + dhy;
						cbz = pbz + dhz;
						nx = nhx;
						ny = nhy;
						nz = nhz;
					} else {
						cbx = pbx - dhx;
						cby = pby - dhy;
						cbz = pbz - dhz;
						nx = -nhx;
						ny = -nhy;
						nz = -nhz;
					}
					dir1x = nwx;
					dir1y = nwy;
					dir1z = nwz;
					dir1l = bw;
					dir2x = ndx;
					dir2y = ndy;
					dir2z = ndz;
					dir2l = bd;
					break;
				case 3:
					if (right3) {
						cbx = pbx + ddx;
						cby = pby + ddy;
						cbz = pbz + ddz;
						nx = ndx;
						ny = ndy;
						nz = ndz;
					} else {
						cbx = pbx - ddx;
						cby = pby - ddy;
						cbz = pbz - ddz;
						nx = -ndx;
						ny = -ndy;
						nz = -ndz;
					}
					dir1x = nwx;
					dir1y = nwy;
					dir1z = nwz;
					dir1l = bw;
					dir2x = nhx;
					dir2y = nhy;
					dir2z = nhz;
					dir2l = bh;
					break;
				}
				dot = nx * ncx + ny * ncy + nz * ncz;
				if (dot < 0) len = ch;
				else len = -ch;
				ccx = pcx + len * ncx;
				ccy = pcy + len * ncy;
				ccz = pcz + len * ncz;
				if (dotc >= 0.999999) {
					tx = -ny;
					ty = nz;
					tz = nx;
				} else {
					tx = nx;
					ty = ny;
					tz = nz;
				}
				len = tx * ncx + ty * ncy + tz * ncz;
				dx = len * ncx - tx;
				dy = len * ncy - ty;
				dz = len * ncz - tz;
				len = Math.sqrt(dx * dx + dy * dy + dz * dz);
				if (len == 0) return;
				len = r / len;
				dx *= len;
				dy *= len;
				dz *= len;
				tx = ccx + dx;
				ty = ccy + dy;
				tz = ccz + dz;
				if (dot < -0.96 || dot > 0.96) {
					r00 = ncx * ncx * 1.5 - 0.5;
					r01 = ncx * ncy * 1.5 - ncz * 0.866025403;
					r02 = ncx * ncz * 1.5 + ncy * 0.866025403;
					r10 = ncy * ncx * 1.5 + ncz * 0.866025403;
					r11 = ncy * ncy * 1.5 - 0.5;
					r12 = ncy * ncz * 1.5 - ncx * 0.866025403;
					r20 = ncz * ncx * 1.5 - ncy * 0.866025403;
					r21 = ncz * ncy * 1.5 + ncx * 0.866025403;
					r22 = ncz * ncz * 1.5 - 0.5;
					px = tx;
					py = ty;
					pz = tz;
					pd = nx * (px - cbx) + ny * (py - cby) + nz * (pz - cbz);
					tx = px - pd * nx - cbx;
					ty = py - pd * ny - cby;
					tz = pz - pd * nz - cbz;
					sd = dir1x * tx + dir1y * ty + dir1z * tz;
					ed = dir2x * tx + dir2y * ty + dir2z * tz;
					if (sd < -dir1l) sd = -dir1l;
					else if (sd > dir1l) sd = dir1l;
					if (ed < -dir2l) ed = -dir2l;
					else if (ed > dir2l) ed = dir2l;
					tx = sd * dir1x + ed * dir2x;
					ty = sd * dir1y + ed * dir2y;
					tz = sd * dir1z + ed * dir2z;
					px = cbx + tx;
					py = cby + ty;
					pz = cbz + tz;
					result.addContactInfo(px, py, pz, nx, ny, nz, pd, b, c, 1, 0, false);
					px = dx * r00 + dy * r01 + dz * r02;
					py = dx * r10 + dy * r11 + dz * r12;
					pz = dx * r20 + dy * r21 + dz * r22;
					px = (dx = px) + ccx;
					py = (dy = py) + ccy;
					pz = (dz = pz) + ccz;
					pd = nx * (px - cbx) + ny * (py - cby) + nz * (pz - cbz);
					if (pd <= 0) {
						tx = px - pd * nx - cbx;
						ty = py - pd * ny - cby;
						tz = pz - pd * nz - cbz;
						sd = dir1x * tx + dir1y * ty + dir1z * tz;
						ed = dir2x * tx + dir2y * ty + dir2z * tz;
						if (sd < -dir1l) sd = -dir1l;
						else if (sd > dir1l) sd = dir1l;
						if (ed < -dir2l) ed = -dir2l;
						else if (ed > dir2l) ed = dir2l;
						tx = sd * dir1x + ed * dir2x;
						ty = sd * dir1y + ed * dir2y;
						tz = sd * dir1z + ed * dir2z;
						px = cbx + tx;
						py = cby + ty;
						pz = cbz + tz;
						result.addContactInfo(px, py, pz, nx, ny, nz, pd, b, c, 2, 0, false);
					}
					px = dx * r00 + dy * r01 + dz * r02;
					py = dx * r10 + dy * r11 + dz * r12;
					pz = dx * r20 + dy * r21 + dz * r22;
					px = (dx = px) + ccx;
					py = (dy = py) + ccy;
					pz = (dz = pz) + ccz;
					pd = nx * (px - cbx) + ny * (py - cby) + nz * (pz - cbz);
					if (pd <= 0) {
						tx = px - pd * nx - cbx;
						ty = py - pd * ny - cby;
						tz = pz - pd * nz - cbz;
						sd = dir1x * tx + dir1y * ty + dir1z * tz;
						ed = dir2x * tx + dir2y * ty + dir2z * tz;
						if (sd < -dir1l) sd = -dir1l;
						else if (sd > dir1l) sd = dir1l;
						if (ed < -dir2l) ed = -dir2l;
						else if (ed > dir2l) ed = dir2l;
						tx = sd * dir1x + ed * dir2x;
						ty = sd * dir1y + ed * dir2y;
						tz = sd * dir1z + ed * dir2z;
						px = cbx + tx;
						py = cby + ty;
						pz = cbz + tz;
						result.addContactInfo(px, py, pz, nx, ny, nz, pd, b, c, 3, 0, false);
					}
				} else {
					sx = tx;
					sy = ty;
					sz = tz;
					sd = nx * (sx - cbx) + ny * (sy - cby) + nz * (sz - cbz);
					sx -= sd * nx;
					sy -= sd * ny;
					sz -= sd * nz;
					if (dot > 0) {
						ex = tx + dcx * 2;
						ey = ty + dcy * 2;
						ez = tz + dcz * 2;
					} else {
						ex = tx - dcx * 2;
						ey = ty - dcy * 2;
						ez = tz - dcz * 2;
					}
					ed = nx * (ex - cbx) + ny * (ey - cby) + nz * (ez - cbz);
					ex -= ed * nx;
					ey -= ed * ny;
					ez -= ed * nz;
					d1x = sx - cbx;
					d1y = sy - cby;
					d1z = sz - cbz;
					d2x = ex - cbx;
					d2y = ey - cby;
					d2z = ez - cbz;
					tx = ex - sx;
					ty = ey - sy;
					tz = ez - sz;
					td = ed - sd;
					dotw = d1x * dir1x + d1y * dir1y + d1z * dir1z;
					doth = d2x * dir1x + d2y * dir1y + d2z * dir1z;
					dot1 = dotw - dir1l;
					dot2 = doth - dir1l;
					if (dot1 > 0) {
						if (dot2 > 0) return;
						t1 = dot1 / (dot1 - dot2);
						sx = sx + tx * t1;
						sy = sy + ty * t1;
						sz = sz + tz * t1;
						sd = sd + td * t1;
						d1x = sx - cbx;
						d1y = sy - cby;
						d1z = sz - cbz;
						dotw = d1x * dir1x + d1y * dir1y + d1z * dir1z;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					} else if (dot2 > 0) {
						t1 = dot1 / (dot1 - dot2);
						ex = sx + tx * t1;
						ey = sy + ty * t1;
						ez = sz + tz * t1;
						ed = sd + td * t1;
						d2x = ex - cbx;
						d2y = ey - cby;
						d2z = ez - cbz;
						doth = d2x * dir1x + d2y * dir1y + d2z * dir1z;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					}
					dot1 = dotw + dir1l;
					dot2 = doth + dir1l;
					if (dot1 < 0) {
						if (dot2 < 0) return;
						t1 = dot1 / (dot1 - dot2);
						sx = sx + tx * t1;
						sy = sy + ty * t1;
						sz = sz + tz * t1;
						sd = sd + td * t1;
						d1x = sx - cbx;
						d1y = sy - cby;
						d1z = sz - cbz;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					} else if (dot2 < 0) {
						t1 = dot1 / (dot1 - dot2);
						ex = sx + tx * t1;
						ey = sy + ty * t1;
						ez = sz + tz * t1;
						ed = sd + td * t1;
						d2x = ex - cbx;
						d2y = ey - cby;
						d2z = ez - cbz;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					}
					dotw = d1x * dir2x + d1y * dir2y + d1z * dir2z;
					doth = d2x * dir2x + d2y * dir2y + d2z * dir2z;
					dot1 = dotw - dir2l;
					dot2 = doth - dir2l;
					if (dot1 > 0) {
						if (dot2 > 0) return;
						t1 = dot1 / (dot1 - dot2);
						sx = sx + tx * t1;
						sy = sy + ty * t1;
						sz = sz + tz * t1;
						sd = sd + td * t1;
						d1x = sx - cbx;
						d1y = sy - cby;
						d1z = sz - cbz;
						dotw = d1x * dir2x + d1y * dir2y + d1z * dir2z;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					} else if (dot2 > 0) {
						t1 = dot1 / (dot1 - dot2);
						ex = sx + tx * t1;
						ey = sy + ty * t1;
						ez = sz + tz * t1;
						ed = sd + td * t1;
						d2x = ex - cbx;
						d2y = ey - cby;
						d2z = ez - cbz;
						doth = d2x * dir2x + d2y * dir2y + d2z * dir2z;
						tx = ex - sx;
						ty = ey - sy;
						tz = ez - sz;
						td = ed - sd;
					}
					dot1 = dotw + dir2l;
					dot2 = doth + dir2l;
					if (dot1 < 0) {
						if (dot2 < 0) return;
						t1 = dot1 / (dot1 - dot2);
						sx = sx + tx * t1;
						sy = sy + ty * t1;
						sz = sz + tz * t1;
						sd = sd + td * t1;
					} else if (dot2 < 0) {
						t1 = dot1 / (dot1 - dot2);
						ex = sx + tx * t1;
						ey = sy + ty * t1;
						ez = sz + tz * t1;
						ed = sd + td * t1;
					}
					if (sd < 0) {
						result.addContactInfo(sx, sy, sz, nx, ny, nz, sd, b, c, 1, 0, false);
					}
					if (ed < 0) {
						result.addContactInfo(ex, ey, ez, nx, ny, nz, ed, b, c, 4, 0, false);
					}
				}
			}
		}
		
	}

}