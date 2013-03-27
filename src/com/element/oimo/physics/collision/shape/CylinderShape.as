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
package com.element.oimo.physics.collision.shape {
	import com.element.oimo.math.Vec3;
	/**
	 * 円柱を表す形状です。
	 * @author saharan
	 */
	public class CylinderShape extends Shape {
		/**
		 * 円柱の半径です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var radius:Number;
		
		/**
		 * 円柱の高さです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var height:Number;
		
		/**
		 * 円柱の高さの半分です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfHeight:Number;
		
		/**
		 * ワールド座標系での円柱の向きを示す単位ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var normalDirection:Vec3;
		
		/**
		 * ワールド座標系での円柱の向きを示す高さの半分の長さのベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfDirection:Vec3;
		
		/**
		 * 新しく CylinderShape オブジェクトを作成します。
		 * @param	radius 円柱の半径
		 * @param	height 円柱の高さ
		 * @param	config 形状の設定
		 */
		public function CylinderShape(radius:Number, height:Number, config:ShapeConfig) {
			this.radius = radius;
			this.height = height;
			halfHeight = height * 0.5;
			position.copy(config.position);
			rotation.copy(config.rotation);
			friction = config.friction;
			restitution = config.restitution;
			mass = Math.PI * radius * radius * height * config.density;
			var inertiaXZ:Number = (1 / 4 * radius * radius + 1 / 12 * height * height) * mass;
			var inertiaY:Number = 1 / 2 * radius * radius;
			localInertia.init(
				inertiaXZ, 0, 0,
				0, inertiaY, 0,
				0, 0, inertiaXZ
			);
			normalDirection = new Vec3();
			halfDirection = new Vec3();
			type = SHAPE_CYLINDER;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function updateProxy():void {
			var len:Number;
			var wx:Number;
			var hy:Number;
			var dz:Number;
			var dirX:Number = rotation.e01;
			var dirY:Number = rotation.e11;
			var dirZ:Number = rotation.e21;
			var xx:Number = dirX * dirX;
			var yy:Number = dirY * dirY;
			var zz:Number = dirZ * dirZ;
			normalDirection.x = dirX;
			normalDirection.y = dirY;
			normalDirection.z = dirZ;
			halfDirection.x = dirX * halfHeight;
			halfDirection.y = dirY * halfHeight;
			halfDirection.z = dirZ * halfHeight;
			wx = 1 - dirX * dirX;
			len = Math.sqrt(wx * wx + xx * yy + xx * zz);
			if (len > 0) len = radius / len;
			wx *= len;
			hy = 1 - dirY * dirY;
			len = Math.sqrt(yy * xx + hy * hy + yy * zz);
			if (len > 0) len = radius / len;
			hy *= len;
			dz = 1 - dirZ * dirZ;
			len = Math.sqrt(zz * xx + zz * yy + dz * dz);
			if (len > 0) len = radius / len;
			dz *= len;
			var w:Number;
			var h:Number;
			var d:Number;
			if (halfDirection.x < 0) w = -halfDirection.x;
			else w = halfDirection.x;
			if (halfDirection.y < 0) h = -halfDirection.y;
			else h = halfDirection.y;
			if (halfDirection.z < 0) d = -halfDirection.z;
			else d = halfDirection.z;
			if (wx < 0) w -= wx;
			else w += wx;
			if (hy < 0) h -= hy;
			else h += hy;
			if (dz < 0) d -= dz;
			else d += dz;
			proxy.init(
				position.x - w, position.x + w,
				position.y - h, position.y + h,
				position.z - d, position.z + d
			);
		}
		
	}

}