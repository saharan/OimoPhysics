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
	 * 箱を表す形状です。
	 * @author saharan
	 */
	public class BoxShape extends Shape {
		/**
		 * 箱の幅です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var width:Number;
		
		/**
		 * 箱の幅の半分です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfWidth:Number;
		
		/**
		 * 箱の高さです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var height:Number;
		
		/**
		 * 箱の高さの半分です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfHeight:Number;
		
		/**
		 * 箱の奥行きです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var depth:Number;
		
		/**
		 * 箱の奥行きの半分です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfDepth:Number;
		
		/**
		 * ワールド座標系での幅の向きを示す単位ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var normalDirectionWidth:Vec3;
		
		/**
		 * ワールド座標系での高さの向きを示す単位ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var normalDirectionHeight:Vec3;
		
		/**
		 * ワールド座標系での奥行きの向きを示す単位ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var normalDirectionDepth:Vec3;
		
		/**
		 * ワールド座標系での幅の向きを示す幅の半分の長さのベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfDirectionWidth:Vec3;
		
		/**
		 * ワールド座標系での高さの向きを示す高さの半分の長さのベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfDirectionHeight:Vec3;
		
		/**
		 * ワールド座標系での奥行きの向きを示す奥行きの半分の長さのベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var halfDirectionDepth:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex1:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex2:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex3:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex4:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex5:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex6:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex7:Vec3;
		
		/**
		 * ワールド座標系での頂点の一つの位置ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var vertex8:Vec3;
		
		/**
		 * 新しく BoxShape オブジェクトを作成します。
		 * @param	width 箱の幅
		 * @param	height 箱の高さ
		 * @param	depth 箱の奥行き
		 * @param	config 形状の設定
		 */
		public function BoxShape(width:Number, height:Number, depth:Number, config:ShapeConfig) {
			this.width = width;
			halfWidth = width * 0.5;
			this.height = height;
			halfHeight = height * 0.5;
			this.depth = depth;
			halfDepth = depth * 0.5;
			position.copy(config.position);
			rotation.copy(config.rotation);
			friction = config.friction;
			restitution = config.restitution;
			mass = width * height * depth * config.density;
			var inertia:Number = mass / 12;
			localInertia.init(
				inertia * (height * height + depth * depth), 0, 0,
				0, inertia * (width * width + depth * depth), 0,
				0, 0, inertia * (width * width + height * height)
			);
			normalDirectionWidth = new Vec3();
			normalDirectionHeight = new Vec3();
			normalDirectionDepth = new Vec3();
			halfDirectionWidth = new Vec3();
			halfDirectionHeight = new Vec3();
			halfDirectionDepth = new Vec3();
			vertex1 = new Vec3();
			vertex2 = new Vec3();
			vertex3 = new Vec3();
			vertex4 = new Vec3();
			vertex5 = new Vec3();
			vertex6 = new Vec3();
			vertex7 = new Vec3();
			vertex8 = new Vec3();
			type = SHAPE_BOX;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function updateProxy():void {
			normalDirectionWidth.x = rotation.e00;
			normalDirectionWidth.y = rotation.e10;
			normalDirectionWidth.z = rotation.e20;
			normalDirectionHeight.x = rotation.e01;
			normalDirectionHeight.y = rotation.e11;
			normalDirectionHeight.z = rotation.e21;
			normalDirectionDepth.x = rotation.e02;
			normalDirectionDepth.y = rotation.e12;
			normalDirectionDepth.z = rotation.e22;
			halfDirectionWidth.x = rotation.e00 * halfWidth;
			halfDirectionWidth.y = rotation.e10 * halfWidth;
			halfDirectionWidth.z = rotation.e20 * halfWidth;
			halfDirectionHeight.x = rotation.e01 * halfHeight;
			halfDirectionHeight.y = rotation.e11 * halfHeight;
			halfDirectionHeight.z = rotation.e21 * halfHeight;
			halfDirectionDepth.x = rotation.e02 * halfDepth;
			halfDirectionDepth.y = rotation.e12 * halfDepth;
			halfDirectionDepth.z = rotation.e22 * halfDepth;
			var wx:Number = halfDirectionWidth.x;
			var wy:Number = halfDirectionWidth.y;
			var wz:Number = halfDirectionWidth.z;
			var hx:Number = halfDirectionHeight.x;
			var hy:Number = halfDirectionHeight.y;
			var hz:Number = halfDirectionHeight.z;
			var dx:Number = halfDirectionDepth.x;
			var dy:Number = halfDirectionDepth.y;
			var dz:Number = halfDirectionDepth.z;
			var x:Number = position.x;
			var y:Number = position.y;
			var z:Number = position.z;
			vertex1.x = x + wx + hx + dx;
			vertex1.y = y + wy + hy + dy;
			vertex1.z = z + wz + hz + dz;
			vertex2.x = x + wx + hx - dx;
			vertex2.y = y + wy + hy - dy;
			vertex2.z = z + wz + hz - dz;
			vertex3.x = x + wx - hx + dx;
			vertex3.y = y + wy - hy + dy;
			vertex3.z = z + wz - hz + dz;
			vertex4.x = x + wx - hx - dx;
			vertex4.y = y + wy - hy - dy;
			vertex4.z = z + wz - hz - dz;
			vertex5.x = x - wx + hx + dx;
			vertex5.y = y - wy + hy + dy;
			vertex5.z = z - wz + hz + dz;
			vertex6.x = x - wx + hx - dx;
			vertex6.y = y - wy + hy - dy;
			vertex6.z = z - wz + hz - dz;
			vertex7.x = x - wx - hx + dx;
			vertex7.y = y - wy - hy + dy;
			vertex7.z = z - wz - hz + dz;
			vertex8.x = x - wx - hx - dx;
			vertex8.y = y - wy - hy - dy;
			vertex8.z = z - wz - hz - dz;
			var w:Number;
			var h:Number;
			var d:Number;
			if (halfDirectionWidth.x < 0) {
				w = -halfDirectionWidth.x;
			} else {
				w = halfDirectionWidth.x;
			}
			if (halfDirectionWidth.y < 0) {
				h = -halfDirectionWidth.y;
			} else {
				h = halfDirectionWidth.y;
			}
			if (halfDirectionWidth.z < 0) {
				d = -halfDirectionWidth.z;
			} else {
				d = halfDirectionWidth.z;
			}
			if (halfDirectionHeight.x < 0) {
				w -= halfDirectionHeight.x;
			} else {
				w += halfDirectionHeight.x;
			}
			if (halfDirectionHeight.y < 0) {
				h -= halfDirectionHeight.y;
			} else {
				h += halfDirectionHeight.y;
			}
			if (halfDirectionHeight.z < 0) {
				d -= halfDirectionHeight.z;
			} else {
				d += halfDirectionHeight.z;
			}
			if (halfDirectionDepth.x < 0) {
				w -= halfDirectionDepth.x;
			} else {
				w += halfDirectionDepth.x;
			}
			if (halfDirectionDepth.y < 0) {
				h -= halfDirectionDepth.y;
			} else {
				h += halfDirectionDepth.y;
			}
			if (halfDirectionDepth.z < 0) {
				d -= halfDirectionDepth.z;
			} else {
				d += halfDirectionDepth.z;
			}
			proxy.init(
				position.x - w, position.x + w,
				position.y - h, position.y + h,
				position.z - d, position.z + d
			);
		}
		
	}

}