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
	/**
	 * 球体を表す形状です。
	 * @author saharan
	 */
	public class SphereShape extends Shape {
		/**
		 * 球体の半径です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var radius:Number;
		
		/**
		 * 新しく SphereShape オブジェクトを作成します。
		 * @param	radius 球体の半径
		 * @param	config 形状の設定
		 */
		public function SphereShape(radius:Number, config:ShapeConfig) {
			this.radius = radius;
			position.copy(config.position);
			rotation.copy(config.rotation);
			friction = config.friction;
			restitution = config.restitution;
			mass = 4 / 3 * Math.PI * radius * radius * radius * config.density;
			var inertia:Number = 2 / 5 * radius * radius * mass;
			localInertia.init(
				inertia, 0, 0,
				0, inertia, 0,
				0, 0, inertia
			);
			type = SHAPE_SPHERE;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function updateProxy():void {
			proxy.init(
				position.x - radius, position.x + radius,
				position.y - radius, position.y + radius,
				position.z - radius, position.z + radius
			);
		}
		
	}

}