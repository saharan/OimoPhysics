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
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Vec3;
	/**
	 * 形状の初期化時に使われる共通設定のクラスです。
	 * このクラスの変数はコピーして使われ、直接参照を持たれることはありません。
	 * @author saharan
	 */
	public class ShapeConfig {
		/**
		 * 重心のワールド座標です。
		 */
		public var position:Vec3;
		
		/**
		 * 回転行列です。
		 * この値を変更すると、剛体ではなく剛体に含まれる形状の
		 * 相対的な角度が変更されることに注意してください。
		 */
		public var rotation:Mat33;
		
		/**
		 * 摩擦係数です。
		 */
		public var friction:Number;
		
		/**
		 * 反発係数です。
		 */
		public var restitution:Number;
		
		/**
		 * 密度です。
		 */
		public var density:Number;
		
		/**
		 * 新しく ShapeConfig オブジェクトを作成します。
		 */
		public function ShapeConfig() {
			position = new Vec3();
			rotation = new Mat33();
			friction = 0.5;
			restitution = 0.5;
			density = 1;
		}
		
	}

}