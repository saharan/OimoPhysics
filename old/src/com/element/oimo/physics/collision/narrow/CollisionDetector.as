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
	import com.element.oimo.physics.collision.shape.Shape;
	/**
	 * より詳細な形状の衝突判定を行うクラスです。
	 * @author saharan
	 */
	public class CollisionDetector {
		/**
		 * detectCollision 関数の引数に指定する形状の順序を、
		 * 反転して受け取るかどうかを表します。
		 */
		public var flip:Boolean;
		
		/**
		 * 新しく CollisionDetector オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function CollisionDetector() {
		}
		
		/**
		 * 二つの形状の詳細な衝突判定を行います。
		 * 形状の種類は指定された物である必要があります。
		 * @param	shape1 形状1
		 * @param	shape2 形状2
		 * @param	result 衝突結果の格納先
		 */
		public function detectCollision(shape1:Shape, shape2:Shape, result:CollisionResult):void {
			throw new Error("detectCollision 関数が継承されていません");
			return -1;
		}
		
	}

}