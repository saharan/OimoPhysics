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
package com.element.oimo.physics.collision.broad {
	/**
	 * 軸並行境界ボックスのクラスです。
	 * 広域衝突判定に用いられます。
	 * @author saharan
	 */
	public class AABB {
		/**
		 * x 軸の最小値です。
		 */
		public var minX:Number;
		
		/**
		 * x 軸の最大値です。
		 */
		public var maxX:Number;
		
		/**
		 * y 軸の最小値です。
		 */
		public var minY:Number;
		
		/**
		 * y 軸の最大値です。
		 */
		public var maxY:Number;
		
		/**
		 * z 軸の最小値です。
		 */
		public var minZ:Number;
		
		/**
		 * z 軸の最大値です。
		 */
		public var maxZ:Number;
		
		/**
		 * 新しく AABB オブジェクトを作成します。
		 * 引数を指定しない場合は、全ての値が0に初期化されます。
		 * @param	minX x 軸の最小値
		 * @param	maxX x 軸の最大値
		 * @param	minY y 軸の最小値
		 * @param	maxY y 軸の最大値
		 * @param	minZ z 軸の最小値
		 * @param	maxZ z 軸の最大値
		 */
		public function AABB(
			minX:Number = 0, maxX:Number = 0,
			minY:Number = 0, maxY:Number = 0,
			minZ:Number = 0, maxZ:Number = 0
		) {
			this.minX = minX;
			this.maxX = maxX;
			this.minY = minY;
			this.maxY = maxY;
			this.minZ = minZ;
			this.maxZ = maxZ;
		}
		
		/**
		 * この AABB を指定された値で初期化します。
		 * 引数を指定しない場合は、全ての値が0に初期化されます。
		 * @param	minX x 軸の最小値
		 * @param	maxX x 軸の最大値
		 * @param	minY y 軸の最小値
		 * @param	maxY y 軸の最大値
		 * @param	minZ z 軸の最小値
		 * @param	maxZ z 軸の最大値
		 */
		public function init(
			minX:Number = 0, maxX:Number = 0,
			minY:Number = 0, maxY:Number = 0,
			minZ:Number = 0, maxZ:Number = 0
		):void {
			this.minX = minX;
			this.maxX = maxX;
			this.minY = minY;
			this.maxY = maxY;
			this.minZ = minZ;
			this.maxZ = maxZ;
		}
		
		/**
		 * この AABB が引数の AABB と交差するかどうか判定します。
		 * @param	aabb AABB
		 * @return 交差する場合は true
		 */
		public function intersect(aabb:AABB):Boolean {
			return maxX > aabb.minX && minX < aabb.maxX && maxY > aabb.minY && minY < aabb.maxY && maxZ > aabb.minZ && minZ < aabb.maxZ;
		}
		
	}

}