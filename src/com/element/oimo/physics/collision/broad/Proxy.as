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
	import com.element.oimo.physics.collision.shape.Shape;
	/**
	 * 広域衝突判定のために、詳細な形状の代わりに使用される
	 * 軸並行境界ボックスのクラスです。
	 * 詳細な形状の衝突判定の回数を削減するため、
	 * 詳細な衝突判定の前に、形状はより単純な形に近似され、
	 * 衝突の可能性がない形状の判定を取り除きます。
	 * @author saharan
	 */
	public class Proxy {
		/**
		 * x 軸の最小値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var minX:Number;
		
		/**
		 * x 軸の最大値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var maxX:Number;
		
		/**
		 * y 軸の最小値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var minY:Number;
		
		/**
		 * y 軸の最大値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var maxY:Number;
		
		/**
		 * z 軸の最小値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var minZ:Number;
		
		/**
		 * z 軸の最大値です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var maxZ:Number;
		
		/**
		 * このプロキシの親となる形状です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var parent:Shape;
		
		/**
		 * 新しく Proxy オブジェクトを作成します。
		 * 引数を指定しない場合は、全ての値が0に初期化されます。
		 * @param	minX x 軸の最小値
		 * @param	maxX x 軸の最大値
		 * @param	minY y 軸の最小値
		 * @param	maxY y 軸の最大値
		 * @param	minZ z 軸の最小値
		 * @param	maxZ z 軸の最大値
		 */
		public function Proxy(
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
		 * このプロキシを指定された値で初期化します。
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
		 * このプロキシが引数のプロキシと交差するかどうか判定します。
		 * @param	proxy プロキシ
		 * @return 交差する場合は true
		 */
		public function intersect(proxy:Proxy):Boolean {
			return maxX > proxy.minX && minX < proxy.maxX && maxY > proxy.minY && minY < proxy.maxY && maxZ > proxy.minZ && minZ < proxy.maxZ;
		}
		
	}

}