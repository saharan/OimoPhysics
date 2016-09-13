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
package com.element.oimo.math {
	/**
	 * 3次元のベクトルを扱うクラスです。
	 * ベクトルは列ベクトルとして扱われます。
	 * オブジェクトの不要な作成を避けるため、
	 * 関数ではほとんどの演算結果は自身のオブジェクトに格納されます。
	 * @author saharan
	 */
	public class Vec3 {
		/**
		 * x 成分です。
		 */
		public var x:Number;
		/**
		 * y 成分です。
		 */
		public var y:Number;
		/**
		 * z 成分です。
		 */
		public var z:Number;
		
		/**
		 * 新しく Vec3 オブジェクトを作成します。
		 * 引数を指定しない場合は、全ての値に0が格納されます。
		 * @param	x 設定する x 成分
		 * @param	y 設定する y 成分
		 * @param	z 設定する z 成分
		 */
		public function Vec3(x:Number = 0, y:Number = 0, z:Number = 0) {
			this.x = x;
			this.y = y;
			this.z = z;
		}
		
		/**
		 * このベクトルの値を指定された値で初期化します。
		 * 引数を指定しない場合は、全ての値に0が格納されます。
		 * @param	x 設定する x 成分
		 * @param	y 設定する y 成分
		 * @param	z 設定する z 成分
		 * @return このオブジェクト
		 */
		public function init(x:Number = 0, y:Number = 0, z:Number = 0):Vec3 {
			this.x = x;
			this.y = y;
			this.z = z;
			return this;
		}
		
		/**
		 * このベクトルを v1 と v2 を加算したベクトルに設定します。
		 * @param	v1 ベクトル1
		 * @param	v2 ベクトル2
		 * @return このオブジェクト
		 */
		public function add(v1:Vec3, v2:Vec3):Vec3 {
			x = v1.x + v2.x;
			y = v1.y + v2.y;
			z = v1.z + v2.z;
			return this;
		}
		
		/**
		 * このベクトルを v1 から v2 を減算したベクトルに設定します。
		 * @param	v1 ベクトル1
		 * @param	v2 ベクトル2
		 * @return このオブジェクト
		 */
		public function sub(v1:Vec3, v2:Vec3):Vec3 {
			x = v1.x - v2.x;
			y = v1.y - v2.y;
			z = v1.z - v2.z;
			return this;
		}
		
		/**
		 * このベクトルを v を s 倍に拡張したベクトルに設定します。
		 * @param	v ベクトル
		 * @param	s スカラー
		 * @return このオブジェクト
		 */
		public function scale(v:Vec3, s:Number):Vec3 {
			x = v.x * s;
			y = v.y * s;
			z = v.z * s;
			return this;
		}
		
		/**
		 * このベクトルと v との内積を返します。
		 * @param	v ベクトル
		 * @return 内積
		 */
		public function dot(v:Vec3):Number {
			return x * v.x + y * v.y + z * v.z;
		}
		
		/**
		 * このベクトルを v1 と v2 の外積のベクトルに設定します。
		 * @param	v1 ベクトル1
		 * @param	v2 ベクトル2
		 * @return このオブジェクト
		 */
		public function cross(v1:Vec3, v2:Vec3):Vec3 {
			var x:Number = v1.y * v2.z - v1.z * v2.y;
			var y:Number = v1.z * v2.x - v1.x * v2.z;
			var z:Number = v1.x * v2.y - v1.y * v2.x;
			this.x = x;
			this.y = y;
			this.z = z;
			return this;
		}
		
		/**
		 * このベクトルを m で v を変換したベクトルに設定します。
		 * @param	m 行列
		 * @param	v ベクトル
		 * @return このオブジェクト
		 */
		public function mulMat(m:Mat33, v:Vec3):Vec3 {
			var x:Number = m.e00 * v.x + m.e01 * v.y + m.e02 * v.z;
			var y:Number = m.e10 * v.x + m.e11 * v.y + m.e12 * v.z;
			var z:Number = m.e20 * v.x + m.e21 * v.y + m.e22 * v.z;
			this.x = x;
			this.y = y;
			this.z = z;
			return this;
		}
		
		/**
		 * このベクトルを v を正規化したベクトルに設定します。
		 * @param	v ベクトル
		 * @return このオブジェクト
		 */
		public function normalize(v:Vec3):Vec3 {
			var length:Number = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
			if (length > 0) length = 1 / length;
			x = v.x * length;
			y = v.y * length;
			z = v.z * length;
			return this;
		}
		
		/**
		 * このベクトルを v を反転したベクトルに設定します。
		 * @param	v ベクトル
		 * @return このオブジェクト
		 */
		public function invert(v:Vec3):Vec3 {
			x = -v.x;
			y = -v.y;
			z = -v.z;
			return this;
		}
		
		/**
		 * このベクトルの大きさを返します。
		 * @return 大きさ
		 */
		public function length():Number {
			return Math.sqrt(x * x + y * y + z * z);
		}
		
		/**
		 * このベクトルの値を v からコピーします。
		 * @param	v ベクトル
		 * @return このオブジェクト
		 */
		public function copy(v:Vec3):Vec3 {
			x = v.x;
			y = v.y;
			z = v.z;
			return this;
		}
		
		/**
		 * この Vec3 オブジェクトを複製します。
		 * @return 複製された Vec3 オブジェクト
		 */
		public function clone():Vec3 {
			return new Vec3(x, y, z);
		}
		
		/**
		 * このベクトルの文字列表現を返します。
		 * @return このベクトルを表す文字列
		 */
		public function toString():String {
			return "Vec3[" + x.toFixed(4) + ", " + y.toFixed(4) + ", " + z.toFixed(4) + "]";
		}
		
	}

}