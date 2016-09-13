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
	 * 3行3列の要素を持つ行列を扱うクラスです。
	 * この行列は、ある三次元座標系から別の三次元座標系への、平行移動を除く変換をサポートします。
	 * 行列は右手系の行列として扱われます。
	 * オブジェクトの不要な作成を避けるため、
	 * 関数ではほとんどの演算結果は自身のオブジェクトに格納されます。
	 * @author saharan
	 */
	public class Mat33 {
		/**
		 * 1行1列目の要素です。
		 */
		public var e00:Number;
		/**
		 * 1行2列目の要素です。
		 */
		public var e01:Number;
		/**
		 * 1行3列目の要素です。
		 */
		public var e02:Number;
		/**
		 * 2行1列目の要素です。
		 */
		public var e10:Number;
		/**
		 * 2行2列目の要素です。
		 */
		public var e11:Number;
		/**
		 * 2行3列目の要素です。
		 */
		public var e12:Number;
		/**
		 * 3行1列目の要素です。
		 */
		public var e20:Number;
		/**
		 * 3行2列目の要素です。
		 */
		public var e21:Number;
		/**
		 * 3行3列目の要素です。
		 */
		public var e22:Number;
		
		/**
		 * 新しく Mat33 オブジェクトを作成します。
		 * 引数を指定しない場合は、単位行列で初期化されます。
		 * @param	e00 設定する1行1列目の要素
		 * @param	e01 設定する1行2列目の要素
		 * @param	e02 設定する1行3列目の要素
		 * @param	e10 設定する2行1列目の要素
		 * @param	e11 設定する2行2列目の要素
		 * @param	e12 設定する2行3列目の要素
		 * @param	e20 設定する3行1列目の要素
		 * @param	e21 設定する3行2列目の要素
		 * @param	e22 設定する3行3列目の要素
		 */
		public function Mat33(
			e00:Number = 1, e01:Number = 0, e02:Number = 0,
			e10:Number = 0, e11:Number = 1, e12:Number = 0,
			e20:Number = 0, e21:Number = 0, e22:Number = 1
		) {
			this.e00 = e00;
			this.e01 = e01;
			this.e02 = e02;
			this.e10 = e10;
			this.e11 = e11;
			this.e12 = e12;
			this.e20 = e20;
			this.e21 = e21;
			this.e22 = e22;
		}
		
		/**
		 * この行列を指定された値で初期化します。
		 * 引数を指定しない場合は、単位行列で初期化されます。
		 * @param	e00 設定する1行1列目の要素
		 * @param	e01 設定する1行2列目の要素
		 * @param	e02 設定する1行3列目の要素
		 * @param	e10 設定する2行1列目の要素
		 * @param	e11 設定する2行2列目の要素
		 * @param	e12 設定する2行3列目の要素
		 * @param	e20 設定する3行1列目の要素
		 * @param	e21 設定する3行2列目の要素
		 * @param	e22 設定する3行3列目の要素
		 * @return このオブジェクト
		 */
		public function init(
			e00:Number = 1, e01:Number = 0, e02:Number = 0,
			e10:Number = 0, e11:Number = 1, e12:Number = 0,
			e20:Number = 0, e21:Number = 0, e22:Number = 1
		):Mat33 {
			this.e00 = e00;
			this.e01 = e01;
			this.e02 = e02;
			this.e10 = e10;
			this.e11 = e11;
			this.e12 = e12;
			this.e20 = e20;
			this.e21 = e21;
			this.e22 = e22;
			return this;
		}
		
		/**
		 * この行列を m1 と m2 を加算した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
		public function add(m1:Mat33, m2:Mat33):Mat33 {
			e00 = m1.e00 + m2.e00;
			e01 = m1.e01 + m2.e01;
			e02 = m1.e02 + m2.e02;
			e10 = m1.e10 + m2.e10;
			e11 = m1.e11 + m2.e11;
			e12 = m1.e12 + m2.e12;
			e20 = m1.e20 + m2.e20;
			e21 = m1.e21 + m2.e21;
			e22 = m1.e22 + m2.e22;
			return this;
		}
		
		/**
		 * この行列を m1 から m2 を減算した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
		public function sub(m1:Mat33, m2:Mat33):Mat33 {
			e00 = m1.e00 - m2.e00;
			e01 = m1.e01 - m2.e01;
			e02 = m1.e02 - m2.e02;
			e10 = m1.e10 - m2.e10;
			e11 = m1.e11 - m2.e11;
			e12 = m1.e12 - m2.e12;
			e20 = m1.e20 - m2.e20;
			e21 = m1.e21 - m2.e21;
			e22 = m1.e22 - m2.e22;
			return this;
		}
		
		/**
		 * この行列を m を s 倍に拡張した行列に設定します。
		 * @param	m 行列
		 * @param	s スカラー
		 * @return このオブジェクト
		 */
		public function scale(m:Mat33, s:Number):Mat33 {
			e00 = m.e00 * s;
			e01 = m.e01 * s;
			e02 = m.e02 * s;
			e10 = m.e10 * s;
			e11 = m.e11 * s;
			e12 = m.e12 * s;
			e20 = m.e20 * s;
			e21 = m.e21 * s;
			e22 = m.e22 * s;
			return this;
		}
		
		/**
		 * この行列を m1 と m2 を合成した行列に設定します。
		 * @param	m1 行列1
		 * @param	m2 行列2
		 * @return このオブジェクト
		 */
		public function mul(m1:Mat33, m2:Mat33):Mat33 {
			var e00:Number = m1.e00 * m2.e00 + m1.e01 * m2.e10 + m1.e02 * m2.e20;
			var e01:Number = m1.e00 * m2.e01 + m1.e01 * m2.e11 + m1.e02 * m2.e21;
			var e02:Number = m1.e00 * m2.e02 + m1.e01 * m2.e12 + m1.e02 * m2.e22;
			var e10:Number = m1.e10 * m2.e00 + m1.e11 * m2.e10 + m1.e12 * m2.e20;
			var e11:Number = m1.e10 * m2.e01 + m1.e11 * m2.e11 + m1.e12 * m2.e21;
			var e12:Number = m1.e10 * m2.e02 + m1.e11 * m2.e12 + m1.e12 * m2.e22;
			var e20:Number = m1.e20 * m2.e00 + m1.e21 * m2.e10 + m1.e22 * m2.e20;
			var e21:Number = m1.e20 * m2.e01 + m1.e21 * m2.e11 + m1.e22 * m2.e21;
			var e22:Number = m1.e20 * m2.e02 + m1.e21 * m2.e12 + m1.e22 * m2.e22;
			this.e00 = e00;
			this.e01 = e01;
			this.e02 = e02;
			this.e10 = e10;
			this.e11 = e11;
			this.e12 = e12;
			this.e20 = e20;
			this.e21 = e21;
			this.e22 = e22;
			return this;
		}
		
		/**
		 * この行列を m と拡大縮小行列を合成したものに設定します。
		 * @param	m 行列
		 * @param	sx x 方向の拡大率
		 * @param	sy　y 方向の拡大率
		 * @param	sz　z 方向の拡大率
		 * @param	prepend 合成順序を逆にする場合は true
		 * @return このオブジェクト
		 */
		public function mulScale(m:Mat33, sx:Number, sy:Number, sz:Number, prepend:Boolean = false):Mat33 {
			var e00:Number;
			var e01:Number;
			var e02:Number;
			var e10:Number;
			var e11:Number;
			var e12:Number;
			var e20:Number;
			var e21:Number;
			var e22:Number;
			if (prepend) {
				e00 = sx * m.e00;
				e01 = sx * m.e01;
				e02 = sx * m.e02;
				e10 = sy * m.e10;
				e11 = sy * m.e11;
				e12 = sy * m.e12;
				e20 = sz * m.e20;
				e21 = sz * m.e21;
				e22 = sz * m.e22;
				this.e00 = e00;
				this.e01 = e01;
				this.e02 = e02;
				this.e10 = e10;
				this.e11 = e11;
				this.e12 = e12;
				this.e20 = e20;
				this.e21 = e21;
				this.e22 = e22;
			} else {
				e00 = m.e00 * sx;
				e01 = m.e01 * sy;
				e02 = m.e02 * sz;
				e10 = m.e10 * sx;
				e11 = m.e11 * sy;
				e12 = m.e12 * sz;
				e20 = m.e20 * sx;
				e21 = m.e21 * sy;
				e22 = m.e22 * sz;
				this.e00 = e00;
				this.e01 = e01;
				this.e02 = e02;
				this.e10 = e10;
				this.e11 = e11;
				this.e12 = e12;
				this.e20 = e20;
				this.e21 = e21;
				this.e22 = e22;
			}
			return this;
		}
		
		/**
		 * この行列を m と回転行列を合成したものに設定します。
		 * @param	m 行列
		 * @param	rad ラジアンでの回転角度
		 * @param	ax 回転軸の x 成分
		 * @param	ay 回転軸の y 成分
		 * @param	az 回転軸の z 成分
		 * @param	prepend 合成順序を逆にする場合は true
		 * @return このオブジェクト
		 */
		public function mulRotate(m:Mat33, rad:Number, ax:Number, ay:Number, az:Number, prepend:Boolean = false):Mat33 {
			var s:Number = Math.sin(rad);
			var c:Number = Math.cos(rad);
			var c1:Number = 1 - c;
			var r00:Number = ax * ax * c1 + c;
			var r01:Number = ax * ay * c1 - az * s;
			var r02:Number = ax * az * c1 + ay * s;
			var r10:Number = ay * ax * c1 + az * s;
			var r11:Number = ay * ay * c1 + c;
			var r12:Number = ay * az * c1 - ax * s;
			var r20:Number = az * ax * c1 - ay * s;
			var r21:Number = az * ay * c1 + ax * s;
			var r22:Number = az * az * c1 + c;
			var e00:Number;
			var e01:Number;
			var e02:Number;
			var e10:Number;
			var e11:Number;
			var e12:Number;
			var e20:Number;
			var e21:Number;
			var e22:Number;
			if (prepend) {
				e00 = r00 * m.e00 + r01 * m.e10 + r02 * m.e20;
				e01 = r00 * m.e01 + r01 * m.e11 + r02 * m.e21;
				e02 = r00 * m.e02 + r01 * m.e12 + r02 * m.e22;
				e10 = r10 * m.e00 + r11 * m.e10 + r12 * m.e20;
				e11 = r10 * m.e01 + r11 * m.e11 + r12 * m.e21;
				e12 = r10 * m.e02 + r11 * m.e12 + r12 * m.e22;
				e20 = r20 * m.e00 + r21 * m.e10 + r22 * m.e20;
				e21 = r20 * m.e01 + r21 * m.e11 + r22 * m.e21;
				e22 = r20 * m.e02 + r21 * m.e12 + r22 * m.e22;
				this.e00 = e00;
				this.e01 = e01;
				this.e02 = e02;
				this.e10 = e10;
				this.e11 = e11;
				this.e12 = e12;
				this.e20 = e20;
				this.e21 = e21;
				this.e22 = e22;
			} else {
				e00 = m.e00 * r00 + m.e01 * r10 + m.e02 * r20;
				e01 = m.e00 * r01 + m.e01 * r11 + m.e02 * r21;
				e02 = m.e00 * r02 + m.e01 * r12 + m.e02 * r22;
				e10 = m.e10 * r00 + m.e11 * r10 + m.e12 * r20;
				e11 = m.e10 * r01 + m.e11 * r11 + m.e12 * r21;
				e12 = m.e10 * r02 + m.e11 * r12 + m.e12 * r22;
				e20 = m.e20 * r00 + m.e21 * r10 + m.e22 * r20;
				e21 = m.e20 * r01 + m.e21 * r11 + m.e22 * r21;
				e22 = m.e20 * r02 + m.e21 * r12 + m.e22 * r22;
				this.e00 = e00;
				this.e01 = e01;
				this.e02 = e02;
				this.e10 = e10;
				this.e11 = e11;
				this.e12 = e12;
				this.e20 = e20;
				this.e21 = e21;
				this.e22 = e22;
			}
			return this;
		}
		
		/**
		 * この行列を m の転置行列に設定します。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
		public function transpose(m:Mat33):Mat33 {
			var e01:Number = m.e10;
			var e02:Number = m.e20;
			var e10:Number = m.e01;
			var e12:Number = m.e21;
			var e20:Number = m.e02;
			var e21:Number = m.e12;
			e00 = m.e00;
			this.e01 = e01;
			this.e02 = e02;
			this.e10 = e10;
			e11 = m.e11;
			this.e12 = e12;
			this.e20 = e20;
			this.e21 = e21;
			e22 = m.e22;
			return this;
		}
		
		/**
		 * この行列を q で表される回転行列に設定します。
		 * @param	q クォータニオン
		 * @return このオブジェクト
		 */
		public function setQuat(q:Quat):Mat33 {
			var x2:Number = 2 * q.x;
			var y2:Number = 2 * q.y;
			var z2:Number = 2 * q.z;
			var xx:Number = q.x * x2;
			var yy:Number = q.y * y2;
			var zz:Number = q.z * z2;
			var xy:Number = q.x * y2;
			var yz:Number = q.y * z2;
			var xz:Number = q.x * z2;
			var sx:Number = q.s * x2;
			var sy:Number = q.s * y2;
			var sz:Number = q.s * z2;
			e00 = 1 - yy - zz;
			e01 = xy - sz;
			e02 = xz + sy;
			e10 = xy + sz;
			e11 = 1 - xx - zz;
			e12 = yz - sx;
			e20 = xz - sy;
			e21 = yz + sx;
			e22 = 1 - xx - yy;
			return this;
		}
		
		/**
		 * この行列を m の逆行列に設定します。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
		public function invert(m:Mat33):Mat33 {
			var det:Number =
				m.e00 * (m.e11 * m.e22 - m.e21 * m.e12) +
				m.e10 * (m.e21 * m.e02 - m.e01 * m.e22) +
				m.e20 * (m.e01 * m.e12 - m.e11 * m.e02)
			;
			if (det != 0) det = 1 / det;
			var t00:Number = m.e11 * m.e22 - m.e12 * m.e21;
			var t01:Number = m.e02 * m.e21 - m.e01 * m.e22;
			var t02:Number = m.e01 * m.e12 - m.e02 * m.e11;
			var t10:Number = m.e12 * m.e20 - m.e10 * m.e22;
			var t11:Number = m.e00 * m.e22 - m.e02 * m.e20;
			var t12:Number = m.e02 * m.e10 - m.e00 * m.e12;
			var t20:Number = m.e10 * m.e21 - m.e11 * m.e20;
			var t21:Number = m.e01 * m.e20 - m.e00 * m.e21;
			var t22:Number = m.e00 * m.e11 - m.e01 * m.e10;
			e00 = det * t00;
			e01 = det * t01;
			e02 = det * t02;
			e10 = det * t10;
			e11 = det * t11; 
			e12 = det * t12;
			e20 = det * t20;
			e21 = det * t21;
			e22 = det * t22;
			return this;
		}
		
		/**
		 * この行列の値を m からコピーします。
		 * @param	m 行列
		 * @return このオブジェクト
		 */
		public function copy(m:Mat33):Mat33 {
			e00 = m.e00;
			e01 = m.e01;
			e02 = m.e02;
			e10 = m.e10;
			e11 = m.e11;
			e12 = m.e12;
			e20 = m.e20;
			e21 = m.e21;
			e22 = m.e22;
			return this;
		}
		
		/**
		 * この行列の値を4行4列の要素を持つ行列 m からコピーします。
		 * 4行目および4列目の要素はコピーされません。
		 * @param	m 4行4列の要素を持つ行列
		 * @return このオブジェクト
		 */
		public function copyMat44(m:Mat44):Mat33 {
			e00 = m.e00;
			e01 = m.e01;
			e02 = m.e02;
			e10 = m.e10;
			e11 = m.e11;
			e12 = m.e12;
			e20 = m.e20;
			e21 = m.e21;
			e22 = m.e22;
			return this;
		}
		
		/**
		 * この Mat33 オブジェクトを複製します。
		 * @return 複製された Mat33 オブジェクト
		 */
		public function clone():Mat33 {
			return new Mat33(e00, e01, e02, e10, e11, e12, e20, e21, e22);
		}
		
		/**
		 * この行列の文字列表現を返します。
		 * @return この行列を表す文字列
		 */
		public function toString():String {
			var text:String =
				"Mat33|" + e00.toFixed(4) + ", " + e01.toFixed(4) + ", " + e02.toFixed(4) + "|\n" +
				"     |" + e10.toFixed(4) + ", " + e11.toFixed(4) + ", " + e12.toFixed(4) + "|\n" +
				"     |" + e20.toFixed(4) + ", " + e21.toFixed(4) + ", " + e22.toFixed(4) + "|"
			;
			return text;
		}
		
	}

}