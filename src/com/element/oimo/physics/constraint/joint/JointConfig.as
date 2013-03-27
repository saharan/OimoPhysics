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
package com.element.oimo.physics.constraint.joint {
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Vec3;
	/**
	 * ジョイントの初期化時に使われる共通設定のクラスです。
	 * このクラスの変数はコピーして使われ、直接参照を持たれることはありません。
	 * @author saharan
	 */
	public class JointConfig {
		/**
		 * 剛体1に対する初期状態での相対接続位置です。
		 */
		public var localRelativeAnchorPosition1:Vec3;
		
		/**
		 * 剛体2に対する初期状態での相対接続位置です。
		 */
		public var localRelativeAnchorPosition2:Vec3;
		
		/**
		 * 剛体1に対する初期状態での回転軸です。
		 * このオプションは一部のジョイントにおいてのみ有効です。
		 */
		public var localAxis1:Vec3;
		
		/**
		 * 剛体2に対する初期状態での回転軸です。
		 * このオプションは一部のジョイントにおいてのみ有効です。
		 */
		public var localAxis2:Vec3;
		
		/**
		 * 接続された剛体同士が衝突するかどうかを表します。
		 */
		public var allowCollision:Boolean;
		
		/**
		 * 新しく JointConfig オブジェクトを作成します。
		 */
		public function JointConfig() {
			localRelativeAnchorPosition1 = new Vec3();
			localRelativeAnchorPosition2 = new Vec3();
			localAxis1 = new Vec3();
			localAxis2 = new Vec3();
		}
		
	}

}