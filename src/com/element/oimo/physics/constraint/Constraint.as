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
package com.element.oimo.physics.constraint {
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	/**
	 * 剛体の拘束を扱うクラスです。
	 * 剛体間の接触やジョイントは全て拘束として処理されます。
	 * @author saharan
	 */
	public class Constraint {
		/**
		 * この拘束の親となるワールドです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var parent:World;
		
		/**
		 * 拘束の対象となる剛体1です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var body1:RigidBody;
		
		/**
		 * 拘束の対象となる剛体2です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var body2:RigidBody;
		
		/**
		 * この拘束がシミュレーションアイランドに追加されたかどうかを示します。
		 * この変数は内部でのみ使用されます。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var addedToIsland:Boolean;
		
		/**
		 * この拘束がスリープ状態かどうかを示します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var sleeping:Boolean;
		
		/**
		 * 新しく Constraint オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function Constraint() {
		}
		
		/**
		 * 拘束処理の前に一度だけ呼び出すべきメソッドです。
		 * @param	timeStep 時間刻み幅
		 * @param	invTimeStep 時間刻み幅の逆数
		 */
		public function preSolve(timeStep:Number, invTimeStep:Number):void {
			throw new Error("preSolve メソッドが継承されていません");
		}
		
		/**
		 * この拘束を処理します。
		 * 通常このメソッドは繰り返し呼び出され、
		 * 拘束の精度は繰り返し回数に左右されます。
		 */
		public function solve():void {
			throw new Error("solve メソッドが継承されていません");
		}
		
		/**
		 * 拘束処理の後に一度だけ呼び出すべきメソッドです。
		 */
		public function postSolve():void {
			throw new Error("postSolve メソッドが継承されていません");
		}
		
	}

}