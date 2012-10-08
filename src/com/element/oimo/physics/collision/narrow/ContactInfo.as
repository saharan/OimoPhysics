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
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * 剛体同士の接触情報を保持するクラスです。
	 * @author saharan
	 */
	public class ContactInfo {
		/**
		 * 接触地点のワールド座標です。
		 */
		public var position:Vec3;
		
		/**
		 * 接触面に対し垂直な法線ベクトルです。
		 */
		public var normal:Vec3;
		
		/**
		 * 形状間に発生した重なりの大きさです。
		 * 正確には形状間の距離を表すので、
		 * 重なりが発生した場合は、この変数は負の値を取ります。
		 */
		public var overlap:Number;
		
		/**
		 * 接触を起こした形状1です。
		 */
		public var shape1:Shape;
		
		/**
		 * 接触を起こした形状2です。
		 */
		public var shape2:Shape;
		
		/**
		 * 接触点の識別データです。
		 */
		public var id:ContactID;
		
		/**
		 * 新しく ContactInfo オブジェクトを作成します。
		 */
		public function ContactInfo() {
			position = new Vec3();
			normal = new Vec3();
			id = new ContactID();
		}
		
	}

}