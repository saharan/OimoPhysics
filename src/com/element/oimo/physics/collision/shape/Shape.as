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
	import com.element.oimo.physics.collision.broad.Proxy;
	import com.element.oimo.physics.constraint.contact.Contact;
	import com.element.oimo.physics.constraint.contact.ContactConnection;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Vec3;
	/**
	 * 剛体に含まれる衝突処理用の形状のクラスです。
	 * @author saharan
	 */
	public class Shape {
		/**
		 * 次に生成される形状の ID の作成に使われます。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public static var nextID:uint = 0;
		
		/**
		 * 定義されていないことを表す形状の種類です。
		 */
		public static const SHAPE_NULL:uint = 0x0;
		
		/**
		 * 球体を表す形状の種類です。
		 */
		public static const SHAPE_SPHERE:uint = 0x1;
		
		/**
		 * 箱を表す形状の種類です。
		 */
		public static const SHAPE_BOX:uint = 0x2;
		
		/**
		 * 円柱を表す形状の種類です。
		 */
		public static const SHAPE_CYLINDER:uint = 0x3;
		
		/**
		 * この形状の固有数値です。
		 * 通常この値は他の形状とかぶることはありません。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var id:uint;
		
		/**
		 * 形状の種類を表します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var type:uint;
		
		/**
		 * 重心のワールド座標です。
		 */
		public var position:Vec3;
		
		/**
		 * 剛体に対する相対位置座標です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativePosition:Vec3;
		
		/**
		 * 剛体に対する初期状態での相対位置座標です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var localRelativePosition:Vec3;
		
		/**
		 * 回転行列です。
		 */
		public var rotation:Mat33;
		
		/**
		 * 剛体に対する相対回転行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativeRotation:Mat33;
		
		/**
		 * 質量です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 質量を変更する場合は setDensity メソッドを使用してください。
		 */
		public var mass:Number;
		
		/**
		 * 初期状態での慣性テンソルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var localInertia:Mat33;
		
		/**
		 * 広域衝突判定に用いられる単純化された形状です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var proxy:Proxy;
		
		/**
		 * 摩擦係数です。
		 */
		public var friction:Number;
		
		/**
		 * 反発係数です。
		 */
		public var restitution:Number;
		
		/**
		 * 形状の親となる剛体です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var parent:RigidBody;
		
		/**
		 * 形状に関与する接触点のリンク配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var contactList:ContactConnection;
		
		/**
		 * 形状の接触点の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numContacts:uint;
		
		/**
		 * 新しい Shape オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function Shape() {
			id = ++nextID;
			position = new Vec3();
			relativePosition = new Vec3();
			localRelativePosition = new Vec3();
			rotation = new Mat33();
			relativeRotation = new Mat33();
			localInertia = new Mat33();
			proxy = new Proxy();
			proxy.parent = this;
		}
		
		/**
		 * この形状のプロキシを更新します。
		 */
		public function updateProxy():void {
			throw new Error("updateProxy メソッドが継承されていません");
		}
		
	}

}