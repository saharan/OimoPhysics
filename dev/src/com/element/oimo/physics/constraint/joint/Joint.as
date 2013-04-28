/* Copyright (c) 2012-2013 EL-EMENT saharan
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
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.constraint.Constraint;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	/**
	 * 剛体同士を繋ぐジョイントのクラスです。
	 * @author saharan
	 */
	public class Joint extends Constraint {
		/**
		 * 定義されていないことを表すジョイントの種類です。
		 */
		public static const JOINT_NULL:uint = 0x0;
		
		/**
		 * 距離ジョイント表すジョイントの種類です。
		 */
		public static const JOINT_DISTANCE:uint = 0x1;
		
		/**
		 * ボールジョイント表すジョイントの種類です。
		 */
		public static const JOINT_BALL:uint = 0x2;
		
		/**
		 * ヒンジジョイント表すジョイントの種類です。
		 */
		public static const JOINT_HINGE:uint = 0x3;
		
		/**
		 * ヒンジ2ジョイント表すジョイントの種類です。
		 */
		public static const JOINT_HINGE2:uint = 0x4;
		
		private var b1Link:JointLink;
		private var b2Link:JointLink;
		
		/**
		 * ジョイントの種類を表します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var type:uint;
		
		/**
		 * 接続された剛体同士が衝突するかどうかを表します。
		 */
		public var allowCollision:Boolean;
		
		/**
		 * 剛体1に対する初期状態での相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var localRelativeAnchorPosition1:Vec3;
		
		/**
		 * 剛体2に対する初期状態での相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var localRelativeAnchorPosition2:Vec3;
		
		/**
		 * 剛体1に対する相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativeAnchorPosition1:Vec3;
		
		/**
		 * 剛体2に対する相対接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativeAnchorPosition2:Vec3;
		
		/**
		 * 剛体1への接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var anchorPosition1:Vec3;
		
		/**
		 * 剛体2への接続位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var anchorPosition2:Vec3;
		
		/**
		 * The previous joint in the world.
		 */
		public var prev:Joint;
		
		/**
		 * The next joint in the world.
		 */
		public var next:Joint;
		
		/**
		 * 新しい Joint オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function Joint() {
			localRelativeAnchorPosition1 = new Vec3();
			localRelativeAnchorPosition2 = new Vec3();
			relativeAnchorPosition1 = new Vec3();
			relativeAnchorPosition2 = new Vec3();
			anchorPosition1 = new Vec3();
			anchorPosition2 = new Vec3();
			b1Link = new JointLink(this);
			b2Link = new JointLink(this);
		}
		
		public function attach():void {
			b1Link.body = body2;
			b2Link.body = body1;
			
			if (body1.jointLink != null) (b1Link.next = body1.jointLink).prev = b1Link;
			else b1Link.next = null;
			body1.jointLink = b1Link;
			body1.numJoints++;
			
			if (body2.jointLink != null) (b2Link.next = body2.jointLink).prev = b2Link;
			else b2Link.next = null;
			body2.jointLink = b2Link;
			body2.numJoints++;
		}
		
		public function detach():void {
			var prev:JointLink = b1Link.prev;
			var next:JointLink = b1Link.next;
			if (prev != null) prev.next = next;
			if (next != null) next.prev = prev;
			if (body1.jointLink == b1Link) body1.jointLink = next;
			b1Link.prev = null;
			b1Link.next = null;
			b1Link.body = null;
			body1.numJoints--;
			
			prev = b2Link.prev;
			next = b2Link.next;
			if (prev != null) prev.next = next;
			if (next != null) next.prev = prev;
			if (body2.jointLink == b2Link) body2.jointLink = next;
			b2Link.prev = null;
			b2Link.next = null;
			b2Link.body = null;
			body2.numJoints--;
			
			b1Link.body = null;
			b2Link.body = null;
		}
		
		/**
		 * 接続されている剛体をスリープ状態から開放します。
		 */
		public function awake():void {
			body1.awake();
			body2.awake();
		}
		
		/**
		 * @inheritDoc
		 */
		override public function preSolve(timeStep:Number, invTimeStep:Number):void {
			super.preSolve(timeStep, invTimeStep);
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			super.solve();
		}
		
		/**
		 * @inheritDoc
		 */
		override public function postSolve():void {
			super.postSolve();
		}
		
	}

}