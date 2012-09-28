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
package com.element.oimo.physics.dynamics {
	import com.element.oimo.physics.collision.broad.BroadPhase;
	import com.element.oimo.physics.collision.broad.BruteForceBroadPhase;
	import com.element.oimo.physics.collision.broad.Pair;
	import com.element.oimo.physics.collision.broad.SweepAndPruneBroadPhase;
	import com.element.oimo.physics.collision.narrow.ContactInfo;
	import com.element.oimo.physics.collision.narrow.CollisionDetector;
	import com.element.oimo.physics.collision.narrow.SphereBoxCollisionDetector;
	import com.element.oimo.physics.collision.narrow.SphereSphereCollisionDetector;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.constraint.contact.Contact;
	import com.element.oimo.physics.test.OimoPhysicsTest;
	import com.element.oimo.physics.util.Performance;
	import flash.utils.getTimer;
	/**
	 * 物理演算ワールドのクラスです。
	 * 全ての物理演算オブジェクトはワールドに追加する必要があります。
	 * @author saharan
	 */
	public class World {
		/**
		 * 追加できる剛体の最大数です。
		 */
		public static const MAX_BODIES:uint = 16384;
		
		/**
		 * 追加できる形状の最大数です。
		 */
		public static const MAX_SHAPES:uint = 32768;
		
		/**
		 * 検出できる接触点の最大数です。
		 */
		public static const MAX_CONTACTS:uint = 65536;
		
		/**
		 * 検出できるプロキシが重なった形状のペアの最大数です。
		 */
		public static const MAX_PAIRS:uint = 65536;
		
		/**
		 * 追加されている剛体の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var rigidBodies:Vector.<RigidBody>;
		
		/**
		 * 追加されている剛体の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numRigidBodies:uint;
		
		/**
		 * 追加されている形状の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shapes:Vector.<Shape>;
		
		/**
		 * 追加されている形状の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numShapes:uint;
		
		/**
		 * プロキシが重なった形状のペアの配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var pairs:Vector.<Pair>;
		
		/**
		 * プロキシが重なった形状のペアの数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numPairs:uint;
		
		/**
		 * 剛体の接触点の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var contacts:Vector.<Contact>;
		
		/**
		 * 剛体の接触点の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numContacts:uint;
		
		/**
		 * 1回のステップで進む時間の長さです。
		 */
		public var timeStep:Number;
		
		/**
		 * ワールドにかかる重力です。
		 */
		public var gravity:Vec3;
		
		/**
		 * 衝突応答の反復処理回数です。
		 * 値が大きいほど、より正確な動きになります。
		 */
		public var iteration:uint;
		
		/**
		 * パフォーマンスの詳細情報です。
		 * 計算に要した時間などが記録されています。
		 */
		public var performance:Performance;
		
		private var broadPhase:BroadPhase;
		
		private var sphereSphereDetector:CollisionDetector;
		private var sphereBoxDetector:CollisionDetector;
		
		private var contactInfos:Vector.<ContactInfo>;
		private var numContactInfos:uint;
		
		/**
		 * 新しく World オブジェクトを作成します。
		 * ワールドのタイムステップは、1秒間でのステップの実行回数から算出されます。
		 * @param stepPerSecond 1秒間でのステップの実行回数
		 */
		public function World(stepPerSecond:Number = 60) {
			timeStep = 1 / stepPerSecond;
			iteration = 8;
			gravity = new Vec3(0, -9.80665, 0);
			rigidBodies = new Vector.<RigidBody>(MAX_BODIES, true);
			shapes = new Vector.<Shape>(MAX_SHAPES, true);
			pairs = new Vector.<Pair>(MAX_PAIRS, true);
			performance = new Performance();
			broadPhase = new SweepAndPruneBroadPhase();
			// broadPhase = new BruteForceBroadPhase();
			sphereSphereDetector = new SphereSphereCollisionDetector();
			sphereBoxDetector = new SphereBoxCollisionDetector();
			contactInfos = new Vector.<ContactInfo>(MAX_CONTACTS, true);
			contacts = new Vector.<Contact>(MAX_CONTACTS, true);
		}
		
		/**
		 * ワールドに剛体を追加します。
		 * 追加された剛体はステップ毎の演算対象になります。
		 * @param	rigidBody 追加する剛体
		 */
		public function addRigidBody(rigidBody:RigidBody):void {
			if (numRigidBodies == MAX_BODIES) {
				throw new Error("これ以上ワールドに剛体を追加することはできません");
			}
			if (rigidBody.parent) {
				throw new Error("一つの剛体を複数ワールドに追加することはできません");
			}
			rigidBodies[numRigidBodies++] = rigidBody;
			var num:uint = rigidBody.numShapes;
			for (var i:int = 0; i < num; i++) {
				addShape(rigidBody.shapes[i]);
			}
			rigidBody.parent = this;
		}
		
		/**
		 * ワールドから剛体を削除します。
		 * 削除する剛体のインデックスを指定した場合は、インデックスのみを使用して削除します。
		 * 削除された物体はステップ毎の演算対象から外されます。
		 * @param	rigidBody 削除する剛体
		 * @param	index 削除する剛体のインデックス
		 */
		public function removeRigidBody(rigidBody:RigidBody, index:int = -1):void {
			if (index < 0) {
				for (var i:int = 0; i < numRigidBodies; i++) {
					if (rigidBody == rigidBodies[i]) {
						index = i;
						break;
					}
				}
				if (index == -1) {
					return;
				}
			} else if (index >= numRigidBodies) {
				throw new Error("削除する剛体のインデックスが範囲外です");
			}
			var remove:RigidBody = rigidBodies[index];
			remove.parent = null;
			var num:uint = rigidBody.numShapes;
			for (var j:int = 0; j < num; j++) {
				removeShape(rigidBody.shapes[j]);
			}
			for (var k:int = index; k < numRigidBodies - 1; k++) {
				rigidBodies[k] = rigidBodies[k + 1];
			}
			rigidBodies[--numRigidBodies] = null;
		}
		
		/**
		 * ワールドに形状を追加します。
		 * <strong>剛体をワールドに追加、およびワールドに追加されている剛体に形状を追加すると、
		 * 自動で形状もワールドに追加されるので、このメソッドは外部から呼ばないでください。</strong>
		 * @param	shape 追加する形状
		 */
		public function addShape(shape:Shape):void {
			if (numShapes == MAX_SHAPES) {
				throw new Error("これ以上ワールドに形状を追加することはできません");
			}
			if (!shape.parent) {
				throw new Error("ワールドに形状を単体で追加することはできません");
			}
			if (shape.parent.parent) {
				throw new Error("一つの形状を複数ワールドに追加することはできません");
			}
			broadPhase.addProxy(shape.proxy);
			shapes[numShapes++] = shape;
		}
		
		/**
		 * ワールドからから形状を削除します。
		 * 削除する形状のインデックスを指定した場合は、インデックスのみを使用して削除します。
		 * <strong>剛体をワールドから削除、およびワールドに追加されている剛体から形状を削除すると、
		 * 自動で形状もワールドから削除されるので、このメソッドは外部から呼ばないでください。</strong>
		 * @param	shape 削除する形状
		 * @param	index 削除する形状のインデックス
		 */
		public function removeShape(shape:Shape, index:int = -1):void {
			if (index < 0) {
				for (var i:int = 0; i < numShapes; i++) {
					if (shape == shapes[i]) {
						index = i;
						break;
					}
				}
				if (index == -1) {
					return;
				}
			} else if (index >= numShapes) {
				throw new Error("削除する形状のインデックスが範囲外です");
			}
			var remove:Shape = shapes[index];
			broadPhase.removeProxy(remove.proxy);
			for (var j:int = index; j < numShapes - 1; j++) {
				shapes[j] = shapes[j + 1];
			}
			shapes[--numShapes] = null;
		}
		
		/**
		 * ワールドの時間をタイムステップ秒だけ進めます。
		 */
		public function step():void {
			var start:int = getTimer();
			for (var i:int = 0; i < numRigidBodies; i++) {
				rigidBodies[i].update(timeStep, gravity);
			}
			performance.updateTime = getTimer() - start;
			collisionDetection();
			collisionResponse();
			performance.totalTime = getTimer() - start;
		}
		
		private function collisionDetection():void {
			collectContactInfos();
			setupContacts();
		}
		
		private function collectContactInfos():void {
			// broad phase
			var start:int = getTimer();
			numPairs = broadPhase.detectPairs(pairs);
			performance.broadPhaseTime = getTimer() - start;
			// narrow phase
			performance.narrowPhaseTime = getTimer();
			numContactInfos = 0;
			for (var i:int = 0; i < numPairs; i++) {
				var pair:Pair = pairs[i];
				var s1:Shape = pair.shape1;
				var s2:Shape = pair.shape2;
				var detector:CollisionDetector = null;
				var flip:Boolean = false;
				switch(s1.type) {
				case Shape.SHAPE_SPHERE:
					switch(s2.type) {
					case Shape.SHAPE_SPHERE:
						detector = sphereSphereDetector;
						break;
					case Shape.SHAPE_BOX:
						detector = sphereBoxDetector;
						break;
					}
					break;
				case Shape.SHAPE_BOX:
					switch(s2.type) {
					case Shape.SHAPE_SPHERE:
						detector = sphereBoxDetector;
						flip = true;
						break;
					case Shape.SHAPE_BOX:
						break;
					}
					break;
				}
				if (detector) {
					numContactInfos = detector.detectCollision(s1, s2, contactInfos, numContactInfos, flip);
					if (numContactInfos == MAX_CONTACTS) {
						return;
					}
				}
			}
		}
		
		private function setupContacts():void {
			numContacts = numContactInfos;
			for (var i:int = 0; i < numContacts; i++) {
				if (!contacts[i]) {
					contacts[i] = new Contact();
				}
				var c:Contact = contacts[i];
				c.setupFromContactInfo(contactInfos[i]);
				// search old contacts
				var s1:Shape = c.shape1;
				var s2:Shape = c.shape2;
				var sc:Vector.<Contact>;
				var numSc:uint;
				if (s1.numContacts < s2.numContacts) {
					sc = s1.contacts;
					numSc = s1.numContacts;
				} else {
					sc = s2.contacts;
					numSc = s2.numContacts;
				}
				for (var j:int = 0; j < numSc; j++) {
					var oc:Contact = sc[j];
					if (
						(oc.shape1 == c.shape1 && oc.shape2 == c.shape2 ||
						oc.shape1 == c.shape2 && oc.shape2 == c.shape1) &&
						oc.id.equals(c.id)
					) {
						// warm starting
						c.normalImpulse = oc.normalImpulse;
						c.tangentImpulse = oc.tangentImpulse;
						c.binormalImpulse = oc.binormalImpulse;
						c.warmStarted = true;
						break;
					}
				}
			}
			performance.narrowPhaseTime = getTimer() - performance.narrowPhaseTime;
		}
		
		private function collisionResponse():void {
			var start:int = getTimer();
			// reset contact counts
			for (var i:int = 0; i < numShapes; i++) {
				shapes[i].numContacts = 0;
			}
			for (var j:int = 0; j < numContacts; j++) {
				contacts[j].preSolve();
			}
			// solve system of equations
			for (var k:int = 0; k < iteration; k++) {
				for (var l:int = 0; l < numContacts; l++) {
					contacts[l].solve();
				}
			}
			for (var m:int = 0; m < numContacts; m++) {
				contacts[m].postSolve();
			}
			performance.constraintsTime = getTimer() - start;
		}
		
	}

}