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
package com.element.oimo.physics.constraint.contact {
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.collision.narrow.ContactID;
	import com.element.oimo.physics.collision.narrow.ContactInfo;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.constraint.Constraint;
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * 二つの剛体間の接触を扱うクラスです。
	 * @author saharan
	 */
	public class Contact extends Constraint {
		/**
		 * 接触地点のワールド座標です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var position:Vec3;
		
		/**
		 * 剛体1に対する接触点の相対位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativePosition1:Vec3;
		
		/**
		 * 剛体2に対する接触点の相対位置です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var relativePosition2:Vec3;
		
		/**
		 * 接触面に対し垂直な法線ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 接線および従法線とは垂直の関係にあり、
		 * 垂直抗力はこの方向にのみ働きます。
		 */
		public var normal:Vec3;
		
		/**
		 * 接触面に対し水平な接線ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 法線および従法線とは垂直の関係にあり、
		 * 摩擦力はこの方向および従法線方向にのみ働きます。
		 */
		public var tangent:Vec3;
		
		/**
		 * 接触面に対し水平な従法線ベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 法線および接線とは垂直の関係にあり、
		 * 摩擦力はこの方向および接線方向にのみ働きます。
		 */
		public var binormal:Vec3;
		
		/**
		 * 剛体間に発生した重なりの大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 重なりを解消するため、各剛体は法線方向に押し出されます。
		 */
		public var overlap:Number;
		
		/**
		 * 法線方向に働いた垂直抗力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var normalImpulse:Number;
		
		/**
		 * 接線方向に働いた摩擦力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var tangentImpulse:Number;
		
		/**
		 * 従法線方向に働いた摩擦力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var binormalImpulse:Number;
		
		/**
		 * 法線方向に働いた重なり解消力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var separateImpulse:Number;
		
		/**
		 * 接触を起こした形状1です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shape1:Shape;
		
		/**
		 * 接触を起こした形状2です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shape2:Shape;
		
		/**
		 * 接触を起こした剛体1です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var rigid1:RigidBody;
		
		/**
		 * 接触を起こした剛体2です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var rigid2:RigidBody;
		
		/**
		 * 接触点の識別データです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var id:ContactID;
		
		/**
		 * この接触が前ステップでも存在したかを示します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var warmStarted:Boolean;
		
		private var normalDenominator:Number;
		private var tangentDenominator:Number;
		private var binormalDenominator:Number;
		
		private var targetNormalVelocity:Number;
		private var targetSeparateVelocity:Number;
		
		private var relativeVelocity:Vec3;
		private var tmp1:Vec3;
		private var tmp2:Vec3;
		
		/**
		 * 新しく Contact オブジェクトを作成します。
		 */
		public function Contact() {
			position = new Vec3();
			relativePosition1 = new Vec3();
			relativePosition2 = new Vec3();
			normal = new Vec3();
			tangent = new Vec3();
			binormal = new Vec3();
			id = new ContactID();
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
			separateImpulse = 0;
			relativeVelocity = new Vec3();
			tmp1 = new Vec3();
			tmp2 = new Vec3();
		}
		
		/**
		 * この拘束を接触点情報から作成します。
		 * @param	contactInfo
		 */
		public function setupFromContactInfo(contactInfo:ContactInfo):void {
			position.copy(contactInfo.position);
			normal.copy(contactInfo.normal);
			tangent.x = -normal.y;
			tangent.y = normal.z;
			tangent.z = normal.x;
			binormal.cross(normal, tangent).normalize(binormal);
			overlap = contactInfo.overlap;
			shape1 = contactInfo.shape1;
			shape2 = contactInfo.shape2;
			rigid1 = shape1.parent;
			rigid2 = shape2.parent;
			relativePosition1.sub(position, rigid1.position);
			relativePosition2.sub(position, rigid2.position);
			id.data1 = contactInfo.id.data1;
			id.data2 = contactInfo.id.data2;
			id.flip = contactInfo.id.flip;
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
			warmStarted = false;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function preSolve():void {
			normalDenominator = calcDenominator(normal);
			tangentDenominator = calcDenominator(tangent);
			binormalDenominator = calcDenominator(binormal);
			relativeVelocity.sub(
				tmp2.cross(rigid2.angularVelocity, relativePosition2).add(tmp2, rigid2.linearVelocity),
				tmp1.cross(rigid1.angularVelocity, relativePosition1).add(tmp1, rigid1.linearVelocity)
			);
			var rvn:Number = normal.dot(relativeVelocity);
			targetNormalVelocity = -(shape1.restitution * shape2.restitution) * rvn;
			if (warmStarted) {
				tmp1.scale(normal, normalImpulse);
				tmp1.add(tmp1, tmp2.scale(tangent, tangentImpulse));
				tmp1.add(tmp1, tmp2.scale(binormal, binormalImpulse));
				rigid1.applyImpulse(position, tmp1);
				rigid2.applyImpulse(position, tmp1.invert(tmp1));
			}
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			var oldImpulse:Number;
			var newImpulse:Number;
			relativeVelocity.sub(
				tmp2.add(tmp2.cross(rigid2.angularVelocity, relativePosition2), rigid2.linearVelocity),
				tmp1.add(tmp1.cross(rigid1.angularVelocity, relativePosition1), rigid1.linearVelocity)
			);
			var rvn:Number = normal.dot(relativeVelocity);
			oldImpulse = normalImpulse;
			newImpulse = (rvn - targetNormalVelocity) * normalDenominator;
			normalImpulse += newImpulse;
			if (normalImpulse > 0) normalImpulse = 0;
			newImpulse = normalImpulse - oldImpulse;
			tmp1.scale(normal, newImpulse);
			rigid1.applyImpulse(position, tmp1);
			rigid2.applyImpulse(position, tmp1.invert(tmp1));
		}
		
		/**
		 * @inheritDoc
		 */
		override public function postSolve():void {
			if (shape1.numContacts < Shape.MAX_CONTACTS) {
				shape1.contacts[shape1.numContacts++] = this;
			}
			if (shape2.numContacts < Shape.MAX_CONTACTS) {
				shape2.contacts[shape2.numContacts++] = this;
			}
		}
		
		private function calcDenominator(normal:Vec3):Number {
			// 1/m1 + 1/m2 + n・([r1×n/I1]×r1 + [r2×n/I2]×r2)
			tmp1.cross(relativePosition1, normal).mulMat(rigid1.invertInertia, tmp1).cross(tmp1, relativePosition1);
			tmp2.cross(relativePosition2, normal).mulMat(rigid2.invertInertia, tmp2).cross(tmp2, relativePosition2);
			return 1 / (rigid1.invertMass + rigid2.invertMass + normal.dot(tmp1.add(tmp1, tmp2)));
		}
		
	}

}