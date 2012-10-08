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
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Vec3;
	import com.element.oimo.physics.collision.narrow.ContactID;
	import com.element.oimo.physics.collision.narrow.ContactInfo;
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.constraint.Constraint;
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * 二つの剛体間の接触拘束を扱うクラスです。
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
		
		private var lVel1:Vec3;
		private var lVel2:Vec3;
		
		private var aVel1:Vec3;
		private var aVel2:Vec3;
		
		private var relPos1X:Number;
		private var relPos1Y:Number;
		private var relPos1Z:Number;
		private var relPos2X:Number;
		private var relPos2Y:Number;
		private var relPos2Z:Number;
		
		private var relVelX:Number;
		private var relVelY:Number;
		private var relVelZ:Number;
		
		private var norX:Number;
		private var norY:Number;
		private var norZ:Number;
		
		private var tanX:Number;
		private var tanY:Number;
		private var tanZ:Number;
		
		private var binX:Number;
		private var binY:Number;
		private var binZ:Number;
		
		private var invM1:Number;
		private var invM2:Number;
		
		private var invI1e00:Number;
		private var invI1e01:Number;
		private var invI1e02:Number;
		private var invI1e10:Number;
		private var invI1e11:Number;
		private var invI1e12:Number;
		private var invI1e20:Number;
		private var invI1e21:Number;
		private var invI1e22:Number;
		private var invI2e00:Number;
		private var invI2e01:Number;
		private var invI2e02:Number;
		private var invI2e10:Number;
		private var invI2e11:Number;
		private var invI2e12:Number;
		private var invI2e20:Number;
		private var invI2e21:Number;
		private var invI2e22:Number;
		
		private var normalDenominator:Number;
		private var tangentDenominator:Number;
		private var binormalDenominator:Number;
		
		private var targetNormalVelocity:Number;
		private var targetSeparateVelocity:Number;
		private var friction:Number;
		private var restitution:Number;
		
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
		
		//----------------------------------------------------------
		//               HELL OF INLINE EXPANSION
		//----------------------------------------------------------
		
		/**
		 * この拘束を接触点情報から作成します。
		 * @param	contactInfo
		 */
		public function setupFromContactInfo(contactInfo:ContactInfo):void {
			position.x = contactInfo.position.x;
			position.y = contactInfo.position.y;
			position.z = contactInfo.position.z;
			
			norX = contactInfo.normal.x;
			norY = contactInfo.normal.y;
			norZ = contactInfo.normal.z;
			normal.x = norX;
			normal.y = norY;
			normal.z = norZ;
			tanX = norY * norX - norZ * norZ;
			tanY = norZ * -norY - norX * norX;
			tanZ = norX * norZ + norY * norY;
			var len:Number = 1 / Math.sqrt(tanX * tanX + tanY * tanY + tanZ * tanZ);
			tanX *= len;
			tanY *= len;
			tanZ *= len;
			tangent.x = tanX;
			tangent.y = tanY;
			tangent.z = tanZ;
			binX = norY * tanZ - norZ * tanY;
			binY = norZ * tanX - norX * tanZ;
			binZ = norX * tanY - norY * tanX;
			binormal.x = binX;
			binormal.y = binY;
			binormal.z = binZ;
			
			overlap = contactInfo.overlap;
			shape1 = contactInfo.shape1;
			shape2 = contactInfo.shape2;
			rigid1 = shape1.parent;
			rigid2 = shape2.parent;
			
			relPos1X = position.x - rigid1.position.x;
			relPos1Y = position.y - rigid1.position.y;
			relPos1Z = position.z - rigid1.position.z;
			relativePosition1.x = relPos1X;
			relativePosition1.y = relPos1Y;
			relativePosition1.z = relPos1Z;
			relPos2X = position.x - rigid2.position.x;
			relPos2Y = position.y - rigid2.position.y;
			relPos2Z = position.z - rigid2.position.z;
			relativePosition2.x = relPos2X;
			relativePosition2.y = relPos2Y;
			relativePosition2.z = relPos2Z;
			
			lVel1 = rigid1.linearVelocity;
			lVel2 = rigid2.linearVelocity;
			aVel1 = rigid1.angularVelocity;
			aVel2 = rigid2.angularVelocity;
			
			invM1 = rigid1.invertMass;
			invM2 = rigid2.invertMass;
			
			var tmpI:Mat33;
			tmpI = rigid1.invertInertia;
			invI1e00 = tmpI.e00;
			invI1e01 = tmpI.e01;
			invI1e02 = tmpI.e02;
			invI1e10 = tmpI.e10;
			invI1e11 = tmpI.e11;
			invI1e12 = tmpI.e12;
			invI1e20 = tmpI.e20;
			invI1e21 = tmpI.e21;
			invI1e22 = tmpI.e22;
			tmpI = rigid2.invertInertia;
			invI2e00 = tmpI.e00;
			invI2e01 = tmpI.e01;
			invI2e02 = tmpI.e02;
			invI2e10 = tmpI.e10;
			invI2e11 = tmpI.e11;
			invI2e12 = tmpI.e12;
			invI2e20 = tmpI.e20;
			invI2e21 = tmpI.e21;
			invI2e22 = tmpI.e22;
			
			id.data1 = contactInfo.id.data1;
			id.data2 = contactInfo.id.data2;
			id.flip = contactInfo.id.flip;
			friction = shape1.friction * shape2.friction;
			restitution = shape1.restitution * shape2.restitution;
			overlap = contactInfo.overlap + 0.05; // 5cm
			if (overlap > 0) overlap = 0;
			// 相乗平均 Geometric mean
			// friction = Math.sqrt(shape1.friction * shape2.friction);
			// restitution = Math.sqrt(shape1.restitution * shape2.restitution);
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
			warmStarted = false;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function preSolve():void {
			var tmp1X:Number;
			var tmp1Y:Number;
			var tmp1Z:Number;
			var tmp2X:Number;
			var tmp2Y:Number;
			var tmp2Z:Number;
			var tmp3X:Number;
			var tmp3Y:Number;
			var tmp3Z:Number;
			
			tmp1X = relPos1Y * norZ - relPos1Z * norY;
			tmp1Y = relPos1Z * norX - relPos1X * norZ;
			tmp1Z = relPos1X * norY - relPos1Y * norX;
			tmp2X = tmp1X * invI1e00 + tmp1Y * invI1e01 + tmp1Z * invI1e02;
			tmp2Y = tmp1X * invI1e10 + tmp1Y * invI1e11 + tmp1Z * invI1e12;
			tmp2Z = tmp1X * invI1e20 + tmp1Y * invI1e21 + tmp1Z * invI1e22;
			tmp3X = tmp2Y * relPos1Z - tmp2Z * relPos1Y;
			tmp3Y = tmp2Z * relPos1X - tmp2X * relPos1Z;
			tmp3Z = tmp2X * relPos1Y - tmp2Y * relPos1X;
			tmp1X = relPos2Y * norZ - relPos2Z * norY;
			tmp1Y = relPos2Z * norX - relPos2X * norZ;
			tmp1Z = relPos2X * norY - relPos2Y * norX;
			tmp2X = tmp1X * invI2e00 + tmp1Y * invI2e01 + tmp1Z * invI2e02;
			tmp2Y = tmp1X * invI2e10 + tmp1Y * invI2e11 + tmp1Z * invI2e12;
			tmp2Z = tmp1X * invI2e20 + tmp1Y * invI2e21 + tmp1Z * invI2e22;
			tmp3X += tmp2Y * relPos2Z - tmp2Z * relPos2Y;
			tmp3Y += tmp2Z * relPos2X - tmp2X * relPos2Z;
			tmp3Z += tmp2X * relPos2Y - tmp2Y * relPos2X;
			normalDenominator = 1 / (invM1 + invM2 + norX * tmp3X + norY * tmp3Y + norZ * tmp3Z);
			
			tmp1X = relPos1Y * tanZ - relPos1Z * tanY;
			tmp1Y = relPos1Z * tanX - relPos1X * tanZ;
			tmp1Z = relPos1X * tanY - relPos1Y * tanX;
			tmp2X = tmp1X * invI1e00 + tmp1Y * invI1e01 + tmp1Z * invI1e02;
			tmp2Y = tmp1X * invI1e10 + tmp1Y * invI1e11 + tmp1Z * invI1e12;
			tmp2Z = tmp1X * invI1e20 + tmp1Y * invI1e21 + tmp1Z * invI1e22;
			tmp3X = tmp2Y * relPos1Z - tmp2Z * relPos1Y;
			tmp3Y = tmp2Z * relPos1X - tmp2X * relPos1Z;
			tmp3Z = tmp2X * relPos1Y - tmp2Y * relPos1X;
			tmp1X = relPos2Y * tanZ - relPos2Z * tanY;
			tmp1Y = relPos2Z * tanX - relPos2X * tanZ;
			tmp1Z = relPos2X * tanY - relPos2Y * tanX;
			tmp2X = tmp1X * invI2e00 + tmp1Y * invI2e01 + tmp1Z * invI2e02;
			tmp2Y = tmp1X * invI2e10 + tmp1Y * invI2e11 + tmp1Z * invI2e12;
			tmp2Z = tmp1X * invI2e20 + tmp1Y * invI2e21 + tmp1Z * invI2e22;
			tmp3X += tmp2Y * relPos2Z - tmp2Z * relPos2Y;
			tmp3Y += tmp2Z * relPos2X - tmp2X * relPos2Z;
			tmp3Z += tmp2X * relPos2Y - tmp2Y * relPos2X;
			tangentDenominator = 1 / (invM1 + invM2 + tanX * tmp3X + tanY * tmp3Y + tanZ * tmp3Z);
			
			tmp1X = relPos1Y * binZ - relPos1Z * binY;
			tmp1Y = relPos1Z * binX - relPos1X * binZ;
			tmp1Z = relPos1X * binY - relPos1Y * binX;
			tmp2X = tmp1X * invI1e00 + tmp1Y * invI1e01 + tmp1Z * invI1e02;
			tmp2Y = tmp1X * invI1e10 + tmp1Y * invI1e11 + tmp1Z * invI1e12;
			tmp2Z = tmp1X * invI1e20 + tmp1Y * invI1e21 + tmp1Z * invI1e22;
			tmp3X = tmp2Y * relPos1Z - tmp2Z * relPos1Y;
			tmp3Y = tmp2Z * relPos1X - tmp2X * relPos1Z;
			tmp3Z = tmp2X * relPos1Y - tmp2Y * relPos1X;
			tmp1X = relPos2Y * binZ - relPos2Z * binY;
			tmp1Y = relPos2Z * binX - relPos2X * binZ;
			tmp1Z = relPos2X * binY - relPos2Y * binX;
			tmp2X = tmp1X * invI2e00 + tmp1Y * invI2e01 + tmp1Z * invI2e02;
			tmp2Y = tmp1X * invI2e10 + tmp1Y * invI2e11 + tmp1Z * invI2e12;
			tmp2Z = tmp1X * invI2e20 + tmp1Y * invI2e21 + tmp1Z * invI2e22;
			tmp3X += tmp2Y * relPos2Z - tmp2Z * relPos2Y;
			tmp3Y += tmp2Z * relPos2X - tmp2X * relPos2Z;
			tmp3Z += tmp2X * relPos2Y - tmp2Y * relPos2X;
			binormalDenominator = 1 / (invM1 + invM2 + binX * tmp3X + binY * tmp3Y + binZ * tmp3Z);
			
			relVelX = (lVel2.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y) - (lVel1.x + aVel1.y * relPos1Z - aVel1.z * relPos1Y);
			relVelY = (lVel2.y + aVel2.z * relPos2X - aVel2.x * relPos2Z) - (lVel1.y + aVel1.z * relPos1X - aVel1.x * relPos1Z);
			relVelZ = (lVel2.z + aVel2.x * relPos2Y - aVel2.y * relPos2X) - (lVel1.z + aVel1.x * relPos1Y - aVel1.y * relPos1X);
			
			var rvn:Number = norX * relVelX + norY * relVelY + norZ * relVelZ;
			if (rvn > -1) rvn = 0;
			targetNormalVelocity = restitution * -rvn;
			if (targetNormalVelocity < -overlap * 2) targetNormalVelocity -= overlap * 2;
			
			if (warmStarted) {
				tmp1X = norX * normalImpulse + tanX * tangentImpulse + binX * binormalImpulse;
				tmp1Y = norY * normalImpulse + tanY * tangentImpulse + binY * binormalImpulse;
				tmp1Z = norZ * normalImpulse + tanZ * tangentImpulse + binZ * binormalImpulse;
				tmp2X = relPos1Y * tmp1Z - relPos1Z * tmp1Y;
				tmp2Y = relPos1Z * tmp1X - relPos1X * tmp1Z;
				tmp2Z = relPos1X * tmp1Y - relPos1Y * tmp1X;
				lVel1.x += tmp1X * invM1;
				lVel1.y += tmp1Y * invM1;
				lVel1.z += tmp1Z * invM1;
				aVel1.x += tmp2X * invI1e00 + tmp2Y * invI1e01 + tmp2Z * invI1e02;
				aVel1.y += tmp2X * invI1e10 + tmp2Y * invI1e11 + tmp2Z * invI1e12;
				aVel1.z += tmp2X * invI1e20 + tmp2Y * invI1e21 + tmp2Z * invI1e22;
				tmp2X = relPos2Y * tmp1Z - relPos2Z * tmp1Y;
				tmp2Y = relPos2Z * tmp1X - relPos2X * tmp1Z;
				tmp2Z = relPos2X * tmp1Y - relPos2Y * tmp1X;
				lVel2.x -= tmp1X * invM2;
				lVel2.y -= tmp1Y * invM2;
				lVel2.z -= tmp1Z * invM2;
				aVel2.x -= tmp2X * invI2e00 + tmp2Y * invI2e01 + tmp2Z * invI2e02;
				aVel2.y -= tmp2X * invI2e10 + tmp2Y * invI2e11 + tmp2Z * invI2e12;
				aVel2.z -= tmp2X * invI2e20 + tmp2Y * invI2e21 + tmp2Z * invI2e22;
			}
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			var oldImpulse1:Number;
			var newImpulse1:Number;
			var oldImpulse2:Number;
			var newImpulse2:Number;
			var rvn:Number;
			var forceX:Number;
			var forceY:Number;
			var forceZ:Number;
			var tmpX:Number;
			var tmpY:Number;
			var tmpZ:Number;
			
			// restitution part
			
			relVelX = (lVel2.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y) - (lVel1.x + aVel1.y * relPos1Z - aVel1.z * relPos1Y);
			relVelY = (lVel2.y + aVel2.z * relPos2X - aVel2.x * relPos2Z) - (lVel1.y + aVel1.z * relPos1X - aVel1.x * relPos1Z);
			relVelZ = (lVel2.z + aVel2.x * relPos2Y - aVel2.y * relPos2X) - (lVel1.z + aVel1.x * relPos1Y - aVel1.y * relPos1X);
			
			rvn = norX * relVelX + norY * relVelY + norZ * relVelZ;
			oldImpulse1 = normalImpulse;
			newImpulse1 = (rvn - targetNormalVelocity) * normalDenominator;
			normalImpulse += newImpulse1;
			if (normalImpulse > 0) normalImpulse = 0;
			newImpulse1 = normalImpulse - oldImpulse1;
			forceX = norX * newImpulse1;
			forceY = norY * newImpulse1;
			forceZ = norZ * newImpulse1;
			tmpX = relPos1Y * forceZ - relPos1Z * forceY;
			tmpY = relPos1Z * forceX - relPos1X * forceZ;
			tmpZ = relPos1X * forceY - relPos1Y * forceX;
			lVel1.x += forceX * invM1;
			lVel1.y += forceY * invM1;
			lVel1.z += forceZ * invM1;
			aVel1.x += tmpX * invI1e00 + tmpY * invI1e01 + tmpZ * invI1e02;
			aVel1.y += tmpX * invI1e10 + tmpY * invI1e11 + tmpZ * invI1e12;
			aVel1.z += tmpX * invI1e20 + tmpY * invI1e21 + tmpZ * invI1e22;
			tmpX = relPos2Y * forceZ - relPos2Z * forceY;
			tmpY = relPos2Z * forceX - relPos2X * forceZ;
			tmpZ = relPos2X * forceY - relPos2Y * forceX;
			lVel2.x -= forceX * invM2;
			lVel2.y -= forceY * invM2;
			lVel2.z -= forceZ * invM2;
			aVel2.x -= tmpX * invI2e00 + tmpY * invI2e01 + tmpZ * invI2e02;
			aVel2.y -= tmpX * invI2e10 + tmpY * invI2e11 + tmpZ * invI2e12;
			aVel2.z -= tmpX * invI2e20 + tmpY * invI2e21 + tmpZ * invI2e22;
			
			// friction part
			
			var max:Number = -normalImpulse * friction;
			relVelX = (lVel2.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y) - (lVel1.x + aVel1.y * relPos1Z - aVel1.z * relPos1Y);
			relVelY = (lVel2.y + aVel2.z * relPos2X - aVel2.x * relPos2Z) - (lVel1.y + aVel1.z * relPos1X - aVel1.x * relPos1Z);
			relVelZ = (lVel2.z + aVel2.x * relPos2Y - aVel2.y * relPos2X) - (lVel1.z + aVel1.x * relPos1Y - aVel1.y * relPos1X);
			
			rvn = tanX * relVelX + tanY * relVelY + tanZ * relVelZ;
			oldImpulse1 = tangentImpulse;
			newImpulse1 = rvn * tangentDenominator;
			tangentImpulse += newImpulse1;
			
			rvn = binX * relVelX + binY * relVelY + binZ * relVelZ;
			oldImpulse2 = binormalImpulse;
			newImpulse2 = rvn * binormalDenominator;
			binormalImpulse += newImpulse2;
			
			var len:Number = tangentImpulse * tangentImpulse + binormalImpulse * binormalImpulse;
			if (len > max * max) { // maximum friction
				len = max / Math.sqrt(len);
				tangentImpulse *= len;
				binormalImpulse *= len;
			}
			
			newImpulse1 = tangentImpulse - oldImpulse1;
			newImpulse2 = binormalImpulse - oldImpulse2;
			
			forceX = tanX * newImpulse1 + binX * newImpulse2;
			forceY = tanY * newImpulse1 + binY * newImpulse2;
			forceZ = tanZ * newImpulse1 + binZ * newImpulse2;
			tmpX = relPos1Y * forceZ - relPos1Z * forceY;
			tmpY = relPos1Z * forceX - relPos1X * forceZ;
			tmpZ = relPos1X * forceY - relPos1Y * forceX;
			lVel1.x += forceX * invM1;
			lVel1.y += forceY * invM1;
			lVel1.z += forceZ * invM1;
			aVel1.x += tmpX * invI1e00 + tmpY * invI1e01 + tmpZ * invI1e02;
			aVel1.y += tmpX * invI1e10 + tmpY * invI1e11 + tmpZ * invI1e12;
			aVel1.z += tmpX * invI1e20 + tmpY * invI1e21 + tmpZ * invI1e22;
			tmpX = relPos2Y * forceZ - relPos2Z * forceY;
			tmpY = relPos2Z * forceX - relPos2X * forceZ;
			tmpZ = relPos2X * forceY - relPos2Y * forceX;
			lVel2.x -= forceX * invM2;
			lVel2.y -= forceY * invM2;
			lVel2.z -= forceZ * invM2;
			aVel2.x -= tmpX * invI2e00 + tmpY * invI2e01 + tmpZ * invI2e02;
			aVel2.y -= tmpX * invI2e10 + tmpY * invI2e11 + tmpZ * invI2e12;
			aVel2.z -= tmpX * invI2e20 + tmpY * invI2e21 + tmpZ * invI2e22;
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
			relativeVelocity.x = (lVel2.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y) - (lVel1.x + aVel1.y * relPos1Z - aVel1.z * relPos1Y);
			relativeVelocity.y = (lVel2.y + aVel2.z * relPos2X - aVel2.x * relPos2Z) - (lVel1.y + aVel1.z * relPos1X - aVel1.x * relPos1Z);
			relativeVelocity.z = (lVel2.z + aVel2.x * relPos2Y - aVel2.y * relPos2X) - (lVel1.z + aVel1.x * relPos1Y - aVel1.y * relPos1X);
		}
		
	}

}