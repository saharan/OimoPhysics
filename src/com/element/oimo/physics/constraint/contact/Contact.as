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
		 * 法線方向の適正質量の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var normalDenominator:Number;
		
		/**
		 * 接線方向の適正質量の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var tangentDenominator:Number;
		
		/**
		 * 従法線方向の適正質量の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var binormalDenominator:Number;
		
		/**
		 * 従法線方向に働いた摩擦力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var binormalImpulse:Number;
		
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
		 * 形状1に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shapeConnection1:ContactConnection;
		
		/**
		 * 形状2に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shapeConnection2:ContactConnection;
		
		/**
		 * 剛体1に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var bodyConnection1:ContactConnection;
		
		/**
		 * 剛体2に対する繋がりです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var bodyConnection2:ContactConnection;
		
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
		
		private var norTorque1X:Number;
		private var norTorque1Y:Number;
		private var norTorque1Z:Number;
		private var norTorque2X:Number;
		private var norTorque2Y:Number;
		private var norTorque2Z:Number;
		
		private var tanTorque1X:Number;
		private var tanTorque1Y:Number;
		private var tanTorque1Z:Number;
		private var tanTorque2X:Number;
		private var tanTorque2Y:Number;
		private var tanTorque2Z:Number;
		
		private var binTorque1X:Number;
		private var binTorque1Y:Number;
		private var binTorque1Z:Number;
		private var binTorque2X:Number;
		private var binTorque2Y:Number;
		private var binTorque2Z:Number;
		
		private var norTorqueUnit1X:Number;
		private var norTorqueUnit1Y:Number;
		private var norTorqueUnit1Z:Number;
		private var norTorqueUnit2X:Number;
		private var norTorqueUnit2Y:Number;
		private var norTorqueUnit2Z:Number;
		
		private var tanTorqueUnit1X:Number;
		private var tanTorqueUnit1Y:Number;
		private var tanTorqueUnit1Z:Number;
		private var tanTorqueUnit2X:Number;
		private var tanTorqueUnit2Y:Number;
		private var tanTorqueUnit2Z:Number;
		
		private var binTorqueUnit1X:Number;
		private var binTorqueUnit1Y:Number;
		private var binTorqueUnit1Z:Number;
		private var binTorqueUnit2X:Number;
		private var binTorqueUnit2Y:Number;
		private var binTorqueUnit2Z:Number;
		
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
		
		private var targetNormalVelocity:Number;
		private var targetSeparateVelocity:Number;
		private var friction:Number;
		private var restitution:Number;
		
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
			shapeConnection1 = new ContactConnection(this);
			shapeConnection2 = new ContactConnection(this);
			bodyConnection1 = new ContactConnection(this);
			bodyConnection2 = new ContactConnection(this);
			id = new ContactID();
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
		}
		
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
			
			overlap = contactInfo.overlap;
			shape1 = contactInfo.shape1;
			shape2 = contactInfo.shape2;
			body1 = shape1.parent;
			body2 = shape2.parent;
			
			bodyConnection1.connectedBody = body2;
			bodyConnection1.connectedShape = shape2;
			shapeConnection1.connectedBody = body2;
			shapeConnection1.connectedShape = shape2;
			bodyConnection2.connectedBody = body1;
			bodyConnection2.connectedShape = shape1;
			shapeConnection2.connectedBody = body1;
			shapeConnection2.connectedShape = shape1;
			
			relPos1X = position.x - body1.position.x;
			relPos1Y = position.y - body1.position.y;
			relPos1Z = position.z - body1.position.z;
			relPos2X = position.x - body2.position.x;
			relPos2Y = position.y - body2.position.y;
			relPos2Z = position.z - body2.position.z;
			
			lVel1 = body1.linearVelocity;
			lVel2 = body2.linearVelocity;
			aVel1 = body1.angularVelocity;
			aVel2 = body2.angularVelocity;
			
			invM1 = body1.invertMass;
			invM2 = body2.invertMass;
			
			var tmpI:Mat33;
			tmpI = body1.invertInertia;
			invI1e00 = tmpI.e00;
			invI1e01 = tmpI.e01;
			invI1e02 = tmpI.e02;
			invI1e10 = tmpI.e10;
			invI1e11 = tmpI.e11;
			invI1e12 = tmpI.e12;
			invI1e20 = tmpI.e20;
			invI1e21 = tmpI.e21;
			invI1e22 = tmpI.e22;
			tmpI = body2.invertInertia;
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
			overlap = contactInfo.overlap;
			normalImpulse = 0;
			tangentImpulse = 0;
			binormalImpulse = 0;
			warmStarted = false;
		}
		
		/**
		 * この接触からの全ての剛体と形状への参照を破棄します。
		 */
		public function removeReferences():void {
			shape1 = null;
			shape2 = null;
			body1 = null;
			body2 = null;
			bodyConnection1.connectedBody = null;
			bodyConnection1.connectedShape = null;
			shapeConnection1.connectedBody = null;
			shapeConnection1.connectedShape = null;
			bodyConnection2.connectedBody = null;
			bodyConnection2.connectedShape = null;
			shapeConnection2.connectedBody = null;
			shapeConnection2.connectedShape = null;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function preSolve(timeStep:Number, invTimeStep:Number):void {
			
			// ----------------------------------------------
			//         calculate relative velocity
			// ----------------------------------------------
			
			relVelX = (lVel2.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y) - (lVel1.x + aVel1.y * relPos1Z - aVel1.z * relPos1Y);
			relVelY = (lVel2.y + aVel2.z * relPos2X - aVel2.x * relPos2Z) - (lVel1.y + aVel1.z * relPos1X - aVel1.x * relPos1Z);
			relVelZ = (lVel2.z + aVel2.x * relPos2Y - aVel2.y * relPos2X) - (lVel1.z + aVel1.x * relPos1Y - aVel1.y * relPos1X);
			var rvn:Number = norX * relVelX + norY * relVelY + norZ * relVelZ;
			
			// ----------------------------------------------
			//        calculate tangent and binormal
			// ----------------------------------------------
			
			tanX = relVelX - rvn * norX;
			tanY = relVelY - rvn * norY;
			tanZ = relVelZ - rvn * norZ;
			var len:Number = tanX * tanX + tanY * tanY + tanZ * tanZ;
			if (len > 1e-2) {
				len = 1 / Math.sqrt(len);
			} else {
				tanX = norY * norX - norZ * norZ;
				tanY = -norZ * norY - norX * norX;
				tanZ = norX * norZ + norY * norY;
				len = 1 / Math.sqrt(tanX * tanX + tanY * tanY + tanZ * tanZ);
			}
			tanX *= len;
			tanY *= len;
			tanZ *= len;
			binX = norY * tanZ - norZ * tanY;
			binY = norZ * tanX - norX * tanZ;
			binZ = norX * tanY - norY * tanX;
			
			// ----------------------------------------------
			// calculate torque axes and angular accelerations
			// ----------------------------------------------
			
			norTorque1X = relPos1Y * norZ - relPos1Z * norY;
			norTorque1Y = relPos1Z * norX - relPos1X * norZ;
			norTorque1Z = relPos1X * norY - relPos1Y * norX;
			norTorque2X = relPos2Y * norZ - relPos2Z * norY;
			norTorque2Y = relPos2Z * norX - relPos2X * norZ;
			norTorque2Z = relPos2X * norY - relPos2Y * norX;
			
			tanTorque1X = relPos1Y * tanZ - relPos1Z * tanY;
			tanTorque1Y = relPos1Z * tanX - relPos1X * tanZ;
			tanTorque1Z = relPos1X * tanY - relPos1Y * tanX;
			tanTorque2X = relPos2Y * tanZ - relPos2Z * tanY;
			tanTorque2Y = relPos2Z * tanX - relPos2X * tanZ;
			tanTorque2Z = relPos2X * tanY - relPos2Y * tanX;
			
			binTorque1X = relPos1Y * binZ - relPos1Z * binY;
			binTorque1Y = relPos1Z * binX - relPos1X * binZ;
			binTorque1Z = relPos1X * binY - relPos1Y * binX;
			binTorque2X = relPos2Y * binZ - relPos2Z * binY;
			binTorque2Y = relPos2Z * binX - relPos2X * binZ;
			binTorque2Z = relPos2X * binY - relPos2Y * binX;
			
			norTorqueUnit1X = norTorque1X * invI1e00 + norTorque1Y * invI1e01 + norTorque1Z * invI1e02;
			norTorqueUnit1Y = norTorque1X * invI1e10 + norTorque1Y * invI1e11 + norTorque1Z * invI1e12;
			norTorqueUnit1Z = norTorque1X * invI1e20 + norTorque1Y * invI1e21 + norTorque1Z * invI1e22;
			norTorqueUnit2X = norTorque2X * invI2e00 + norTorque2Y * invI2e01 + norTorque2Z * invI2e02;
			norTorqueUnit2Y = norTorque2X * invI2e10 + norTorque2Y * invI2e11 + norTorque2Z * invI2e12;
			norTorqueUnit2Z = norTorque2X * invI2e20 + norTorque2Y * invI2e21 + norTorque2Z * invI2e22;
			
			tanTorqueUnit1X = tanTorque1X * invI1e00 + tanTorque1Y * invI1e01 + tanTorque1Z * invI1e02;
			tanTorqueUnit1Y = tanTorque1X * invI1e10 + tanTorque1Y * invI1e11 + tanTorque1Z * invI1e12;
			tanTorqueUnit1Z = tanTorque1X * invI1e20 + tanTorque1Y * invI1e21 + tanTorque1Z * invI1e22;
			tanTorqueUnit2X = tanTorque2X * invI2e00 + tanTorque2Y * invI2e01 + tanTorque2Z * invI2e02;
			tanTorqueUnit2Y = tanTorque2X * invI2e10 + tanTorque2Y * invI2e11 + tanTorque2Z * invI2e12;
			tanTorqueUnit2Z = tanTorque2X * invI2e20 + tanTorque2Y * invI2e21 + tanTorque2Z * invI2e22;
			
			binTorqueUnit1X = binTorque1X * invI1e00 + binTorque1Y * invI1e01 + binTorque1Z * invI1e02;
			binTorqueUnit1Y = binTorque1X * invI1e10 + binTorque1Y * invI1e11 + binTorque1Z * invI1e12;
			binTorqueUnit1Z = binTorque1X * invI1e20 + binTorque1Y * invI1e21 + binTorque1Z * invI1e22;
			binTorqueUnit2X = binTorque2X * invI2e00 + binTorque2Y * invI2e01 + binTorque2Z * invI2e02;
			binTorqueUnit2Y = binTorque2X * invI2e10 + binTorque2Y * invI2e11 + binTorque2Z * invI2e12;
			binTorqueUnit2Z = binTorque2X * invI2e20 + binTorque2Y * invI2e21 + binTorque2Z * invI2e22;
			
			// ----------------------------------------------
			//         calculate impulse denominators
			// ----------------------------------------------
			
			var tmp1X:Number;
			var tmp1Y:Number;
			var tmp1Z:Number;
			var tmp2X:Number;
			var tmp2Y:Number;
			var tmp2Z:Number;
			tmp1X = norTorque1X * invI1e00 + norTorque1Y * invI1e01 + norTorque1Z * invI1e02;
			tmp1Y = norTorque1X * invI1e10 + norTorque1Y * invI1e11 + norTorque1Z * invI1e12;
			tmp1Z = norTorque1X * invI1e20 + norTorque1Y * invI1e21 + norTorque1Z * invI1e22;
			tmp2X = tmp1Y * relPos1Z - tmp1Z * relPos1Y;
			tmp2Y = tmp1Z * relPos1X - tmp1X * relPos1Z;
			tmp2Z = tmp1X * relPos1Y - tmp1Y * relPos1X;
			tmp1X = norTorque2X * invI2e00 + norTorque2Y * invI2e01 + norTorque2Z * invI2e02;
			tmp1Y = norTorque2X * invI2e10 + norTorque2Y * invI2e11 + norTorque2Z * invI2e12;
			tmp1Z = norTorque2X * invI2e20 + norTorque2Y * invI2e21 + norTorque2Z * invI2e22;
			tmp2X += tmp1Y * relPos2Z - tmp1Z * relPos2Y;
			tmp2Y += tmp1Z * relPos2X - tmp1X * relPos2Z;
			tmp2Z += tmp1X * relPos2Y - tmp1Y * relPos2X;
			normalDenominator = 1 / (invM1 + invM2 + norX * tmp2X + norY * tmp2Y + norZ * tmp2Z);
			
			tmp1X = tanTorque1X * invI1e00 + tanTorque1Y * invI1e01 + tanTorque1Z * invI1e02;
			tmp1Y = tanTorque1X * invI1e10 + tanTorque1Y * invI1e11 + tanTorque1Z * invI1e12;
			tmp1Z = tanTorque1X * invI1e20 + tanTorque1Y * invI1e21 + tanTorque1Z * invI1e22;
			tmp2X = tmp1Y * relPos1Z - tmp1Z * relPos1Y;
			tmp2Y = tmp1Z * relPos1X - tmp1X * relPos1Z;
			tmp2Z = tmp1X * relPos1Y - tmp1Y * relPos1X;
			tmp1X = tanTorque2X * invI2e00 + tanTorque2Y * invI2e01 + tanTorque2Z * invI2e02;
			tmp1Y = tanTorque2X * invI2e10 + tanTorque2Y * invI2e11 + tanTorque2Z * invI2e12;
			tmp1Z = tanTorque2X * invI2e20 + tanTorque2Y * invI2e21 + tanTorque2Z * invI2e22;
			tmp2X += tmp1Y * relPos2Z - tmp1Z * relPos2Y;
			tmp2Y += tmp1Z * relPos2X - tmp1X * relPos2Z;
			tmp2Z += tmp1X * relPos2Y - tmp1Y * relPos2X;
			tangentDenominator = 1 / (invM1 + invM2 + tanX * tmp2X + tanY * tmp2Y + tanZ * tmp2Z);
			
			tmp1X = binTorque1X * invI1e00 + binTorque1Y * invI1e01 + binTorque1Z * invI1e02;
			tmp1Y = binTorque1X * invI1e10 + binTorque1Y * invI1e11 + binTorque1Z * invI1e12;
			tmp1Z = binTorque1X * invI1e20 + binTorque1Y * invI1e21 + binTorque1Z * invI1e22;
			tmp2X = tmp1Y * relPos1Z - tmp1Z * relPos1Y;
			tmp2Y = tmp1Z * relPos1X - tmp1X * relPos1Z;
			tmp2Z = tmp1X * relPos1Y - tmp1Y * relPos1X;
			tmp1X = binTorque2X * invI2e00 + binTorque2Y * invI2e01 + binTorque2Z * invI2e02;
			tmp1Y = binTorque2X * invI2e10 + binTorque2Y * invI2e11 + binTorque2Z * invI2e12;
			tmp1Z = binTorque2X * invI2e20 + binTorque2Y * invI2e21 + binTorque2Z * invI2e22;
			tmp2X += tmp1Y * relPos2Z - tmp1Z * relPos2Y;
			tmp2Y += tmp1Z * relPos2X - tmp1X * relPos2Z;
			tmp2Z += tmp1X * relPos2Y - tmp1Y * relPos2X;
			binormalDenominator = 1 / (invM1 + invM2 + binX * tmp2X + binY * tmp2Y + binZ * tmp2Z);
			
			// ----------------------------------------------
			//           calculate initial forces
			// ----------------------------------------------
			
			if (warmStarted) {
				tangentImpulse *= 0.95;
				binormalImpulse *= 0.95;
				tmp1X = norX * normalImpulse + tanX * tangentImpulse + binX * binormalImpulse;
				tmp1Y = norY * normalImpulse + tanY * tangentImpulse + binY * binormalImpulse;
				tmp1Z = norZ * normalImpulse + tanZ * tangentImpulse + binZ * binormalImpulse;
				lVel1.x += tmp1X * invM1;
				lVel1.y += tmp1Y * invM1;
				lVel1.z += tmp1Z * invM1;
				aVel1.x += norTorqueUnit1X * normalImpulse + tanTorqueUnit1X * tangentImpulse + binTorqueUnit1X * binormalImpulse;
				aVel1.y += norTorqueUnit1Y * normalImpulse + tanTorqueUnit1Y * tangentImpulse + binTorqueUnit1Y * binormalImpulse;
				aVel1.z += norTorqueUnit1Z * normalImpulse + tanTorqueUnit1Z * tangentImpulse + binTorqueUnit1Z * binormalImpulse;
				lVel2.x -= tmp1X * invM2;
				lVel2.y -= tmp1Y * invM2;
				lVel2.z -= tmp1Z * invM2;
				aVel2.x -= norTorqueUnit2X * normalImpulse + tanTorqueUnit2X * tangentImpulse + binTorqueUnit2X * binormalImpulse;
				aVel2.y -= norTorqueUnit2Y * normalImpulse + tanTorqueUnit2Y * tangentImpulse + binTorqueUnit2Y * binormalImpulse;
				aVel2.z -= norTorqueUnit2Z * normalImpulse + tanTorqueUnit2Z * tangentImpulse + binTorqueUnit2Z * binormalImpulse;
				rvn = 0; // disable bouncing
			}
			
			// ----------------------------------------------
			//           calculate target velocity
			// ----------------------------------------------
			
			if (rvn > -1) {
				rvn = 0; // disable bouncing
			}
			targetNormalVelocity = restitution * -rvn;
			var separationalVelocity:Number = -overlap - 0.005; // allow 0.5cm overlap
			if (separationalVelocity > 0) {
				separationalVelocity *= invTimeStep * 0.05;
				if (targetNormalVelocity < separationalVelocity) {
					targetNormalVelocity = separationalVelocity;
				}
			}
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			var error:Number;
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
			
			rvn = 
				(lVel2.x - lVel1.x) * norX + (lVel2.y - lVel1.y) * norY + (lVel2.z - lVel1.z) * norZ +
				aVel2.x * norTorque2X + aVel2.y * norTorque2Y + aVel2.z * norTorque2Z -
				aVel1.x * norTorque1X - aVel1.y * norTorque1Y - aVel1.z * norTorque1Z
			;
			oldImpulse1 = normalImpulse;
			newImpulse1 = (rvn - targetNormalVelocity) * normalDenominator * 1.4; // SOR
			normalImpulse += newImpulse1;
			if (normalImpulse > 0) normalImpulse = 0;
			newImpulse1 = normalImpulse - oldImpulse1;
			forceX = norX * newImpulse1;
			forceY = norY * newImpulse1;
			forceZ = norZ * newImpulse1;
			lVel1.x += forceX * invM1;
			lVel1.y += forceY * invM1;
			lVel1.z += forceZ * invM1;
			aVel1.x += norTorqueUnit1X * newImpulse1;
			aVel1.y += norTorqueUnit1Y * newImpulse1;
			aVel1.z += norTorqueUnit1Z * newImpulse1;
			lVel2.x -= forceX * invM2;
			lVel2.y -= forceY * invM2;
			lVel2.z -= forceZ * invM2;
			aVel2.x -= norTorqueUnit2X * newImpulse1;
			aVel2.y -= norTorqueUnit2Y * newImpulse1;
			aVel2.z -= norTorqueUnit2Z * newImpulse1;
			
			// friction part
			
			var max:Number = -normalImpulse * friction;
			relVelX = lVel2.x - lVel1.x;
			relVelY = lVel2.y - lVel1.y;
			relVelZ = lVel2.z - lVel1.z;
			rvn =
				relVelX * tanX + relVelY * tanY + relVelZ * tanZ +
				aVel2.x * tanTorque2X + aVel2.y * tanTorque2Y + aVel2.z * tanTorque2Z -
				aVel1.x * tanTorque1X - aVel1.y * tanTorque1Y - aVel1.z * tanTorque1Z
			;
			oldImpulse1 = tangentImpulse;
			newImpulse1 = rvn * tangentDenominator;
			tangentImpulse += newImpulse1;
			
			rvn =
				relVelX * binX + relVelY * binY + relVelZ * binZ +
				aVel2.x * binTorque2X + aVel2.y * binTorque2Y + aVel2.z * binTorque2Z -
				aVel1.x * binTorque1X - aVel1.y * binTorque1Y - aVel1.z * binTorque1Z
			;
			oldImpulse2 = binormalImpulse;
			newImpulse2 = rvn * binormalDenominator;
			binormalImpulse += newImpulse2;
			
			// cone friction clamp
			var len:Number = tangentImpulse * tangentImpulse + binormalImpulse * binormalImpulse;
			if (len > max * max) {
				len = max / Math.sqrt(len);
				tangentImpulse *= len;
				binormalImpulse *= len;
			}
			
			newImpulse1 = tangentImpulse - oldImpulse1;
			newImpulse2 = binormalImpulse - oldImpulse2;
			
			forceX = tanX * newImpulse1 + binX * newImpulse2;
			forceY = tanY * newImpulse1 + binY * newImpulse2;
			forceZ = tanZ * newImpulse1 + binZ * newImpulse2;
			lVel1.x += forceX * invM1;
			lVel1.y += forceY * invM1;
			lVel1.z += forceZ * invM1;
			aVel1.x += tanTorqueUnit1X * newImpulse1 + binTorqueUnit1X * newImpulse2;
			aVel1.y += tanTorqueUnit1Y * newImpulse1 + binTorqueUnit1Y * newImpulse2;
			aVel1.z += tanTorqueUnit1Z * newImpulse1 + binTorqueUnit1Z * newImpulse2;
			lVel2.x -= forceX * invM2;
			lVel2.y -= forceY * invM2;
			lVel2.z -= forceZ * invM2;
			aVel2.x -= tanTorqueUnit2X * newImpulse1 + binTorqueUnit2X * newImpulse2;
			aVel2.y -= tanTorqueUnit2Y * newImpulse1 + binTorqueUnit2Y * newImpulse2;
			aVel2.z -= tanTorqueUnit2Z * newImpulse1 + binTorqueUnit2Z * newImpulse2;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function postSolve():void {
			relativePosition1.x = relPos1X;
			relativePosition1.y = relPos1Y;
			relativePosition1.z = relPos1Z;
			relativePosition2.x = relPos2X;
			relativePosition2.y = relPos2Y;
			relativePosition2.z = relPos2Z;
			normal.x = norX;
			normal.y = norY;
			normal.z = norZ;
			tangent.x = tanX;
			tangent.y = tanY;
			tangent.z = tanZ;
			binormal.x = binX;
			binormal.y = binY;
			binormal.z = binZ;
		}
		
	}

}