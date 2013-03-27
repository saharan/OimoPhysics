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
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * 剛体間の二点の距離を拘束するジョイントです。
	 * @author saharan
	 */
	public class DistanceJoint extends Joint {
		/**
		 * 保とうとする剛体間の距離です。
		 */
		public var distance:Number;
		
		/**
		 * 拘束力の大きさです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var impulse:Number;
		
		private var posError:Number;
		
		private var lVel1:Vec3;
		private var lVel2:Vec3;
		
		private var aVel1:Vec3;
		private var aVel2:Vec3;
		
		private var norX:Number;
		private var norY:Number;
		private var norZ:Number;
		
		private var relPos1X:Number;
		private var relPos1Y:Number;
		private var relPos1Z:Number;
		private var relPos2X:Number;
		private var relPos2Y:Number;
		private var relPos2Z:Number;
		
		private var norTorque1X:Number;
		private var norTorque1Y:Number;
		private var norTorque1Z:Number;
		private var norTorque2X:Number;
		private var norTorque2Y:Number;
		private var norTorque2Z:Number;
		
		private var norTorqueUnit1X:Number;
		private var norTorqueUnit1Y:Number;
		private var norTorqueUnit1Z:Number;
		private var norTorqueUnit2X:Number;
		private var norTorqueUnit2Y:Number;
		private var norTorqueUnit2Z:Number;
		
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
		
		private var targetNormalVelocity:Number;
		
		/**
		 * 新しい DistanceJoint オブジェクトを作成します。
		 * @param	rigid1 剛体1
		 * @param	rigid2 剛体2
		 * @param	distance 保とうとする距離
		 * @param	config ジョイントの設定
		 */
		public function DistanceJoint(rigid1:RigidBody, rigid2:RigidBody, distance:Number, config:JointConfig) {
			this.body1 = rigid1;
			this.body2 = rigid2;
			connection1.connected = rigid2;
			connection2.connected = rigid1;
			this.distance = distance;
			allowCollision = config.allowCollision;
			localRelativeAnchorPosition1.copy(config.localRelativeAnchorPosition1);
			localRelativeAnchorPosition2.copy(config.localRelativeAnchorPosition2);
			type = JOINT_DISTANCE;
			
			lVel1 = this.body1.linearVelocity;
			lVel2 = this.body2.linearVelocity;
			aVel1 = this.body1.angularVelocity;
			aVel2 = this.body2.angularVelocity;
			
			invM1 = this.body1.invertMass;
			invM2 = this.body2.invertMass;
			
			impulse = 0;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function preSolve(timeStep:Number, invTimeStep:Number):void {
			var tmpM:Mat33;
			var tmp1X:Number;
			var tmp1Y:Number;
			var tmp1Z:Number;
			var tmp2X:Number;
			var tmp2Y:Number;
			var tmp2Z:Number;
			
			// ----------------------------------------------
			//              calculate positions
			// ----------------------------------------------
			
			tmpM = body1.rotation;
			tmp1X = localRelativeAnchorPosition1.x;
			tmp1Y = localRelativeAnchorPosition1.y;
			tmp1Z = localRelativeAnchorPosition1.z;
			relPos1X = relativeAnchorPosition1.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
			relPos1Y = relativeAnchorPosition1.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
			relPos1Z = relativeAnchorPosition1.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
			tmpM = body2.rotation;
			tmp1X = localRelativeAnchorPosition2.x;
			tmp1Y = localRelativeAnchorPosition2.y;
			tmp1Z = localRelativeAnchorPosition2.z;
			relPos2X = relativeAnchorPosition2.x = tmp1X * tmpM.e00 + tmp1Y * tmpM.e01 + tmp1Z * tmpM.e02;
			relPos2Y = relativeAnchorPosition2.y = tmp1X * tmpM.e10 + tmp1Y * tmpM.e11 + tmp1Z * tmpM.e12;
			relPos2Z = relativeAnchorPosition2.z = tmp1X * tmpM.e20 + tmp1Y * tmpM.e21 + tmp1Z * tmpM.e22;
			
			// ----------------------------------------------
			//               calculate normal
			// ----------------------------------------------
			
			norX = (anchorPosition2.x = relPos2X + body2.position.x) - (anchorPosition1.x = relPos1X + body1.position.x);
			norY = (anchorPosition2.y = relPos2Y + body2.position.y) - (anchorPosition1.y = relPos1Y + body1.position.y);
			norZ = (anchorPosition2.z = relPos2Z + body2.position.z) - (anchorPosition1.z = relPos1Z + body1.position.z);
			tmp1X = Math.sqrt(norX * norX + norY * norY + norZ * norZ);
			posError = distance - tmp1X;
			if (tmp1X > 0) tmp1X = 1 / tmp1X;
			norX *= tmp1X;
			norY *= tmp1X;
			norZ *= tmp1X;
			
			// ----------------------------------------------
			// calculate torque axes and angular accelerations
			// ----------------------------------------------
			
			tmpM = body1.invertInertia;
			invI1e00 = tmpM.e00;
			invI1e01 = tmpM.e01;
			invI1e02 = tmpM.e02;
			invI1e10 = tmpM.e10;
			invI1e11 = tmpM.e11;
			invI1e12 = tmpM.e12;
			invI1e20 = tmpM.e20;
			invI1e21 = tmpM.e21;
			invI1e22 = tmpM.e22;
			tmpM = body2.invertInertia;
			invI2e00 = tmpM.e00;
			invI2e01 = tmpM.e01;
			invI2e02 = tmpM.e02;
			invI2e10 = tmpM.e10;
			invI2e11 = tmpM.e11;
			invI2e12 = tmpM.e12;
			invI2e20 = tmpM.e20;
			invI2e21 = tmpM.e21;
			invI2e22 = tmpM.e22;
			
			norTorque1X = relPos1Y * norZ - relPos1Z * norY;
			norTorque1Y = relPos1Z * norX - relPos1X * norZ;
			norTorque1Z = relPos1X * norY - relPos1Y * norX;
			norTorque2X = relPos2Y * norZ - relPos2Z * norY;
			norTorque2Y = relPos2Z * norX - relPos2X * norZ;
			norTorque2Z = relPos2X * norY - relPos2Y * norX;
			
			norTorqueUnit1X = norTorque1X * invI1e00 + norTorque1Y * invI1e01 + norTorque1Z * invI1e02;
			norTorqueUnit1Y = norTorque1X * invI1e10 + norTorque1Y * invI1e11 + norTorque1Z * invI1e12;
			norTorqueUnit1Z = norTorque1X * invI1e20 + norTorque1Y * invI1e21 + norTorque1Z * invI1e22;
			norTorqueUnit2X = norTorque2X * invI2e00 + norTorque2Y * invI2e01 + norTorque2Z * invI2e02;
			norTorqueUnit2Y = norTorque2X * invI2e10 + norTorque2Y * invI2e11 + norTorque2Z * invI2e12;
			norTorqueUnit2Z = norTorque2X * invI2e20 + norTorque2Y * invI2e21 + norTorque2Z * invI2e22;
			
			// ----------------------------------------------
			//         calculate impulse denominator
			// ----------------------------------------------
			
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
			
			// ----------------------------------------------
			//           calculate initial forces
			// ----------------------------------------------
			
			tmp1X = norX * impulse;
			tmp1Y = norY * impulse;
			tmp1Z = norZ * impulse;
			lVel1.x += tmp1X * invM1;
			lVel1.y += tmp1Y * invM1;
			lVel1.z += tmp1Z * invM1;
			aVel1.x += norTorqueUnit1X * impulse;
			aVel1.y += norTorqueUnit1Y * impulse;
			aVel1.z += norTorqueUnit1Z * impulse;
			lVel2.x -= tmp1X * invM2;
			lVel2.y -= tmp1Y * invM2;
			lVel2.z -= tmp1Z * invM2;
			aVel2.x -= norTorqueUnit2X * impulse;
			aVel2.y -= norTorqueUnit2Y * impulse;
			aVel2.z -= norTorqueUnit2Z * impulse;
			
			// ----------------------------------------------
			//           calculate target velocity
			// ----------------------------------------------
			
			if (posError < -0.005) posError += 0.005; // allow 0.5cm error
			else if (posError > 0.005) posError -= 0.005;
			else posError = 0;
			targetNormalVelocity = posError * invTimeStep * 0.05;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			var newImpulse:Number;
			var rvn:Number;
			var forceX:Number;
			var forceY:Number;
			var forceZ:Number;
			var tmpX:Number;
			var tmpY:Number;
			var tmpZ:Number;
			rvn = 
				(lVel2.x - lVel1.x) * norX + (lVel2.y - lVel1.y) * norY + (lVel2.z - lVel1.z) * norZ +
				aVel2.x * norTorque2X + aVel2.y * norTorque2Y + aVel2.z * norTorque2Z -
				aVel1.x * norTorque1X - aVel1.y * norTorque1Y - aVel1.z * norTorque1Z
			;
			newImpulse = (rvn - targetNormalVelocity) * normalDenominator;
			impulse += newImpulse;
			forceX = norX * newImpulse;
			forceY = norY * newImpulse;
			forceZ = norZ * newImpulse;
			lVel1.x += forceX * invM1;
			lVel1.y += forceY * invM1;
			lVel1.z += forceZ * invM1;
			aVel1.x += norTorqueUnit1X * newImpulse;
			aVel1.y += norTorqueUnit1Y * newImpulse;
			aVel1.z += norTorqueUnit1Z * newImpulse;
			lVel2.x -= forceX * invM2;
			lVel2.y -= forceY * invM2;
			lVel2.z -= forceZ * invM2;
			aVel2.x -= norTorqueUnit2X * newImpulse;
			aVel2.y -= norTorqueUnit2Y * newImpulse;
			aVel2.z -= norTorqueUnit2Z * newImpulse;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function postSolve():void {
		}
		
	}

}