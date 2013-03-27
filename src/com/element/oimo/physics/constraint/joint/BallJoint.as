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
	 * 剛体間の一点を拘束するジョイントです。
	 * @author saharan
	 */
	public class BallJoint extends Joint {
		/**
		 * 拘束力のベクトルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * この値は蓄積され、次ステップでも使い回されます。
		 */
		public var impulse:Vec3;
		
		private var impulseX:Number;
		private var impulseY:Number;
		private var impulseZ:Number;
		
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
		
		private var xTorqueUnit1X:Number;
		private var xTorqueUnit1Y:Number;
		private var xTorqueUnit1Z:Number;
		private var xTorqueUnit2X:Number;
		private var xTorqueUnit2Y:Number;
		private var xTorqueUnit2Z:Number;
		
		private var yTorqueUnit1X:Number;
		private var yTorqueUnit1Y:Number;
		private var yTorqueUnit1Z:Number;
		private var yTorqueUnit2X:Number;
		private var yTorqueUnit2Y:Number;
		private var yTorqueUnit2Z:Number;
		
		private var zTorqueUnit1X:Number;
		private var zTorqueUnit1Y:Number;
		private var zTorqueUnit1Z:Number;
		private var zTorqueUnit2X:Number;
		private var zTorqueUnit2Y:Number;
		private var zTorqueUnit2Z:Number;
		
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
		
		// denominator matrix (Point-To-Point mass matrix)
		private var d00:Number;
		private var d01:Number;
		private var d02:Number;
		private var d10:Number;
		private var d11:Number;
		private var d12:Number;
		private var d20:Number;
		private var d21:Number;
		private var d22:Number;
		
		private var targetVelX:Number;
		private var targetVelY:Number;
		private var targetVelZ:Number;
		
		/**
		 * 新しい BallJoint オブジェクトを作成します。
		 * @param	rigid1 剛体1
		 * @param	rigid2 剛体2
		 * @param	config ジョイントの設定
		 */
		public function BallJoint(rigid1:RigidBody, rigid2:RigidBody, config:JointConfig) {
			this.body1 = rigid1;
			this.body2 = rigid2;
			connection1.connected = rigid2;
			connection2.connected = rigid1;
			allowCollision = config.allowCollision;
			localRelativeAnchorPosition1.copy(config.localRelativeAnchorPosition1);
			localRelativeAnchorPosition2.copy(config.localRelativeAnchorPosition2);
			type = JOINT_BALL;
			
			lVel1 = this.body1.linearVelocity;
			lVel2 = this.body2.linearVelocity;
			aVel1 = this.body1.angularVelocity;
			aVel2 = this.body2.angularVelocity;
			
			invM1 = this.body1.invertMass;
			invM2 = this.body2.invertMass;
			
			impulse = new Vec3();
			impulseX = 0;
			impulseY = 0;
			impulseZ = 0;
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
			var t00:Number;
			var t01:Number;
			var t02:Number;
			var t10:Number;
			var t11:Number;
			var t12:Number;
			var t20:Number;
			var t21:Number;
			var t22:Number;
			var u00:Number;
			var u01:Number;
			var u02:Number;
			var u10:Number;
			var u11:Number;
			var u12:Number;
			var u20:Number;
			var u21:Number;
			var u22:Number;
			
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
			anchorPosition1.x = relPos1X + body1.position.x;
			anchorPosition1.y = relPos1Y + body1.position.y;
			anchorPosition1.z = relPos1Z + body1.position.z;
			anchorPosition2.x = relPos2X + body2.position.x;
			anchorPosition2.y = relPos2Y + body2.position.y;
			anchorPosition2.z = relPos2Z + body2.position.z;
			
			// ----------------------------------------------
			//        calculate angular accelerations
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
			
			xTorqueUnit1X = relPos1Z * invI1e01 - relPos1Y * invI1e02;
			xTorqueUnit1Y = relPos1Z * invI1e11 - relPos1Y * invI1e12;
			xTorqueUnit1Z = relPos1Z * invI1e21 - relPos1Y * invI1e22;
			xTorqueUnit2X = relPos2Z * invI2e01 - relPos2Y * invI2e02;
			xTorqueUnit2Y = relPos2Z * invI2e11 - relPos2Y * invI2e12;
			xTorqueUnit2Z = relPos2Z * invI2e21 - relPos2Y * invI2e22;
			
			yTorqueUnit1X = -relPos1Z * invI1e00 + relPos1X * invI1e02;
			yTorqueUnit1Y = -relPos1Z * invI1e10 + relPos1X * invI1e12;
			yTorqueUnit1Z = -relPos1Z * invI1e20 + relPos1X * invI1e22;
			yTorqueUnit2X = -relPos2Z * invI2e00 + relPos2X * invI2e02;
			yTorqueUnit2Y = -relPos2Z * invI2e10 + relPos2X * invI2e12;
			yTorqueUnit2Z = -relPos2Z * invI2e20 + relPos2X * invI2e22;
			
			zTorqueUnit1X = relPos1Y * invI1e00 - relPos1X * invI1e01;
			zTorqueUnit1Y = relPos1Y * invI1e10 - relPos1X * invI1e11;
			zTorqueUnit1Z = relPos1Y * invI1e20 - relPos1X * invI1e21;
			zTorqueUnit2X = relPos2Y * invI2e00 - relPos2X * invI2e01;
			zTorqueUnit2Y = relPos2Y * invI2e10 - relPos2X * invI2e11;
			zTorqueUnit2Z = relPos2Y * invI2e20 - relPos2X * invI2e21;
			
			// ----------------------------------------------
			//         calculate impulse denominator
			// ----------------------------------------------
			
			// calculate Point-To-Point mass matrix
			// from impulse equation
			// 
			// M = ([/m] - [r^][/I][r^]) ^ -1
			// 
			// where:
			// 
			// [/m] = |1/m, 0, 0|
			//        |0, 1/m, 0|
			//        |0, 0, 1/m|
			// 
			// [r^] = |0, -rz, ry|
			//        |rz, 0, -rx|
			//        |-ry, rx, 0|
			// 
			// [/I] = Inverted moment inertia
			
			d00 = invM1 + invM2;
			d01 = 0;
			d02 = 0;
			d10 = 0;
			d11 = d00;
			d12 = 0;
			d20 = 0;
			d21 = 0;
			d22 = d00;
			t01 = -relPos1Z;
			t02 = relPos1Y;
			t10 = relPos1Z;
			t12 = -relPos1X;
			t20 = -relPos1Y;
			t21 = relPos1X;
			u00 = invI1e01 * t10 + invI1e02 * t20;
			u01 = invI1e00 * t01 + invI1e02 * t21;
			u02 = invI1e00 * t02 + invI1e01 * t12;
			u10 = invI1e11 * t10 + invI1e12 * t20;
			u11 = invI1e10 * t01 + invI1e12 * t21;
			u12 = invI1e10 * t02 + invI1e11 * t12;
			u20 = invI1e21 * t10 + invI1e22 * t20;
			u21 = invI1e20 * t01 + invI1e22 * t21;
			u22 = invI1e20 * t02 + invI1e21 * t12;
			d00 -= t01 * u10 + t02 * u20;
			d01 -= t01 * u11 + t02 * u21;
			d02 -= t01 * u12 + t02 * u22;
			d10 -= t10 * u00 + t12 * u20;
			d11 -= t10 * u01 + t12 * u21;
			d12 -= t10 * u02 + t12 * u22;
			d20 -= t20 * u00 + t21 * u10;
			d21 -= t20 * u01 + t21 * u11;
			d22 -= t20 * u02 + t21 * u12;
			t01 = -relPos2Z;
			t02 = relPos2Y;
			t10 = relPos2Z;
			t12 = -relPos2X;
			t20 = -relPos2Y;
			t21 = relPos2X;
			u00 = invI2e01 * t10 + invI2e02 * t20;
			u01 = invI2e00 * t01 + invI2e02 * t21;
			u02 = invI2e00 * t02 + invI2e01 * t12;
			u10 = invI2e11 * t10 + invI2e12 * t20;
			u11 = invI2e10 * t01 + invI2e12 * t21;
			u12 = invI2e10 * t02 + invI2e11 * t12;
			u20 = invI2e21 * t10 + invI2e22 * t20;
			u21 = invI2e20 * t01 + invI2e22 * t21;
			u22 = invI2e20 * t02 + invI2e21 * t12;
			d00 -= t01 * u10 + t02 * u20;
			d01 -= t01 * u11 + t02 * u21;
			d02 -= t01 * u12 + t02 * u22;
			d10 -= t10 * u00 + t12 * u20;
			d11 -= t10 * u01 + t12 * u21;
			d12 -= t10 * u02 + t12 * u22;
			d20 -= t20 * u00 + t21 * u10;
			d21 -= t20 * u01 + t21 * u11;
			d22 -= t20 * u02 + t21 * u12;
			tmp1X = 1 / (d00 * (d11 * d22 - d21 * d12) + d10 * (d21 * d02 - d01 * d22) + d20 * (d01 * d12 - d11 * d02));
			t00 = (d11 * d22 - d12 * d21) * tmp1X;
			t01 = (d02 * d21 - d01 * d22) * tmp1X;
			t02 = (d01 * d12 - d02 * d11) * tmp1X;
			t10 = (d12 * d20 - d10 * d22) * tmp1X;
			t11 = (d00 * d22 - d02 * d20) * tmp1X;
			t12 = (d02 * d10 - d00 * d12) * tmp1X;
			t20 = (d10 * d21 - d11 * d20) * tmp1X;
			t21 = (d01 * d20 - d00 * d21) * tmp1X;
			t22 = (d00 * d11 - d01 * d10) * tmp1X;
			d00 = t00;
			d01 = t01;
			d02 = t02;
			d10 = t10;
			d11 = t11;
			d12 = t12;
			d20 = t20;
			d21 = t21;
			d22 = t22;
			
			// ----------------------------------------------
			//           calculate initial forces
			// ----------------------------------------------
			
			lVel1.x += impulseX * invM1;
			lVel1.y += impulseY * invM1;
			lVel1.z += impulseZ * invM1;
			aVel1.x += xTorqueUnit1X * impulseX + yTorqueUnit1X * impulseY + zTorqueUnit1X * impulseZ;
			aVel1.y += xTorqueUnit1Y * impulseX + yTorqueUnit1Y * impulseY + zTorqueUnit1Y * impulseZ;
			aVel1.z += xTorqueUnit1Z * impulseX + yTorqueUnit1Z * impulseY + zTorqueUnit1Z * impulseZ;
			lVel2.x -= impulseX * invM2;
			lVel2.y -= impulseY * invM2;
			lVel2.z -= impulseZ * invM2;
			aVel2.x -= xTorqueUnit2X * impulseX + yTorqueUnit2X * impulseY + zTorqueUnit2X * impulseZ;
			aVel2.y -= xTorqueUnit2Y * impulseX + yTorqueUnit2Y * impulseY + zTorqueUnit2Y * impulseZ;
			aVel2.z -= xTorqueUnit2Z * impulseX + yTorqueUnit2Z * impulseY + zTorqueUnit2Z * impulseZ;
			
			// ----------------------------------------------
			//           calculate target velocity
			// ----------------------------------------------
			
			targetVelX = anchorPosition2.x - anchorPosition1.x;
			targetVelY = anchorPosition2.y - anchorPosition1.y;
			targetVelZ = anchorPosition2.z - anchorPosition1.z;
			tmp1X = Math.sqrt(targetVelX * targetVelX + targetVelY * targetVelY + targetVelZ * targetVelZ);
			if (tmp1X < 0.005) {
				targetVelX = 0;
				targetVelY = 0;
				targetVelZ = 0;
			} else {
				tmp1X = (0.005 - tmp1X) / tmp1X * invTimeStep * 0.05;
				targetVelX *= tmp1X;
				targetVelY *= tmp1X;
				targetVelZ *= tmp1X;
			}
		}
		
		/**
		 * @inheritDoc
		 */
		override public function solve():void {
			var relVelX:Number;
			var relVelY:Number;
			var relVelZ:Number;
			var newImpulseX:Number;
			var newImpulseY:Number;
			var newImpulseZ:Number;
			relVelX = lVel2.x - lVel1.x + aVel2.y * relPos2Z - aVel2.z * relPos2Y - aVel1.y * relPos1Z + aVel1.z * relPos1Y - targetVelX;
			relVelY = lVel2.y - lVel1.y + aVel2.z * relPos2X - aVel2.x * relPos2Z - aVel1.z * relPos1X + aVel1.x * relPos1Z - targetVelY;
			relVelZ = lVel2.z - lVel1.z + aVel2.x * relPos2Y - aVel2.y * relPos2X - aVel1.x * relPos1Y + aVel1.y * relPos1X - targetVelZ;
			newImpulseX = relVelX * d00 + relVelY * d01 + relVelZ * d02;
			newImpulseY = relVelX * d10 + relVelY * d11 + relVelZ * d12;
			newImpulseZ = relVelX * d20 + relVelY * d21 + relVelZ * d22;
			impulseX += newImpulseX;
			impulseY += newImpulseY;
			impulseZ += newImpulseZ;
			lVel1.x += newImpulseX * invM1;
			lVel1.y += newImpulseY * invM1;
			lVel1.z += newImpulseZ * invM1;
			aVel1.x += xTorqueUnit1X * newImpulseX + yTorqueUnit1X * newImpulseY + zTorqueUnit1X * newImpulseZ;
			aVel1.y += xTorqueUnit1Y * newImpulseX + yTorqueUnit1Y * newImpulseY + zTorqueUnit1Y * newImpulseZ;
			aVel1.z += xTorqueUnit1Z * newImpulseX + yTorqueUnit1Z * newImpulseY + zTorqueUnit1Z * newImpulseZ;
			lVel2.x -= newImpulseX * invM2;
			lVel2.y -= newImpulseY * invM2;
			lVel2.z -= newImpulseZ * invM2;
			aVel2.x -= xTorqueUnit2X * newImpulseX + yTorqueUnit2X * newImpulseY + zTorqueUnit2X * newImpulseZ;
			aVel2.y -= xTorqueUnit2Y * newImpulseX + yTorqueUnit2Y * newImpulseY + zTorqueUnit2Y * newImpulseZ;
			aVel2.z -= xTorqueUnit2Z * newImpulseX + yTorqueUnit2Z * newImpulseY + zTorqueUnit2Z * newImpulseZ;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function postSolve():void {
			impulse.x = impulseX;
			impulse.y = impulseY;
			impulse.z = impulseZ;
		}
		
	}

}