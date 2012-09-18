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
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.math.Mat33;
	import com.element.oimo.math.Quat;
	import com.element.oimo.math.Vec3;
	/**
	 * 剛体のクラスです。
	 * 剛体は衝突処理用に単数あるいは複数の形状を持ち、
	 * それぞれ個別にパラメーターを設定できます。
	 * @author saharan
	 */
	public class RigidBody {
		/**
		 * 動的な剛体を表す剛体の種類です。
		 */
		public static const BODY_DYNAMIC:uint = 0x0;
		
		/**
		 * 静的な剛体を表す剛体の種類です。
		 */
		public static const BODY_STATIC:uint = 0x1;
		
		/**
		 * 一つの剛体に追加できる形状の最大数です。
		 */
		public static const MAX_SHAPES:uint = 64;
		
		/**
		 * 剛体の種類を表します。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 剛体の種類を変更する場合は、必ず
		 * setupMass メソッドの引数に設定したい種類を指定してください。
		 */
		public var type:uint;
		
		/**
		 * 重心のワールド座標です。
		 */
		public var position:Vec3;
		
		/**
		 * 並進速度です。
		 */
		public var linearVelocity:Vec3;
		
		/**
		 * 姿勢を表すクォータニオンです。
		 */
		public var orientation:Quat;
		
		/**
		 * 姿勢を表す回転行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 回転行列は、ステップ毎にクォータニオンから再計算されます。
		 */
		public var rotation:Mat33;
		
		/**
		 * 角速度です。
		 */
		public var angularVelocity:Vec3;
		
		/**
		 * 質量です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 質量は setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
		public var mass:Number;
		
		/**
		 * 質量の逆数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 質量は setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
		public var invertMass:Number;
		
		/**
		 * ワールド系での慣性テンソルの逆行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * ワールド系での慣性テンソルの逆行列は、ステップ毎に
		 * 姿勢と初期状態の慣性テンソルの逆数から再計算されます。
		 */
		public var invertInertia:Mat33;
		
		/**
		 * 初期状態での慣性テンソルです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 慣性テンソルは setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
		public var localInertia:Mat33;
		
		/**
		 * 初期状態での慣性テンソルの逆行列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 * 
		 * 慣性テンソルは setupMass メソッドを呼び出すと、
		 * 含まれている形状から自動で再計算されます。
		 */
		public var invertLocalInertia:Mat33;
		
		/**
		 * 剛体に含まれている形状の配列です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var shapes:Vector.<Shape>;
		
		/**
		 * 剛体に含まれている形状の数です。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var numShapes:uint;
		
		/**
		 * この剛体が追加されているワールドです。
		 * <strong>この変数は外部から変更しないでください。</strong>
		 */
		public var parent:World;
		
		/**
		 * 新しく RigidBody オブジェクトを作成します。
		 */
		public function RigidBody() {
			position = new Vec3();
			linearVelocity = new Vec3();
			orientation = new Quat();
			rotation = new Mat33();
			angularVelocity = new Vec3();
			invertInertia = new Mat33();
			localInertia = new Mat33();
			invertLocalInertia = new Mat33();
			shapes = new Vector.<Shape>(MAX_SHAPES, true);
		}
		
		/**
		 * 剛体に形状を追加します。
		 * 形状を追加した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
		 * @param	shape 追加する形状
		 */
		public function addShape(shape:Shape):void {
			if (numShapes == MAX_SHAPES) {
				throw new Error("これ以上剛体に形状を追加することはできません");
			}
			if (shape.parent) {
				throw new Error("一つの形状を複数剛体に追加することはできません");
			}
			shapes[numShapes++] = shape;
			shape.parent = this;
			if (parent) {
				parent.addShape(shape);
			}
		}
		
		/**
		 * 剛体から形状を削除します。
		 * 削除する形状のインデックスを指定した場合は、インデックスのみを使用して削除します。
		 * 形状を削除した場合、次のステップ開始までに setupMass メソッドを呼び出してください。
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
			remove.parent = null;
			if (parent) {
				parent.removeShape(remove);
			}
			for (var j:int = index; j < numShapes - 1; j++) {
				shapes[j] = shapes[j + 1];
			}
			shapes[--numShapes] = null;
		}
		
		/**
		 * 重心座標・質量・慣性テンソルなどを算出します。
		 * type に BODY_STATIC を指定すると、この剛体は空間に対して固定されます。
		 * @param	type 剛体の種類
		 */
		public function setupMass(type:uint = BODY_DYNAMIC):void {
			this.type = type;
			position.init();
			mass = 0;
			localInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
			var invRot:Mat33 = new Mat33(); // = rotation ^ -1
			invRot.transpose(rotation);
			var tmpM:Mat33 = new Mat33();
			var tmpV:Vec3 = new Vec3();
			var denom:Number = 0;
			for (var i:int = 0; i < numShapes; i++) {
				var shape:Shape = shapes[i];
				// relativeRotation = (rotation ^ -1) * shape.rotation
				shape.relativeRotation.mul(invRot, shape.rotation);
				position.add(position, tmpV.scale(shape.position, shape.mass));
				denom += shape.mass;
				mass += shape.mass;
				// inertia = (rotation ^ -1) * localInertia * rotation
				tmpM.mul(tmpM.mul(tmpM.transpose(shape.relativeRotation), shape.localInertia), shape.relativeRotation);
				localInertia.add(localInertia, tmpM);
			}
			position.scale(position, 1 / denom);
			invertMass = 1 / mass;
			var xy:Number = 0;
			var yz:Number = 0;
			var zx:Number = 0;
			for (var j:int = 0; j < numShapes; j++) {
				shape = shapes[j];
				var relPos:Vec3 = shape.localRelativePosition;
				relPos.sub(shape.position, position).mulMat(invRot, relPos);
				localInertia.e00 += shape.mass * (relPos.y * relPos.y + relPos.z * relPos.z);
				localInertia.e11 += shape.mass * (relPos.x * relPos.x + relPos.z * relPos.z);
				localInertia.e22 += shape.mass * (relPos.x * relPos.x + relPos.y * relPos.y);
				xy -= shape.mass * relPos.x * relPos.y;
				yz -= shape.mass * relPos.y * relPos.z;
				zx -= shape.mass * relPos.z * relPos.x;
			}
			localInertia.e01 = xy;
			localInertia.e10 = xy;
			localInertia.e02 = zx;
			localInertia.e20 = zx;
			localInertia.e12 = yz;
			localInertia.e21 = yz;
			invertLocalInertia.invert(localInertia);
			if (type == BODY_STATIC) {
				invertMass = 0;
				invertLocalInertia.init(0, 0, 0, 0, 0, 0, 0, 0, 0);
			}
		}
		
		/**
		 * 剛体の運動を時間積分し、形状などの情報を更新します。
		 * このメソッドはワールドのステップを呼ぶと自動で呼び出されるので、
		 * 通常は外部から呼ぶ必要はありません。
		 * @param	timeStep
		 * @param	gravity
		 */
		public function update(timeStep:Number, gravity:Vec3):void {
			if (type == BODY_STATIC) {
				linearVelocity.init();
				angularVelocity.init();
			} else if (type == BODY_DYNAMIC) {
				position.x += linearVelocity.x * timeStep;
				position.y += linearVelocity.y * timeStep;
				position.z += linearVelocity.z * timeStep;
				linearVelocity.x += gravity.x * timeStep;
				linearVelocity.y += gravity.y * timeStep;
				linearVelocity.z += gravity.z * timeStep;
				var dq:Quat = new Quat(0, angularVelocity.x, angularVelocity.y, angularVelocity.z);
				orientation.add(orientation, dq.mul(dq, orientation).scale(dq, timeStep * 0.5)).normalize(orientation);
			} else {
				throw new Error("未定義の剛体の種類です");
			}
			rotation.setQuat(orientation);
			invertInertia.mul(invertInertia.mul(invertInertia.transpose(rotation), invertLocalInertia), rotation);
			for (var i:int = 0; i < numShapes; i++) {
				var shape:Shape = shapes[i];
				shape.position.add(position, shape.relativePosition.mulMat(rotation, shape.localRelativePosition));
				shape.rotation.mul(rotation, shape.relativeRotation);
				shape.updateProxy();
			}
		}
		
		public function applyImpulse(position:Vec3, force:Vec3):void {
			linearVelocity.x += force.x * invertMass;
			linearVelocity.y += force.y * invertMass;
			linearVelocity.z += force.z * invertMass;
			var rel:Vec3 = new Vec3();
			rel.sub(position, this.position).cross(rel, force).mulMat(invertInertia, rel);
			angularVelocity.x += rel.x;
			angularVelocity.y += rel.y;
			angularVelocity.z += rel.z;
		}
		
	}

}