package com.element.oimo.physics.collision.broad {
	import com.element.oimo.physics.constraint.joint.Joint;
	import com.element.oimo.physics.dynamics.RigidBody;
	/**
	 * 広域衝突判定を行うクラスです。
	 * 広域衝突判定では、詳細な衝突判定の回数を削減するため、
	 * 詳細な形状の代わりに、近似された単純な形を用いて計算されます。
	 * 広域衝突判定の後、衝突の可能性がある形状のペアのみに、詳細な衝突判定が行われます。
	 * @author saharan
	 */
	public class BroadPhase {
		/**
		 * プロキシが重なった形状のペアを検索した回数です。
		 * アルゴリズムが総当りの場合、この数は形状の数を n とすると n * (n - 1) / 2 と表せます。
		 */
		public var numPairChecks:uint;
		/**
		 * 新しく BroadPhase オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function BroadPhase() {
		}
		
		/**
		 * 判定対象のプロキシを追加します。
		 * @param	proxy 追加するプロキシ
		 */
		public function addProxy(proxy:Proxy):void {
			throw new Error("addProxy 関数が継承されていません");
		}
		
		/**
		 * 判定対象のプロキシを削除します。
		 * @param	proxy 削除するプロキシ
		 */
		public function removeProxy(proxy:Proxy):void {
			throw new Error("removeProxy 関数が継承されていません");
		}
		
		/**
		 * 衝突の可能性がある形状のペアを計算します。
		 * @param	pairs 形状のペアの格納先
		 * @return 形状のペアの数
		 */
		public function detectPairs(pairs:Vector.<Pair>):uint {
			throw new Error("collectPairs 関数が継承されていません");
		}
		
		/**
		 * 指定された剛体で構成されるペアが有効であるかどうかを判断します。
		 * @param	b1 剛体1
		 * @param	b2 剛体2
		 * @return ペアが有効なら true
		 */
		public function isAvailablePair(b1:RigidBody, b2:RigidBody):Boolean {
			if (b1.type == RigidBody.BODY_STATIC && b2.type == RigidBody.BODY_STATIC) {
				return false;
			}
			var joints:Vector.<Joint>;
			var numJoints:uint;
			if (b1.numJoints < b2.numJoints) {
				joints = b1.joints;
				numJoints = b1.numJoints;
			} else {
				joints = b2.joints;
				numJoints = b2.numJoints;
			}
			for (var i:int = 0; i < numJoints; i++) {
				var joint:Joint = joints[i];
				if (
					!joint.allowCollide &&
					(joint.rigid1 == b1 && joint.rigid2 == b2 ||
					joint.rigid1 == b2 && joint.rigid2 == b1)
				) {
					return false;
				}
			}
			return true;
		}
		
	}

}