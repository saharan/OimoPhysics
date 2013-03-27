package com.element.oimo.physics.collision.broad {
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.constraint.joint.Joint;
	import com.element.oimo.physics.constraint.joint.JointConnection;
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
		 * プロキシが重なった形状のペアを検索した回数です。
		 * アルゴリズムが総当りの場合、この数は形状の数を n とすると n * (n - 1) / 2 と表せます。
		 */
		public var numPairChecks:uint;
		
		private var bufferSize:uint;
		
		/**
		 * 新しく BroadPhase オブジェクトを作成します。
		 * <strong>このコンストラクタは外部から呼び出さないでください。</strong>
		 */
		public function BroadPhase() {
			bufferSize = 256;
			pairs = new Vector.<Pair>(bufferSize, true);
			for (var i:int = 0; i < bufferSize; i++) {
				pairs[i] = new Pair();
			}
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
		 * 指定された形状で構成されるペアが有効であるかどうかを判断します。
		 * @param	s1 形状1
		 * @param	s2 形状2
		 * @return ペアが有効なら true
		 */
		public function isAvailablePair(s1:Shape, s2:Shape):Boolean {
			var b1:RigidBody = s1.parent;
			var b2:RigidBody = s2.parent;
			if (
				b1 == b2 || // same parents
				(b1.type == RigidBody.BODY_STATIC || b1.sleeping) &&
				(b2.type == RigidBody.BODY_STATIC || b2.sleeping) // static (or sleeping) objects
			) {
				return false;
			}
			var jc:JointConnection;
			if (b1.numJoints < b2.numJoints) jc = b1.jointList;
			else jc = b2.jointList;
			while (jc != null) {
				var joint:Joint = jc.parent;
				if (
					!joint.allowCollision &&
					(joint.body1 == b1 && joint.body2 == b2 ||
					joint.body1 == b2 && joint.body2 == b1)
				) {
					return false;
				}
				jc = jc.next;
			}
			return true;
		}
		
		/**
		 * 衝突の可能性がある形状のペアを計算します。
		 */
		public function detectPairs():void {
			while (numPairs > 0) {
				var pair:Pair = pairs[--numPairs];
				pair.shape1 = null;
				pair.shape2 = null;
			}
			collectPairs();
		}
		
		protected function collectPairs():void {
			throw new Error("collectPairs 関数が継承されていません");
		}
		
		protected function addPair(s1:Shape, s2:Shape):void {
			if (numPairs == bufferSize) { // expand pair buffer
				var newBufferSize:uint = bufferSize << 1;
				var newPairs:Vector.<Pair> = new Vector.<Pair>(newBufferSize, true);
				for (var i:int = 0; i < bufferSize; i++) {
					newPairs[i] = pairs[i];
				}
				for (i = bufferSize; i < newBufferSize; i++) {
					newPairs[i] = new Pair();
				}
				pairs = newPairs;
				bufferSize = newBufferSize;
			}
			var pair:Pair = pairs[numPairs++];
			pair.shape1 = s1;
			pair.shape2 = s2;
		}
		
	}

}