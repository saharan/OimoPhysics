package com.element.oimo.physics.collision.broad {
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
		
	}

}