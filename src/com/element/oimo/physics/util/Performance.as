package com.element.oimo.physics.util {
	/**
	 * ワールドが物理演算に要した時間などを記録するクラスです。
	 * 特に表記がない場合、時間の単位はミリ秒です。
	 * @author saharan
	 */
	public class Performance {
		/**
		 * 広域衝突判定に要した時間です。
		 */
		public var broadPhaseTime:uint;
		
		/**
		 * 詳細な衝突判定に要した時間です。
		 */
		public var narrowPhaseTime:uint;
		
		/**
		 * 拘束の計算に要した時間です。
		 */
		public var constraintsTime:uint;
		
		/**
		 * 位置などの更新に要した時間です。
		 */
		public var updateTime:uint;
		
		/**
		 * ステップ計算に要した合計時間です。
		 */
		public var totalTime:uint;
		
		/**
		 * 新しく Performance オブジェクトを作成します。
		 */
		public function Performance() {
		}
		
	}

}