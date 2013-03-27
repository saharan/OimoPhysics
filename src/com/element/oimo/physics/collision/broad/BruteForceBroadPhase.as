package com.element.oimo.physics.collision.broad {
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	import flash.utils.getTimer;
	/**
	 * 総当りアルゴリズムを使用して広域衝突判定を行うクラスです。
	 * <strong>このアルゴリズムは速度検証以外には非推奨です。</strong>
	 * 総当り判定は形状の数に対し、常に O(n^2) の計算量を要求するため、
	 * 形状の増え方に比べ、負荷の増え方が非常に高くなります。
	 * @author saharan
	 */
	public class BruteForceBroadPhase extends BroadPhase {
		private var proxyPool:Vector.<Proxy>;
		private var numProxies:uint;
		
		/**
		 * 新しく BruteForceBroadPhase オブジェクトを作成します。
		 */
		public function BruteForceBroadPhase() {
			proxyPool = new Vector.<Proxy>(World.MAX_SHAPES, true);
		}
		
		/**
		 * @inheritDoc
		 */
		override public function addProxy(proxy:Proxy):void {
			proxyPool[numProxies] = proxy;
			numProxies++;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function removeProxy(proxy:Proxy):void {
			var idx:int = -1;
			for (var i:int = 0; i < numProxies; i++) {
				if (proxyPool[i] == proxy) {
					idx = i;
					break;
				}
			}
			if (idx == -1) {
				return;
			}
			for (var j:int = idx; j < numProxies - 1; j++) {
				proxyPool[j] = proxyPool[j + 1];
			}
			proxyPool[numProxies] = null;
			numProxies--;
		}
		
		override protected function collectPairs():void {
			numPairChecks = numProxies * (numProxies - 1) >> 1;
			for (var i:int = 0; i < numProxies; i++) {
				var p1:Proxy = proxyPool[i];
				var s1:Shape = p1.parent;
				for (var j:int = i + 1; j < numProxies; j++) {
					var p2:Proxy = proxyPool[j];
					var s2:Shape = p2.parent;
					if (
						p1.maxX < p2.minX || p1.minX > p2.maxX ||
						p1.maxY < p2.minY || p1.minY > p2.maxY ||
						p1.maxZ < p2.minZ || p1.minZ > p2.maxZ ||
						!isAvailablePair(s1, s2)
					) {
						continue;
					}
					addPair(s1, s2);
				}
			}
		}
		
	}

}