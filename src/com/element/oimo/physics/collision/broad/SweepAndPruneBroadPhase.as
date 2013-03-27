package com.element.oimo.physics.collision.broad {
	import com.element.oimo.physics.collision.shape.Shape;
	import com.element.oimo.physics.dynamics.RigidBody;
	import com.element.oimo.physics.dynamics.World;
	import flash.utils.getTimer;
	/**
	 * Sweep And Prune アルゴリズムを使用して広域衝突判定を行うクラスです。
	 * プロキシの要素は各軸についてソートされ、
	 * 衝突の可能性がある形状のペアを効率的に計算することができます。
	 * ワールドに対し多数の形状がワープおよび高速で移動するような場面では、
	 * このアルゴリズムは好ましくない結果を生み出す可能性があります。
	 * @author saharan
	 */
	public class SweepAndPruneBroadPhase extends BroadPhase {
		private var proxyPoolAxis:Vector.<Vector.<Proxy>>;
		private var sortAxis:uint;
		private var numProxies:uint;
		
		/**
		 * 新しく SweepAndPruneBroadPhase オブジェクトを作成します。
		 */
		public function SweepAndPruneBroadPhase() {
			sortAxis = 0;
			proxyPoolAxis = new Vector.<Vector.<Proxy>>(3, true);
			proxyPoolAxis[0] = new Vector.<Proxy>(World.MAX_SHAPES, true);
			proxyPoolAxis[1] = new Vector.<Proxy>(World.MAX_SHAPES, true);
			proxyPoolAxis[2] = new Vector.<Proxy>(World.MAX_SHAPES, true);
		}
		
		/**
		 * @inheritDoc
		 */
		override public function addProxy(proxy:Proxy):void {
			proxyPoolAxis[0][numProxies] = proxy;
			proxyPoolAxis[1][numProxies] = proxy;
			proxyPoolAxis[2][numProxies] = proxy;
			numProxies++;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function removeProxy(proxy:Proxy):void {
			removeProxyAxis(proxy, proxyPoolAxis[0]);
			removeProxyAxis(proxy, proxyPoolAxis[1]);
			removeProxyAxis(proxy, proxyPoolAxis[2]);
			numProxies--;
		}
		
		override protected function collectPairs():void {
			numPairChecks = 0;
			var proxyPool:Vector.<Proxy> = proxyPoolAxis[sortAxis];
			var result:uint;
			if (sortAxis == 0) {
				insertionSortX(proxyPool);
				sweepX(proxyPool);
			} else if (sortAxis == 1) {
				insertionSortY(proxyPool);
				sweepY(proxyPool);
			} else {
				insertionSortZ(proxyPool);
				sweepZ(proxyPool);
			}
		}
		
		private function sweepX(proxyPool:Vector.<Proxy>):void {
			var center:Number;
			var sumX:Number = 0;
			var sumX2:Number = 0;
			var sumY:Number = 0;
			var sumY2:Number = 0;
			var sumZ:Number = 0;
			var sumZ2:Number = 0;
			var invNum:Number = 1 / numProxies;
			var bodyStatic:uint = RigidBody.BODY_STATIC;
			for (var i:int = 0; i < numProxies; i++) {
				var p1:Proxy = proxyPool[i];
				center = p1.minX + p1.maxX;
				sumX += center;
				sumX2 += center * center;
				center = p1.minY + p1.maxY;
				sumY += center;
				sumY2 += center * center;
				center = p1.minZ + p1.maxZ;
				sumZ += center;
				sumZ2 += center * center;
				var s1:Shape = p1.parent;
				for (var j:int = i + 1; j < numProxies; j++) {
					var p2:Proxy = proxyPool[j];
					numPairChecks++;
					if (p1.maxX < p2.minX) {
						break;
					}
					var s2:Shape = p2.parent;
					if (
						p1.maxY < p2.minY || p1.minY > p2.maxY ||
						p1.maxZ < p2.minZ || p1.minZ > p2.maxZ ||
						!isAvailablePair(s1, s2)
					) {
						continue;
					}
					addPair(s1, s2);
				}
			}
			sumX = sumX2 - sumX * sumX * invNum;
			sumY = sumY2 - sumY * sumY * invNum;
			sumZ = sumZ2 - sumZ * sumZ * invNum;
			if (sumX > sumY) {
				if (sumX > sumZ) {
					sortAxis = 0;
				} else {
					sortAxis = 2;
				}
			} else if (sumY > sumZ) {
				sortAxis = 1;
			} else {
				sortAxis = 2;
			}
		}
		
		private function sweepY(proxyPool:Vector.<Proxy>):void {
			var center:Number;
			var sumX:Number = 0;
			var sumX2:Number = 0;
			var sumY:Number = 0;
			var sumY2:Number = 0;
			var sumZ:Number = 0;
			var sumZ2:Number = 0;
			var invNum:Number = 1 / numProxies;
			var bodyStatic:uint = RigidBody.BODY_STATIC;
			for (var i:int = 0; i < numProxies; i++) {
				var p1:Proxy = proxyPool[i];
				center = p1.minX + p1.maxX;
				sumX += center;
				sumX2 += center * center;
				center = p1.minY + p1.maxY;
				sumY += center;
				sumY2 += center * center;
				center = p1.minZ + p1.maxZ;
				sumZ += center;
				sumZ2 += center * center;
				var s1:Shape = p1.parent;
				for (var j:int = i + 1; j < numProxies; j++) {
					var p2:Proxy = proxyPool[j];
					numPairChecks++;
					if (p1.maxY < p2.minY) {
						break;
					}
					var s2:Shape = p2.parent;
					if (
						p1.maxX < p2.minX || p1.minX > p2.maxX ||
						p1.maxZ < p2.minZ || p1.minZ > p2.maxZ ||
						!isAvailablePair(s1, s2)
					) {
						continue;
					}
					addPair(s1, s2);
				}
			}
			sumX = sumX2 - sumX * sumX * invNum;
			sumY = sumY2 - sumY * sumY * invNum;
			sumZ = sumZ2 - sumZ * sumZ * invNum;
			if (sumX > sumY) {
				if (sumX > sumZ) {
					sortAxis = 0;
				} else {
					sortAxis = 2;
				}
			} else if (sumY > sumZ) {
				sortAxis = 1;
			} else {
				sortAxis = 2;
			}
		}
		
		private function sweepZ(proxyPool:Vector.<Proxy>):void {
			var center:Number;
			var sumX:Number = 0;
			var sumX2:Number = 0;
			var sumY:Number = 0;
			var sumY2:Number = 0;
			var sumZ:Number = 0;
			var sumZ2:Number = 0;
			var invNum:Number = 1 / numProxies;
			var bodyStatic:uint = RigidBody.BODY_STATIC;
			for (var i:int = 0; i < numProxies; i++) {
				var p1:Proxy = proxyPool[i];
				center = p1.minX + p1.maxX;
				sumX += center;
				sumX2 += center * center;
				center = p1.minY + p1.maxY;
				sumY += center;
				sumY2 += center * center;
				center = p1.minZ + p1.maxZ;
				sumZ += center;
				sumZ2 += center * center;
				var s1:Shape = p1.parent;
				for (var j:int = i + 1; j < numProxies; j++) {
					var p2:Proxy = proxyPool[j];
					numPairChecks++;
					if (p1.maxZ < p2.minZ) {
						break;
					}
					var s2:Shape = p2.parent;
					if (
						p1.maxX < p2.minX || p1.minX > p2.maxX ||
						p1.maxY < p2.minY || p1.minY > p2.maxY ||
						!isAvailablePair(s1, s2)
					) {
						continue;
					}
					addPair(s1, s2);
				}
			}
			sumX = sumX2 - sumX * sumX * invNum;
			sumY = sumY2 - sumY * sumY * invNum;
			sumZ = sumZ2 - sumZ * sumZ * invNum;
			if (sumX > sumY) {
				if (sumX > sumZ) {
					sortAxis = 0;
				} else {
					sortAxis = 2;
				}
			} else if (sumY > sumZ) {
				sortAxis = 1;
			} else {
				sortAxis = 2;
			}
		}
		
		private function removeProxyAxis(proxy:Proxy, proxyPool:Vector.<Proxy>):void {
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
		}
		
		private function insertionSortX(proxyPool:Vector.<Proxy>):void {
			if (numProxies == 1)
				return;
			for (var i:int = 1; i < numProxies; i++) {
				var insert:Proxy = proxyPool[i];
				if (proxyPool[i - 1].minX > insert.minX) {
					var j:int = i;
					do {
						proxyPool[j] = proxyPool[j - 1];
						j--;
					} while (j > 0 && proxyPool[j - 1].minX > insert.minX);
					proxyPool[j] = insert;
				}
			}
		}
		
		private function insertionSortY(proxyPool:Vector.<Proxy>):void {
			if (numProxies == 1)
				return;
			for (var i:int = 1; i < numProxies; i++) {
				var insert:Proxy = proxyPool[i];
				if (proxyPool[i - 1].minY > insert.minY) {
					var j:int = i;
					do {
						proxyPool[j] = proxyPool[j - 1];
						j--;
					} while (j > 0 && proxyPool[j - 1].minY > insert.minY);
					proxyPool[j] = insert;
				}
			}
		}
		
		private function insertionSortZ(proxyPool:Vector.<Proxy>):void {
			if (numProxies == 1)
				return;
			for (var i:int = 1; i < numProxies; i++) {
				var insert:Proxy = proxyPool[i];
				if (proxyPool[i - 1].minZ > insert.minZ) {
					var j:int = i;
					do {
						proxyPool[j] = proxyPool[j - 1];
						j--;
					} while (j > 0 && proxyPool[j - 1].minZ > insert.minZ);
					proxyPool[j] = insert;
				}
			}
		}
		
	}

}