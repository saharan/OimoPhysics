package com.element.oimo.physics.collision.broadphase {
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
		private const axes:Vector.<SAPAxis> = new Vector.<SAPAxis>(3, true);
		private const radixCnt1:Vector.<int> = new Vector.<int>(4096, true);
		private const radixCnt2:Vector.<int> = new Vector.<int>(4096, true);
		private const radixCnt3:Vector.<int> = new Vector.<int>(4096, true);
		private const radixCnt4:Vector.<int> = new Vector.<int>(4096, true);
		private var index1:int;
		private var index2:int;
		private var numValues:uint;
		
		/**
		 * 新しく SweepAndPruneBroadPhase オブジェクトを作成します。
		 */
		public function SweepAndPruneBroadPhase() {
			axes[0] = new SAPAxis();
			axes[1] = new SAPAxis();
			axes[2] = new SAPAxis();
			index1 = 0;
			index2 = 1;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function addProxy(proxy:Proxy):void {
			axes[0].insertValue(proxy.sap.minX);
			axes[0].insertValue(proxy.sap.maxX);
			axes[1].insertValue(proxy.sap.minY);
			axes[1].insertValue(proxy.sap.maxY);
			axes[2].insertValue(proxy.sap.minZ);
			axes[2].insertValue(proxy.sap.maxZ);
			numValues += 2;
		}
		
		/**
		 * @inheritDoc
		 */
		override public function removeProxy(proxy:Proxy):void {
			axes[0].deleteValue(proxy.sap.minX);
			axes[0].deleteValue(proxy.sap.maxX);
			axes[1].deleteValue(proxy.sap.minY);
			axes[1].deleteValue(proxy.sap.maxY);
			axes[2].deleteValue(proxy.sap.minZ);
			axes[2].deleteValue(proxy.sap.maxZ);
			numValues -= 2;
		}
		
		override protected function collectPairs():void {
			var axis1:SAPAxis = axes[index1];
			var axis2:SAPAxis = axes[index2];
			//axis1.sort();
			//axis2.sort();
			sort2(axis1, axis2);
			var count1:int = axis1.calculateTestCount();
			var count2:int = axis2.calculateTestCount();
			numPairChecks = 0;
			var values:Vector.<SAPValue>;
			if (count1 < count2) {
				values = axis1.values;
			} else {
				values = axis2.values;
				index1 ^= index2;
				index2 ^= index1;
				index1 ^= index2;
			}
			index2 = (index1 | index2) ^ 3;
			var actives:SAPProxy = null;
			var numActives:int = 0;
			for (var i:int = 0; i < numValues; i++) {
				var v1:SAPValue = values[i];
				var p1:Proxy = v1.aabb;
				var sp1:SAPProxy = p1.sap;
				if ((v1.value2 & 0x40000000) == 0) {
					var s1:Shape = p1.parent;
					for (var sp:SAPProxy = actives; sp != null; sp = sp.next) {
						var p2:Proxy = sp.parent;
						var s2:Shape = p2.parent;
						numPairChecks++;
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
					sp1.next = actives;
					actives = sp1;
				} else {
					if (sp1 == actives) {
						actives = actives.next;
					} else {
						sp = actives;
						do {
							var next:SAPProxy = sp.next;
							if (next == sp1) {
								sp.next = next.next;
								break;
							}
							sp = next;
						} while (sp != null);
					}
				}
			}
		}
		
		private function sort2(a1:SAPAxis, a2:SAPAxis):void {
			const size:int = numValues;
			const values1:Vector.<SAPValue> = a1.values;
			const values2:Vector.<SAPValue> = a2.values;
			const valuesBuf1:Vector.<SAPValue> = a1.valuesBuf;
			const valuesBuf2:Vector.<SAPValue> = a2.valuesBuf;
			const val1:Vector.<int> = a1.radixVal;
			const val2:Vector.<int> = a2.radixVal;
			const buf1:Vector.<int> = a1.radixBuf;
			const buf2:Vector.<int> = a2.radixBuf;
			const cnt1:Vector.<int> = radixCnt1;
			const cnt2:Vector.<int> = radixCnt2;
			const cnt3:Vector.<int> = radixCnt3;
			const cnt4:Vector.<int> = radixCnt4;
			var i:int;
			var j:int;
			var p:int;
			var q:int;
			// -------------------------------------------------- sort 1-12 bits
			for (i = 0; i < 4096; i++) {
				cnt1[i] = 0;
				cnt2[i] = 0;
				cnt3[i] = 0;
				cnt4[i] = 0;
			}
			for (i = 0, j = size; i < size; i++, j++) {
				var v:SAPValue;
				v = values1[i];
				valuesBuf1[i] = v;
				cnt1[(val1[i] = v.value2) & 0xfff]++; // count up
				v = values2[i];
				valuesBuf2[i] = v;
				cnt2[(val2[i] = v.value2) & 0xfff]++; // count up
			}
			p = cnt1[0];
			q = cnt2[0];
			for (i = 1; i < 4096; i++) {
				p = cnt1[i] += p;
				q = cnt2[i] += q;
			}
			for (i = size - 1; i >= 0; i--) {
				cnt3[buf1[q = --cnt1[(p = val1[i]) & 0xfff]] = p >> 12 & 0xfff]++; // count up
				buf1[q + size] = i; // index
				cnt4[buf2[q = --cnt2[(p = val2[i]) & 0xfff]] = p >> 12 & 0xfff]++; // count up
				buf2[q + size] = i; // index
			}
			// -------------------------------------------------- sort 13-24 bits
			p = cnt3[0];
			q = cnt4[0];
			for (i = 1; i < 4096; i++) {
				p = cnt3[i] += p;
				q = cnt4[i] += q;
			}
			i = size - 1;
			j = i + size;
			while (i >= 0) {
				values1[--cnt3[buf1[i]]] = valuesBuf1[buf1[j]];
				values2[--cnt4[buf2[i]]] = valuesBuf2[buf2[j]];
				i--;
				j--;
			}
		}
		
	}

}