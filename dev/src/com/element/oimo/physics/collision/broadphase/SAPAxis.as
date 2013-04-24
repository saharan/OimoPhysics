package com.element.oimo.physics.collision.broadphase {
	import flash.utils.getTimer;
	/**
	 * ...
	 * @author saharan
	 */
	public class SAPAxis {
		public var stack:Vector.<int>;
		public var values:Vector.<SAPValue>;
		public var valuesBuf:Vector.<SAPValue>;
		private var numValues:int;
		private var size:int;
		
		public var radixVal:Vector.<int>;
		public var radixBuf:Vector.<int>;
		public const radixCnt:Vector.<int> = new Vector.<int>(4096, true);
		
		public function SAPAxis() {
			size = 512;
			stack = new Vector.<int>(size << 1, true);
			radixVal = new Vector.<int>(size << 1, true);
			radixBuf = new Vector.<int>(size << 1, true);
			values = new Vector.<SAPValue>(size, true);
			valuesBuf = new Vector.<SAPValue>(size, true);
		}
		
		public function insertValue(value:SAPValue):void {
			if (numValues == size) {
				size <<= 1;
				stack = new Vector.<int>(size << 1, true);
				radixVal = new Vector.<int>(size << 1, true);
				radixBuf = new Vector.<int>(size << 1, true);
				valuesBuf = new Vector.<SAPValue>(size, true);
				var newValues:Vector.<SAPValue> = new Vector.<SAPValue>(size, true);
				for (var i:int = 0; i < numValues; i++) {
					newValues[i] = values[i];
				}
				values = newValues;
			}
			values[numValues++] = value;
		}
		
		public function deleteValue(value:SAPValue):void {
			var target:Number = value.value;
			var left:int = 0;
			var right:int = --numValues;
			while (left < right) { // binary search
				var mid:int = left + (right - left >> 1);
				if (values[mid].value < target) {
					left = mid + 1;
				} else {
					right = mid;
				}
			}
			for (var i:int = left; i <= numValues; i++) {
				if (values[i] == value) {
					while (++i <= numValues) {
						values[i - 1] = values[i];
					}
					values[i] = null;
				}
			}
		}
		
		public function sort():void {
			var stackCount:int = 2;
			stack[0] = 0;
			stack[1] = numValues - 1;
			while (stackCount > 0) {
				var right:int = stack[--stackCount];
				var left:int = stack[--stackCount];
				var diff:int = right - left;
				var tmp:SAPValue;
				if (diff > 16) { // quick sort
					var mid:int = left + (diff >> 1);
					tmp = values[mid];
					values[mid] = values[right];
					values[right] = tmp;
					var pivot:Number = tmp.value;
					var i:int = left - 1;
					var j:int = right;
					while (true) {
						var vi:SAPValue;
						var vj:SAPValue;
						do {
							vi = values[++i];
						} while (vi.value < pivot);
						do {
							vj = values[--j];
						} while (pivot < vj.value && j != left);
						if (i >= j) break;
						values[i] = vj;
						values[j] = vi;
					}
					values[right] = values[i];
					values[i] = tmp;
					stack[stackCount++] = left;
					stack[stackCount++] = i - 1;
					stack[stackCount++] = i + 1;
					stack[stackCount++] = right;
				} else { // insertion sort
					for (i = left + 1; i <= right; i++) {
						tmp = values[i];
						pivot = tmp.value;
						if (values[i - 1].value > pivot) {
							j = i;
							do {
								values[j] = values[--j];
							} while (j > left && values[j - 1].value > pivot);
							values[j] = tmp;
						}
					}
				}
			}
		}
		
		public function radixSort():void {
			const size:int = numValues;
			var buff:Vector.<int> = radixBuf;
			var work:Vector.<int> = radixVal;
			var count:Vector.<int> = radixCnt;
			for (var i:int = 0; i < size; i++) {
				buff[i] = values[i].value2;
				buff[i + size] = i;
			}
			for (var shift:int = 0; shift < 24; shift += 8) {
				for (i = 0; i < 256; i++) {
					count[i] = 0;
				}
				for (i = 0; i < size; i++) {
					++count[buff[i] >> shift & 0xff];
				}
				for (i = 1; i < 256; i++) {
					count[i] += count[i - 1];
				}
				i = size;
				var j:int = size << 1;
				while (i > 0) {
					var p:int = buff[--i];
					var q:int = --count[p >> shift & 0xff];
					work[q] = p;
					work[q + size] = buff[--j];
				}
				var tmp:Vector.<int> = buff;
				buff = work;
				work = tmp;
			}
		}
		
		public function calculateTestCount():int {
			var num:int = 0;
			var sum:int = 0;
			for (var i:int = 0; i < numValues; i++) {
				if ((values[i].value2 & 0x40000000) != 0) {
					num--;
				} else {
					sum += num;
					num++;
				}
			}
			return sum;
		}
		
		public function test():void {
			for (var i:int = 0; i < numValues; i++) {
				if ((values[i].value2 & 0x40000000) == 0 && values[i].value > values[i].pair.value) throw new Error();
			}
		}
		
		public function print():void {
			var text:String = "[";
			for (var i:int = 0; i < numValues; i++) {
				if (i != 0) text += ", ";
				text += values[i].value.toFixed(2);
			}
			text += "]";
			trace(text);
		}
		
	}

}