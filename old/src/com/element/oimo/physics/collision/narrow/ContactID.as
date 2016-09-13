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
package com.element.oimo.physics.collision.narrow {
	/**
	 * 接触点の識別のためのクラスです。
	 * @author saharan
	 */
	public class ContactID {
		/**
		 * 識別に使われる一つ目のデータです。
		 */
		public var data1:uint;
		
		/**
		 * 識別に使われる二つ目のデータです。
		 */
		public var data2:uint;
		
		/**
		 * 識別データが反転しているかどうかを表します。
		 */
		public var flip:Boolean;
		
		/**
		 * 新しく ContactID オブジェクトを作成します。
		 */
		public function ContactID() {
		}
		
		public function equals(id:ContactID):Boolean {
			return flip == id.flip ? data1 == id.data1 && data2 == id.data2 : data2 == id.data1 && data1 == id.data2;
		}
		
	}

}