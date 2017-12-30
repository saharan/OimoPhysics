// highlighter adapted from code.haxe.org
(function (console) { "use strict";
var EReg = function(r,opt) {
	opt = opt.split("u").join("");
	this.r = new RegExp(r,opt);
};
EReg.prototype = {
	replace: function(s,by) {
		return s.replace(this.r,by);
	}
};
var Highlighter = function() { };
Highlighter.main = function() {
	js.JQuery("pre code").each(function() {
		var el = js.JQuery(this);
		if(!el.hasClass("highlighted")) {
			el.html(Highlighter.syntaxHighlight(el.html()));
			el.addClass("highlighted");
		}
	});
};
Highlighter.syntaxHighlight = function(html) {
	var kwds = ["abstract","trace","break","case","cast","class","continue","default","do","dynamic","else","enum","extends","extern","for","function","if","implements","import","in","inline","interface","macro","new","override","package","private","public","return","static","switch","throw","try","typedef","untyped","using","var","while"];
	var kwds1 = new EReg("\\b(" + kwds.join("|") + ")\\b","g");
	var vals = ["null","true","false","this"];
	var vals1 = new EReg("\\b(" + vals.join("|") + ")\\b","g");
	var types = new EReg("\\b([A-Z][a-zA-Z0-9]*)\\b","g");
	html = kwds1.replace(html,"<span class='kwd'>$1</span>");
	html = vals1.replace(html,"<span class='val'>$1</span>");
	html = types.replace(html,"<span class='type'>$1</span>");
	html = new EReg("(\"[^\"]*\")","g").replace(html,"<span class='str'>$1</span>");
	html = new EReg("(//.+\n)","g").replace(html,"<span class='cmt'>$1</span>");
	html = new EReg("(/\\*\\*?[^*]*\\*?\\*/)","g").replace(html,"<span class='cmt'>$1</span>");
	return html;
};
var q = window.jQuery;
var js = js || {}
js.JQuery = q;
Highlighter.main();
})(typeof console != "undefined" ? console : {log:function(){}});