
var webai_ros_util = function(opt){

    google.load("search", "1");
    google.load("feeds",  "1");
    google.load("jquery", "1.4.3");

    this.$WA  = crocro.webAi;
    this.cWSrch   = new this.$WA.WebSrch();
    this.cJpKw    = new this.$WA.JpKw();
    this.cJpSntnc = new this.$WA.JpSntnc();
    this.hitCnt = 0 ;
    this.response = "" ;
    this.keywords = "" ;

    this.cWSrch.ready(function(){
	this.cWSrch
	    .brand("gglBrnd")
	    .init({
		type : "web",
		opt : function(obj) {
		    obj.setResultSetSize(google.search.Search.LARGE_RESULTSET);
		}
	    })
	    .init({
		type : "blg",
		opt : function(obj) {
		    obj.setResultSetSize(google.search.Search.LARGE_RESULTSET);
		}
	    })
	    .setEachCall(function(){ console.log("searching") ; })
	    .start();
    }.bind(this));

    webai_ros_util.prototype.search = function(qStr){
	var kwArr = [];
	this.cJpKw.reset();
	this.cJpSntnc.reset();
	this.cJpKw.avoidStr = qStr;
	this.cWSrch
	    .reset()
	    .srch({
		type : "web",
		page : 1,
		key : qStr,
		res : function(res, cursor) {
		    var outStr = "";
		    if (cursor) {
			this.hitCnt = cursor.estimatedResultCount ;
		    }
		    if (! res || res.length <= 0) {
			console.log("no response") ;
			this.cWSrch.cmndsBreak();
		    } else {
			this.response = res ;
			var buf = "" ;
			for (var i = 0; i < res.length; i ++) {
			    var r = res[i];
			    this.cJpKw.addSrc(r.titleNoFormatting);
			    this.cJpKw.addSrc(r.content);
			    //console.log( "[" + r.titleNoFormatting + "] " + r.content) ;
			    buf += "<br/>[" + r.titleNoFormatting + "]<br/> " + r.content + "<br />";
			}
			document.getElementById("search_contents").innerHTML = buf ;
		    }
		    this.keywords = this.cJpKw.getStrArr({prmAll:true}) ;
		    document.getElementById("search_keywords").innerHTML = this.keywords.join("<br />") ;
		    //kwArr = cJpKw.getStrArr();
		}.bind(this)
	    })
	    .start();
    }
}

var wru = new webai_ros_util() ;