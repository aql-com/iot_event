//============
//Webpage Code
//============
const char indexpageCode[6500] = {
"<!DOCTYPE html>\
<head>\
  <title> System Status </title>\
</head>\
<html>\
<!--CSS-->\
<style>\
  body {background-color: #3760A3}\
  h4 {font-family: arial; text-align: center; color: white;}\
  .card\
  {\
     background: #3760A3;\
     padding: 10px;\
     font-weight: bold;\
     <!--font: 25px calibri;-->\
     text-align: center;\
     box-sizing: border-box;\
     color: white;\
     margin:20px;\
     box-shadow: 0px 2px 15px 15px rgba(0,0,0,0.75);\
  }\
</style>\
<!--HTML-->\
<body>\
  <div class=\"card\">\
    <h2><span style=\"display: inline-block; width: 300px;\">System Status</span></h2>\
    <h3>\
  <form>\
	<table>\
	<tr><td style='width: 120px'>Unit ID:</td><td></td><td><span id=\"uid\"> </span></td></tr>\
	<tr><td>Unit Time:</td><td></td><td><span id=\"utime\"> </span></td></tr>\
	</table></h3>\
    <br><h4>\
<!--	<br> \
      POT Value : <span style=\"color:yellow\" id=\"POTvalue\">0</span>-->\
    </h4>\
 <h4>\
    <!--button onclick=\"DoStart()\">Start</button><br><br>\
    <button onclick=\"DoStop()\">Stop</button><br><br>-->\
	</h4>\
  </form>\
  <button onclick=\"DoHelp()\">Help</button>\
  <span id=\"helpDIV\"></span>\<span id=\"dbg1\"></span>\<span id=\"dbg2\"></span><span id=\"dbg3\"></span>\<span id=\"dbg4\"></span>\
  </div>\
  <div id=\"footer\" style=\"text-align: center;\">Copyright aql 2023.</div>\r\n\
  <!--JavaScript-->\
  <script>\r\n\
    setInterval(function() { updateStatus(); }, 1000);\r\n\
    //--\r\n\
	function getXMLValue(xmlData, field) {\r\n\
	try {\r\n\
		if(xmlData.getElementsByTagName(field)[0].firstChild.nodeValue)\r\n\
			return xmlData.getElementsByTagName(field)[0].firstChild.nodeValue;\r\n\
		else\r\n\
			return null;\r\n\
		} catch(err) { return null; }\r\n\
	}\r\n\
    function updateStatus()\r\n\
    {\r\n\
	  var i = 0;\r\n\
      var UpdateRequest = new XMLHttpRequest();\r\n\
      UpdateRequest.onreadystatechange = function()\r\n\
      {\r\n\
        if(this.readyState == 4 && this.status == 200)\r\n\
        {\r\n\
		xmlpkt = this.responseXML;\r\n\
		console.log(this);\r\n\
		console.log(this.responseText);\r\n\
        //document.getElementById(\"POTvalue\").innerHTML = getXMLValue(xmlpkt,'tst');\r\n\
        document.getElementById('uid').innerHTML = getXMLValue(xmlpkt,'uid');\r\n\
        document.getElementById(\"utime\").innerHTML = getXMLValue(xmlpkt,'time');\r\n\
		for(i = 0; i < 8; i++)\r\n\
		{document.getElementById(\"ev\"+i).innerHTML = getXMLValue(xmlpkt,'ev'+i);\r\n\
        document.getElementById(\"en\"+i).innerHTML = getXMLValue(xmlpkt,'en'+i);\r\n\
        document.getElementById(\"md\"+i).innerHTML = getXMLValue(xmlpkt,'md'+i);\r\n\
        document.getElementById(\"da\"+i).innerHTML = getXMLValue(xmlpkt,'da'+i);\r\n\
        document.getElementById(\"st\"+i).innerHTML = getXMLValue(xmlpkt,'st'+i);\r\n\
        document.getElementById(\"du\"+i).innerHTML = getXMLValue(xmlpkt,'du'+i);\r\n\
        document.getElementById(\"pw\"+i).innerHTML = getXMLValue(xmlpkt,'pw'+i);\r\n\
        document.getElementById(\"ru\"+i).innerHTML = getXMLValue(xmlpkt,'ru'+i);\r\n\
        document.getElementById(\"rp\"+i).innerHTML = getXMLValue(xmlpkt,'rp'+i);\r\n\
        document.getElementById(\"dr\"+i).innerHTML = getXMLValue(xmlpkt,'dr'+i);}\r\n\
        document.getElementById(\"status\").innerHTML = getXMLValue(xmlpkt,'stat');\r\n\
        document.getElementById(\"pwm\").innerHTML = getXMLValue(xmlpkt,'pwm');\r\n\
		i = getXMLValue(xmlpkt,'bon');\r\n\
		document.getElementById(\"dbg4\").innerHTML = i;\r\n\
		if (i == 0) document.getElementById(\"boff\").checked = true;\r\n\
		if (i == 1) document.getElementById(\"b25\").checked = true;\r\n\
		if (i == 2) document.getElementById(\"b50\").checked = true;\r\n\
		if (i == 3) document.getElementById(\"b75\").checked = true;\r\n\
		if (i == 4) document.getElementById(\"b100\").checked = true;\r\n\
//		document.getElementById('bon').checked  = getXMLValue(xmlpkt,'bon');\r\n\
//		document.getElementById('boff').checked = 1 - getXMLValue(xmlpkt,'bon');\r\n\
//		document.getElementById('boff').checked = getXMLValue(xmlpkt,'boff');\r\n\
//		if (getXMLValue(xmlpkt,'bon') == true)  document.getElementById(\"dbg1\").innerHTML = \"on=True\";\
		//if(getXMLValue(xmlpkt,'bon'))  document.getElementById('bon').checked  = true;\r\n\
		//if(getXMLValue(xmlpkt,'boff')) document.getElementById('boff').checked = true;\r\n\
		//if(getXMLValue(xmlpkt,'bsch')) document.getElementById('bsch').checked = true;\r\n\
        }\r\n\
      };\r\n\
      UpdateRequest.open(\"GET\", \"readPOT\", true);\r\n\
      UpdateRequest.send();\r\n\
    }\r\n\
    //--\r\n\
    function DoHelp()\r\n\
    {\r\n\
      var x = document.getElementById(\"helpDIV\");\r\n\
      var message = \"help message\";\r\n\
      if (x.innerHTML == \"\") x.innerHTML = message;\r\n\
      else x.innerHTML = \"\";\r\n\
    }\r\n\
    function DoSubmit()\r\n\
    {\r\n\
	document.forms[0].submit();\
     }\r\n\
  </script>\r\n\
</body>\
</html>\
"};

