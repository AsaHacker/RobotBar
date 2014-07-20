
usage
=============

* roslaunch rosbridge_server rosbridge_websocket.launch
* google-chrome $your_speech_recog.html

result
=============

$ rostopic echo /speech_recog/all
data:  こんにちは/0.8113591074943542  コニチワ/0  こにちわ/0  こんにち
わ/0  こにちは/0  コンニチワ/0
\---
data:  今日はいい天気ですね/0.8241590261459351  今日もいい天気ですね/0
今日はいい天気ですねー/0  今日はいい天気ですねぇ/0  今日は元気ですね/0

thanks
===================

* Thanks, google web speech api
* https://www.google.com/intl/ja/chrome/demos/speech.html


