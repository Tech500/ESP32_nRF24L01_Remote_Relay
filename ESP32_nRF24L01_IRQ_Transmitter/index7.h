//index7.h
const char HTML7[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <div class="container">
    <header>
      <br><br>This is original H264 video encoded by IP camera; server doesn't do any transcoding.  Wyze Cam v3 video feeds
      <br>Wyze-Bridge Docker, container; which provides webRTC video URL.  Camera maybe offline; depending on battery discharge state.
      <br><br>
    </header>
    <main>
      <iframe class="iframe" width="1300" height="731"src="http://c600.duckdns.org:8889/wyze-cam-v3/" frameborder="0"></iframe> 
    </main>
    <footer>
      <h2><a href="server return link goes here... " >ESP32 Server</a></h2>  
    </footer>
  </div>
</body>
</html>
)====";
