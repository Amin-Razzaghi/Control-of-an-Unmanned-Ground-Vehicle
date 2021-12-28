#include <ESP8266WiFi.h>

const char* ssid = "mahboobe";
const char* password = "mahboobe";

WiFiServer server(80);

//int output_pin = 2;

void setup() {
  Serial.begin(9600);
  delay(10);

  //pinMode(output_pin, OUTPUT);
  //digitalWrite(output_pin, 1);

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  server.begin();
  Serial.println("Server started");

  Serial.println(WiFi.localIP());
}

void loop() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }

//  Serial.println("new client");
  while (!client.available()) {
    delay(1);
  }

  String req = client.readStringUntil('\r');
  //Serial.println(req);
  client.flush();

  if (req.indexOf("/W") != -1) {
    Serial.println("W");
  }
  else if (req.indexOf("/S") != -1) {
    Serial.println("S");
  }
  else if (req.indexOf("/A") != -1) {
    Serial.println("A");
  }
  else if (req.indexOf("/D") != -1) {
    Serial.println("D");
  }
  else if (req.indexOf("/Q") != -1) {
    Serial.println("Q");
  }
  else if (req.indexOf("/E") != -1) {
    Serial.println("E");
  }
  else if (req.indexOf("/B") != -1) {
    Serial.println("B");
  }

  client.flush();

  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
  s += "<head>";
  s += "<meta charset=\"utf-8\">";
  //s += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  s += "<script src=\"https://code.jquery.com/jquery-2.1.3.min.js\"></script>";
  s += "<link rel=\"stylesheet\" href=\"https://maxcdn.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css\">";
  s += "</head>";
  s += "<body style=\"background-color: wheat\">";

  
  s += "</body>";

  s += "</br>";
  s += "</br>";
  s += "</br>";
  s += "</br>";
  s += "</br>";
  s += "</br>";

  
  s += "<div class=\"row justify-content-md-center\">";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"Q\" onmousedown=\"FORWARDLEFT()\">";
    s += "</div>";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"W\" onmousedown=\"FORWARD()\">";
    s += "</div>";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"E\" onmousedown=\"FORWARDRIGHT()\">";
    s += "</div>";
  s += "</div>";
  s += "</br>";
  s += "</br>";
  s += "<div class=\"row justify-content-md-center\">";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"A\" onclick=\"LEFT()\">";
    s += "</div>";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-danger btn-lg\" value=\"B\" onclick=\"STOP()\">";
    s += "</div>";
    s += "<div class=\"col-1 \">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"D\" onclick=\"RIGHT()\">";
    s += "</div>";
  s += "</div>";
  s += "</br>";
  s += "</br>";
  s += "<div class=\"row justify-content-md-center\">";
    s += "<div class=\"col-1\">";
      s += "<input type=\"button\" class=\"btn btn-primary btn-lg\" value=\"S\" onclick=\"BACKWARD()\">";
    s += "</div>";
  s += "</div>";
  s += "<script>";
    s += "function LEFT() {$.get(\"/A\");}";
    s += "function RIGHT() {$.get(\"/D\");}";
    s += "function BACKWARD() {$.get(\"/S\");}";
    s += "function FORWARD() {$.get(\"/W\");}";
    s += "function STOP() {$.get(\"/B\");}";
    s += "function FORWARDLEFT() {$.get(\"/Q\");}";
    s += "function FORWARDRIGHT() {$.get(\"/E\");}";
  s += "</script>";

  client.print(s);
  delay(1);
  //Serial.println("Client disconnected");
}
