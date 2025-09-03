#include <WiFi.h>
#include <HTTPClient.h>
#include <SPIFFS.h>

const char* ssid = "Wifi Name";
const char* password = "Wifi Password";

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.print("Connecting to....");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries++;
    if (retries > 60) {
      Serial.println("\nFailed to Connect.");
      while (true);
    }
  }
  Serial.println();
  Serial.print("WiFi Connected to the IP address: ");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    while (true);
  }

  HTTPClient http;
  String url = "https://www.google.com";
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println("HTTP Response Code: " + String(httpCode));
    Serial.println("Downloaded Data:");
    Serial.println(payload);

    File file = SPIFFS.open("/downloaded.txt", FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing");
    } else {
      file.print(payload);
      file.close();
      Serial.println("Data saved to /downloaded.txt");
    }

    file = SPIFFS.open("/downloaded.txt", FILE_READ);
    Serial.println("Read-back from SPIFFS:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("Error on HTTP request: " + String(httpCode));
  }

  http.end();
}

void loop() {
  delay(1000);
}
