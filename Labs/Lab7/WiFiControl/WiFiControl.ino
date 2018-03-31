// Uses ESP8266 12-F (AI-Thinker Variant)
// Program as Adafruit Feather HUZZAH
// Flash: 4M
// No Debug
// lwIP: v2 Lower Memory
// CPU: 80MHz
// Baud: 115200
// Erase: Sketch Only

// Uses: ESP8266FS from https://github.com/esp8266/arduino-esp8266fs-plugin/releases/download/0.2.0/ESP8266FS-0.2.0.zip
// Uses: WebSocketServer from: https://github.com/Links2004/arduinoWebSockets

// Default IP: http://192.168.4.1

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "FS.h"

/* Set these to your desired credentials. */
const char *ssid = "Team15";
const char *password = "16311thereisnospoon";

ESP8266WiFiMulti WiFiMulti;

ESP8266WebServer server(80);
WebSocketsServer socket = WebSocketsServer(81);

String ctrl_page;

/*
	WebSocket Messaging Protocol:
	Symbol followed by 3 bytes.

	# -> Set RGB Color (B1 -> R, B2 -> G, B3 -> B)
	$ -> Command Servos (B1 -> Pan Servo Angle, B2 -> Tilt Servo Angle, B3 -> N/A)
*/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
	switch(type){
		case WStype_DISCONNECTED:
			// Do Nothing
		break;

		case WStype_CONNECTED: {
			IPAddress ip = socket.remoteIP(num);
			socket.sendTXT(num, "Connected");
		}
	 	break;

		case WStype_TEXT:
			if(payload[0] == '#'){ // 						- Set RGB Color
				uint32_t rgb = (uint32_t) strtol((const char*) &payload[1], NULL, 16);

				int valR = (rgb>>16) & 0xFF;
				int valG = (rgb>>8) & 0xFF;
				int valB = (rgb>>0) & 0xFF;
				Serial.write(1); Serial.write(0); Serial.write(valR);
				Serial.write(1); Serial.write(1); Serial.write(valG);
				Serial.write(1); Serial.write(2); Serial.write(valB);
			} else if(payload[0] == '$'){ //			- Command Servo
				uint32_t comm = (uint32_t) strtol((const char*) &payload[1], NULL, 16);

				int valPan = (comm>>16) & 0xFF;
				int valTilt = (comm>>8) & 0xFF;
				Serial.write(10); Serial.write(1); Serial.write(valPan);
				Serial.write(10); Serial.write(2); Serial.write(valTilt);
			}
		break;
	}
}

// Open HTML File Stored in SPIFFS Flash Memory
String openFile(String addr){
	SPIFFS.begin();
	File file = SPIFFS.open("/"+addr, "r");

	if(file){ // Successfully Opened
		String html = "";
		while(file.available()){
			html += file.readStringUntil('\n') + "\n";
		}
		file.close();
		return html;
	} else{
		return "Server Error. File: " + addr + " could not be opened from SPIFFS.";
	}
}

void handleRoot() {
	server.send(200, "text/html", ctrl_page);
}

void setup() {
	Serial.begin(115200);

	for(uint8_t i = 0; i<4; i++){ // Wait for Boot to Complete before Using Serial
		Serial.flush();
		delay(1000);
	}

	ctrl_page = openFile("ctrl.html");

	// Create Access Point:
	WiFi.softAP(ssid, password);

	IPAddress myIP = WiFi.softAPIP();

	// Open Web Socket:
	socket.begin();
	socket.onEvent(webSocketEvent);

	MDNS.begin("cmu.com");

	// Set Server Paths:
	server.on("/", handleRoot);
	server.begin();

	// Register Ports with mDNS:
	MDNS.addService("http", "tcp", 80);
	MDNS.addService("ws", "tcp", 81);
}

long last_laser = 0;
int laser_state = 0;

void loop() {
	socket.loop();
	server.handleClient();

	read_uart_message();
	// if(millis()-last_laser > 1000){
	// 	last_laser = millis();
	// 	laser_state = !laser_state;
	// 	Serial.write(20); Serial.write(laser_state); Serial.write(0);
	// }

}

/* ESP-INCOMING UART MESSAGING PROTOCOL:
 - 2 Byte Message:
 -- 1: Command   2: Value
  Comm 1 -> Pass Odometry X Integer to Socket
 	Comm 2 -> Pass Odometry X Decimal to Socket
 	Comm 3 -> Pass Odometry Y Integer to Socket
	Comm 4 -> Pass Odometry Y Decimal to Socket
 	Comm 5 -> Pass Odometry TH Integer to Socket
	Comm 6 -> Pass Odometry TH Decimal to Socket
  */
void read_uart_message(){
  static char msg_byte = 0; // Current Message Byte being Processed
  static int curr_msg[2] = {0,0};

  if(Serial.available() > 0){
    curr_msg[msg_byte] = (int) Serial.read();

    if(msg_byte == 1){
      process_uart_message(curr_msg);
      msg_byte = 0;
                                                                                // Serial.print(curr_msg[0]); Serial.print(curr_msg[1]); Serial.println(curr_msg[2]);
    } else{
      msg_byte++;
    }
  }
}

void process_uart_message(int* msg){
  switch(msg[0]){ // Command
    case 1: //                                - Pass Odometry X Integer to Socket
    	socket.broadcastTXT("X" + String(msg[1]));
    break;
    case 2: //                                - Pass Odometry X Decimal to Socket
    	socket.broadcastTXT("x" + String(msg[1]));
    break;
    case 3: //                                - Pass Odometry Y Integer to Socket
    	socket.broadcastTXT("Y" + String(msg[1]));
    break;
    case 4: //                                - Pass Odometry Y Decimal to Socket
    	socket.broadcastTXT("y" + String(msg[1]));
    break;
    case 5: //                                - Pass Odometry TH Integer to Socket
    	socket.broadcastTXT("T" + String(msg[1]));
    break;
    case 6: //                                - Pass Odometry TH Decimal to Socket
    	socket.broadcastTXT("t" + String(msg[1]));
    break;

    default:
      // Do Nothing.
    break;
  }
}
