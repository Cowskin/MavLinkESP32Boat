#include "HardwareSerial.h" 
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <Servo.h>

#define WIFI_SSID "MAVLink"
#define WIFI_PASSWORD "12345678"
#define MAVLINK_PORT 14550

// Usage: connect to Wi-Fi network and use QGroundControl to open MAVLink connection

WiFiUDP udp;
HardwareSerial MySerial(2); // Use hardware serial port 2
TinyGPSPlus gps;
unsigned long lastGPSSend = 0; // Store the last send time
const unsigned long gpsSendInterval = 1000; // Interval to send GPS data

// SERVO CONFIGURATIONS
Servo motor1;
Servo motor2;
const int motorPin1 = 12;  // Motor 1 control pin
const int motorPin2 = 13;  // Motor 2 control pin

const int minPulse = 70; // 1ms pulse (minimum throttle)
const int midPulse = 90;
const int maxPulse = 160; // 2ms pulse (maximum throttle)

void setup() {
  Serial.begin(115200);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  udp.begin(MAVLINK_PORT);

  Serial.println("Initializing Serial2 for GPS...");
  MySerial.begin(9600, SERIAL_8N1, 16, 17); // Initialize Serial2 for GPS with RX on GPIO16 and TX on GPIO17
  Serial.println("Serial2 initialized.");

   // Attach motors to their respective pins
    motor1.attach(motorPin1);
    motor2.attach(motorPin2);

    // Set initial positions to the minimum pulse width (1ms)
    motor1.write(motorPin1, midPulse);
    motor2.write(motorPin2, midPulse);
}

void loop() {
  
  receiveMAVLink();

  // Send GPS data at intervals''''
  if (millis() - lastGPSSend >= gpsSendInterval) {
    sendMAVLink();
    sendGPSMAVLink();
    lastGPSSend = millis(); // Update the last send time
  }
}

void sendMAVLink() {
  static uint32_t lastSent = 0;
  if (millis() - lastSent < 1000) return; // Send every second

  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send buffer over UDP
	udp.beginPacket("255.255.255.255", 14550);
	udp.write(buf, len);
	udp.endPacket();

  lastSent = millis();
}

void receiveMAVLink() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  // Read UDP packet
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  udp.read(buf, MAVLINK_MAX_PACKET_LEN);

  // Parse MAVLink message
  mavlink_message_t msg;
  mavlink_status_t status;
  for (int i = 0; i < packetSize; i++) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          //Serial.println("Received HEARTBEAT");
          //sendHeartbeat();
          break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
          // Can be used to control the vehicle
          mavlink_manual_control_t manualControl;
          mavlink_msg_manual_control_decode(&msg, &manualControl);
          //Serial.print("Received MANUAL_CONTROL:");
          //Serial.print(" x=");
          //Serial.print(manualControl.x);
          //Serial.print(" y=");
          //Serial.print(manualControl.y);
          //Serial.print(" z=");
          //Serial.print(manualControl.z);
          //Serial.print(" r=");
          //Serial.println(manualControl.r);
          
          controlMotors(manualControl.z, manualControl.y);
          break;
        case MAVLINK_MSG_ID_COMMAND_LONG: // Check for COMMAND_LONG (ID 76)
          mavlink_command_long_t commandLong;
          Serial.print("Received message with ID ");
          Serial.println(commandLong.command);
          mavlink_msg_command_long_decode(&msg, &commandLong);
          // Check the command ID
          if (commandLong.command == 181 || commandLong.command == 183) {
              sendCustomResponse(commandLong.command);
          }
          break;
        default:
          Serial.print("Received message with ID ");
          Serial.println(msg.msgid);
          break;
      }
    }
  }
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Prepare HEARTBEAT message
  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                             MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  udp.beginPacket("255.255.255.255", MAVLINK_PORT);
  udp.write(buf, len);
  udp.endPacket();
  Serial.println("Sent HEARTBEAT");
}

void sendCustomResponse(uint8_t commandId) {
    
    uint8_t result = MAV_RESULT_ACCEPTED; // Set the result for acknowledgment
    uint8_t progress = 0; // Set progress, if applicable (usually 0)
    int32_t result_param2 = 0; // Additional result parameter (can be used for more context)
    uint8_t target_system = 1; // Your system ID
    uint8_t target_component = MAV_COMP_ID_AUTOPILOT1; // Your component ID
    Serial.print("Sending COMMAND_ACK for command ID: ");
    Serial.println(commandId);
    // Prepare MAVLink message
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(1, 1, &msg, MAVLINK_MSG_ID_COMMAND_LONG, result, progress, result_param2, target_system, target_component);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // Send the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    udp.beginPacket("255.255.255.255", MAVLINK_PORT);
    udp.write(buf, len);
    udp.endPacket();
}


// Function to control motors based on x and y values
void controlMotors(int16_t z, int16_t y) {
  // Map z value to pulse width
  int pulseZ = map(z, -1000, 1000, minPulse, maxPulse);
  int pulseY = map(y, -1000, 1000, minPulse, maxPulse);

   if (z > 0) {
    Serial.println("left motor move forward");
    motor1.write(motorPin1, pulseZ);
  } else if (z < 0) {
    Serial.println("left motor move backward");
    motor1.write(motorPin1, pulseZ);
  } else {
    Serial.println("left motor stop");
    motor1.write(motorPin1, midPulse); // Stop the motor
  }

  if (y > 0) {
    Serial.println("right motor move forward");
    motor2.write(motorPin2, pulseY); 
  } else if (y < 0) {
    Serial.println("right motor move backward");
    motor2.write(motorPin2, pulseY);
  } else {
    Serial.println("right motor stop");
    motor2.write(motorPin2, midPulse); // Stop the motor
  }
}


void sendGPSMAVLink() {
  // Check if there is any data available on Serial2
  if (MySerial.available()) {
    //Serial.println("Data available from GPS:");
    while (MySerial.available()) {
      char c = MySerial.read(); // Read a character from Serial2
      // Serial.print(c); // Print the character to the Serial monitor
      if (gps.encode(c))
      {
        // displayInfo();
        // Check if GPS data is valid
        if (gps.location.isValid()) {
          // Gather GPS data
          uint64_t time_usec = micros();
          uint8_t fix_type = gps.hdop.isValid() ? 3 : 0; // Example fix type, adjust based on your logic
          int32_t lat = gps.location.lat() * 1e7;
          int32_t lon = gps.location.lng() * 1e7;
          int32_t alt = gps.altitude.meters() * 1000;
          uint16_t eph = gps.hdop.isValid() ? gps.hdop.hdop() * 100 : UINT16_MAX;
          uint16_t epv = UINT16_MAX; // Vertical dilution of position not provided by TinyGPS++
          uint16_t vel = gps.speed.isValid() ? gps.speed.kmph() * 27.78 : UINT16_MAX; // Convert km/h to cm/s
          uint16_t cog = gps.course.isValid() ? gps.course.deg() * 100 : UINT16_MAX;
          uint8_t satellites_visible = gps.satellites.value();
          int32_t alt_ellipsoid = UINT32_MAX; // Altitude above WGS84, not provided by TinyGPS++
          uint32_t h_acc = UINT32_MAX; // Horizontal accuracy, not provided by TinyGPS++
          uint32_t v_acc = UINT32_MAX; // Vertical accuracy, not provided by TinyGPS++
          uint32_t vel_acc = UINT32_MAX; // Speed accuracy, not provided by TinyGPS++
          uint32_t hdg_acc = UINT32_MAX; // Heading accuracy, not provided by TinyGPS++
          uint16_t yaw = 36000; // Yaw, use 36000 for north as per the documentation

          // Pack the MAVLink message
          mavlink_message_t msg;
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];

          mavlink_msg_gps_raw_int_pack(
            1,
            MAV_COMP_ID_AUTOPILOT1,
            &msg,
            time_usec,
            fix_type,
            lat,
            lon,
            alt,
            eph,
            epv,
            vel,
            cog,
            satellites_visible,
            alt_ellipsoid,
            h_acc,
            v_acc,
            vel_acc,
            hdg_acc,
            yaw
          );

          // Copy the message to the send buffer
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          udp.beginPacket("255.255.255.255", MAVLINK_PORT);
          udp.write(buf, len);
          udp.endPacket();
          // Serial.println("Sent GPS data via MAVLink"); 
        }
      }
    }
  } else {
    // Serial.println("No data available from GPS.");
  }


  

  //delay(1000); // Wait for a second before checking again
		

      // Generate GPS_RAW_INT message based on parsed data from ATGM336H library
      // mavlink_msg_gps_raw_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, 
      //                            micros(),  // Use micros() for timestamp
      //                            Save_Data.fix_type,  // Assuming fix type is available in library
      //                            (int32_t)(Save_Data.latitude * 1e7),  // Latitude scaled by 1e7
      //                            (int32_t)(Save_Data.longitude * 1e7), // Longitude scaled by 1e7
      //                            Save_Data.altitude,
      //                            0, 0, 0, // Not used for GPS_RAW_INT
      //                            Save_Data.satellites);  // Assuming satellites visible is available in library
      // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
      // udp.beginPacket("255.255.255.255", MAVLINK_PORT);
      // udp.write(buf, len);
      // udp.endPacket();
      //Serial.println("Sent GPS data via MAVLink");  
}


void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}