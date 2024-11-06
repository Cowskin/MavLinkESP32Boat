#include <HardwareSerial.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <MAVLink.h>
#include <Servo.h>
#include "Submarine.hpp"

#define WIFI_SSID "MAVLink"
#define WIFI_PASSWORD "12345678"
#define MAVLINK_PORT 14550

WiFiUDP udp;
HardwareSerial MySerial(2);  // Use hardware serial port 2 GPIO 16 + GPIO 17
TinyGPSPlus gps;
unsigned long lastGPSSend = 0;               // Store the last send time
const unsigned long gpsSendInterval = 1000;  // Interval to send GPS data

// Track MAVLink heartbeat status
unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatTimeout = 3000;  // 3 seconds

// Parameter structure definition
struct Parameter {
  const char *name;
  int32_t *value;
  int32_t defaultValue;
};

// SERVO CONFIGURATIONS
Servo motor1;
Servo motor2;

const int motorPin1 = 14;  // Motor 1 control pin
const int motorPin2 = 13;  // Motor 2 control pin

Preferences preferences;

const int defaultMinPulse = 30;
const int defaultMaxPulse = 160;
const int defaultMidPulse = 90;

int minPulse = defaultMinPulse;  // 1ms pulse (minimum throttle)
int midPulse = defaultMidPulse;
int left_maxPulse = defaultMaxPulse;   // 2ms pulse (maximum throttle)
int right_maxPulse = defaultMaxPulse;  // 2ms pulse (maximum throttle)
int com_rc_in_mode = 1;

// Store last manual control values of z and y
int16_t lastZ = 0;
int16_t lastX = 0;

Parameter parameters[] = {
  { "COM_RC_IN_MODE", &com_rc_in_mode, 1},
  { "MOTOR_min", &minPulse, defaultMinPulse },
  { "MOTOR_mid", &midPulse, defaultMidPulse },
  { "MOTOR_l_max", &left_maxPulse, defaultMaxPulse },
  { "MOTOR_r_max", &right_maxPulse, defaultMaxPulse }
};

const int numParameters = sizeof(parameters) / sizeof(parameters[0]);

Submarine submarine(1, 0, 2, 6, 4, 5);

void printBinary(uint16_t value) {
  Serial.print("bX");
  for (int i = 15; i >= 0; i--) {
    Serial.print((value >> i) & 1);  // Shift and mask to get each bit
  }
}

// Function to check if a specific bit is enabled
bool isBitEnabled(uint16_t value, int bitPosition) {
  return (value & (1 << bitPosition)) != 0;  // Check if the bit at bitPosition is set
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  udp.begin(MAVLINK_PORT);

  preferences.begin("myfile", false);
  loadParametersFromPreferences();

  Serial.println("Initializing Serial2 for GPS...");
  MySerial.begin(9600, SERIAL_8N1, 16, 17);  // Initialize Serial2 for GPS with RX on GPIO16 and TX on GPIO17
  Serial.println("Serial2 initialized.");

  // Attach motors to their respective pins
  motor1.attach(motorPin1);
  motor2.attach(motorPin2);

  // Set initial positions to the minimum pulse width (1ms)
  motor1.write(midPulse);
  motor2.write(midPulse);
}

void loadParametersFromPreferences() {
  Serial.printf("Loaded parameters from Preferences:\n");
  for (int i = 0; i < sizeof(parameters) / sizeof(Parameter); ++i) {
    int32_t storedValue = preferences.getInt(parameters[i].name, parameters[i].defaultValue);
    *(parameters[i].value) = storedValue;  // Directly set the value from preferences or default
    Serial.printf("%s set to: %d\n", parameters[i].name, *(parameters[i].value));
  }
}

void send_mavlink_message(mavlink_message_t *msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  udp.beginPacket("255.255.255.255", MAVLINK_PORT);
  udp.write(buf, len);
  udp.endPacket();
}

void loop() {
  receiveMAVLink();
  static bool motorsNeutral = false;

  // Check if heartbeat has timed out
  if (millis() - lastHeartbeatTime > heartbeatTimeout) {
    if (!motorsNeutral) {
      Serial.println("MAVLink heartbeat lost. Setting motors to neutral.");
      // Set motors to a safe position
      motor1.write(midPulse);
      motor2.write(midPulse);
      motorsNeutral = true;  // Set the flag to true
    }
  } else {
    if (motorsNeutral) {
      Serial.println("MAVLink connection established.");
      motorsNeutral = false;
    }
  }

  // Send GPS data at intervals''''
  if (millis() - lastGPSSend >= gpsSendInterval) {
    sendMAVLink();
    sendGPSMAVLink();
    lastGPSSend = millis();  // Update the last send time
  }
}

void sendMAVLink() {
  static uint32_t lastSent = 0;
  if (millis() - lastSent < 1000) return;  // Send every second

  // Generate HEARTBEAT message buffer
  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, 0, MAV_STATE_ACTIVE);
  send_mavlink_message(&msg);

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
          lastHeartbeatTime = millis();
          break;
        case MAVLINK_MSG_ID_MANUAL_CONTROL:
          // Can be used to control the vehicle
          mavlink_manual_control_t manualControl;
          mavlink_msg_manual_control_decode(&msg, &manualControl);

          if (manualControl.buttons != 0 || manualControl.buttons2 != 0) {
            Serial.print("Received joystick button event:");
            Serial.print(" joystick button(0-15)=");
            printBinary(manualControl.buttons);
            Serial.print(" joystick buttons2(16-31)=");
            printBinary(manualControl.buttons2);
            Serial.println();  // New line after printing

            // Check if bit 0 is enabled (remember: bit indexing starts from 0)
            if (isBitEnabled(manualControl.buttons, 0)) {
              Serial.println("Bit 0 in manualControl.buttons(0-15) is enabled. Button (A) is Pressed -- submarine Down");
              submarine.down();
            } 
            else if  (isBitEnabled(manualControl.buttons, 13)) {
              Serial.println("Bit 13 in manualControl.buttons(0-15) is enabled. Button (Y) is Pressed -- submarine Up");
              submarine.up();
            } 
            else if  (isBitEnabled(manualControl.buttons, 3)) {
              Serial.println("Bit 3 in manualControl.buttons(0-15) is enabled. Button (LB-Left Bumper) is Pressed -- submarine Claw Open");
              submarine.clawOpen();
            } 
            else if  (isBitEnabled(manualControl.buttons, 5)) {
              Serial.println("Bit 5 in manualControl.buttons(0-15) is enabled. Button (RB-Right Bumper) is Pressed -- submarine Claw Close");
              submarine.clawClose();
            } 
          }

          // Print only non-zero values for s, t, aux1, aux2, aux3, aux4, aux5, aux6
          if (manualControl.s != 0) {
            Serial.print(" Pitch (s): ");
            Serial.println(manualControl.s);
          }
          if (manualControl.t != 0) {
            Serial.print(" Roll (t): ");
            Serial.println(manualControl.t);
          }
          if (manualControl.aux1 != 0) {
            Serial.print(" Aux1: ");
            Serial.println(manualControl.aux1);
          }
          if (manualControl.aux2 != 0) {
            Serial.print(" Aux2: ");
            Serial.println(manualControl.aux2);
          }
          if (manualControl.aux3 != 0) {
            Serial.print(" Aux3: ");
            Serial.println(manualControl.aux3);
          }
          if (manualControl.aux4 != 0) {
            Serial.print(" Aux4: ");
            Serial.println(manualControl.aux4);
          }
          if (manualControl.aux5 != 0) {
            Serial.print(" Aux5: ");
            Serial.println(manualControl.aux5);
          }
          if (manualControl.aux6 != 0) {
            Serial.print(" Aux6: ");
            Serial.println(manualControl.aux6);
          }
          // handle x y z r
          controlMotors(manualControl.z, manualControl.x);
          break;
        case MAVLINK_MSG_ID_COMMAND_LONG:  // Check for COMMAND_LONG (ID 76)
          mavlink_command_long_t commandLong;
          Serial.print("Received LONG message with ID ");
          Serial.println(commandLong.command);
          mavlink_msg_command_long_decode(&msg, &commandLong);
          // Check the command ID
          if (commandLong.command == 181 || commandLong.command == 183) {
            sendCustomResponse(commandLong.command);
          }
          else if (commandLong.command == MAV_CMD_PREFLIGHT_STORAGE){
            Serial.println("Factory reset command received.");
            resetToFactoryDefaults();
            sendCustomResponse(commandLong.command);
          }
          break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
          mavlink_handle_para_list_req();
          break;
        case MAVLINK_MSG_ID_PARAM_SET:     // Add case for PARAM_SET
          mavlink_handle_param_set(&msg);  // Call the function to handle PARAM_SET
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
          mvalink_handle_miss_list_req();
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
  // Prepare HEARTBEAT message
  mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &msg,
                             MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC, 0, 0, 0);
  send_mavlink_message(&msg);
}

void sendCustomResponse(uint8_t commandId) {

  uint8_t result = MAV_RESULT_ACCEPTED;               // Set the result for acknowledgment
  uint8_t progress = 0;                               // Set progress, if applicable (usually 0)
  int32_t result_param2 = 0;                          // Additional result parameter (can be used for more context)
  uint8_t target_system = 1;                          // Your system ID
  uint8_t target_component = MAV_COMP_ID_AUTOPILOT1;  // Your component ID
  Serial.print("Sending COMMAND_ACK for command ID: ");
  Serial.println(commandId);
  // Prepare MAVLink message
  mavlink_message_t msg;
  mavlink_msg_command_ack_pack(1, 1, &msg, commandId, result, progress, result_param2, target_system, target_component);
  // Send the message
  send_mavlink_message(&msg);
}

// Function to control motors based on z and y values
void controlMotors(int16_t z, int16_t x) {
  // Skip the operation if z and y are the same as last values
  if (z == lastZ && x == lastX) {
    return;
  }
  Serial.print("Current z: ");
  Serial.print(z);
  Serial.print(", Current x: ");
  Serial.println(x);

  // Map absolute values of z and y to pulse widths
  int pulseZForward = map(z, 500, 1000, midPulse, left_maxPulse);  
  int pulseZBackward = map(z, 0, 500, minPulse, midPulse);         
  int pulseXForward = map(x, 0, 1000, midPulse, right_maxPulse);   
  int pulseXBackward = map(x, -1000, 0, minPulse, midPulse);

  // Control left motor
  if (z > 550) {
    Serial.print("Pulse for left motor forward: ");
    Serial.println(pulseZForward);
    motor1.write(pulseZForward);
  } else if (z < 450) {
    Serial.print("Pulse for left motor backward: ");
    Serial.println(pulseZBackward);
    motor1.write(pulseZBackward);
  } else {
    Serial.print("left motor stop: ");
    Serial.println(midPulse);
    motor1.write(midPulse);  // Stop the motor
  }

  // Control right motor
  if (x > 50) {
    Serial.print("Pulse for right motor forward: ");
    Serial.println(pulseXForward);
    motor2.write(pulseXForward);
  } else if (x < -100) {
    Serial.print("Pulse for right motor backward: ");
    Serial.println(pulseXBackward);
    motor2.write(pulseXBackward);
  } else {
    Serial.print("right motor stop: ");
    Serial.println(midPulse);
    motor2.write(midPulse);  // Stop the motor
  }

  // Update last values
  lastZ = z;
  lastX = x;
}

void sendGPSMAVLink() {
  // Check if there is any data available on Serial2
  if (MySerial.available()) {
    //Serial.println("Data available from GPS:");
    while (MySerial.available()) {
      char c = MySerial.read();  // Read a character from Serial2
      // Serial.print(c); // Print the character to the Serial monitor
      if (gps.encode(c)) {
        // displayInfo();
        // Check if GPS data is valid
        if (gps.location.isValid()) {
          // Gather GPS data
          uint64_t time_usec = micros();
          uint8_t fix_type = gps.hdop.isValid() ? 3 : 0;  // Example fix type, adjust based on your logic
          int32_t lat = gps.location.lat() * 1e7;
          int32_t lon = gps.location.lng() * 1e7;
          int32_t alt = gps.altitude.meters() * 1000;
          uint16_t eph = gps.hdop.isValid() ? gps.hdop.hdop() * 100 : UINT16_MAX;
          uint16_t epv = UINT16_MAX;                                                   // Vertical dilution of position not provided by TinyGPS++
          uint16_t vel = gps.speed.isValid() ? gps.speed.kmph() * 27.78 : UINT16_MAX;  // Convert km/h to cm/s
          uint16_t cog = gps.course.isValid() ? gps.course.deg() * 100 : UINT16_MAX;
          uint8_t satellites_visible = gps.satellites.value();
          int32_t alt_ellipsoid = UINT32_MAX;  // Altitude above WGS84, not provided by TinyGPS++
          uint32_t h_acc = UINT32_MAX;         // Horizontal accuracy, not provided by TinyGPS++
          uint32_t v_acc = UINT32_MAX;         // Vertical accuracy, not provided by TinyGPS++
          uint32_t vel_acc = UINT32_MAX;       // Speed accuracy, not provided by TinyGPS++
          uint32_t hdg_acc = UINT32_MAX;       // Heading accuracy, not provided by TinyGPS++
          uint16_t yaw = 36000;                // Yaw, use 36000 for north as per the documentation

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
            yaw);

          send_mavlink_message(&msg);
          // Serial.println("Sent GPS data via MAVLink");
        }
      }
    }
  } else {
    // Serial.println("No data available from GPS.");
  }
}

void send_single_parameter(int index) {
  if (index < 0 || index >= numParameters) {
    Serial.println("Invalid parameter index.");
    return;
  }

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_param_value_t mvl_param;

  // Set the parameter details
  strcpy(mvl_param.param_id, parameters[index].name);  // Copy parameter ID
  mvl_param.param_type = MAV_PARAM_TYPE_INT32;         // Parameter type is int32

  // Use a union to assign int32 value to float field
  union {
    float float_value;
    int32_t int_value;
  } value;
  value.int_value = *(parameters[index].value);
  mvl_param.param_value = value.float_value;

  // Set param count and index
  mvl_param.param_count = numParameters;
  mvl_param.param_index = index;

  // Encode and send the parameter message
  mavlink_msg_param_value_encode(1, MAV_COMP_ID_AUTOPILOT1, &msg, &mvl_param);
  send_mavlink_message(&msg);

  // Print parameter and value in one line
  Serial.printf("Sent parameter: %s = %d\n", parameters[index].name, *(parameters[index].value));
}

void mavlink_handle_para_list_req() {
  for (int i = 0; i < numParameters; i++) {
    send_single_parameter(i);
  }
}


void mvalink_handle_miss_list_req() {
  // Send the MISSION_COUNT message with 0 count
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Define mission type and opaque ID (0 for no specific plan)
  uint8_t mission_type = 0;  // Set to your desired mission type if applicable
  uint32_t opaque_id = 0;    // Set to 0 as per your request

  // Pack the MISSION_COUNT message with a count of 0
  mavlink_msg_mission_count_pack(
    1,                       // system_id
    MAV_COMP_ID_AUTOPILOT1,  // component_id
    &msg,
    1,                       // target_system
    MAV_COMP_ID_AUTOPILOT1,  // target_component
    0,                       // count of mission items
    mission_type,            // mission type
    opaque_id                // opaque ID
  );

  // Send the MISSION_COUNT message
  send_mavlink_message(&msg);
  Serial.println("Sent MISSION_COUNT with count = 0");
}

void mavlink_handle_param_set(mavlink_message_t *msg) {
  mavlink_param_set_t param_set;
  mavlink_msg_param_set_decode(msg, &param_set);

  // Search for the parameter by name
  for (int i = 0; i < numParameters; i++) {
    if (strcmp(param_set.param_id, parameters[i].name) == 0) {
      // Decode the float value and update the parameter
      union {
        float float_value;
        int32_t int_value;
      } value;
      value.float_value = param_set.param_value;  // Set from MAVLink parameter value
       *(parameters[i].value) = value.int_value; 
      preferences.putInt(parameters[i].name, value.int_value);
      Serial.printf("Parameter %s updated to %d\n", parameters[i].name, value.int_value);

      // Send ACK with updated parameter value
      send_single_parameter(i);
      return;
    }
  }

  Serial.printf("Unknown parameter: %s\n", param_set.param_id);

  // Optional: Send an error ACK (here, using an arbitrary -1 value)
  mavlink_message_t error_msg;
  mavlink_msg_param_value_pack(1, 200, &error_msg,
                               param_set.param_id,
                               -1,  // Indicate error with an invalid value
                               MAV_PARAM_TYPE_INT32,
                               numParameters, -1);
  send_mavlink_message(&error_msg);
}

void displayInfo() {
  Serial.print(F("Location: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
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
  } else {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void resetToFactoryDefaults() {
  // Reset parameters to their default values
  minPulse = defaultMinPulse;
  midPulse = defaultMidPulse;
  left_maxPulse = defaultMaxPulse;
  right_maxPulse = defaultMaxPulse;

  for (int i = 0; i < sizeof(parameters) / sizeof(Parameter); ++i) {
    *(parameters[i].value) = parameters[i].defaultValue;
    preferences.putInt(parameters[i].name, parameters[i].defaultValue);
    Serial.printf("%s set to: %d\n", parameters[i].name, *(parameters[i].value));
  }

  Serial.println("Factory reset performed. Parameters reset to defaults.");
}