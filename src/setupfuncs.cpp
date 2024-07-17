#ifndef SETUPFUNCS
#define SETUPFUNCS

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

void OTA_Setup();
void ROS_Setup(std_msgs::Float32MultiArray &VAR);
void WIFI_Setup(const IPAddress IPV4, const IPAddress GATEWAY, const IPAddress SUBNET);

// OTA setup function
void OTA_Setup() {

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Tracker");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"temqueverissoai");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ROS setup function
void ROS_Setup(IPAddress SERVER, const uint16_t SERVERPORT {
  
    // ROS variables and functions
    ros::NodeHandle nh; // Node handle object
    geometry_msgs::Twist rosvel;
    std_msgs::Float32MultiArray encoderReadings;
    void ROS_Setup(std_msgs::Float32MultiArray &VAR);
    void velCb(const geometry_msgs::Twist &MSG);
    ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &velCb);
    ros::Publisher encoderFb("encoder_fb", &encoderReadings);

    VAR.layout.dim = (std_msgs::MultiArrayDimension*)malloc(sizeof(std_msgs::MultiArrayDimension));
    VAR.layout.dim[0].label = "encoderFb";
    VAR.layout.dim[0].size = 2;
    VAR.layout.dim[0].stride = 2;
    VAR.layout.data_offset = 0;
    VAR.data_length = 2;
    VAR.data = (float*)malloc(sizeof(float) * 2);

    nh.getHardware()->setConnection(SERVER, SERVERPORT);
    nh.initNode();
    nh.subscribe(cmd_vel);
    nh.advertise(encoderFb);
    while (!nh.connected()) {
    ArduinoOTA.handle();
    nh.spinOnce();
    Serial.print(".");
    wait(800);
    }
    Serial.println("I come to serve, not to be served (Connected to master).");
}

void WIFI_Setup(const IPAddress IPV4, const IPAddress GATEWAY, const IPAddress SUBNET) {
   if (!WiFi.config(IPV4, GATEWAY, SUBNET)) {
    Serial.println("The static IP setup failed.");
  }
  WiFi.begin(SSID, PASSWD);
  WiFi.mode(WIFI_STA);
  Serial.println("Connecting to WIFI...");
  while (WiFi.status() != WL_CONNECTED) {
    ArduinoOTA.handle();
    Serial.print(".");
    wait(800);
  }
  Serial.printf("\nConnected to WIFI.\n");
}

#endif // SETUPFUNCS