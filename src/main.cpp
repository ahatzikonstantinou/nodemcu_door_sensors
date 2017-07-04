#include <FS.h> //for WiFiManager this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//needed for library
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>  //https://github.com/bblanchon/ArduinoJson
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager
#include "NTPClient.h"    //https://github.com/stelgenhof/NTPClient

#define PIN_SWITCH 5 // ahat: this is GPIO5 = D1 in nodemcu v3
#define PIN_LED 13
#define PIN_FLASH 0

int previousFlashState = 1; //ahat: this is important. 0: PRESSED, 1: RELEASED. previousFlashState must start with 1
                            //or else as soon as the first input is read it will look like FLASH was pressed

// default values
// const char* ssid = "Security";
// const char* password = "312ggp12";
char mqtt_server[40];
char mqtt_port[6];
char publish_topic[256];
char subscribe_topic[256];

#define SENSOR_COUNT 5
#define SENSOR_GPIO_SIZE 3
#define SENSOR_NAME_SIZE 128
typedef struct
{
  char gpio[SENSOR_GPIO_SIZE];
  char name[SENSOR_NAME_SIZE];
  int previousState;
} Sensor;
Sensor sensors[SENSOR_COUNT];

// char sensors_gpio[SENSOR_COUNT][SENSOR_GPIO_SIZE];
// char sensors_name[SENSOR_COUNT][SENSOR_NAME_SIZE];
// int sensors_previousState[SENSOR_COUNT];

// char sensor1_gpio[2];
// char sensor2_gpio[2];
// char sensor3_gpio[2];
// char sensor4_gpio[2];
// char sensor5_gpio[2];
// char sensor1_name[128];
// char sensor2_name[128];
// char sensor3_name[128];
// char sensor4_name[128];
// char sensor5_name[128];

WiFiClient espClient;
PubSubClient client( espClient );

void mqttReconnect()
{
  // Loop until we're reconnected
  while( !client.connected() )
  {
    Serial.print( "Attempting MQTT connection..." );
    // Attempt to connect
    if( client.connect( "ESP8266 Client" ) )
    {
      Serial.println( "connected" );
      // ... and subscribe to topic
      client.subscribe( subscribe_topic );
    }
    else
    {
      Serial.print( "failed, rc=" );
      Serial.print( client.state() );
      Serial.println( " try again in 2 seconds" );
      // Wait 5 seconds before retrying
      delay( 2000 );
    }
  }
  client.loop();
}

void loopMqttConnect()
{
  if( !client.connected() )
  {
    mqttReconnect();
  }
}

bool isNumber( char* chars )
{
  // Serial.printf( "Testing if %s is a number\n", chars );
  if( strlen( chars ) == 0 )
  {
    // Serial.printf( "String '%s' is empty, so not a number\n", chars );
    return false;
  }

  for( int i = 0 ; i < strlen( chars ) ; i++ )
  {
    char c = chars[i];
    // Serial.printf( "Testing char '%c'\n", c );
    if( c != '0' && c != '1' && c != '2' && c != '3' && c != '4' && c != '5' && c != '6' && c != '7' && c != '8' && c != '9' )
    {
      // Serial.println( "Returning false" );
      return false;
    }
  }
  // Serial.println( "Returning true" );
  return true;

  // char *endptr;
  // strtol(chars, &endptr, 10);
  //
  // bool success = (*endptr == '\0'); /* read the manual in the link to understand this please */
  // free( endptr );
  // return success;
}

void sensorSetup()
{
  for( int i = 0 ; i < SENSOR_COUNT ; i++ )
  {
    if( !isNumber( sensors[i].gpio ) )
    {
      continue;
    }
    // Serial.printf( "Will set pin of sensor[%d], gpio='%s'\n", i, sensors[i].gpio );
    int pin = String( sensors[i].gpio ).toInt();
    pinMode( pin, INPUT_PULLUP );
    sensors[i].previousState = ! digitalRead( pin );
    // Serial.printf( "Finished setting pin of sensor[%d], gpio='%s'\n", i, sensors[i].gpio );
  }
}

/* ahat: replaced with WiFiManager
void wifiSetup()
{
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay( 2000 );

  Serial.print( "Connecting to " );
  Serial.println( ssid );
  WiFi.begin( ssid, password );
  while( WiFi.status() != WL_CONNECTED )
  {
    delay( 500 );
    Serial.print( "." );
  }
  Serial.println( "" );
  Serial.println( "WiFi connected" );
  Serial.println( "IP address: " );
  Serial.println( WiFi.localIP() );
}
*/

void flashSetup()
{
  pinMode( PIN_FLASH, INPUT_PULLUP );
  //ahat: this is important. 0: PRESSED, 1: RELEASED. previousFlashState must start with 1
  //or else as soon as the first input is read it will look like FLASH was pressed
  previousFlashState = 1; // Starting with Flash RELEASED
  Serial.print( "Starting with previousFlashState:" );
  Serial.println( previousFlashState );
}

void scanNetworks()
{
  Serial.println("scan start");


  int n = WiFi.scanNetworks();// WiFi.scanNetworks will return the number of networks found
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE)?" ":"*");
      delay(10);
    }
  }
  Serial.println("");

  // Wait a bit before scanning again
  delay(5000);
}

void mqttPublish( String sensorName, char* message )
{
    if( client.connected() )
    {
      String topic = publish_topic + String( "/" ) + sensorName;
      char _topic[ sizeof( topic ) + 1 ]; topic.toCharArray( _topic, sizeof( _topic ) + 1 ) ;
      Serial.printf( "Publishing: [%s] ", _topic );
      Serial.println( message );
      client.publish( _topic, message );
    }
    else
    {
      Serial.println( "Cannot publish because client is not connected." );
    }
}

void loopReadSensorMqttPublish( bool changesOnly )
{
  for( int i = 0 ; i < SENSOR_COUNT ; i++ )
  {
    // Serial.printf( "Reading sensor[%d] gpio='%s', name='%s'\n", i, sensors[i].gpio, sensors[i].name );
    if( !isNumber( sensors[i].gpio ) )
    {
      continue;
    }
    // Serial.printf( "Will string read pin of sensor[%d], gpio:'%s'\n", i, sensors[i].gpio );
    int pin = String( sensors[i].gpio ).toInt();
    // Serial.printf( "Will digitalRead pin %d\n", pin );
    int inputState = digitalRead( pin );
    // Serial.printf( "Will compare inputState %d with previousState: %d\n", inputState, sensors[i].previousState );
    if( !changesOnly || inputState != sensors[i].previousState )
    {
      Serial.printf( "Sensor[%d] gpio='%s' is ", i, sensors[i].gpio );
      if( inputState )
      {
        Serial.println( "OPEN" );
        mqttPublish( sensors[i].gpio, "OPEN" );
      }
      else
      {
        Serial.println( "CLOSED" );
        mqttPublish( sensors[i].gpio, "CLOSED" );
      }
      sensors[i].previousState = inputState;
    }
    // Serial.printf( "Finised reading sensor[%d]\n", i );
  }
  // Serial.println( "Finished reading all sensors" );
  client.loop();
}

void callback( char* topic, byte* payload, unsigned int length)
{
  Serial.print( "Message arrived [" );
  Serial.print( topic );
  Serial.print( "] " );
  for (int i=0;i<length;i++)
  {
    char receivedChar = (char)payload[i];
    Serial.print(receivedChar);
    // if (receivedChar == '0')
    // {
    //   // ESP8266 outputs are "reversed"
    //   digitalWrite( PIN_LED, HIGH );
    // }
    // if (receivedChar == '1')
    // {
    //   digitalWrite( PIN_LED, LOW );
    // }
  }
  Serial.println();

  loopReadSensorMqttPublish( false );
}

void mqttSetup()
{
  // Serial.printf( "Will try to read mqtt_port %s in a String\n", mqtt_port );
  String port( mqtt_port );
  // Serial.printf( "Will try to set mqtt server to %s and port to %d\n", mqtt_server, port.toInt() );
  client.setServer( mqtt_server, port.toInt() );
  // Serial.printf( "Will try to set mqtt callback\n" );
  client.setCallback( callback );
}

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback ()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// if autoConnect = true wifimanager will attempt to connect with previous known SSID and password
// else it will try ondemand configuration
void setupWifiManager( bool autoConnect )
{
  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(publish_topic, json["publish_topic"]);
          strcpy(subscribe_topic, json["subscribe_topic"]);
          for( int i = 0 ; i < SENSOR_COUNT ; i++ )
          {
            String id = "sensor" + String(i+1) + "_gpio";
            strncpy( sensors[i].gpio, json[ id ], SENSOR_GPIO_SIZE );
            id = "sensor" + String(i+1) + "_name";
            strncpy( sensors[i].name, json[ id ], SENSOR_NAME_SIZE );
            Serial.printf( "Loaded json in sensor[%d]: gpio='%s', name='%s'\n", i, sensors[i].gpio, sensors[i].name );
          }
          // strcpy(sensor1_gpio, json["sensor1_gpio"]);
          // strcpy(sensor2_gpio, json["sensor2_gpio"]);
          // strcpy(sensor3_gpio, json["sensor3_gpio"]);
          // strcpy(sensor4_gpio, json["sensor4_gpio"]);
          // strcpy(sensor5_gpio, json["sensor5_gpio"]);
          // strcpy(sensor1_name, json["sensor1_name"]);
          // strcpy(sensor2_name, json["sensor2_name"]);
          // strcpy(sensor3_name, json["sensor3_name"]);
          // strcpy(sensor4_name, json["sensor4_name"]);
          // strcpy(sensor5_name, json["sensor5_name"]);
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_publish_topic( "publish_topic", "publish topic", publish_topic, 256);
  WiFiManagerParameter custom_subscribe_topic( "subscribe_topic", "subscribe topic", subscribe_topic, 256);

  // WiFiManagerParameter* sensor_param[SENSOR_COUNT*2];
  // WiFiManagerParameter* sp;
  // for( int i = 0 ; i < SENSOR_COUNT ; i++ )
  // {
  //   int paramIndex = i*2;
  //   String id = "sensor" + String( i + 1 ) + "_gpio";
  //   char _id[ sizeof( id ) + 1 ]; id.toCharArray( _id, sizeof( _id ) + 1 ) ;
  //   String placeholder = "sensor" + String( i + 1 ) + " gpio";
  //   char _placeholder[ sizeof( placeholder ) + 1 ]; placeholder.toCharArray( _placeholder, sizeof( _placeholder ) + 1 ) ;
  //   String defaultValue = NULL != sensors[i].gpio ? sensors[i].gpio : "" ;
  //   char _defaultValue[ sizeof( defaultValue ) + 1 ]; defaultValue.toCharArray( _defaultValue, sizeof( _defaultValue ) + 1 ) ;
  //   sp = new WiFiManagerParameter( _id, _placeholder, _defaultValue, SENSOR_GPIO_SIZE );
  //   sensor_param[ paramIndex ] = sp;
  //   Serial.printf( "Created WMParam[%d] %s, %s, %s, %d\n, ", paramIndex, _id, _placeholder, _defaultValue, SENSOR_GPIO_SIZE );
  //   Serial.printf( "sp: %d, sensor_param[%d]:%d\n", sp, paramIndex, sensor_param );
  //   Serial.printf( "Sensor_param[%]: %s, %s, %s, %d\n", paramIndex, sensor_param[paramIndex]->getID(), sensor_param[paramIndex]->getPlaceholder(), sensor_param[paramIndex]->getValue(), sensor_param[paramIndex]->getValueLength() );
  //   delete sp;
  //
  //   id = "sensor" + String( i + 1 ) + "_name";
  //   char name_id[ sizeof( id ) + 1 ]; id.toCharArray( name_id, sizeof( name_id ) + 1 ) ;
  //   placeholder = "sensor" + String( i + 1 ) + " name";
  //   char name_placeholder[ sizeof( placeholder ) + 1 ]; placeholder.toCharArray( name_placeholder, sizeof( name_placeholder ) + 1 ) ;
  //   defaultValue = NULL != sensors[i].name ? sensors[i].name : "" ;
  //   char name_defaultValue[ sizeof( defaultValue ) + 1 ]; defaultValue.toCharArray( name_defaultValue, sizeof( name_defaultValue ) + 1 ) ;
  //   // sp = new WiFiManagerParameter( name_id, name_placeholder, name_defaultValue, SENSOR_NAME_SIZE );
  //   // sensor_param[ paramIndex + 1 ] = sp;
  //   Serial.printf( "Created WMParam[%d] %s, %s, %s, %d\n, ", (paramIndex + 1), name_id, name_placeholder, name_defaultValue, SENSOR_NAME_SIZE );
  // }
  WiFiManagerParameter custom_sensor1_gpio( "sensor1_gpio", "sensor1 gpio", sensors[0].gpio, SENSOR_GPIO_SIZE);
  WiFiManagerParameter custom_sensor1_name( "sensor1_name", "sensor1 name", sensors[0].name, SENSOR_NAME_SIZE);
  WiFiManagerParameter custom_sensor2_gpio( "sensor2_gpio", "sensor2 gpio", sensors[1].gpio, SENSOR_GPIO_SIZE);
  WiFiManagerParameter custom_sensor2_name( "sensor2_name", "sensor2 name", sensors[1].name, SENSOR_NAME_SIZE);
  WiFiManagerParameter custom_sensor3_gpio( "sensor3_gpio", "sensor3 gpio", sensors[2].gpio, SENSOR_GPIO_SIZE);
  WiFiManagerParameter custom_sensor3_name( "sensor3_name", "sensor3 name", sensors[2].name, SENSOR_NAME_SIZE);
  WiFiManagerParameter custom_sensor4_gpio( "sensor4_gpio", "sensor4 gpio", sensors[3].gpio, SENSOR_GPIO_SIZE);
  WiFiManagerParameter custom_sensor4_name( "sensor4_name", "sensor4 name", sensors[3].name, SENSOR_NAME_SIZE);
  WiFiManagerParameter custom_sensor5_gpio( "sensor5_gpio", "sensor5 gpio", sensors[4].gpio, SENSOR_GPIO_SIZE);
  WiFiManagerParameter custom_sensor5_name( "sensor5_name", "sensor5 name", sensors[4].name, SENSOR_NAME_SIZE);
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  // wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  //add all your parameters here
  wifiManager.addParameter( &custom_mqtt_server );
  wifiManager.addParameter( &custom_mqtt_port );
  wifiManager.addParameter( &custom_publish_topic );
  wifiManager.addParameter( &custom_subscribe_topic );
  // for( int i = 0 ; i < SENSOR_COUNT*2 ; i++ )
  // {
    // wifiManager.addParameter( sensor_param[i] );
    // Serial.printf( "Sensor_param[%]: %s, %s, %s, %d\n", i, sensor_param[i]->getID(), sensor_param[i]->getPlaceholder(), sensor_param[i]->getValue(), sensor_param[i]->getValueLength() );
  // }
  wifiManager.addParameter( &custom_sensor1_gpio );
  wifiManager.addParameter( &custom_sensor1_name );
  wifiManager.addParameter( &custom_sensor2_gpio );
  wifiManager.addParameter( &custom_sensor2_name );
  wifiManager.addParameter( &custom_sensor3_gpio );
  wifiManager.addParameter( &custom_sensor3_name );
  wifiManager.addParameter( &custom_sensor4_gpio );
  wifiManager.addParameter( &custom_sensor4_name );
  wifiManager.addParameter( &custom_sensor5_gpio );
  wifiManager.addParameter( &custom_sensor5_name );

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  if( autoConnect )
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.autoConnect("AutoConnectAP", "password"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }
  else
  {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay( 2000 );
    if (!wifiManager.startConfigPortal("OnDemandAP"))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //read updated parameters
  strcpy( mqtt_server, custom_mqtt_server.getValue() );
  strcpy( mqtt_port, custom_mqtt_port.getValue() );
  strcpy( publish_topic, custom_publish_topic.getValue() );
  strcpy( subscribe_topic, custom_subscribe_topic.getValue() );
  // for( int i = 0 ; i < SENSOR_COUNT*2 ; i = i + 2 )
  // {
  //   strcpy( sensors[i].gpio, sensor_param[i]->getValue() );
  //   strcpy( sensors[i].name, sensor_param[i+1]->getValue() );

    // //release memory
    // delete sensor_param[i];
    // delete sensor_param[i+1];
  // }
  strncpy( sensors[0].gpio, custom_sensor1_gpio.getValue(), SENSOR_GPIO_SIZE );
  strncpy( sensors[0].name, custom_sensor1_name.getValue(), SENSOR_NAME_SIZE );
  strncpy( sensors[1].gpio, custom_sensor2_gpio.getValue(), SENSOR_GPIO_SIZE );
  strncpy( sensors[1].name, custom_sensor2_name.getValue(), SENSOR_NAME_SIZE );
  strncpy( sensors[2].gpio, custom_sensor3_gpio.getValue(), SENSOR_GPIO_SIZE );
  strncpy( sensors[2].name, custom_sensor3_name.getValue(), SENSOR_NAME_SIZE );
  strncpy( sensors[3].gpio, custom_sensor4_gpio.getValue(), SENSOR_GPIO_SIZE );
  strncpy( sensors[3].name, custom_sensor4_name.getValue(), SENSOR_NAME_SIZE );
  strncpy( sensors[4].gpio, custom_sensor5_gpio.getValue(), SENSOR_GPIO_SIZE );
  strncpy( sensors[4].name, custom_sensor5_name.getValue(), SENSOR_NAME_SIZE );
  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["publish_topic"] = publish_topic;
    json["subscribe_topic"] = subscribe_topic;
    for( int i = 0 ; i < SENSOR_COUNT ; i++ )
    {
      String gpio = "sensor" + String( i + 1 ) + "_gpio";
      json[ gpio ] = sensors[i].gpio;
      String name = "sensor" + String( i + 1 ) + "_name";
      json[ name ] = sensors[i].name;
    }
    // json["sensor1_gpio"] = sensor1_gpio;
    // json["sensor1_name"] = sensor1_name;
    // json["sensor2_gpio"] = sensor2_gpio;
    // json["sensor2_name"] = sensor2_name;
    // json["sensor3_gpio"] = sensor3_gpio;
    // json["sensor3_name"] = sensor3_name;
    // json["sensor4_name"] = sensor4_name;
    // json["sensor4_gpio"] = sensor4_gpio;
    // json["sensor5_name"] = sensor5_name;
    // json["sensor5_gpio"] = sensor5_gpio;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());

}

void loopReadFlash()
{
  int inputState = digitalRead( PIN_FLASH );
  if( inputState != previousFlashState )
  {
    // Serial.printf( "In loopReadFlash: previousFlashState = %d, inputState = %d\n", previousFlashState, inputState );
    Serial.print("Flash is " );
    if( inputState )
    {
      Serial.println( "RELEASED" );
      // start wifimanager to reconfigure connection to wifi
      setupWifiManager( false );

      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");
    }
    else
    {
      Serial.println( "PRESSED" );
    }
    previousFlashState = inputState;
  }

  delay( 100 );
}

void setup()
{
  Serial.begin( 115200 );
  // wifiSetup();
  flashSetup();
  Serial.println( "flashSetup finished" );
  setupWifiManager( true );
  Serial.println( "setupWifiManager finished" );
  sensorSetup();
  Serial.println( "sensorSetup finished" );
  mqttSetup();
  Serial.println( "mqttSetup finished" );

  NTP.init((char *)"pool.ntp.org", UTC0300);
  NTP.setPollingInterval(60); // Poll every minute

  Serial.println( "Setup done" );
}

void loop()
{
  // scanNetworks();
  loopMqttConnect();

  loopReadSensorMqttPublish( true );

  loopReadFlash();

  // example use of NTP
  Serial.printf( "Current time: %s - First synchronized at: %s.\n", NTP.getTimeDate( now() ), NTP.getTimeDate( NTP.getFirstSync() ) );
  // delay( 2000 );
}
