/*  Simple GPS tracker
    2-26-23
    Author / Modifier - Stephen Witty switty@level500.com

    Project - create simple GPS tracker using RAK4631 - based on modified version from RAK WisBlock

    Project should be considered POC / Demo code

    Code based RAK WisBlock GPS tracker example code with modifications
    Base code was taken from Arduino IDE examples that RAK included - GPS tracker solution

    Using RAK4631 with RAK1910 (GPS module) in slot A of baseboard

    V1 - 2-26-23  Initial dev work
*/

//Original RAK comment header below - 3-axis acceleration code was removed among other things removed/changed/added 

/**
 * @file GPS_Tracker.ino
 * @author rakwireless.com
 * @brief This sketch demonstrate a GPS tracker that collect location from a uBlox M7 GNSS sensor
 *    and send the data to lora gateway.
 *    It uses a 3-axis acceleration sensor to detect movement of the tracker
 * @version 0.2
 * @date 2021-04-30
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */

#include <Arduino.h>
#include <LoRaWan-RAK4630.h> //http://librarymanager/All#SX126x
//#include <SPI.h>
//#include "Wire.h"
#include <TinyGPS.h>        //http://librarymanager/All#TinyGPS
#include "keys.h"

#define VERSION "1"

TinyGPS gps;

String tmp_data = "";
int direction_S_N = 0;  //0--S, 1--N
int direction_E_W = 0;  //0--E, 1--W

// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                     /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t gCurrentClass = CLASS_A;          /* class definition*/
LoRaMacRegion_t gCurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm gCurrentConfirm = LMH_UNCONFIRMED_MSG;          /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
 */
static lmh_param_t lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };
                                       
// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 20000                        /**< Defines for user timer, the application data transmission interval. 10s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];        //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }

  // Initialize LoRa chip.
  lora_rak4630_init();

  Serial.println("=====================================");
  Serial.print("Simple GPS Tracker Version: ");
  Serial.println(VERSION);

  //gps init
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, 0);
  delay(1000);
  digitalWrite(WB_IO2, 1);
  delay(1000);
  
  Serial1.begin(9600);
  while (!Serial1);
  Serial.println("gps uart init ok!");
  
  //creat a user timer to send data to server period
  uint32_t err_code;

  err_code = timers_init();
  if (err_code != 0)
  {
    Serial.printf("timers_init failed - %d\n", err_code);
    return;
  }

  // Setup the EUIs and Keys
  lmh_setDevEui(nodeDeviceEUI);
  lmh_setAppEui(nodeAppEUI);
  lmh_setAppKey(nodeAppKey);

  // Initialize LoRaWan
  err_code = lmh_init(&lora_callbacks, lora_param_init, doOTAA, gCurrentClass, gCurrentRegion);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
}

void loop()
{
  
}

/**@brief LoRa function for handling HasJoined event.
 */
void lorawan_has_joined_handler(void)
{
  Serial.println("Network Joined!");

  lmh_error_status ret = lmh_class_request(gCurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
}

/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, gCurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  lmh_error_status error = lmh_send(&m_lora_app_data, gCurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
  TimerStart(&appTimer);
}

/**@brief Function for analytical direction.
 */
void direction_parse(String tmp)
{
    if (tmp.indexOf(",E,") != -1)
    {
        direction_E_W = 0;
    }
    else
    {
        direction_E_W = 1;
    }
    
    if (tmp.indexOf(",S,") != -1)
    {
        direction_S_N = 0;
    }
    else
    {
        direction_S_N = 1;
    }
}

/**@brief Function for handling a LoRa tx timer timeout event.
 */
String data = "";
void tx_lora_periodic_handler(void)
{ 
  bool newData = false;
  
  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
     // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      tmp_data += c;
      if (gps.encode(c))// Did a new valid sentence come in?
          newData = true;
    }
  }
  
  direction_parse(tmp_data);
  tmp_data = "";
  float flat, flon;
  int32_t ilat, ilon;
  if (newData)
  {
    unsigned long age;  
    gps.f_get_position(&flat, &flon, &age);
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat;
    ilat = flat * 100000;
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon;
    ilon = flon * 100000;
    memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
    m_lora_app_data.port = gAppPort;
    m_lora_app_data.buffer[0] = 0x09;
    //lat data
    m_lora_app_data.buffer[1] = (ilat & 0xFF000000) >> 24;
    m_lora_app_data.buffer[2] = (ilat & 0x00FF0000) >> 16;
    m_lora_app_data.buffer[3] = (ilat & 0x0000FF00) >> 8;
    m_lora_app_data.buffer[4] =  ilat & 0x000000FF;
    if(direction_S_N == 0)
    {
      m_lora_app_data.buffer[5] = 'S';    
    }
    else
    {
      m_lora_app_data.buffer[5] = 'N';    
    }
    //lon data
    m_lora_app_data.buffer[6] = (ilon & 0xFF000000) >> 24;
    m_lora_app_data.buffer[7] = (ilon & 0x00FF0000) >> 16;
    m_lora_app_data.buffer[8] = (ilon & 0x0000FF00) >> 8;
    m_lora_app_data.buffer[9] =  ilon & 0x000000FF;
    if(direction_E_W == 0)
    {
     m_lora_app_data.buffer[10] = 'E';
    }
    else
    {
      m_lora_app_data.buffer[10] = 'W';
    }
    m_lora_app_data.buffsize = 11;
    send_lora_frame();
    }
    else
    {
      Serial.println("*************** No Location Found");
      TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
      TimerStart(&appTimer);
    }
  }

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
uint32_t timers_init(void)
{
  TimerInit(&appTimer, tx_lora_periodic_handler);
  return 0;
}
