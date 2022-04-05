#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SH1107Wire.h"
#include "Seeed_BME280.h"
#include <Wire.h>
#include "RunningMedian.h"
#include "logger.h"
#include "credentials.h"

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

//    60000  ->   1 min
//   120000  ->   2 min
//   300000  ->   5 min
//   600000  ->  10 min
//   900000  ->  15 min
//  1200000  ->  20 min
//  1800000  ->  30 min
//  3600000  ->  60 min
/*the application data transmission duty cycle.  value in [ms].*/
uint32_t sleepTime = 1200000;
uint32_t appTxDutyCycle = sleepTime;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;
/*!
  Number of trials to transmit the frame, if the LoRaMAC layer did not
  receive an acknowledgment. The MAC performs a datarate adaptation,
  according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
  to the following table:

  Transmission nb | Data Rate
  ----------------|-----------
  1 (first)       | DR
  2               | DR
  3               | max(DR-1,0)* 4               | max(DR-1,0)
  5               | max(DR-2,0)
  6               | max(DR-2,0)
  7               | max(DR-3,0)
  8               | max(DR-3,0)

  Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
  the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

#define DEFAULT_LOG_LEVEL Error // DEBUG: set the Debug for more logging statements

#define WAKE_UP_PIN GPIO7

#define MINBATT 3300
#define MAXBATT 3700

#define CONTACT_BUTTON GPIO5
#define DISPLAY_JUMPER GPIO6

#define MIN_READING_SAMPLES 7
#define MAX_READING_WAIT_TIME 10000

extern SH1107Wire display;
BME280 bme280;
RunningMedian samples = RunningMedian(MIN_READING_SAMPLES);

int distance, humidity, temperature, batteryVoltage, batteryLevel, batteryPct, uptimeCount;
long pressure;
bool accelWoke = false;

char buffer[40];

void measureDistance() {
  logger::debug("Distance Measuring started");
  unsigned long startTime = millis();
  while (!samples.isFull()) {
    if ((startTime + MAX_READING_WAIT_TIME) < millis()) {
      logger::err("Error: Timed out reading distance data");
      break;
    }
    int result = readDistanceSample();
    if (result >= 0) {
      samples.add(result);
    }
    delay(150);
  }
  if (!samples.isFull()) {
    logger::err("Error: Could not measure distance");
    distance = -1;
  } else {
    logger::debug("Distance Measuring finished!");
    distance = samples.getMedian() / 10; // convert from mm to cm
  }
  samples.clear();
}

int readDistanceSample() {
  logger::debug("readDistanceSample");
  byte data[4];
  size_t len = Serial1.readBytesUntil('255', data, 4);
  Serial1.flush();
  if (len == 4) {
    if (data[0] == 0xff) {
      int sum = (data[0] + data[1] + data[2]) & 0x00FF;
      if (sum == data[3]) {
        float result = (data[1] << 8) + data[2];
        if (result > 250) {
          return result;
        } else {
          return 0;
        }
      } else {
        logger::err("ERROR: Checksum");
      }
    }
  } else if (len == 0) {
    logger::err("ERROR: No Serial1 Data");
  } else {
    logger::err("ERROR: Serial1 Data not matching");
  }
  return -1;
}

void measureAir() {
  logger::debug("measureAir");
  temperature = bme280.getTemperature();
  humidity = bme280.getHumidity();
  pressure = bme280.getPressure();
  //altitude = bme280.calcAltitude(pressure);
}

void measureBatteryData() {
  logger::debug("measureBatteryData");
  //detachInterrupt(USER_KEY);
  batteryVoltage = getBatteryVoltage();
  batteryLevel = (BoardGetBatteryLevel() / 254) * 100;
  batteryPct = map(batteryVoltage, MINBATT, MAXBATT, 0, 100);
  if (batteryPct < 0) {
    batteryPct = 0;
  } else if (batteryPct > 100) {
    batteryPct = 100;
  }
  //pinMode(USER_KEY, INPUT);
  //attachInterrupt(USER_KEY, onWakeUp, RISING);
}

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
  logger::info("Reading Sensor values");
  measureBatteryData();

  logger::debug("Turn on Vext");
  digitalWrite(Vext, LOW);
  if (LoRaWAN.isDisplayEnabled()) {
    displayUpTimeCount();
    delay(1000);
  } else {
    delay(500);
  }

  logger::debug("start bme280");
  if (!bme280.init()) {
    logger::err("Error: Could not init BME");
  }
  logger::debug("bme280 started");
  delay(500); //delay to let sensor settle

  logger::debug("Everything is now ready");

  measureDistance();
  measureAir();
  int leaking = digitalRead(CONTACT_BUTTON);

  appDataSize = 18;

  // water level distance
  appData[0] = highByte(distance);
  appData[1] = lowByte(distance);

  // water leak
  appData[2] = lowByte(leaking);

  // temperature
  int temperatureMasked = temperature * 100;
  appData[3] = highByte(temperatureMasked);
  appData[4] = lowByte(temperatureMasked);

  // humidity
  appData[5] = highByte(humidity);
  appData[6] = lowByte(humidity);

 // pressure
  appData[7] = (byte) ((pressure & 0xFF000000) >> 24 );
  appData[8] = (byte) ((pressure & 0x00FF0000) >> 16 );
  appData[9] = (byte) ((pressure & 0x0000FF00) >> 8  );
  appData[10] = (byte) ((pressure & 0X000000FF)      );

  // battery-voltate
  appData[11] = highByte(batteryVoltage);
  appData[12] = lowByte(batteryVoltage);

  // battery-pct
  appData[13] = lowByte(batteryPct);

  // battery-level
  appData[14] = highByte(batteryLevel);
  appData[15] = lowByte(batteryLevel);

  // counter
  appData[16] = highByte(uptimeCount);
  appData[17] = lowByte(uptimeCount);

  logger::info("Distance: %d", distance);
  logger::info("Temperature: %d", temperature);
  logger::info("Humidity: %d", humidity);
  logger::info("Pressure: %d", pressure);
  logger::info("BatteryVoltage: %d", batteryVoltage);
  logger::info("BatteryLevel: %d", batteryLevel);
  logger::info("BatteryPct: %d", batteryPct);
  logger::info("UptimeCount: %d", uptimeCount);

  if (LoRaWAN.isDisplayEnabled()) {
    display.init();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.clear();

    sprintf(buffer, "Data fetched");
    display.drawString(58, 0, buffer);

    sprintf(buffer, "Dist: %d [cm]", distance);
    display.drawString(58, 10, buffer);

    sprintf(buffer, "Tmp: %d [C]", temperature);
    display.drawString(58, 22, buffer);

    sprintf(buffer, "Hum: %d [%%]", humidity);
    display.drawString(58, 33, buffer);

    if (leaking == HIGH) {
      sprintf(buffer, "Leak: Ja");
    } else {
      sprintf(buffer, "Leak: Nein");
    }
    display.drawString(58, 44, buffer);

    display.display();
    delay(2000);
  }
  uptimeCount ++;
}

void setDisplayState() {
  if (digitalRead(DISPLAY_JUMPER) == HIGH && !LoRaWAN.isDisplayEnabled()) {
    logger::info("Enable display and rgb");
    LoRaWAN.enableDisplay();
    LoRaWAN.enableRgb();
  } else if (digitalRead(DISPLAY_JUMPER) == LOW && LoRaWAN.isDisplayEnabled()) {
    logger::info("Disable display and rgb");
    LoRaWAN.disableDisplay();
    LoRaWAN.disableRgb();
  }
}

void displayUpTimeCount() {
  display.init();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.clear();
  sprintf(buffer, "%d", uptimeCount);
  display.drawString(58, 22, buffer);
  display.display();
}

void onWakeUp() {
  delay(10);
  if (digitalRead(WAKE_UP_PIN) == HIGH) {
    Serial.println(F("Woke up by WAKE_UP_PIN during sleep"));
    LoRaWAN.enableDisplay();
    LoRaWAN.enableRgb();
    accelWoke = true;
    delay(50);
  }
}

void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  logger::debug("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    logger::debug("REV DATA: %02X \r\n", mcpsIndication->Buffer[i]);
  }
  if (mcpsIndication->Port == 4) {
    int newSleepTime = mcpsIndication->Buffer[1] | (mcpsIndication->Buffer[0] << 8);
    sleepTime  = newSleepTime * 1000;
    logger::info("Changed Sleep Time to: %d", sleepTime);
  }
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial1.setTimeout(10);

  logger::set_serial(Serial);
  logger::set_level(logger::DEFAULT_LOG_LEVEL);

  pinMode(Vext, OUTPUT);
  pinMode(CONTACT_BUTTON, INPUT);

  pinMode(DISPLAY_JUMPER, INPUT_PULLUP);
  attachInterrupt(DISPLAY_JUMPER, setDisplayState, CHANGE);

  if (digitalRead(DISPLAY_JUMPER) == HIGH) {
    LoRaWAN.enableDisplay();
    LoRaWAN.enableRgb();
  }

#if(AT_SUPPORT)
  enableAt();
#endif
  LoRaWAN.displayMcuInit();
  deviceState = DEVICE_STATE_INIT;
  LoRaWAN.ifskipjoin();

  accelWoke = false;
  pinMode(WAKE_UP_PIN, INPUT_PULLUP);
  attachInterrupt(WAKE_UP_PIN, onWakeUp, RISING);
}

void loop() {

  switch ( deviceState )
  {
    case DEVICE_STATE_INIT:
      {
#if(LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
#if(AT_SUPPORT)
        getDevParam();
#endif
        logger::debug("DEVICE_STATE_INIT");
        printDevParam();
        LoRaWAN.init(loraWanClass, loraWanRegion);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        logger::debug("DEVICE_STATE_JOIN");
        LoRaWAN.displayJoining();
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        logger::debug("DEVICE_STATE_SEND");
        LoRaWAN.displaySending();
        prepareTxFrame( appPort );
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        logger::debug("DEVICE_STATE_CYCLE");
        // Schedule next packet transmission
        appTxDutyCycle = sleepTime;
        txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        logger::info("Go to sleep for: %d sec", (int) (txDutyCycleTime / 1000.0));
        delay(100);
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        if (accelWoke) {
          if (IsLoRaMacNetworkJoined) {
            LoRaWAN.displaySending();
            prepareTxFrame( appPort );
            LoRaWAN.send();
            setDisplayState();
          }
          accelWoke = false;
        }
        LoRaWAN.displayAck();
        LoRaWAN.sleep();
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  }
}
