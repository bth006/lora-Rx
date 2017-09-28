// rf95_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing server
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95  if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example rf95_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with
// the RFM95W, Adafruit Feather M0 with RFM95

#define DEBUG   //If you comment this line, the DPRINT & DPRINTLN lines are defined as blank.
#ifdef DEBUG    //Macros are usually in all capital letters.
  #define DPRINT(...)    Serial.print(__VA_ARGS__)     //DPRINT is a macro, debug print
  #define DPRINTln(...)  Serial.println(__VA_ARGS__)   //DPRINTLN is a macro, debug print with new line
#else
  #define DPRINT(...)     //now defines a blank line
  #define DPRINTln(...)   //now defines a blank line
#endif

#include <SPI.h>
#include <RH_RF95.h>
long readVcc(void);
// Singleton instance of the radio driver
//RH_RF95 rf95;
RH_RF95 rf95(10, 2); // Select, interupt
//RH_RF95 rf95(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
//RH_RF95 rf95(8, 3); // Adafruit Feather M0 with RFM95

// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB



struct payloadDataStruct{
  byte nodeID;
  byte rssi;
  int voltage;
}rxpayload;

payloadDataStruct txpayload;

byte tx_buf[sizeof(txpayload)] = {0};
int led = 4;
const byte nodeID=0;

void setup()

{
  txpayload.nodeID=nodeID;
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);  //reset pin

  delay(1000);
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
//  pinMode(4, OUTPUT);
//  digitalWrite(4, HIGH);

  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while (!Serial) ; // Wait for serial port to be available
  if (!rf95.init())
    DPRINT("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
//  driver.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);
DPRINTln("init ok");
rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);//th
delay(100);

}

void loop()
{

  if (rf95.available())
  {

    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      digitalWrite(led, HIGH);
//      RH_RF95::printBuffer("request: ", buf, len);
      //DPRINT("got request: ");

      //rf95.printBuffer("Got :", buf, len);
        memcpy(&rxpayload, buf, sizeof(rxpayload));
    DPRINT(millis()/1000);
DPRINT(" nodeID = ");DPRINT(rxpayload.nodeID);
    DPRINT(" payload size ");   DPRINT(sizeof(rxpayload));
        DPRINT(" remote voltage = ");DPRINT(rxpayload.voltage);
          DPRINT(" remote rssi = ");DPRINT(rxpayload.rssi);


      //DPRINTln((char*)buf);
      DPRINT(" Local RSSI: ");
      DPRINT(rf95.lastRssi(), DEC);
      //delay(1000);//th


      // Send a reply
      //uint8_t data[] = "And hello back to you";
      //rf95.send(data, sizeof(data));


      byte absrssi = abs(rf95.lastRssi());
      txpayload.rssi = absrssi;
      txpayload.voltage=(int)readVcc();



      memcpy(tx_buf, &txpayload, sizeof(txpayload) );
        byte zize=sizeof(txpayload);

    rf95.send((uint8_t *)tx_buf, zize);



      rf95.waitPacketSent();
      DPRINTln(" Sent a reply");
       digitalWrite(led, LOW);
    }
    else
    {
      DPRINTln("recv failed");
    }
  }
}

long readVcc() { long result; // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert while (bit_is_set(ADCSRA,ADSC));
  result = ADCL; result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
