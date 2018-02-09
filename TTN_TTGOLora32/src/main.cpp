// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

// 20180209 - added support for selecting spreading factor using PRG button on TTGO 
//            display of current frequency and SF on OLED

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define BUTTON_GPIO 0  // define the button GPIO port 

unsigned int counter = 0;

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

/*************************************
 * TODO: Change the following keys
 * NwkSKey: network session key, AppSKey: application session key, and DevAddr: end-device address
 *************************************/
static u1_t NWKSKEY[16] = { .... };  // Paste here the key in MSB format

static u1_t APPSKEY[16] = { .... };  // Paste here the key in MSB format

static u4_t DEVADDR = 0x00000000;   // Put here the device id in hexadecimal form.

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;
char TTN_response[30];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

// Defining lookup table for translating enum to SF label
// there must be a more elegant way, but for now it works
// enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };}
static String sf_table[9] = {
    "SF12",
    "SF11",
    "SF10",
    "SF9",
    "SF8",
    "SF7",
    "*SF7B",
    "*FSK",
    "*NONE"
};

// set initial spreading factor index (3 = SF9)
static byte currentSF = 3;

void do_send(osjob_t* j){
    // Payload to send (uplink)
    static uint8_t message[] = "Hello World!";

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        display.clear();
        display.drawString (0, 0, "Packet queued...");
        display.drawString (0, 12, String(LMIC.freq) + " Mhz");
        display.drawString (0, 24, sf_table[currentSF]);

        display.drawString (0, 50, String (++counter));
        display.display ();
        Serial.println(F("Sending uplink packet..."));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        display.clear();
        display.setFont (ArialMT_Plain_10);
        display.drawString (0, 0, "EV_TXCOMPLETE event!");
        display.drawString (0, 12, String(LMIC.freq) + " Mhz");
        display.drawString (0, 24, sf_table[currentSF]);


        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
          Serial.println(F("Received ack"));
          display.drawString (0, 20, "Received ACK.");
        }

        if (LMIC.dataLen) {
          int i = 0;
          // data received in rx slot after tx
          Serial.print(F("Data Received: "));
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();

          display.drawString (0, 30, "Received DATA.");
          for ( i = 0 ; i < LMIC.dataLen ; i++ )
            TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
          TTN_response[i] = 0;
          display.drawString (0, 38, String(TTN_response));
        }

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN, LOW);
        display.drawString (0, 50, String (counter));
        display.display ();
    }
}

int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_RESERVE_S)&&EFUSE_RD_CHIP_VER_RESERVE_V) ;
}

void printESPRevision() {
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG) ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.print("EFUSE_RD_CHIP_VER_RESERVE_S ");
  Serial.println(EFUSE_RD_CHIP_VER_RESERVE_S, BIN);

  Serial.print("EFUSE_RD_CHIP_VER_RESERVE_V ");
  Serial.println(EFUSE_RD_CHIP_VER_RESERVE_V, BIN);

  Serial.println();

  Serial.print("Chip Revision (official version): ");
  Serial.println(getChipRevision());

  Serial.print("Chip Revision from shift Operation ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);

}

void setup() {
    Serial.begin(115200);
    delay(1500);   // Give time for the seral monitor to start up
    Serial.println(F("Starting..."));

    printESPRevision();

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);
    
    // setup button
    pinMode(BUTTON_GPIO, INPUT_PULLUP);

   // reset the OLED
   pinMode(OLED_RESET,OUTPUT);
   digitalWrite(OLED_RESET, LOW);
   delay(50);
   digitalWrite(OLED_RESET, HIGH);

   display.init ();
   display.flipScreenVertically ();
   display.setFont (ArialMT_Plain_10);

   display.setTextAlignment (TEXT_ALIGN_LEFT);

   display.drawString (0, 0, "Init!");
   display.display ();

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Set static session parameters.
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Setup SF (use static variable currentSF), TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = static_cast<_dr_eu868_t>(currentSF);

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    LMIC_setDrTxpow(static_cast<_dr_eu868_t>(currentSF),14);

    // Start job
    do_send(&sendjob);
}

bool _btn_isreleased = true;
void loop() {
    os_runloop_once();
    
    //detect button press and change SF
    //TODO: take code out of loop routine
    if (digitalRead(BUTTON_GPIO) == LOW && _btn_isreleased) {

        _btn_isreleased = false;

        currentSF++;
        if (currentSF > 8) {
            currentSF = 0;
        }
        // change SF
        LMIC.dn2Dr = static_cast<_dr_eu868_t>(currentSF);

        // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
        //LMIC_setDrTxpow(DR_SF11,14);
        LMIC_setDrTxpow(static_cast<_dr_eu868_t>(currentSF),14);

        // debounce
        delay(50);

        display.clear();
        display.setFont (ArialMT_Plain_10);
        display.drawString (0, 0, "SF Changed");
        display.setFont (ArialMT_Plain_16);
        display.drawString (0, 12, sf_table[currentSF]);
        display.display();

    }

    if (digitalRead(BUTTON_GPIO) == HIGH && !_btn_isreleased) { _btn_isreleased = true; }

}
