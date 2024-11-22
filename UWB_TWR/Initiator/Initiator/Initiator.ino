/*
   Copyright (c) 2024 by Ashutosh Dhekne <dhekne@gatech.edu>
   This program runs on the UWB device connected to the login device.  

   You are free to:
Share — copy and redistribute the material in any medium or format
The licensor cannot revoke these freedoms as long as you follow the license terms.
Under the following terms:
Attribution — You must give appropriate credit , provide a link to the license, and indicate if changes were made . 
You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
NonCommercial — You may not use the material for commercial purposes .
NoDerivatives — If you remix, transform, or build upon the material, you may not distribute the modified material.
No additional restrictions — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.
Notices:
You do not have to comply with the license for elements of the material in the public domain or where your use is permitted by an applicable exception or limitation .

No warranties are given. The license may not give you all of the permissions necessary for your intended use. 
For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
*/
#include "genericFunctions.h"
#include "RangingContainer.h"
#include <time.h>
#include<TimeLib.h>
#include<Wire.h>
#include "RTClib.h"

//Debug levels
int DebugUWB_L1 = 0;
int DebugUWB_L2 = 0;
int DebugWebserverComms = 1;
int DebugCrypto = 1;
int DebugUWB_LRXTO = 0;

//UWB Globals
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin

// packet sent status and count
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;
String message;

//UWB Messages
byte tx_poll_msg[MAX_POLL_LEN] = {POLL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_resp_msg[MAX_RESP_LEN] = {RESP_MSG_TYPE, 0x02, 0, 0, 0, 0};
byte tx_final_msg[MAX_FINAL_LEN] = {FINAL_MSG_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_dist_msg[MAX_DIST_LEN] = {DIST_EST_MSG_TYPE, 0, 0, 0, 0, 0, 0};
byte tx_send_keys[MAX_KEYS_TYPE_LEN] = {SEND_KEYS_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte rx_send_keys_ack[MAX_KEYS_TYPE_LEN] = {SEND_KEYS_ACK_TYPE, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int response_counter = 0;
char rx_msg_char[200];
byte rx_packet[128];
char myDevID=0;

//UWB Ranging
unsigned int seq;
char currentDeviceIndex;
uint64_t RespTs;
Ranging thisRange;
int dist;
int attempt=0;
#define MAX_ATTEMPTS 100

//UWB State Machine
typedef enum states {STATE_IDLE, STATE_POLL, STATE_RESP_EXPECTED, STATE_FINAL_SEND, STATE_TWR_DONE, STATE_RESP_SEND, STATE_FINAL_EXPECTED, STATE_OTHER_POLL_EXPECTED, STATE_RESP_PENDING, STATE_DIST_EST_EXPECTED, STATE_DIST_EST_SEND, STATE_TIGHT_LOOP,
                     STATE_RECEIVE, STATE_PRESYNC, STATE_SYNC, STATE_ANCHOR, STATE_TAG, STATE_FIRST_START, STATE_OBLIVION, STATE_ACK_EXPECTED, STATE_SEND_KEY_IV, STATE_KEY_IV_ACK_EXPECTED
                    } STATES;
volatile uint8_t current_state = STATE_IDLE;

//UWB Error Handling
int RX_TO_COUNT = 0;
int BEEP_PIN = 6;

//UWB CIR
//The most relevant samples for channel analysis are typically in the first part 
// of the accumulator, where the main signal path and early multipath components are captured.
uint8_t cir_data[1016]; // 254 complex samples, maximum size of DW1000 ACC register



void setup() {
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  //Serial.print("Waiting for instructions...");

  //Initialize UWB
  randomSeed(analogRead(0));
  Serial.println(F("-----------------------------"));
  Serial.println(F("-->    UWB Initiator      <--"));
  Serial.println(F("-----------------------------"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  current_state = STATE_POLL;
}


//UWB Support Functions
void receiver(uint16_t rxtoval = 0 ) {
  received = false;
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(false);
  if (rxtoval > 0) {
    DW1000.setRxTimeout(rxtoval);
  } else {
    DW1000.setRxTimeout(rxtoval);
  }
  DW1000.startReceive();
}

void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  RX_TO_COUNT = 0;
  // status change on reception success

  DW1000.getData(rx_packet, DW1000.getDataLength());
  //  Serial.println("Received something...");
  received = true;
  
//  byte2char(rx_packet, 24);
//  Serial.println(rx_msg_char);

  DW1000.getAccMem(cir_data, 0, 254);
  Serial.write(cir_data, 1016);
	// for (int i=0;i<500;i=i+4)
	// {
	// 	Serial.print(cir_data[i+1], HEX);
	// 	Serial.print(cir_data[i], HEX);
	// 	Serial.print(", ");
	// 	Serial.print(cir_data[i+3], HEX);
	// 	Serial.print(cir_data[i+2], HEX);
	// 	Serial.println("");
	// }
}

void handleError() {
  if (current_state == STATE_RECEIVE)
  {
    current_state = STATE_RECEIVE;
  } else if(current_state == STATE_ANCHOR)
  {
    current_state = STATE_ANCHOR;
  }
  else {
    current_state = STATE_POLL;
  }
  RxTimeout = true;
  error = true;
  Serial.println("ERROR");

}

void handleRxTO() {
  RX_TO_COUNT++;
  if (DebugUWB_L1 == 1) {
    Serial.println("RXTO");
  }
  if (DebugUWB_LRXTO == 1) {
    printState();
  }
  RxTimeout = true;
}



void loop() {
  int recvd_resp_seq;
  
  //Take actions based on the current state
  switch(current_state) {
    case STATE_IDLE:
      break;
    case STATE_POLL:
    {
      /*if (attempt>MAX_ATTEMPTS) {
        Serial.println("---Could Not communicate with token device---");
        current_state=STATE_IDLE;
        digitalWrite(BEEP_PIN, 1);
        delay(500);
        //digitalWrite(6, 0);
        //digitalWrite(12, 0);
        digitalWrite(BEEP_PIN, 0);
        delay(50);
        digitalWrite(BEEP_PIN, 1);
        delay(250);
        digitalWrite(BEEP_PIN, 0);
        delay(50);
        digitalWrite(BEEP_PIN, 1);
        delay(100);
        digitalWrite(BEEP_PIN, 0);
        delay(50);
        attempt = 0;
        break;
      }*/
      if (DebugUWB_L1 == 1) {
        Serial.println("State: STATE_POLL");
      }
      seq++;
      tx_poll_msg[SRC_IDX] = myDevID;
      tx_poll_msg[DST_IDX] = BROADCAST_ID;
      tx_poll_msg[SEQ_IDX] = seq & 0xFF;
      tx_poll_msg[SEQ_IDX + 1] = seq >> 8;
      generic_send(tx_poll_msg, sizeof(tx_poll_msg), POLL_MSG_POLL_TX_TS_IDX, SEND_DELAY_FIXED);

      current_state = STATE_RESP_EXPECTED;

      while (!sendComplete) {
      };
      if (DebugUWB_L1 == 1) {
        Serial.println("Poll out");
      }

      //current_time_us = get_time_us();
      sendComplete = false;
      receiver(TYPICAL_RX_TIMEOUT);
      break;
    }
    case STATE_RESP_EXPECTED: {
      if (DebugUWB_L1 == 1) {
        Serial.println("State: STATE_RESP_EXPECTED");
      }
      
      //Handle multiple UWB responders later.
      if (received) {
        received = false;
        if (rx_packet[DST_IDX] == myDevID && rx_packet[0] == RESP_MSG_TYPE) {
            if (DebugUWB_L1 == 1) {
              Serial.println("Recieved response!");
            }
            recvd_resp_seq = rx_packet[SEQ_IDX] +  ((uint16_t)rx_packet[SEQ_IDX + 1] << 8);
            currentDeviceIndex = rx_packet[SRC_IDX];
            
            if(recvd_resp_seq==seq)
            {
              DW1000Time rxTS;
              DW1000.getReceiveTimestamp(rxTS);
              RespTs = rxTS.getTimestamp();
              current_state = STATE_FINAL_SEND;
            }
        } else {
            received = false;
            receiver(TYPICAL_RX_TIMEOUT);
        }
      } else {
        if (RxTimeout == true) {
          if (DebugUWB_L1 == 1 || DebugUWB_L2 == 1) {
            Serial.println("RX TO");
          }
          RxTimeout = false;
          current_state = STATE_POLL; //Need a max try and give up
          attempt++;
        }
      }
      break;
    }
    case STATE_FINAL_SEND:
    {
      if (DebugUWB_L1 == 1) {
        Serial.println("State: STATE_FINAL_SEND");
      }
      tx_final_msg[SRC_IDX] = myDevID;
      tx_final_msg[DST_IDX] = currentDeviceIndex;
      tx_final_msg[SEQ_IDX] = seq & 0xFF;
      tx_final_msg[SEQ_IDX + 1] = seq >> 8;
      any_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], RespTs);
      FIXED_DELAY = 4;
      generic_send(tx_final_msg, MAX_FINAL_LEN, FINAL_MSG_FINAL_TX_TS_IDX, SEND_DELAY_FIXED);
      while (!sendComplete);
      sendComplete = false;
      if (DebugUWB_L1 == 1) {
        Serial.println("Final out");
      }
      current_state = STATE_DIST_EST_EXPECTED;
      receiver(TYPICAL_RX_TIMEOUT);
      break;
    }
    case STATE_DIST_EST_EXPECTED:
    {
      //Read the distance estimate.
      //Check if the token is near enough
      if (received == true)
      {
        received = false;
        if (rx_packet[DST_IDX] == myDevID && rx_packet[0] == DIST_EST_MSG_TYPE) {
          long dist;
          dist = rx_packet[DIST_EST_MSG_DIST_MEAS_IDX];
          dist |= rx_packet[DIST_EST_MSG_DIST_MEAS_IDX+1]<<8;
          dist |= rx_packet[DIST_EST_MSG_DIST_MEAS_IDX+2]<<16;
          dist |= rx_packet[DIST_EST_MSG_DIST_MEAS_IDX+3]<<24;
          // Serial.print(millis());
          // Serial.print(": Distance: ");
          // Serial.println(dist);
          //digitalWrite(BEEP_PIN, 1);
          //delay(100);
          //digitalWrite(BEEP_PIN, 0);
         
          current_state = STATE_POLL;
        } else {
          received = false;
          receiver(TYPICAL_RX_TIMEOUT);
        }
      } else {
        if (RxTimeout == true) {
          if (DebugUWB_L1 == 1 || DebugUWB_L2 == 1) {
            Serial.println("RX TO");
          }
          RxTimeout = false;
          current_state = STATE_POLL; //Need a max try and give up
        }
      }
      break;
    }
  }
}

//Other support functions

void printState() {
  switch(current_state){
    case STATE_IDLE: Serial.println("STATE_IDLE"); break;
    case STATE_POLL: Serial.println("STATE_POLL"); break;
    case STATE_RESP_EXPECTED: Serial.println("STATE_RESP_EXPECTED"); break;
    case STATE_FINAL_SEND: Serial.println("STATE_FINAL_SEND"); break;
    case STATE_TWR_DONE: Serial.println("STATE_TWR_DONE"); break;
    case STATE_RESP_SEND: Serial.println("STATE_RESP_SEND"); break;
    case STATE_FINAL_EXPECTED: Serial.println("STATE_FINAL_EXPECTED");break;
    case STATE_OTHER_POLL_EXPECTED: Serial.println("STATE_OTHER_POLL_EXPECTED");break;
    case STATE_RESP_PENDING: Serial.println("STATE_RESP_PENDING"); break;
    case STATE_DIST_EST_EXPECTED: Serial.println("STATE_DIST_EST_EXPECTED");break;
    case STATE_DIST_EST_SEND: Serial.println("STATE_DIST_EST_SEND"); break;
    case STATE_RECEIVE: Serial.println("STATE_RECEIVE"); break;
    default: Serial.print("Unknown State: "); Serial.println(current_state); break;                
  }
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
