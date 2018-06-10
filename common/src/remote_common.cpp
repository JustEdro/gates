#ifndef COMMON_LIB_PACKET
#define COMMON_LIB_PACKET

#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>

typedef enum {COMMAND_NOOP = 0, COMMAND_OPEN = 1, COMMAND_CLOSE = 2, COMMAND_PING = 3} command;
typedef enum {RESPONSE_OK = 0, RESPONSE_BUSY = 1, RESPONSE_ERROR = 2} response;


#define REQUEST_PACKAGE_PAYLOAD_SIZE 16
#define RESPONSE_PACKAGE_PAYLOAD_SIZE 4

struct RequestPackage {
  command com;
  uint8_t counter;

  byte payload[REQUEST_PACKAGE_PAYLOAD_SIZE];

  String commandText (){
    switch (com){
      case COMMAND_NOOP:
        return "COMMAND_NOOP";
        break;
      case COMMAND_OPEN:
        return "COMMAND_OPEN";
        break;
      case COMMAND_CLOSE:
        return "COMMAND_CLOSE";
        break;
      case COMMAND_PING:
        return "COMMAND_PING";
        break;

      default:
        return "COMMAND_NOTHING";
        break;
    }
  }
};



struct ResponsePackage {
  response resp;
  uint8_t counter;

  byte payload[RESPONSE_PACKAGE_PAYLOAD_SIZE];

  String commandText (){
    switch (resp){
      case RESPONSE_OK:
        return "RESPONSE_OK";
        break;
      case RESPONSE_BUSY:
        return "RESPONSE_BUSY";
        break;
      case RESPONSE_ERROR:
        return "RESPONSE_ERROR";
        break;

      default:
        return "RESPONSE_NOTHING";
        break;
    }
  }
};


const uint64_t addressess[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // 1 - read, 2 - write on reading state


void switchToRead (RF24& radio){
  radio.startListening();
}

void switchToWrite (RF24& radio){
  radio.stopListening();
}


#endif // def COMMON_LIB_PACKET
