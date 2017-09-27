#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <../../common/src/remote_common.cpp>

// relay pins
#define PIN_K1 2
#define PIN_K2 3
#define PIN_K3 4
#define PIN_K4 5

//  state machine delays
#define ARMED_MAX_DURATION   10000
#define OPENING_MAX_DURATION 4000
#define CLOSING_MAX_DURATION 3000


RF24 radio(9,10);


typedef enum {STATE_IDLE, STATE_WAKEUP, STATE_ARMED, STATE_OPENING, STATE_CLOSING} state;


int counter = 1;
int failCounter = 0;
//byte data[32];


unsigned long stateStartTime;
state currentState = STATE_IDLE;
uint8_t k1State, k2State, k3State, k4State;

RequestPackage data;
ResponsePackage resp;
uint8_t commandCounter = 0;


void resetTimer(){
  stateStartTime = millis(); //
}



void switchState(state newState){
  currentState = newState;
  resetTimer();
}


// updates state and performs digitalWrite if needed
void setRelayStates(uint8_t k1, uint8_t k2, uint8_t k3, uint8_t k4){
  if (k1State != k1) {
    k1State = k1;
    digitalWrite(PIN_K1, k1);
    //Serial.println("k1State != k1");
  }
  if (k2State != k2) {
    k2State = k2;
    digitalWrite(PIN_K2, k2);
    //Serial.println("k2State != k2");
  }
  if (k3State != k3) {
    k3State = k3;
    digitalWrite(PIN_K3, k3);
    //Serial.println("k3State != k3");
  }
  if (k4State != k4) {
    k4State = k4;
    digitalWrite(PIN_K4, k4);
    //Serial.println("k4State != k4");
  }
}


bool initComplete = false;

void setup(){
  Serial.begin(9600); //открываем порт для связи с ПК
  Serial.println("System started (BASE)");

  /// radio
  // активировать модуль
  if ( radio.begin() ) {
    Serial.println("Radio connected");
    initComplete = true;
  } else {
    Serial.println("Radio NOT RESPONDING");
  }

  if (initComplete) {
    radio.begin(); //активировать модуль
    radio.setAutoAck(1);         //режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(2,15);     //(время между попыткой достучаться, число попыток)

    radio.setPayloadSize(sizeof(data));     //размер пакета, в байтах
    radio.setChannel(0x60);  // выбираем канал (в котором нет шумов!)
    radio.setCRCLength(RF24_CRC_16);   // RF24_CRC_8, RF24_CRC_16
    radio.setPALevel (RF24_PA_MAX);   // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate (RF24_250KBPS); // RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

    radio.enableAckPayload(); // включаем возможность ответить на входящий пакет

    radio.openWritingPipe(addressess[0]);
    radio.openReadingPipe(1, addressess[1]);

    switchToRead(radio);
    radio.powerUp(); //начать работу

    Serial.println("Radio initialized");
  }

  // vars
  k1State = LOW;
  k2State = LOW;
  k3State = LOW;
  k4State = LOW;

  // relay
  pinMode(PIN_K1, OUTPUT);
  pinMode(PIN_K2, OUTPUT);
  pinMode(PIN_K3, OUTPUT);
  pinMode(PIN_K4, OUTPUT);

  setRelayStates(HIGH, HIGH, HIGH, HIGH);
}


void loop() {
  bool dataRead = false;
  byte pipeNo;

  if ( radio.available(&pipeNo) ) {

    radio.read(&data, sizeof(data));

    Serial.println("Got command: " + data.commandText());
    dataRead = true;

    commandCounter++;
    resp.counter = commandCounter;
  }


  // state machine
  switch(currentState){
    case STATE_IDLE:
      setRelayStates(HIGH, HIGH, HIGH, HIGH);

      if (dataRead && data.com == COMMAND_OPEN){
        Serial.println("Changing to STATE_OPENING");
        switchState(STATE_OPENING);
      }

      if (dataRead && data.com == COMMAND_CLOSE){
        Serial.println("Changing to STATE_CLOSING");
        switchState(STATE_CLOSING);
      }

      if (dataRead && data.com == COMMAND_PING){
        Serial.println("Pong");
        
      }

      resp.resp = RESPONSE_OK;
      break;

    case STATE_OPENING:
      // 1-st relay activates open solenoid
      setRelayStates(LOW, HIGH, HIGH, HIGH);

      // fall back to IDLE
      if ( millis() - stateStartTime >= OPENING_MAX_DURATION ){
        Serial.println("Changing to STATE_IDLE by timer");
        switchState(STATE_IDLE);
      }

      break;

    case STATE_CLOSING:
      // 2-nd relay activates close solenoid
      setRelayStates(HIGH, LOW, HIGH, HIGH);

      // fall back to IDLE
      if ( millis() - stateStartTime >= CLOSING_MAX_DURATION ){
        Serial.println("Changing to STATE_IDLE by timer");
        switchState(STATE_IDLE);
      }
      break;

    default:
      // nothing

      break;
  }

  if ( dataRead ) {
    radio.writeAckPayload( pipeNo, &resp, sizeof(resp) );
  }

}
