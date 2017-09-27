#include <Arduino.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <../../common/src/remote_common.cpp>

RF24 radio(9,10); // "создать" модуль на пинах 9 и 10

// buttons
#define PIN_BTN_A 3
#define PIN_BTN_B 4
// LIGHTS
#define LED_A 6
#define LED_B 7
#define LED_C 8

#define ARMED_MAX_DURATION   300  //
#define OPENING_MAX_DURATION 1000  //
#define CLOSING_MAX_DURATION 1000   //
#define WAKEUP_MAX_DURATION  300   //


uint8_t l1State, l2State, l3State;



typedef enum {STATE_IDLE, STATE_OPENING_COMMAND, STATE_CLOSING_COMMAND} state;


unsigned long stateStartTime;
state currentState = STATE_IDLE;

int counter = 0;

RequestPackage request;
ResponsePackage resp;


void debugLed (uint8_t led, uint8_t state) {
  if (state == LOW) {
    Serial.print("Led pin ");
    Serial.print(led, DEC);
    Serial.print(" switched to LOW");
  } else {
    Serial.print("Led pin ");
    Serial.print(led, DEC);
    Serial.print(" switched to HIGH");
  }
  Serial.println(" ");
}

// updates state and performs digitalWrite if needed
void setLedStates(uint8_t l1, uint8_t l2, uint8_t l3){
  if (l1State != l1) {
    l1State = l1;
    digitalWrite(LED_A, l1);
    debugLed(LED_A, l1);
  }
  if (l2State != l2) {
    l2State = l2;
    digitalWrite(LED_B, l2);
    debugLed(LED_B, l2);
  }
  if (l3State != l3) {
    l3State = l3;
    digitalWrite(LED_C, l3);
    debugLed(LED_C, l3);
  }
}




void switchState(state newState){
  currentState = newState;
  stateStartTime = millis(); //
}



void sendCommand(command com) {
  counter++;

  request.com = com;

  request.payload[0] = counter & 0xFF;
  request.payload[1] = (counter >> 8) & 0xFF;

  switchToWrite(radio);

  if(!radio.write(&request, sizeof(request))){   //Write to the FIFO buffers
     Serial.println("Sending failed");

     setLedStates(LOW, LOW, HIGH);
     delay(500);
     setLedStates(LOW, LOW, LOW);

  } else {
    if( radio.isAckPayloadAvailable() ){ // если прилетел ответ на запрос

      radio.read( &resp, sizeof(resp) );
      Serial.print("Got response: ");
      Serial.println(resp.commandText());
    } else {
      Serial.println("NO RESPONSE");
    }
  }

  switchToRead(radio);

  Serial.println("Sent: " + request.commandText());
}




bool initComplete = false;

void setup()   {
  Serial.begin(9600);
  Serial.println("System started (REMOTE)");

  /// radio
  // активировать модуль
  if ( radio.begin() ) {
    Serial.println("Radio connected");
    initComplete = true;
  } else {
    Serial.println("Radio NOT RESPONDING");
  }

  if (initComplete) {
    radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(2,15);     // (время между попыткой достучаться, число попыток)

    radio.setPayloadSize(sizeof(request));   // размер пакета, в байтах


    radio.setChannel(0x60);  //выбираем канал (в котором нет шумов!)
    radio.setCRCLength(RF24_CRC_16);
    radio.setPALevel(RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate(RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

    radio.enableAckPayload(); // включаем возможность ответить на входящий пакет

    radio.openWritingPipe(addressess[1]);
    radio.openReadingPipe(1, addressess[0]);

    switchToRead(radio);
    radio.powerUp(); //начать работу

    Serial.println("Radio initialized");

    randomSeed(analogRead(0));
    for(int i=0; i < REQUEST_PACKAGE_PAYLOAD_SIZE; i++){
       request.payload[i] = random(255);  //Load the buffer with random request
       //request.payload[i] = 0;
    }

    // buttons
    pinMode(PIN_BTN_A, INPUT_PULLUP);
    pinMode(PIN_BTN_B, INPUT_PULLUP);
  }

  // light
  pinMode(LED_A, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_C, OUTPUT);

  l1State = LOW;
  l2State = LOW;
  l3State = LOW;

  setLedStates(LOW, LOW, LOW);

  if (! initComplete)
    setLedStates(LOW, LOW, HIGH);
}


void loop() {
  if (! initComplete) return;

  uint8_t btnA, btnB;
  btnA = digitalRead(PIN_BTN_A);
  btnB = digitalRead(PIN_BTN_B);

  switch(currentState){
    case STATE_IDLE:
      setLedStates(LOW, LOW, LOW);

      if (btnA == LOW){
        Serial.println("Changing to STATE_OPENING_COMMAND");
        switchState(STATE_OPENING_COMMAND);

        sendCommand(COMMAND_OPEN);
      }

      if (btnB == LOW){
        Serial.println("Changing to STATE_CLOSING_COMMAND");
        switchState(STATE_CLOSING_COMMAND);

        sendCommand(COMMAND_CLOSE);
      }
      break;

    case STATE_OPENING_COMMAND:
      setLedStates(HIGH, LOW, LOW);

      // fall back to STATE_IDLE
      if ( millis() - stateStartTime >= OPENING_MAX_DURATION ){
        Serial.println("Changing to STATE_IDLE state by timer");
        switchState(STATE_IDLE);
      }
      break;

    case STATE_CLOSING_COMMAND:
      setLedStates(LOW, HIGH, LOW);

      // fall back to STATE_IDLE
      if ( millis() - stateStartTime >= CLOSING_MAX_DURATION ){
        Serial.println("Changing to STATE_IDLE state by timer");
        switchState(STATE_IDLE);
      }
      break;
    default:
      // nothing

      break;
  }
}
