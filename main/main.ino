
#include <Arduino.h>

#include <avr/wdt.h>

#define VREF_STARTUP_MICROS (25)
#define BAUD_RATE 9600

#define Batt_PIN PIN_PB1
#define Pv_PIN PIN_PB0

#include <avr/io.h>
#include <util/delay.h>
#include <IRremote.h>

int Motion = PIN_PC3;

void VREF_init(void);
void DAC0_setVal(uint8_t val);
void DAC0_init(void);

unsigned long startTime;
unsigned long stopTime;
int hours, minutes, seconds;
boolean stopCounting = false;

IRrecv IR(12);

unsigned long irCode;
#define code1 3125149440
#define code2 3108437760
#define code3 3091726080
#define code4 3141861120
#define code5 3208707840
#define code6 3158572800
#define code7 4161273600
#define code8 3927310080
#define code9 4127850240
#define codeAsterisk 3910598400
#define code0 3860463360
#define codePound 4061003520
#define codeUP 3877175040
#define codeLeft 4144561920
#define codeOK 3810328320
#define codeRight 2774204160
#define codeDown 2907897600

String data1;

int value = 0;
String st = "off";
String tx = " ";
String ma = "off";

int solar_off = 207;

int dimx = 0;
int t1 = 28;
int t2 = 23;
int t3 = 21;
int t4 = 16;

int t0 = 0;

int MotionST;
int pas = 20;
int VBatt;
int VPv;

void setup() {
  IR.enableIRIn();
  Serial.begin(BAUD_RATE);
  pinMode(5, OUTPUT);
  pinMode(Motion, INPUT);
  wdt_enable(WDTO_15MS);
}

void stopCount() {
  startTime = millis();
  dimx = t0;
  t1 = t0;
  t2 = t0;
  t3 = t0;
  t4 = t0;
}

void VREF_init(void) {
  VREF.CTRLA |= VREF_DAC0REFSEL_4V34_gc; /* Voltage reference at 1.5V */
  VREF.CTRLB |= VREF_DAC0REFEN_bm;       /* DAC0/AC0 reference enable: enabled */
  _delay_us(VREF_STARTUP_MICROS);        /* Wait VREF start-up time */
}

void DAC0_setVal(uint8_t val) {
  DAC0.DATA = val;
}

void DAC0_init(void) {
  PORTA.PIN6CTRL &= ~PORT_ISC_gm;              /* Disable digital input buffer */
  PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc; /* Disable pull-up resistor */
  PORTA.PIN6CTRL &= ~PORT_PULLUPEN_bm;         /* Enable DAC, Output Buffer, Run in Standby */
  DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm | DAC_RUNSTDBY_bm;
}

void dim(int di) {
  VREF_init();
  DAC0_init();
  if (di == 0) {
    digitalWrite(5, LOW);
  } else if (di > 0) {
    digitalWrite(5, HIGH);
  }
  DAC0_setVal(di);
}

void remote() {
  if (IR.decode()) {
    irCode = IR.decodedIRData.decodedRawData;
    if (irCode == code1) {
      data1 = "Code 1 ";
    } else if (irCode == code2) {
      data1 = "Code 2 ";
    } else if (irCode == code3) {
      data1 = "Code 3 ";
    } else if (irCode == code4) {
      data1 = "Code 4 ";
    } else if (irCode == code5) {
      data1 = "Code 5 ";
    } else if (irCode == code6) {
      data1 = "Code 6 ";
    } else if (irCode == code7) {
      data1 = "Code 7 ";
    } else if (irCode == code8) {
      data1 = "Code 8 ";
    } else if (irCode == code9) {
      data1 = "Code 9 ";
    } else if (irCode == codeAsterisk) {
      data1 = "Code * ";
    } else if (irCode == code0) {
      data1 = "Code 0 ";
    } else if (irCode == codePound) {
      data1 = "Code Pound ";
    } else if (irCode == codeUP) {
      data1 = "Code UP ";
    } else if (irCode == codeLeft) {
      data1 = "Code LEFT ";
    } else if (irCode == codeOK) {
      data1 = "Code OK ";
    } else if (irCode == codeRight) {
      data1 = "Code RIGHT ";
    } else if (irCode == codeDown) {
      data1 = "Code DOWN ";
      stopCount();
    } else {
      data1 = "None";
    }
    IR.resume();
  }
}

void ct() {
  unsigned long currentTime = millis();
  hours = (currentTime - startTime) / 3600000;
  minutes = ((currentTime - startTime) % 3600000) / 60000;
  seconds = ((currentTime - startTime) % 60000) / 1000;

  if ((hours >= 0) && (hours < 1)) {
    dimx = t1;

  } else if ((hours >= 1) && (hours < 3)) {
    dimx = t2;

  } else if ((hours >= 3 ) && (hours < 4)) {
    dimx = t3;

  } else if (hours >= 4) {
    dimx = t4;
    if (MotionST != 0) {
      dimx = 20;
      delay(10000);
    }
  }
  dim(dimx);
}

void sendata() {
  VBatt = analogRead(Batt_PIN);
  VPv = analogRead(Pv_PIN);
  MotionST = digitalRead(Motion);

  String  data1 = "Dim=" + String(dimx) + " ,VBatt=" + String(VBatt) + " ,VPv=" + String(VPv) + " ,irCode=" + data1 + " ,Motion=" + String(MotionST) + " ,Hours=" + String(hours) + " ,Minutes=" + String(minutes) + " ,Seconds=" + String(seconds) + "\n";
  Serial.print(data1);
}

void loop() {
  //  remote();
  sendata();
  delay(300);


  //  if (VPv < 200) {
  //    delay(18000);
  //    t1 = 28;
  //    t2 = 23;
  //    t3 = 21;
  //    t4 = 16;
  //    if (data1 == "Code DOWN ") {
  //      stopCount();
  //    } else if (data1 == "Code UP ") {
  //      t1 = 28;
  //      t2 = 23;
  //      t3 = 21;
  //      t4 = 16;
  //    }
  //    ct();
  //  }
  //
  //  if (VPv > 200) {
  //    stopCount();
  //  }
  //
  //  if (VBatt < 500) {
  //    digitalWrite(5, LOW);
  //    stopCount();
  //    while (VBatt < 800) {
  //      //      int VBatt1 = analogRead(Batt_PIN);
  //      delay(10);
  //    }
  //
  //  }

  //  dim(dimx);
}
