
/* UNIVERSIDADE FEDERAL DE UBERLANDIA
   Biomedical Engineering

   Autors: Ítalo G S Fernandes

   contact: italogsfernandes@gmail.
   URLs: https://github.com/italogsfernandes/

  Requisites: Library Timer One[https://github.com/PaulStoffregen/TimerOne]
            [DueTimer](https://github.com/ivanseidel/DueTimer)

  This code aims to simulate EMG signals from 4 channels representing 6
  different movements of the hand
  and send this to the serial interface as ASCII text or as binary data.

  When selected the binary data mode this is the format of the packet
  For 1 channel:
  Packet:  START | MSB  | LSB  | END
  Exemple:  '$' | 0x01 | 0x42 | '\n'
  For n channels:
  Packet:  START | MSB1  | LSB1  | MSB...  | LSB...  | MSB_n  | LSB_n  | END
  Exemple:  '$' |  0x01 |  0x42 |  0x01   |  0x33   |  0x02   |  0xB4  | '\n'
*/
//Uncomment the next line to activate the sending of data as text
//This is usefull when you are using the serial plotter tool (Ctrl+shift+L)

#define ADC_ACQUIRER
//#define AVR_ACQUIRER
//#define SIMULATOR
#define PLOTTER_SERIAL


//Libraries
#ifdef SIMULATOR
#include "SignalGenerator.h"
#endif

///////////
//Timers //
///////////
#define FREQ_AQUIRE          200                    //Frequency in Hz
#define INTERVAL_MS_AQUIRE   1000 / FREQ_AQUIRE     //Interval in milliseconds
#define INTERVAL_US_AQUIRE   1000000 / FREQ_AQUIRE  //Interval in microseconds

// Arduino DUE
#if defined(__arm__) && defined(__SAM3X8E__)
#include<DueTimer.h>
#define SETUP_TIMER()   Timer3.attachInterrupt(timerDataAcq).setFrequency(FREQ_AQUIRE)
#define START_TIMER()   Timer3.start()
#define STOP_TIMER()    Timer3.stop()
#endif

// Arduino UNO, NANO (and others 328P based boards), MEGA e ATtiny85
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATtiny85__)
#include<TimerOne.h>
#define SETUP_TIMER()   Timer1.initialize()
#define START_TIMER()   Timer1.attachInterrupt(timerDataAcq,INTERVAL_US_AQUIRE)
#define STOP_TIMER()    Timer1.stop()
#endif

////////////
//Defines //
////////////
#define UART_BAUDRATE 115200
#define PACKET_START  '$'
#define PACKET_END    '\n'
#define QNT_CH        1

////////////////
//Global data //
////////////////
#ifdef SIMULATOR
SignalGenerator_t my_generator;
uint16_t generated_value;
float eeg_read_values[QNT_CH];
#endif

uint16_t adc_read_values[QNT_CH];

void timerDataAcq();
void sendData();
void showData();


////////////////////////////////////
//Pins and variables for stimulus //
////////////////////////////////////
#define PINO_FONE 5
#define PINO_TRIGGER_IN 6

#define FREQ_BASE 500 //Hz
#define FREQ_ODD 550 //Hz

long randNumber;
int freq_now = FREQ_BASE;
int freq_next = FREQ_BASE;

unsigned long time_now, alarm_time;


/////////////
// ADS8344 //
/////////////
#include<SPI.h>

#define ADC_CS_PINO 10
#define ADC_BUSY_PINO 9

#define ADC_CONVERSION_REGISTER 0x84 //0b 1000 0100

SPISettings adc_spi_settings(8000000, MSBFIRST, SPI_MODE0);

uint8_t adc_reading_msb, adc_reading_lsb;

//////////////////
//Main Function //
//////////////////
void setup() {
#ifdef ADC_ACQUIRER
  SPI.begin();
  pinMode(ADC_CS_PINO, OUTPUT); digitalWrite(ADC_CS_PINO, HIGH);
  pinMode(ADC_BUSY_PINO, INPUT);
  setup_adc();
#endif
  Serial.begin(UART_BAUDRATE);

  pinMode(PINO_FONE, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PINO_TRIGGER_IN, INPUT_PULLUP);

  #ifdef SIMULATOR
  my_generator.setOffset(0);
  my_generator.setAmplitude(1);
  my_generator.setWaveform(EEG_BASE_WAVE);
  #endif

  randomSeed(analogRead(5));
  time_now = millis();
  alarm_time = time_now + 500; // alarme daqui a 500ms

  SETUP_TIMER();
  START_TIMER();
}

void loop() {
  time_now = millis();
  if (time_now == alarm_time) {
    alarm_time = time_now + 500; // proximo alarme daqui a 500ms

    // seleciona estimulo
    randNumber = random(10);
    freq_next = (randNumber == 0) ? FREQ_ODD : FREQ_BASE; // random
    freq_next = (freq_now == FREQ_ODD) ? FREQ_BASE : freq_next; // dont repeat

    // Go signal go
    digitalWrite(LED_BUILTIN, freq_now == FREQ_ODD);
    tone(PINO_FONE, freq_now, 200);
    freq_now = freq_next;
  }
}

///////////////////
//Aquire Routine //
///////////////////
void timerDataAcq() {
#ifdef SIMULATOR
  my_generator.generate_value(eeg_read_values);
  for (int i = 0; i < QNT_CH; i++) {
    adc_read_values[i] = (uint16_t) eeg_read_values[i];
  }
#endif

#ifdef ADC_ACQUIRER
  adc_read_values[0] = read_adc();
#endif

#ifdef AVR_ACQUIRER
  adc_read_values[0] = analogRead(A0);
#endif

  //Sending the value
#ifndef PLOTTER_SERIAL
  sendData();
#else
  showData();
#endif
}

void sendData() {
  Serial.write(PACKET_START);
  for (int i = 0; i < QNT_CH; i++) {
    Serial.write(adc_read_values[i] >> 8);
    Serial.write(adc_read_values[i]);
  }
  Serial.write(PACKET_END);
}

void showData() {
  for (int i = 0; i < QNT_CH; i++) {
    Serial.print(adc_read_values[i] + 1024 * i);
    Serial.print("\t");
  }
  Serial.println();
}

#ifdef ADC_ACQUIRER
void setup_adc() {
  read_adc();
}

uint16_t read_adc() {
  digitalWrite(ADC_CS_PINO, LOW);
  SPI.beginTransaction(adc_spi_settings);
  SPI.transfer(ADC_CONVERSION_REGISTER);
  digitalWrite(ADC_CS_PINO, HIGH);

  delayMicroseconds(2);

  digitalWrite(ADC_CS_PINO, LOW);
  adc_reading_msb = SPI.transfer(0);
  adc_reading_lsb = SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(ADC_CS_PINO, HIGH);
  SPI.endTransaction();

  return (adc_reading_msb << 8 | adc_reading_lsb);
}
#endif
