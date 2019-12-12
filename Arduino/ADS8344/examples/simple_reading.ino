/*
  WALBAX TECH

  Please help us to make some good stuff like this:
  Buy me a beer :)

  Description: A lazy code for reading the ADC ADS8344 using an Arduino UNO.

  I'm serious about the need of help, if you want to make some libraries like
  that one or you feel confortable coding, we are hiring, contact-us.
  Contact Link: https://bit.ly/2KIPR9c

  ADC Connections:
  - VREF: Vcc
  - COM: GND
  Arduino UNO - DEVICE ADS8344:
  - CS/SS (PIN 10) : ADC CS
  - DIN/MISO (PIN 12 or ICSP-1): ADC DOUT
  - DOUT/MOSI (PIN 11 or ICSP-4): ADC Din
  - DCLK/SCK (PIN 13 or ICSP-3): ADC DCLK
*/

#include<SPI.h>

#define sampFreq 1024 //frequencia de amostragem HZ
#define ts 1000/sampFreq //período em ms
#define St 0x24
#define Et 0x0A
#define EEG 0x45
#define TR 0x54
Timer T; //timer de aquisição
int funcEvent;

//13 => sck
//12 => MISO
//11 => MOSI
//10 =>SS

#define CS_pin 10
#define AD_BUSY 9
#define NULL_SPI 0x00

#define AD_CONFIG 0x86  //0b 1000 0110
#define AD_CONVERT 0x84 //0b 1000 0100

//#define AD_CONFIG 0x96  //0b1001 0110
//#define AD_CONVERT 0x94 //0b1001 0100

#define CH_DFLT 0
SPISettings settings_conv_ad(8000000, MSBFIRST, SPI_MODE0);


//ganhos do circuito
#define G2 7
#define G1 6
#define G0 5

void setup() {
  // put your setup code here, to run once:

  SPI.begin();
  pinMode(CS_pin, OUTPUT);
  digitalWrite(CS_pin, HIGH);
  pinMode(AD_BUSY, INPUT);
  Serial.begin(115200);

  //ganho programável
  pinMode(G2, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(G0, OUTPUT);

  digitalWrite(G2,LOW);
  digitalWrite(G1,LOW);
  digitalWrite(G0,HIGH);

  setup_conversor(CH_DFLT);
  //T.every(ts,Aquis); //ativa o timer

}


/**
   Do the same of read_conversor but ignorates the result
*/
void setup_conversor(uint8_t channel) {
  SPI.beginTransaction(settings_conv_ad);

  SPI.transfer(AD_CONFIG);//Enviando comando para configurar
  //esperar 100ns, que nesse caso seria o tempo de instrução(?)
  digitalWrite(CS_pin, HIGH);
  delayMicroseconds(2); //espera 2us tempo de converter
  digitalWrite(CS_pin, LOW);
  //while(digitalRead(AD_BUSY)); //tira esse e esperar 200ns, que é o tempo de instrução(?)
  SPI.transfer(NULL_SPI);
  SPI.transfer(NULL_SPI);
  SPI.transfer(NULL_SPI); //IDLE

  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();
}


/**
   Send a packet to request a reading, waits the busy signal to become low, and send 2 null values to read the msb and lsb.
*/
void read_conversor(uint8_t channel) {
  uint8_t conv_msb, conv_lsb;
  uint16_t leitura_ad;
  digitalWrite(CS_pin, LOW);
  SPI.beginTransaction(settings_conv_ad);

  SPI.transfer(AD_CONVERT);//0 | (channel << 4));//Enviando comando para aquisição de dados
  //esperar 100ns, que nesse caso seria o tempo de instrução(?)

  digitalWrite(CS_pin, HIGH);
  delayMicroseconds(2); //espera 2us tempo de converter

  digitalWrite(CS_pin, LOW);
  //while(digitalRead(AD_BUSY)); //tira esse e esperar 200ns, que é o tempo de instrução(?)
  conv_msb = SPI.transfer(NULL_SPI);
  conv_lsb = SPI.transfer(NULL_SPI);
  SPI.transfer(NULL_SPI); //IDLE

  digitalWrite(CS_pin, HIGH);
  SPI.endTransaction();

  leitura_ad = conv_msb << 8 | conv_lsb;
  SendTR();
  EnviaDados(leitura_ad);
  //Serial.println(leitura_ad);

}

//envia os dados pela porta serial
void EnviaDados(uint16_t ad)
{

  Serial.write(St);
  Serial.write(EEG);

  Serial.write(ad);
  Serial.write(ad >> 8);
  Serial.write(Et);

}

void SendTR()
{
  uint16_t ad = analogRead(A0);//ler o valor da entrada analogica
  Serial.write(St);
  Serial.write(TR);
  Serial.write(ad);//envia o valor pela porta serial
  Serial.write(ad>>8);
  Serial.write(Et);
}
void Aquis()
{
  read_conversor(CH_DFLT);
}
 void GainControl(char G)
{
    switch(G)
    {
        case 'a'://0
            digitalWrite(G2,LOW);
            digitalWrite(G1,LOW);
            digitalWrite(G0,LOW);
            break;
        case 'b'://1
            digitalWrite(G2,LOW);
            digitalWrite(G1,LOW);
            digitalWrite(G0,HIGH);
            break;
        case 'c'://2
            digitalWrite(G2,LOW);
            digitalWrite(G1,HIGH);
            digitalWrite(G0,LOW);
            break;
        case 'd'://4
            digitalWrite(G2,LOW);
            digitalWrite(G1,HIGH);
            digitalWrite(G0,HIGH);
            break;
        case 'e'://8
            digitalWrite(G2,HIGH);
            digitalWrite(G1,LOW);
            digitalWrite(G0,LOW);
            break;
        case 'f'://16
            digitalWrite(G2,HIGH);
            digitalWrite(G1,LOW);
            digitalWrite(G0,HIGH);
            break;
        case 'g': //32
            digitalWrite(G2,HIGH);
            digitalWrite(G1,HIGH);
            digitalWrite(G0,LOW);
            break;
        case 'h': //64
            digitalWrite(G2,HIGH);
            digitalWrite(G1,HIGH);
            digitalWrite(G0,HIGH);
            break;
    }
}
void loop() {
  // put your main code here, to run repeatedly:
  T.update();
  if (Serial.available()) {
    char value = Serial.read();
    switch (value) {
      case 's': //setup
        setup_conversor(CH_DFLT);
        break;
      case 'r': //read
        T.every(ts,Aquis); //ativa o timer
        read_conversor(CH_DFLT);
        break;
      case 'p': //pause
        T.stop(funcEvent); //para o timer
        break;
      default:
        GainControl(value);
    }
  }
}
