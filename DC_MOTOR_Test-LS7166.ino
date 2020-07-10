// https://github.com/cacycleworks/Arduino_LS7166/blob/master/ArduinoDRO.ino
// Quadrature counter decoder LS7166 control lines
byte ic_nRD = 27; //PG1
byte ic_CnD = 43; //PA1
byte ic_nWR = 26; //PG0
byte ic_nCS = 44; //PA0

#define in1   5   //PE5
#define in2   15  //PB7
#define en    16  //PG3
#define val   45  //PF0

#define C2CMINMICRON 221200
#define LS7166INISET 0

const int ad[8]={28,29,30,31,32,33,34,35};

unsigned long EncoderCount; // this is current encode position read from LS7166

void setup() {
  pinMode(ic_nRD, OUTPUT);
  pinMode(ic_CnD, OUTPUT);
  pinMode(ic_nWR, OUTPUT);
  pinMode(ic_nCS, OUTPUT);

  digitalWrite(ic_nRD, HIGH);
  digitalWrite(ic_CnD, HIGH);
  digitalWrite(ic_nWR, HIGH);
  digitalWrite(ic_nCS, HIGH);
  delayMicroseconds(50);
  
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  pinMode (en, OUTPUT);
  pinMode (val, INPUT);

  digitalWrite(en, HIGH);
  digitalWrite(in1, LOW);

  Serial.begin(115200);
  
  EncoderCount=0;
  init_7166();
}

void cd(boolean CnD) {
  digitalWrite(ic_CnD,CnD); // 0인경우 DATA, 1인경우 CONTROL
}

void cs(boolean nCS) {
  digitalWrite(ic_nCS,nCS); // 0인경우 7166 접근 허용
}

void wr(boolean nWR) {
  digitalWrite(ic_nWR,nWR); // Twr min 60ns, 0인경우 DATA 쓰기 시작, 1인경우 쓰기 중단
}

void rd(boolean nRD) {
  digitalWrite(ic_nRD, nRD); // Trd max 11ns, 0인경우 읽기 시작, 1인경우 중단
}

void ctrl_7166( byte control ){ 
  DDRC = 0xff; // sets Arduino Digital pins 0-7 all as outputs
  //PORTC = control;
  for(int i=0;i<8;i++){
    digitalWrite(ad[i],bitRead(control,i));
    }
  latchWR_7166();
  cs(1);
}

void latchWR_7166(){
  wr(0);
  delayMicroseconds(50);
  wr(1);
}

void write_7166(unsigned long Data ){
  DDRC = 0xff; // sets Arduino Digital pins 0-7 all as outputs
  PORTC =  (unsigned char)Data;
    latchWR_7166();
    
  Data >>= 8;
  PORTC =  (unsigned char)Data;
    latchWR_7166();

  Data >>= 8;
  PORTC =  (unsigned char)Data;
    latchWR_7166();
    cs(1);
}

void init_7166() {
  EncoderCount = 0;  
  DDRC = 0xff; 
  delayMicroseconds(50);

  cs(0);cd(1);rd(1);  
  ctrl_7166(0x20);    //Performs master reset  
  delayMicroseconds(50);

  cs(0);cd(1);rd(1);
  ctrl_7166(0x40);  // ICR 셋업, 카운터 인풋모드 시작
  delayMicroseconds(50);

  cs(0);cd(1);rd(1);
  ctrl_7166(0x80);  // NOMAL 카운터
  delayMicroseconds(50);

  cs(0);cd(1);rd(1);
  ctrl_7166(0xC3);  // 카운터 리셋
  delayMicroseconds(50);

  cs(0);cd(1);rd(1);
  ctrl_7166(0x48);  // ICR, Enable A, B Input
  delayMicroseconds(50);
  
  cd(0);rd(1);cs(0);
  write_7166( LS7166INISET );
}

byte latchRD_7166(){
  byte dataread=0;  // nRD should already be high 읽기 중단이 유지 되어야 함
  DDRC = 0x00;
  rd(0); delayMicroseconds(50);
  for(int i=0;i<8;i++){digitalWrite(ad[i],1);}
  dataread = digitalRead(ad[0])|(digitalRead(ad[1])<<1)|(digitalRead(ad[2])<<2)|(digitalRead(ad[3])<<3)|(digitalRead(ad[4])<<4)|(digitalRead(ad[5])<<5)|(digitalRead(ad[6])<<6)|(digitalRead(ad[7])<<7);
  delayMicroseconds(50);
  rd(1);
  return dataread;
}

unsigned long read_7166(){
  unsigned long tmp=0, Data=0;
  DDRC = 0xff; // sets Arduino Digital pins 0-7 all as outputs
  cd(1);rd(1);cs(0);
  ctrl_7166(0x03);
  delayMicroseconds(50);
    
  // read 1st byte, LSB
  cd(0);wr(1);cs(0);
  tmp = latchRD_7166(); // byte 0, lsB
  Data |= tmp;     //  << 8 추가  // read 2nd byte, the middle byte
  cd(0);wr(1);cs(0);
  tmp = latchRD_7166(); // byte 1  
  tmp = tmp << 8; // shift over 8 bits
  Data |= tmp;  // read 3rd byte, the MSB
  cd(0);wr(1);cs(0);
  tmp = latchRD_7166(); // byte 1
  tmp = tmp << 16;     // shift over 16 bits this tmp is MSB
  Data |= tmp;
  cs(1);
  return Data;
}
/////////////////////////////////////////////////////////////////////////
void loop() {

  wr(1); cs(1); cd(1); rd(1);
  int vol = analogRead(val);
  int pwm = map(vol, 0, 1023, 0, 255);
  analogWrite(in2, pwm);
  
  EncoderCount = read_7166();
  Serial.print("Encoder Count = ");
  Serial.println(EncoderCount, DEC);
  delay(500);
}

void delay_t(int t) {
  int i;
  for(i=0; i<t; i++){ 
  asm("nop");
  }
}
