#include <NintendoExtensionCtrl.h>
#include <EEPROM.h>

ClassicController classic;

#define LED A0

#define CYBER_WAIT  25
#define JOYDRV_WAIT 1000000L
#define AD_CNV_DATA1 64
#define AD_CNV_DATA2 192

#define IF_MODE_ATARI 0
#define IF_MODE_CPSF  1
#define IF_MODE_CYBER 2
#define IF_MODE_JOYDRV 3

#define MODE_CHG_TIME 3000 // 3秒

unsigned long mode_time;
byte cnv_mode;
volatile byte cyber_if_flg;
volatile byte joydrv_if_flg;
volatile byte atari_data_SELL;
volatile byte atari_data_SELH;
volatile byte joydrv_snddata[16];
 //  0: X X X X X X SELECT START
 //  1: R4 R3 R2 R1 L4 L3 L2 L1
 //  2: Z' Z Y X D C B A
 //  3: R右 R左 R下 R上 L右 L左 L下 L上
 //  4: L上下アナログ
 //  5: L左右アナログ
 //  6: R上下アナログ
 //  7: R左右アナログ
 //  8: 左アナログボタン1
 //  9: 左アナログボタン2
 // 10: 左アナログボタン3
 // 11: 左アナログボタン4
 // 12: 右アナログボタン1
 // 13: 右アナログボタン2
 // 14: 右アナログボタン3
 // 15: 右アナログボタン4

#define CLASSIC_BYTE_COUNT (6)

//void controller_decode_bytes(uint8_t *buf, size_t len, struct ClassicController *myStruct) {
//void controller_decode_bytes(uint8_t *buf, size_t len) {
void controller_decode_bytes() {

//  if ((buf == NULL) || (len < CLASSIC_BYTE_COUNT)) { return; }

/*
  Serial.print(buf[0]&B00011111,BIN);
  Serial.print(" ");
  Serial.println(buf[1]&B00011111,BIN);

  myStruct->LeftX = (buf[0] & (64 - 1)) - 32; // 0 to 63
  myStruct->LeftY = (buf[1] & (64 - 1)) - 32; // 0 to 63 -> -32 to 31
  
  myStruct->RightX = (((buf[2] >> 7) & 1) + ((buf[1] >> 6) & 3) * 2 + ((buf[0] >> 6) & 3) * 8) - 16; // 0 to 31 -> -16 to 15
  myStruct->RightY = (buf[2] & (32 - 1)) - 16; // 0 to 31 -> -16 to 15

  myStruct->LeftT = (((buf[2] >> 5) & 3) * 8 + ((buf[3] >> 5) & 7));
  myStruct->RightT = (buf[3] & (32 - 1));

  myStruct->ButtonDown = ((buf[4] & (1 << 6)) == 0);
  myStruct->ButtonLeft = ((buf[5] & (1 << 1)) == 0);
  myStruct->ButtonUp = ((buf[5] & (1 << 0)) == 0);
  myStruct->ButtonRight = ((buf[4] & (1 << 7)) == 0);
  myStruct->ButtonSelect = ((buf[4] & (1 << 4)) == 0);
  myStruct->ButtonHome = ((buf[4] & (1 << 3)) == 0);
  myStruct->ButtonStart = ((buf[4] & (1 << 2)) == 0);
  myStruct->ButtonY = ((buf[5] & (1 << 5)) == 0);
  myStruct->ButtonX = ((buf[5] & (1 << 3)) == 0);
  myStruct->ButtonA = ((buf[5] & (1 << 4)) == 0);
  myStruct->ButtonB = ((buf[5] & (1 << 6)) == 0);
  myStruct->ButtonL = ((buf[4] & (1 << 5)) == 0);
  myStruct->ButtonR = ((buf[4] & (1 << 1)) == 0);
  myStruct->ButtonZL = ((buf[5] & (1 << 7)) == 0);
  myStruct->ButtonZR = ((buf[5] & (1 << 2)) == 0);
*/
  int i;
  int d_pointer;

  joydrv_snddata[0] = joydrv_snddata[1] = joydrv_snddata[2] = joydrv_snddata[3] = B11111111;
  joydrv_snddata[4] = joydrv_snddata[5] = joydrv_snddata[6] = joydrv_snddata[7] = B10000000;
  for(i=8;i<16;i++) joydrv_snddata[i] = 0;
/*
  joydrv_snddata[5] = byte(buf[d_pointer+1]); // L左右アナログ
  joydrv_snddata[4] = byte(buf[d_pointer+2]); // L上下アナログ
  joydrv_snddata[7] = byte(buf[d_pointer+3]); // R左右アナログ
  joydrv_snddata[6] = byte(buf[d_pointer+4]); // R上下アナログ
  joydrv_snddata[8]  = byte(buf[d_pointer+8]); // 左アナログボタン
  joydrv_snddata[12] = byte(buf[d_pointer+9]); // 右アナログボタン
  joydrv_snddata[3] = ps_udlr_data[byte(buf[d_pointer+5])&B00001111];
*/
  // classic.leftJoyX;

  if(classic.dpadUp()) // UP
    joydrv_snddata[3] &= B11111110;
  if(classic.dpadDown()) // DOWN
    joydrv_snddata[3] &= B11111101;
  if(classic.dpadLeft()) // LEFT
    joydrv_snddata[3] &= B11111011;
  if(classic.dpadRight()) // RIGHT
    joydrv_snddata[3] &= B11110111;
 
  if(classic.buttonB()) // Aボタン (cross)
    joydrv_snddata[2] &= B11111110;
  if(classic.buttonA()) // Bボタン (circle)
    joydrv_snddata[2] &= B11111101;
  if(classic.buttonR()) // R1ボタン(R)
    joydrv_snddata[1] &= B11101111;
  if(classic.buttonZR()) // R2ボタン(ZR)
    joydrv_snddata[1] &= B11011111;
//  if(buf[d_pointer+6]&0x0080) // R3ボタン
//    joydrv_snddata[1] &= B10111111;
  if(classic.buttonY()) // Xボタン (square)
    joydrv_snddata[2] &= B11101111;
  if(classic.buttonX()) // Yボタン (triangle)
    joydrv_snddata[2] &= B11011111;
  if(classic.buttonL()) // L1ボタン(L)
    joydrv_snddata[1] &= B11111110;
  if(classic.buttonZL()) // L2ボタン(ZL)
    joydrv_snddata[1] &= B11111101;
//  if(buf[d_pointer+6]&0x0040) // L3ボタン
//    joydrv_snddata[1] &= B11111011;

  if(classic.buttonPlus()) // STARTボタン
    joydrv_snddata[0] &= B11111110;
  if(classic.buttonMinus()) // SELECTボタン
    joydrv_snddata[0] &= B11111101;
}

void setup() {
  int i,j;

  Serial.begin(115200);
  classic.begin();

  while (!classic.connect()) {
    Serial.println("Classic Controller not detected!");
    delay(1000);
  }

  atari_data_SELL = atari_data_SELH = B11111111;

  for(j=0;j<=3;j++) joydrv_snddata[j] = B11111111;
  for(j=4;j<=7;j++) joydrv_snddata[j] = B10000000;
  for(j=8;j<=15;j++) joydrv_snddata[j] = 0;


  DDRD = B11111011;
  cnv_mode = EEPROM.read(0);
  if(cnv_mode > IF_MODE_JOYDRV) cnv_mode = IF_MODE_ATARI;

  set_cnv_mode();

  mode_time = millis();  /* 起動時の時間 */
  cyber_if_flg = 0; /* 通信要求クリア */
  joydrv_if_flg = 0; /* 通信要求クリア */
}

void set_cnv_mode()
{
  int i,j;

  detachInterrupt(0);
  switch(cnv_mode) {
    case IF_MODE_ATARI:
      for(i=0;i<2;i++) {
        delay(600);
        digitalWrite(LED, HIGH); // LED13 ON
        delay(600);
        digitalWrite(LED, LOW); // LED13 OFF
      }
      DDRB = B00111111;
      PORTB = B11111111;
      attachInterrupt(0, int_cpsfatari, CHANGE);
      break;
    case IF_MODE_CPSF:
      for(i=0;i<2;i++) {
        delay(800);
        for(j=0;j<2;j++) {
          delay(300);
          digitalWrite(LED, HIGH); // LED13 ON
          delay(300);
          digitalWrite(LED, LOW); // LED13 OFF
        }
      }
      DDRB = B00111111;
      PORTB = B11111111;
      attachInterrupt(0, int_cpsfatari, CHANGE);
      break;
    case IF_MODE_CYBER:
      for(i=0;i<2;i++) {
        delay(800);
        for(j=0;j<3;j++) {
          delay(150);
          digitalWrite(LED, HIGH); // LED13 ON
          delay(150);
          digitalWrite(LED, LOW); // LED13 OFF
        }
      }
      DDRB = B00111111;
      PORTB = B11111111;
      attachInterrupt(0, int_cyber, FALLING);
      break;
    case IF_MODE_JOYDRV:
      for(i=0;i<2;i++) {
        delay(800);
        for(j=0;j<4;j++) {
          delay(150);
          digitalWrite(LED, HIGH); // LED13 ON
          delay(150);
          digitalWrite(LED, LOW); // LED13 OFF
        }
      }
      DDRB = B00001111;
      PORTB = B11111111;
      attachInterrupt(0, int_joydrv, FALLING);
      break;
  }
}

void loop() {
  byte cyber_data[12];
    // 上位4bitはステータス、下位4bitがデータ
    //  0 : A+A' B+B' C D
    //  1 : E1 E2 F(START) G(SELECT)
    //  2 : 左上下 上位4bit
    //  3 : 左左右 上位4bit
    //  4 : 右上下 上位4bit
    //  5 : 右左右 上位4bit
    //  6 : 左上下 下位4bit
    //  7 : 左左右 下位4bit
    //  8 : 右上下 下位4bit
    //  9 : 右左右 下位4bit
    // 10 : A B A' B'
    // 11 : 1 1 1 1

  int joydrv_port;
  byte atari_work_SELL;
  byte atari_work_SELH;
  byte flg_chg_mode;
  int i;
  unsigned long now_time;

  boolean success = classic.update();  // Get new data from the controller

  if (!success) {  // Ruh roh
    Serial.println("Controller disconnected!");
    delay(1000);
  }
  else {
    controller_decode_bytes();        // データ分析
/****************************************************************************/

    now_time = millis(); // 現在の起動からの時間
    if(!(joydrv_snddata[0]&0x03)) { //SELECT + STARTボタン
      if(!(joydrv_snddata[2]&0x01)) // Aボタン
        flg_chg_mode = IF_MODE_ATARI;
      else if(!(joydrv_snddata[2]&0x02)) // Bボタン
        flg_chg_mode = IF_MODE_CPSF;
      else if(!(joydrv_snddata[2]&0x10)) // Xボタン
        flg_chg_mode = IF_MODE_CYBER;
      else if(!(joydrv_snddata[2]&0x20)) // Yボタン
        flg_chg_mode = IF_MODE_JOYDRV;
      else mode_time = now_time;
      if((now_time - mode_time) >= MODE_CHG_TIME) {
        cnv_mode = flg_chg_mode;
        EEPROM.write(0, cnv_mode);
        set_cnv_mode();
        mode_time = now_time = millis(); // 現在の起動からの時間
        cyber_if_flg = 0; /* 通信要求クリア */
        joydrv_if_flg = 0; /* 通信要求クリア */
      }
    }
    else mode_time = now_time;
  
    /************************* CYBER STICK MODE ***************************/
    if (cyber_if_flg == 1) {
      for(i=0;i<12;i+=2) {
        cyber_data[i]=B11001111;
        cyber_data[i+1]=B11011111;
      }
      /* 左左右アナログ */
      cyber_data[3] &= ((joydrv_snddata[5]>>4)|B11110000); // 上位4ビット
      cyber_data[7] &= (joydrv_snddata[5]|B11110000); // 下位4ビット
      /* 左上下アナログ */
      cyber_data[2] &= ((joydrv_snddata[4]>>4)|B11110000); // 上位4ビット
      cyber_data[6] &= (joydrv_snddata[4]|B11110000); // 下位4ビット
      /* 右左右アナログ */
      cyber_data[5] &= ((joydrv_snddata[7]>>4)|B11110000); // 上位4ビット
      cyber_data[9] &= (joydrv_snddata[7]|B11110000); // 下位4ビット
      /* 右上下アナログ */
      cyber_data[4] &= ((joydrv_snddata[6]>>4)|B11110000); // 上位4ビット
      cyber_data[8] &= (joydrv_snddata[6]|B11110000); // 下位4ビット
      if(!(joydrv_snddata[2]&0x01)) { // Aボタン
        cyber_data[0]  &= B11110111;
        cyber_data[10] &= B11110111;
      }
      if(!(joydrv_snddata[2]&0x02)) { // Bボタン
        cyber_data[0]  &= B11111011;
        cyber_data[10] &= B11111011;
      }
      if(!(joydrv_snddata[1]&0x10)) // R1ボタン
        cyber_data[0] &= B11111101;
      if(!(joydrv_snddata[2]&0x10)) { // Xボタン
        cyber_data[0]  &= B11110111;
        cyber_data[10] &= B11111101;
      }
      if(!(joydrv_snddata[2]&0x20)) { // Yボタン
        cyber_data[0]  &= B11111011;
        cyber_data[10] &= B11111110;
      }
      if(!(joydrv_snddata[1]&0x01)) // L1ボタン
        cyber_data[0] &= B11111110;
      if(!(joydrv_snddata[1]&0x20)) // R2ボタン
        cyber_data[1] &= B11110111;
      if(!(joydrv_snddata[1]&0x02)) // L2ボタン
        cyber_data[1] &= B11111011;
      if(!(joydrv_snddata[0]&0x01)) // STARTボタン
        cyber_data[1] &= B11111101;
      if(!(joydrv_snddata[0]&0x02)) // SELECTボタン
        cyber_data[1] &= B11111110;
      
      for(i=0;i<12;i++) {
        PORTB = cyber_data[i];
        delayMicroseconds(CYBER_WAIT);
      }
      PORTB = B11111111;
      cyber_if_flg = 0;
    }
    /******************************* JOYDRV MODE *********************************/
    else if (joydrv_if_flg == 1) {
      joydrv_if_flg = 0;
  
      joydrv_port = rcv_joydrv();

/*      if(joydrv_port < 0) {
        Serial.println(F("JOYDRV rcv ERROR1"));
        goto joydrvif_error;
      }*/

      /* PC本体側のBUSY終了待ち */
      for(i=0;i<JOYDRV_WAIT;i++) {
        if(PINB&B00010000) break;
      }
      if(i>=JOYDRV_WAIT) {
        Serial.print(F("JOYDRV BUSY ERROR"));
        goto joydrvif_error;
      }
  
      for(i=0;i<16;i++) {
        if(snd_joydrv(joydrv_snddata[i])<0) {
          Serial.print(F("JOYDRV snd ERROR"));
          Serial.println(i,DEC);
          goto joydrvif_error;
        }
      }
  joydrvif_error:
      PORTB = B11111111;
    }
    /******************************* ATARI MODE *********************************/
    else if(cnv_mode == IF_MODE_ATARI) {
      atari_work_SELL = (joydrv_snddata[3]|B11110000);
      atari_work_SELH = ((joydrv_snddata[3]>>4)|B11110000);
      if(joydrv_snddata[5] < AD_CNV_DATA1) atari_work_SELL &= B11111011; // 左
      else if(joydrv_snddata[5] > AD_CNV_DATA2) atari_work_SELL &= B11110111; // 右
      if(joydrv_snddata[4] < AD_CNV_DATA1) atari_work_SELL &= B11111110; // 上
      else if(joydrv_snddata[4] > AD_CNV_DATA2) atari_work_SELL &= B11111101; // 下
      if(joydrv_snddata[7] < AD_CNV_DATA1) atari_work_SELH &= B11111011; // 左
      else if(joydrv_snddata[7] > AD_CNV_DATA2) atari_work_SELH &= B11110111; // 右
      if(joydrv_snddata[6] < AD_CNV_DATA1) atari_work_SELH &= B11111110; // 上
      else if(joydrv_snddata[6] > AD_CNV_DATA2) atari_work_SELH &= B11111101; // 下
      if(!(joydrv_snddata[0]&0x01)) // STARTボタン
        atari_work_SELL &= B11110011;
      if(!(joydrv_snddata[0]&0x02)) // SELECTボタン
        atari_work_SELL &= B11111100;
      if(!(joydrv_snddata[2]&0x20) || !(joydrv_snddata[2]&0x01)) { // AボタンorYボタン
        atari_work_SELL &= B11101111;
        atari_work_SELH &= B11101111;
      }
      if(!(joydrv_snddata[2]&0x10) || !(joydrv_snddata[2]&0x02)) { // BボタンorXボタン
        atari_work_SELL &= B11011111;
        atari_work_SELH &= B11011111;
      }
      atari_data_SELL = atari_work_SELL;
      atari_data_SELH = atari_work_SELH;
            
      PORTB = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
    }
    /******************************* CPSF MODE *********************************/
    else if(cnv_mode == IF_MODE_CPSF) {
      atari_work_SELL = (joydrv_snddata[3]|B11110000);
      atari_work_SELH = B11111111;
      if(joydrv_snddata[5] < AD_CNV_DATA1) atari_work_SELL &= B11111011; // 左
      else if(joydrv_snddata[5] > AD_CNV_DATA2) atari_work_SELL &= B11110111; // 右
      if(joydrv_snddata[4] < AD_CNV_DATA1) atari_work_SELL &= B11111110; // 上
      else if(joydrv_snddata[4] > AD_CNV_DATA2) atari_work_SELL &= B11111101; // 下
      if(!(joydrv_snddata[0]&0x01)) // STARTボタン
        atari_work_SELH &= B11011111;
      if(!(joydrv_snddata[0]&0x02)) // SELECTボタン
        atari_work_SELH &= B11110111;
      if(!(joydrv_snddata[2]&0x01)) // Aボタン
        atari_work_SELL &= B11101111;
      if(!(joydrv_snddata[2]&0x02)) // Bボタン
        atari_work_SELL &= B11011111;
      if(!(joydrv_snddata[1]&0x10)) // R1ボタン
        atari_work_SELH &= B11111110;
      if(!(joydrv_snddata[2]&0x10)) // Xボタン
        atari_work_SELH &= B11111011;
      if(!(joydrv_snddata[2]&0x20)) // Yボタン
        atari_work_SELH &= B11111101;
      if(!(joydrv_snddata[1]&0x01)) // L1ボタン
        atari_work_SELH &= B11101111;
      atari_data_SELL = atari_work_SELL;
      atari_data_SELH = atari_work_SELH;
  
      PORTB = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
    }
  }
}


int rcv_joydrv()
{
  int d_work;
  int i;
  long j;

  d_work = 0;
  for(i=0;i<4;i++) {
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINB&B00010000)) {
        PORTB &= B11111110;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINB&B00010000) {
        d_work = d_work<<1;
        if(PINB&B00100000) d_work|=1;
        PORTB |= B00000001;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;

    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINB&B00100000)) {
        PORTB &= B11111101;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINB&B00100000) {
        d_work = d_work<<1;
        if(PINB&B00010000) d_work|=1;
        PORTB |= B00000010;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
  }
  return d_work;
}

int snd_joydrv(byte snd_data)
{
  byte d_work;
  int i;
  long j;

  d_work = snd_data;
  for(i=0;i<4;i++) {
    if(d_work&B10000000) PORTB |= B00000010;
    else PORTB &= B11111101;
    d_work = d_work<<1;
    PORTB &= B11111110;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINB&B00010000)) {
        PORTB |= B00000001;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINB&B00010000) break;
    }
    if(j>=JOYDRV_WAIT) return -1;

    if(d_work&B10000000) PORTB |= B00000001;
    else PORTB &= B11111110;
    d_work = d_work<<1;
    PORTB &= B11111101;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINB&B00100000)) {
        PORTB |= B00000010;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINB&B00100000) break;
    }
    if(j>=JOYDRV_WAIT) return -1;
  }
  return 0;
}

void int_cpsfatari()
{
  PORTB = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
}

void int_cyber()
{
  cyber_if_flg = 1;
}

void int_joydrv()
{
  joydrv_if_flg = 1;
}
