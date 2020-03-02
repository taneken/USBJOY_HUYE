// USBコントローラ -> ATARIインタフェース
// Copyright
//   メインプログラム、コントローラタイプ(TYPE_PS4, TYPE_MDmini) :
//     HUYE 2018-2020
//   コントローラタイプ(TYPE_PSC, TYPE_RAP3, TYPE_RAP4, TYPE_SNES, TYPE_RETROFREAK) : 
//     たねけん 2020
//
#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

#define TYPE_PS4 0
#define TYPE_MDmini 1
#define TYPE_PSC 2
#define TYPE_RAP3 3
#define TYPE_RAP4 4
#define TYPE_SNES 5
#define TYPE_RETROFREAK 6

#define MAX_JOYSTICK 4
#define CYBER_WAIT  25
#define JOYDRV_WAIT 1000000L
#define AD_CNV_DATA1 64
#define AD_CNV_DATA2 192

#define IF_MODE_ATARI 7
#define IF_MODE_CPSF  6
#define IF_MODE_CYBER 5
#define IF_MODE_JOYDRV 4

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

const struct {
  uint16_t vid;
  uint16_t pid;
  int joy_type;
} Tbl_cnv_data[] = {
  {0x054c, 0x09cc, TYPE_PS4}, // PS4標準コントローラ
  {0x0ca3, 0x0024, TYPE_MDmini}, // MDmini標準コントローラ
  {0x054c, 0x0ba0, TYPE_PS4}, // PS4 Wireless Adapter
  {0x054c, 0x0cda, TYPE_PSC}, // PlayStation Classic USB Controller
  {0x0f0d, 0x0011, TYPE_RAP3}, // HORI Real Arcade Pro.3 SA PS3コントローラ
  {0x0f0d, 0x008b, TYPE_RAP3}, // HORI RAP V HAYABUSA Controller(PS3)
  {0x0f0d, 0x008a, TYPE_PS4},  // HORI RAP V HAYABUSA Controller(PS4)
  {0x0f0d, 0x0066, TYPE_PS4},  // HORIPAD FPS+(PS4)
  {0x0f0d, 0x00ee, TYPE_PS4},  // HORI ワイヤードコントローラライト for PS4-102
  {0x0583, 0x2060, TYPE_SNES}, // iBUFFALO SNES CLASSIC USB GAMEPAD
  {0x1345, 0x1040, TYPE_SNES}, // RetroFreak GAME CONTROLLER
  {0x0413, 0x502b, TYPE_RETROFREAK}, // RetroFreak CONTROLLER ADAPTER
  {0x046d, 0xc219, TYPE_PS4},  // Logicool Wireless Gamepad F710
  {0, 0, -1} // データ終端
};

byte cnv_mode;
volatile byte cyber_if_flg;
volatile byte joydrv_if_flg;
volatile byte atari_data_SELL;
volatile byte atari_data_SELH;
volatile byte joydrv_snddata[MAX_JOYSTICK][16];
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
volatile struct {
  byte motor1;
  byte motor2;
  byte led_r;
  byte led_g;
  byte led_b;
  byte flush_on;
  byte flush_off;
  bool flg_change;
} stick_ctrldata[MAX_JOYSTICK];

const byte ps_udlr_data[16] = {B11111110, B11110110, B11110111, B11110101, B11111101, B11111001, B11111011, B11111010,
                               B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111, B11111111};

const byte psc_udlr_data[16] = {B11111010, B11111110, B11110110, B11111111, B11111011, B11111111, B11110111, B11111111,
                                B11111001, B11111101, B11110101, B11111111, B11111111, B11111111, B11111111, B11111111};

class JoystickHID : public HIDUniversal {
public:
  JoystickHID(USB *p) :
  HIDUniversal(p) { };

protected:
  virtual void ParseHIDData(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf)
  {
    int i;
    int port_no;
    int cnv_pointer;
    int d_pointer;
    uint8_t w_buf[32];

    port_no=(bAddress>=9 ? bAddress-9 : 0);

    cnv_pointer=0;
    while(Tbl_cnv_data[cnv_pointer].joy_type != -1) {
      if(Tbl_cnv_data[cnv_pointer].vid == VID && Tbl_cnv_data[cnv_pointer].pid == PID) break;
      cnv_pointer++;
    }

/*    Serial.print(VID, HEX);
    Serial.print(F(":"));
    Serial.print(PID, HEX);
    Serial.print(F(":"));
    Serial.println(buf[0], HEX);*/

    // デバッグ用VID/PID buf[x]表示
    char strString[256];
    sprintf(strString, "VID:%04X PID:%04X cnv_pointer:%02X\t",VID,PID,cnv_pointer);
    Serial.print(strString);
    Serial.print(" ");
    for (int i=0; i<len; i++ ) {
      PrintHex<uint8_t > (buf[i], 0);
      Serial.print(" ");
    }
    Serial.println("");
    
    joydrv_snddata[port_no][0] = joydrv_snddata[port_no][1] = joydrv_snddata[port_no][2] = joydrv_snddata[port_no][3] = B11111111;
    joydrv_snddata[port_no][4] = joydrv_snddata[port_no][5] = joydrv_snddata[port_no][6] = joydrv_snddata[port_no][7] = B10000000;
    for(i=8;i<16;i++) joydrv_snddata[port_no][i] = 0;

    // コントローラ別処理
    switch(Tbl_cnv_data[cnv_pointer].joy_type) {
      case TYPE_MDmini: // MDmini
        joydrv_snddata[port_no][5] = byte(buf[3]); // 左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[4]); // 上下アナログ

        if(buf[5]&0x0040) // Aボタン
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[5]&0x0020) // Bボタン
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[6]&0x0002) // Cボタン
          joydrv_snddata[port_no][2] &= B11111011;
        if(buf[5]&0x0080) // Xボタン
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[5]&0x0010) // Yボタン
          joydrv_snddata[port_no][2] &= B11011111;
        if(buf[6]&0x0001) // Zボタン
          joydrv_snddata[port_no][2] &= B10111111;

        if(buf[6]&0x0020) // STARTボタン
          joydrv_snddata[port_no][0] &= B11111110;
        if(buf[6]&0x0010) // SELECT(MODE)ボタン
          joydrv_snddata[port_no][0] &= B11111101;

        if(stick_ctrldata[port_no].flg_change)
          stick_ctrldata[port_no].flg_change = false;

        break;

      case TYPE_RAP3: // RAP (PS3mode)
        // [[ HORI Real Arcade Pro.V HAYABUSA(PS3 MODE) ------------------------------------------------------
        joydrv_snddata[port_no][3] = ps_udlr_data[byte(buf[2])&B00001111];

        if(buf[0]&0x0004) // Aボタン (cross)
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[0]&0x0002) // Bボタン (circle)
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[0]&0x0020) // R1ボタン
          joydrv_snddata[port_no][1] &= B11101111;
        if(buf[0]&0x0080) // R2ボタン
          joydrv_snddata[port_no][1] &= B11011111;
        if(buf[1]&0x0008) // R3ボタン
          joydrv_snddata[port_no][1] &= B10111111;
        if(buf[0]&0x0008) // Xボタン (square)
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[0]&0x0001) // Yボタン (triangle)
          joydrv_snddata[port_no][2] &= B11011111;
        if(buf[0]&0x0010) // L1ボタン
          joydrv_snddata[port_no][1] &= B11111110;
        if(buf[0]&0x0040) // L2ボタン
          joydrv_snddata[port_no][1] &= B11111101;
        if(buf[1]&0x0004) // L3ボタン
          joydrv_snddata[port_no][1] &= B11111011;

        if(buf[1]&0x0002) // STARTボタン
          joydrv_snddata[port_no][0] &= B11111110;
        if(buf[1]&0x0001) // SELECTボタン
          joydrv_snddata[port_no][0] &= B11111101;

        if(stick_ctrldata[port_no].flg_change)
          stick_ctrldata[port_no].flg_change = false;
        break;

      case TYPE_PSC:
        // [[ Playstation Classic Controller Parser ----------------------------------------------------------

        joydrv_snddata[port_no][3] = psc_udlr_data[(byte(buf[1])>>2)&B00001111];

        if(buf[0]&0x0004) // Aボタン (cross)
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[0]&0x0002) // Bボタン (circle)
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[0]&0x0080) // R1ボタン
          joydrv_snddata[port_no][1] &= B11101111;
        if(buf[0]&0x0020) // R2ボタン
          joydrv_snddata[port_no][1] &= B11011111;
        if(buf[0]&0x0008) // Xボタン (square)
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[0]&0x0001) // Yボタン (triangle)
          joydrv_snddata[port_no][2] &= B11011111;
        if(buf[0]&0x0040) // L1ボタン
          joydrv_snddata[port_no][1] &= B11111110;
        if(buf[0]&0x0010) // L2ボタン
          joydrv_snddata[port_no][1] &= B11111101;

        if(buf[1]&0x0002) // STARTボタン
          joydrv_snddata[port_no][0] &= B11111110;
        if(buf[1]&0x0001) // SELECTボタン
          joydrv_snddata[port_no][0] &= B11111101;

        if(stick_ctrldata[port_no].flg_change)
          stick_ctrldata[port_no].flg_change = false;
        break;

      case TYPE_SNES:
        // [[ iBUFFALO SNES CLASSIC USB GAMEPAD Parser -------------------------------------------------------
        joydrv_snddata[port_no][5] = byte(buf[0]); // 左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[1]); // 上下アナログ

        if(buf[2]&0x0001) // Aボタン (circle)
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[2]&0x0002) // Bボタン (cross)
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[2]&0x0004) // Xボタン (triangle)
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[2]&0x0008) // Yボタン (square)
          joydrv_snddata[port_no][2] &= B11011111;
        if(buf[2]&0x0010) // R1ボタン
          joydrv_snddata[port_no][1] &= B11101111;
        if(buf[2]&0x0020) // L1ボタン
          joydrv_snddata[port_no][1] &= B11111110;
        if(buf[2]&0x0040) // STARTボタン
          joydrv_snddata[port_no][0] &= B11111110;
        if(buf[2]&0x0080) // SELECTボタン
          joydrv_snddata[port_no][0] &= B11111101;

        if(stick_ctrldata[port_no].flg_change)
          stick_ctrldata[port_no].flg_change = false;
        break;

      case TYPE_RETROFREAK:
        // レトロフリーク コントローラアダプター -------------------------------------------------------------
        switch(byte(buf[0])) {
          case 1: // NES??
            joydrv_snddata[port_no][5] = byte(buf[1]); // 左右アナログ
            joydrv_snddata[port_no][4] = byte(buf[2]); // 上下アナログ
    
            if(buf[3]&0x0001) // Aボタン
              joydrv_snddata[port_no][2] &= B11111110;
            if(buf[3]&0x0002) // Bボタン
              joydrv_snddata[port_no][2] &= B11111101;
            if(buf[3]&0x0004) // STARTボタン
              joydrv_snddata[port_no][0] &= B11111110;
            if(buf[3]&0x0008) // SELECTボタン
              joydrv_snddata[port_no][0] &= B11111101;
    
            if(stick_ctrldata[port_no].flg_change)
              stick_ctrldata[port_no].flg_change = false;
            break;
          case 2: // SFC
            joydrv_snddata[port_no][5] = byte(buf[1]); // 左右アナログ
            joydrv_snddata[port_no][4] = byte(buf[2]); // 上下アナログ
    
            if(buf[3]&0x0001) // Aボタン
              joydrv_snddata[port_no][2] &= B11111110;
            if(buf[3]&0x0002) // Bボタン
              joydrv_snddata[port_no][2] &= B11111101;
            if(buf[3]&0x0004) // Xボタン (triangle)
              joydrv_snddata[port_no][2] &= B11101111;
            if(buf[3]&0x0008) // Yボタン (square)
              joydrv_snddata[port_no][2] &= B11011111;
            if(buf[3]&0x0010) // R1ボタン
              joydrv_snddata[port_no][1] &= B11101111;
            if(buf[3]&0x0020) // L1ボタン
              joydrv_snddata[port_no][1] &= B11111110;
            if(buf[3]&0x0040) // STARTボタン
              joydrv_snddata[port_no][0] &= B11111110;
            if(buf[3]&0x0080) // SELECTボタン
              joydrv_snddata[port_no][0] &= B11111101;
    
            if(stick_ctrldata[port_no].flg_change)
              stick_ctrldata[port_no].flg_change = false;
            break;
          case 3: // MegaDrive 6button
            joydrv_snddata[port_no][5] = byte(buf[1]); // 左右アナログ
            joydrv_snddata[port_no][4] = byte(buf[2]); // 上下アナログ
    
            if(buf[3]&0x0001) // Aボタン
              joydrv_snddata[port_no][2] &= B11111110;
            if(buf[3]&0x0002) // Bボタン
              joydrv_snddata[port_no][2] &= B11111101;
            if(buf[3]&0x0004) // Cボタン
              joydrv_snddata[port_no][2] &= B11111011;
            if(buf[3]&0x0008) // Xボタン
              joydrv_snddata[port_no][2] &= B11101111;
            if(buf[3]&0x0010) // Yボタン
              joydrv_snddata[port_no][2] &= B11011111;
            if(buf[3]&0x0020) // Zボタン
              joydrv_snddata[port_no][2] &= B10111111;
            if(buf[3]&0x0040) // STARTボタン
              joydrv_snddata[port_no][0] &= B11111110;
            if(buf[3]&0x0080) // MODEボタン
              joydrv_snddata[port_no][0] &= B11111101;
    
            if(stick_ctrldata[port_no].flg_change)
              stick_ctrldata[port_no].flg_change = false;
            break;
          case 4: // PCE?? PCE 6bは??
            joydrv_snddata[port_no][5] = byte(buf[1]); // 左右アナログ
            joydrv_snddata[port_no][4] = byte(buf[2]); // 上下アナログ
    
            if(buf[3]&0x0001) // Aボタン
              joydrv_snddata[port_no][2] &= B11111110;
            if(buf[3]&0x0002) // Bボタン
              joydrv_snddata[port_no][2] &= B11111101;
            if(buf[3]&0x0004) // STARTボタン
              joydrv_snddata[port_no][0] &= B11111110;
            if(buf[3]&0x0008) // SELECTボタン
              joydrv_snddata[port_no][0] &= B11111101;
    
            if(stick_ctrldata[port_no].flg_change)
              stick_ctrldata[port_no].flg_change = false;
            break;
          case 5: // FC
            joydrv_snddata[port_no][5] = byte(buf[1]); // 左右アナログ
            joydrv_snddata[port_no][4] = byte(buf[2]); // 上下アナログ
    
            if(buf[3]&0x0001) // Aボタン
              joydrv_snddata[port_no][2] &= B11111110;
            if(buf[3]&0x0002) // Bボタン
              joydrv_snddata[port_no][2] &= B11111101;
            if(buf[3]&0x0004) // STARTボタン
              joydrv_snddata[port_no][0] &= B11111110;
            if(buf[3]&0x0008) // SELECTボタン
              joydrv_snddata[port_no][0] &= B11111101;
    
            if(stick_ctrldata[port_no].flg_change)
              stick_ctrldata[port_no].flg_change = false;
            break;

        }
        break;

      case TYPE_PS4: // PS4
      defult: // 標準
        if(buf[0] == 0x01) d_pointer = 0;
        else if(buf[0] == 0x11) {
          if (len < 4) return;
          d_pointer = 2;
        }
        else return;

        joydrv_snddata[port_no][5] = byte(buf[d_pointer+1]); // L左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[d_pointer+2]); // L上下アナログ
        joydrv_snddata[port_no][7] = byte(buf[d_pointer+3]); // R左右アナログ
        joydrv_snddata[port_no][6] = byte(buf[d_pointer+4]); // R上下アナログ
        joydrv_snddata[port_no][8]  = byte(buf[d_pointer+8]); // 左アナログボタン
        joydrv_snddata[port_no][12] = byte(buf[d_pointer+9]); // 右アナログボタン

        joydrv_snddata[port_no][3] = ps_udlr_data[byte(buf[d_pointer+5])&B00001111];

        if(buf[d_pointer+5]&0x0020) // Aボタン (cross)
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[d_pointer+5]&0x0040) // Bボタン (circle)
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[d_pointer+6]&0x0002) // R1ボタン
          joydrv_snddata[port_no][1] &= B11101111;
        if(buf[d_pointer+6]&0x0008) // R2ボタン
          joydrv_snddata[port_no][1] &= B11011111;
        if(buf[d_pointer+6]&0x0080) // R3ボタン
          joydrv_snddata[port_no][1] &= B10111111;
        if(buf[d_pointer+5]&0x0010) // Xボタン (square)
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[d_pointer+5]&0x0080) // Yボタン (triangle)
          joydrv_snddata[port_no][2] &= B11011111;
        if(buf[d_pointer+6]&0x0001) // L1ボタン
          joydrv_snddata[port_no][1] &= B11111110;
        if(buf[d_pointer+6]&0x0004) // L2ボタン
          joydrv_snddata[port_no][1] &= B11111101;
        if(buf[d_pointer+6]&0x0040) // L3ボタン
          joydrv_snddata[port_no][1] &= B11111011;

        if((buf[d_pointer+6]&0x0020) || (buf[d_pointer+7]&0x0001)) // STARTボタン
          joydrv_snddata[port_no][0] &= B11111110;
        if(buf[d_pointer+6]&0x0010) // SELECTボタン
          joydrv_snddata[port_no][0] &= B11111101;

        if(stick_ctrldata[port_no].flg_change) {
          memset(w_buf, 0, sizeof(w_buf));
          w_buf[0]  = 0x05;
          w_buf[1]  = 0xFF;
          w_buf[4]  = stick_ctrldata[port_no].motor2;
          w_buf[5]  = stick_ctrldata[port_no].motor1;
          w_buf[6]  = stick_ctrldata[port_no].led_r;
          w_buf[7]  = stick_ctrldata[port_no].led_g;
          w_buf[8]  = stick_ctrldata[port_no].led_b;
          w_buf[9]  = stick_ctrldata[port_no].flush_on;
          w_buf[10] = stick_ctrldata[port_no].flush_off;

          stick_ctrldata[port_no].flg_change = false;
          SndRpt(sizeof(w_buf), w_buf);
        }
        break;
    }
  }
};

USB Usb;
USBHub Hub(&Usb);
JoystickHID Hid1(&Usb);
JoystickHID Hid2(&Usb);
JoystickHID Hid3(&Usb);
JoystickHID Hid4(&Usb);

void setup() {
  int i,j;

  PORTB |= B00000111;
  cyber_if_flg = 0;
  joydrv_if_flg = 0;

  atari_data_SELL = atari_data_SELH = B11111111;

  for(i=0;i<MAX_JOYSTICK;i++) {
    for(j=0;j<=3;j++) joydrv_snddata[i][j] = B11111111;
    for(j=4;j<=7;j++) joydrv_snddata[i][j] = B10000000;
    for(j=8;j<=15;j++) joydrv_snddata[i][j] = 0;
    stick_ctrldata[i].motor1=0;
    stick_ctrldata[i].motor2=0;
    stick_ctrldata[i].led_r=0;
    stick_ctrldata[i].led_g=0;
    stick_ctrldata[i].led_b=0;
    stick_ctrldata[i].flush_on=0;
    stick_ctrldata[i].flush_off=0;
    stick_ctrldata[i].flg_change=false;
  }

  cnv_mode = (PINB&B00000111);

  Serial.begin(115200);
  while (!Serial);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }

  switch(cnv_mode) {
    case IF_MODE_ATARI:
    case IF_MODE_CPSF:
      DDRC |= B00111111;
      PORTC = B11111111;
      attachInterrupt(0, int_cpsfatari, CHANGE);
      break;
    case IF_MODE_CYBER:
      DDRC |= B00111111;
      PORTC = B11111111;
      attachInterrupt(0, int_cyber, FALLING);
      break;
    case IF_MODE_JOYDRV:
      DDRC |= B00001111;
      PORTC = B11111111;
      attachInterrupt(0, int_joydrv, FALLING);
      break;
  }
  Serial.print(F("\r\nUSB controller -> ATARI ver 1.20\r\n"));

  cyber_if_flg = 0; /* 起動時割込み誤検知防止 */
  joydrv_if_flg = 0; /* 起動時割込み誤検知防止 */
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
  int motor1;
  int motor2;
  byte atari_work_SELL;
  byte atari_work_SELH;
  int i;

  Usb.Task();

  if (cyber_if_flg == 1) {
    for(i=0;i<12;i+=2) {
      cyber_data[i]=B11001111;
      cyber_data[i+1]=B11011111;
    }
    /* 左左右アナログ */
    cyber_data[3] &= ((joydrv_snddata[0][5]>>4)|B11110000); // 上位4ビット
    cyber_data[7] &= (joydrv_snddata[0][5]|B11110000); // 下位4ビット
    /* 左上下アナログ */
    cyber_data[2] &= ((joydrv_snddata[0][4]>>4)|B11110000); // 上位4ビット
    cyber_data[6] &= (joydrv_snddata[0][4]|B11110000); // 下位4ビット
    /* 右左右アナログ */
    cyber_data[5] &= ((joydrv_snddata[0][7]>>4)|B11110000); // 上位4ビット
    cyber_data[9] &= (joydrv_snddata[0][7]|B11110000); // 下位4ビット
    /* 右上下アナログ */
    cyber_data[4] &= ((joydrv_snddata[0][6]>>4)|B11110000); // 上位4ビット
    cyber_data[8] &= (joydrv_snddata[0][6]|B11110000); // 下位4ビット
    if(!(joydrv_snddata[0][2]&0x01)) { // Aボタン
      cyber_data[0]  &= B11110111;
      cyber_data[10] &= B11110111;
    }
    if(!(joydrv_snddata[0][2]&0x02)) { // Bボタン
      cyber_data[0]  &= B11111011;
      cyber_data[10] &= B11111011;
    }
    if(!(joydrv_snddata[0][1]&0x10)) // R1ボタン
      cyber_data[0] &= B11111101;
    if(!(joydrv_snddata[0][2]&0x10)) { // Xボタン
      cyber_data[0]  &= B11110111;
      cyber_data[10] &= B11111101;
    }
    if(!(joydrv_snddata[0][2]&0x20)) { // Yボタン
      cyber_data[0]  &= B11111011;
      cyber_data[10] &= B11111110;
    }
    if(!(joydrv_snddata[0][1]&0x01)) // L1ボタン
      cyber_data[0] &= B11111110;
    if(!(joydrv_snddata[0][1]&0x20)) // R2ボタン
      cyber_data[1] &= B11110111;
    if(!(joydrv_snddata[0][1]&0x02)) // L2ボタン
      cyber_data[1] &= B11111011;
    if(!(joydrv_snddata[0][0]&0x01)) // STARTボタン
      cyber_data[1] &= B11111101;
    if(!(joydrv_snddata[0][0]&0x02)) // SELECTボタン
      cyber_data[1] &= B11111110;
    
    for(i=0;i<12;i++) {
      PORTC = cyber_data[i];
      delayMicroseconds(CYBER_WAIT);
    }
    PORTC = B11111111;
    cyber_if_flg = 0;
  }
  else if (joydrv_if_flg == 1) {
    joydrv_if_flg = 0;

    joydrv_port = rcv_joydrv();
    if(joydrv_port < 0 || joydrv_port >= MAX_JOYSTICK) {
      Serial.println(F("JOYDRV rcv ERROR1"));
      goto joydrvif_error;
    }
    motor1 = rcv_joydrv();
    if(motor1 < 0) {
      Serial.println(F("JOYDRV rcv ERROR2"));
      goto joydrvif_error;
    }
    motor2 = rcv_joydrv();
    if(motor2 < 0) {
      Serial.println(F("JOYDRV rcv ERROR3"));
      goto joydrvif_error;
    }
    if(stick_ctrldata[joydrv_port].motor1 != byte(motor1) || stick_ctrldata[joydrv_port].motor2 != byte(motor2))
      stick_ctrldata[joydrv_port].flg_change = true;
    stick_ctrldata[joydrv_port].motor1 = byte(motor1);
    stick_ctrldata[joydrv_port].motor2 = byte(motor2);

    /* PC本体側のBUSY終了待ち */
    for(i=0;i<JOYDRV_WAIT;i++) {
      if(PINC&B00010000) break;
    }
    if(i>=JOYDRV_WAIT) {
      Serial.print(F("JOYDRV BUSY ERROR"));
      goto joydrvif_error;
    }

    for(i=0;i<16;i++) {
      if(snd_joydrv(joydrv_snddata[joydrv_port][i])<0) {
        Serial.print(F("JOYDRV snd ERROR"));
        Serial.println(i,DEC);
        goto joydrvif_error;
      }
    }
joydrvif_error:
    PORTC = B11111111;
  }
  else if(cnv_mode == IF_MODE_ATARI) {
    atari_work_SELL = (joydrv_snddata[0][3]|B11110000);
    atari_work_SELH = ((joydrv_snddata[0][3]>>4)|B11110000);
    if(joydrv_snddata[0][5] < AD_CNV_DATA1) atari_work_SELL &= B11111011; // 左
    else if(joydrv_snddata[0][5] > AD_CNV_DATA2) atari_work_SELL &= B11110111; // 右
    if(joydrv_snddata[0][4] < AD_CNV_DATA1) atari_work_SELL &= B11111110; // 上
    else if(joydrv_snddata[0][4] > AD_CNV_DATA2) atari_work_SELL &= B11111101; // 下
    if(joydrv_snddata[0][7] < AD_CNV_DATA1) atari_work_SELH &= B11111011; // 左
    else if(joydrv_snddata[0][7] > AD_CNV_DATA2) atari_work_SELH &= B11110111; // 右
    if(joydrv_snddata[0][6] < AD_CNV_DATA1) atari_work_SELH &= B11111110; // 上
    else if(joydrv_snddata[0][6] > AD_CNV_DATA2) atari_work_SELH &= B11111101; // 下
    if(!(joydrv_snddata[0][0]&0x01)) // STARTボタン
      atari_work_SELL &= B11110011;
    if(!(joydrv_snddata[0][0]&0x02)) // SELECTボタン
      atari_work_SELL &= B11111100;
    if(!(joydrv_snddata[0][2]&0x20) || !(joydrv_snddata[0][2]&0x01)) { // AボタンorYボタン
      atari_work_SELL &= B11101111;
      atari_work_SELH &= B11101111;
    }
    if(!(joydrv_snddata[0][2]&0x10) || !(joydrv_snddata[0][2]&0x02)) { // BボタンorXボタン
      atari_work_SELL &= B11011111;
      atari_work_SELH &= B11011111;
    }
    atari_data_SELL = atari_work_SELL;
    atari_data_SELH = atari_work_SELH;
          
    PORTC = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
  }
  else if(cnv_mode == IF_MODE_CPSF) {
    atari_work_SELL = (joydrv_snddata[0][3]|B11110000);
    atari_work_SELH = B11111111;
    if(joydrv_snddata[0][5] < AD_CNV_DATA1) atari_work_SELL &= B11111011; // 左
    else if(joydrv_snddata[0][5] > AD_CNV_DATA2) atari_work_SELL &= B11110111; // 右
    if(joydrv_snddata[0][4] < AD_CNV_DATA1) atari_work_SELL &= B11111110; // 上
    else if(joydrv_snddata[0][4] > AD_CNV_DATA2) atari_work_SELL &= B11111101; // 下
    if(!(joydrv_snddata[0][0]&0x01)) // STARTボタン
      atari_work_SELH &= B11011111;
    if(!(joydrv_snddata[0][0]&0x02)) // SELECTボタン
      atari_work_SELH &= B11110111;
    if(!(joydrv_snddata[0][2]&0x01)) // Aボタン
      atari_work_SELL &= B11101111;
    if(!(joydrv_snddata[0][2]&0x02)) // Bボタン
      atari_work_SELL &= B11011111;
    if(!(joydrv_snddata[0][1]&0x10)) // R1ボタン
      atari_work_SELH &= B11111110;
    if(!(joydrv_snddata[0][2]&0x10)) // Xボタン
      atari_work_SELH &= B11111011;
    if(!(joydrv_snddata[0][2]&0x20)) // Yボタン
      atari_work_SELH &= B11111101;
    if(!(joydrv_snddata[0][1]&0x01) || !(joydrv_snddata[0][1]&0x20)) // L1ボタン,R2ボタン
      atari_work_SELH &= B11101111;
    atari_data_SELL = atari_work_SELL;
    atari_data_SELH = atari_work_SELH;

    PORTC = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
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
      if(!(PINC&B00010000)) {
        PORTC &= B11111110;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINC&B00010000) {
        d_work = d_work<<1;
        if(PINC&B00100000) d_work|=1;
        PORTC |= B00000001;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;

    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINC&B00100000)) {
        PORTC &= B11111101;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINC&B00100000) {
        d_work = d_work<<1;
        if(PINC&B00010000) d_work|=1;
        PORTC |= B00000010;
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
    if(d_work&B10000000) PORTC |= B00000010;
    else PORTC &= B11111101;
    d_work = d_work<<1;
    PORTC &= B11111110;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINC&B00010000)) {
        PORTC |= B00000001;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINC&B00010000) break;
    }
    if(j>=JOYDRV_WAIT) return -1;

    if(d_work&B10000000) PORTC |= B00000001;
    else PORTC &= B11111110;
    d_work = d_work<<1;
    PORTC &= B11111101;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(!(PINC&B00100000)) {
        PORTC |= B00000010;
        break;
      }
    }
    if(j>=JOYDRV_WAIT) return -1;
    for(j=0;j<JOYDRV_WAIT;j++) {
      if(PINC&B00100000) break;
    }
    if(j>=JOYDRV_WAIT) return -1;
  }
  return 0;
}

void int_cpsfatari()
{
  PORTC = ((PIND&B00000100) ? atari_data_SELH : atari_data_SELL);
}

void int_cyber()
{
  cyber_if_flg = 1;
}

void int_joydrv()
{
  joydrv_if_flg = 1;
}
