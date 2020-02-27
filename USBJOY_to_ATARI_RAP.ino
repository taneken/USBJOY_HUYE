#include <usbhid.h>
#include <hiduniversal.h>
#include <usbhub.h>

//* debug option
//#define _DEBUG

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

char strString[256];

//*********************************************************************************************
//* VID/PID TABLE
//*********************************************************************************************
const struct {
  uint16_t vid;
  uint16_t pid;
  int joy_type; // 0:PS4,1:MDmini
} Tbl_vidpid[] = {
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
  {0, 0, -1} // データ終端
};
//*********************************************************************************************

volatile bool isUP, isDOWN, isLEFT, isRIGHT;
volatile bool isCIRCLE, isCROSS, isTRI, isSQUARE;
volatile bool isL1, isR1, isL2, isR2, isL3, isR3;
volatile bool isSTART, isSELECT;
volatile bool isANALOG1, isANALOG2, isANALOG3, isANALOG4;


byte cnv_mode;
volatile byte cyber_if_flg;
volatile byte joydrv_if_flg;
volatile byte cpsfatari_data1;
volatile byte cpsfatari_data2;
volatile byte cyber_data[12];
volatile byte joydrv_snddata[MAX_JOYSTICK][16];
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

void DBG( byte data )
{
#ifdef _DEBUG
  Serial.print(data,HEX);
  Serial.print( " " );
#endif
}

class JoystickHID : public HIDUniversal {
public:
  JoystickHID(USB *p) :
  HIDUniversal(p) { };

protected:
  virtual void ParseHIDData(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf)
  {
    int i;
    int port_no;

    isUP = isDOWN = isLEFT = isRIGHT = isCIRCLE = isCROSS = isTRI = isSQUARE = 0;
    isL1 = isR1 = isL2 = isR2 = isL3 = isR3 = isSTART = isSELECT = 0;

    port_no=(bAddress>=9 ? bAddress-9 : 0);

    i=0;
    while(Tbl_vidpid[i].joy_type != -1) {
      if(Tbl_vidpid[i].vid == VID && Tbl_vidpid[i].pid == PID)
        break;
      i++;
    }

    // デバッグ用VID/PID buf[x]表示
    sprintf(strString, "VID:%04X PID:%04X PortNo:%02X\t",VID,PID,port_no);
    Serial.print(strString);
    Serial.print(" ");
    for (int i=0; i<len; i++ ) {
      PrintHex<uint8_t > (buf[i], 0);
      Serial.print(" ");
    }
    
    switch(Tbl_vidpid[i].joy_type) {
      case TYPE_PS4: // PS4
        // [[ Playstation 4 GENERIC Controller ]] -----------------------------------------------------------

        // Analog Stick
        if(byte(buf[1]) < AD_CNV_DATA1) isLEFT = true;  // L左
        if(byte(buf[1]) > AD_CNV_DATA2) isRIGHT = true; // L右
        isANALOG1 = buf[1];
        if(byte(buf[2]) < AD_CNV_DATA1) isUP = true;    // L上
        if(byte(buf[2]) > AD_CNV_DATA2) isDOWN = true;  // L下
        isANALOG2 = buf[2];

        if(byte(buf[3]) < AD_CNV_DATA1) isLEFT = true;  // R左
        if(byte(buf[3]) > AD_CNV_DATA2) isRIGHT = true; // R右
        isANALOG3 = buf[3];
        if(byte(buf[4]) < AD_CNV_DATA1) isUP = true;    // R上
        if(byte(buf[4]) > AD_CNV_DATA2) isDOWN = true;  // R下
        isANALOG4 = buf[4];
             
        // D-Pad
        switch(byte(buf[5])&B00001111) {
          case 0: isUP = true;            break;
          case 1: isUP = isRIGHT = true;  break;
          case 2: isRIGHT = true;         break;
          case 3: isRIGHT = isDOWN = true;break;
          case 4: isDOWN = true;          break;
          case 5: isDOWN = isLEFT = true; break;
          case 6: isLEFT = true;          break;
          case 7: isUP = isLEFT = true;   break;
        }

        // Botton
                                                                          // buf[0] neutral // 0000 1000b  08
        isSQUARE =  ( buf[5] & 0x10 );    if ( isSQUARE ) DBG("Y");       // Y .. square    // 0001 1000b  18
        isCROSS  =  ( buf[5] & 0x20 );    if ( isCROSS ) DBG("B");        // B .. cross     // 0010 1000b  28
        isCIRCLE =  ( buf[5] & 0x40 );    if ( isCIRCLE ) DBG("A");       // A .. circle    // 0100 1000b  48
        isTRI    =  ( buf[5] & 0x80 );    if ( isTRI ) DBG("X");          // X .. triangle  // 1000 1000b  88
        //----------------------------------------------------------------------------------------------------
                                                                          // buf[1] neutral // 0000 0000b  00
        isL1     =  ( buf[6] & 0x01 );    if ( isL1 ) DBG("L1");                            // 0000 0001b  01
        isR1     =  ( buf[6] & 0x02 );    if ( isR1 ) DBG("R1");                            // 0000 0010b  02
        isL2     =  ( buf[6] & 0x04 );    if ( isL2 ) DBG("L2");                            // 0000 0100b  04
        isR2     =  ( buf[6] & 0x08 );    if ( isR2 ) DBG("R2");                            // 0000 1000b  08
        isSELECT =  ( buf[6] & 0x10 );    if ( isSELECT ) DBG("Select");                    // 0001 0000b  10
        isSTART  =  ( buf[6] & 0x20 );    if ( isSTART )  DBG("Start");                     // 0010 0000b  20
        isL3     =  ( buf[6] & 0x40 );    if ( isL3 ) DBG("L3");                            // 0100 0000b  40
        isR3     =  ( buf[6] & 0x80 );    if ( isR3 ) DBG("R3");                            // 1000 0000b  80
        //----------------------------------------------------------------------------------------------------
        Input_MASTER(port_no, len, buf);
        break;
      case TYPE_MDmini: // MDmini
        Input_MDmini(port_no, len, buf);
        break;
      case TYPE_RAP3: // RAP (PS3mode)
        // [[ HORI Real Arcade Pro.V HAYABUSA(PS3 MODE) ------------------------------------------------------

        // アナログはとりあえずなし
        
        switch(byte(buf[2])&B00001111) {
          case 0: isUP = true;            break;
          case 1: isUP = isRIGHT = true;  break;
          case 2: isRIGHT = true;         break;
          case 3: isRIGHT = isDOWN = true;break;
          case 4: isDOWN = true;          break;
          case 5: isDOWN = isLEFT = true; break;
          case 6: isLEFT = true;          break;
          case 7: isUP = isLEFT = true;   break;
        }

        // Botton
                                                                          // buf[0] neutral // 0000 0000b  00
        isSQUARE =  ( buf[0] & 0x01 );    if ( isSQUARE ) DBG("Y");       // Y .. square    // 0000 0001b  01
        isCROSS  =  ( buf[0] & 0x02 );    if ( isCROSS ) DBG("B");        // B .. cross     // 0000 0010b  02
        isCIRCLE =  ( buf[0] & 0x04 );    if ( isCIRCLE ) DBG("A");       // A .. circle    // 0000 0100b  04
        isTRI    =  ( buf[0] & 0x08 );    if ( isTRI ) DBG("X");          // X .. triangle  // 0000 1000b  08
        isL1     =  ( buf[0] & 0x10 );    if ( isL1 ) DBG("L1");                            // 0001 0000b  10
        isR1     =  ( buf[0] & 0x20 );    if ( isR1 ) DBG("R1");                            // 0010 0000b  20
        isL2     =  ( buf[0] & 0x40 );    if ( isL2 ) DBG("L2");                            // 0100 0000b  40
        isR2     =  ( buf[0] & 0x80 );    if ( isR2 ) DBG("R2");                            // 1000 0000b  80
        //----------------------------------------------------------------------------------------------------
                                                                          // buf[1] neutral    0000 0000b  00
        isSELECT =  ( buf[1] & 0x01 );    if ( isSELECT ) DBG("Select");                    // 0000 0001b  01
        isSTART  =  ( buf[1] & 0x02 );    if ( isSTART )  DBG("Start");                     // 0000 0010b  02
        isL3     =  ( buf[1] & 0x04 );    if ( isL3 ) DBG("L3");                            // 0000 0100b  04
        isR3     =  ( buf[1] & 0x08 );    if ( isR3 ) DBG("R3");                            // 0000 1000b  08
        //----------------------------------------------------------------------------------------------------
        Input_MASTER(port_no, len, buf);
        break;

      case TYPE_PSC:
        // [[ Playstation Classic Controller Parser ----------------------------------------------------------
                                                                          // buf[0] neutral    0000 0000b  00
        isL2     =  ( buf[0] & 0x10 );    if ( isL2 ) DBG("L2");                            // 0001 0000b  10
        isR2     =  ( buf[0] & 0x20 );    if ( isR2 ) DBG("R2");                            // 0010 0000b  20
        isL1     =  ( buf[0] & 0x40 );    if ( isL1 ) DBG("L1");                            // 0100 0000b  40
        isR1     =  ( buf[0] & 0x80 );    if ( isR1 ) DBG("R1");                            // 1000 0000b  80
        isTRI    =  ( buf[0] & 0x01 );    if ( isTRI ) DBG("X");          // X .. triangle  // 0000 0001b  01
        isCIRCLE =  ( buf[0] & 0x02 );    if ( isCIRCLE ) DBG("A");       // A .. circle    // 0000 0010b  02
        isCROSS  =  ( buf[0] & 0x04 );    if ( isCROSS ) DBG("B");        // B .. cross     // 0000 0100b  04
        isSQUARE =  ( buf[0] & 0x08 );    if ( isSQUARE ) DBG("Y");       // Y .. square    // 0000 1000b  08
        //----------------------------------------------------------------------------------------------------
                                                                          // buf[1] neutral    0001 0100b  14
        isSELECT =  ( buf[1] & 0x01 );    if ( isSELECT ) DBG("Select");                    // 0001 0101b  15
        isSTART  =  ( buf[1] & 0x02 );    if ( isSTART )  DBG("Start");                     // 0001 0110b  16
        isLEFT   = !( buf[1] & (1<<2)) & !( buf[1] & (1<<3));   if ( isLEFT ) DBG("LT");    // 0001 0000b  10
        isRIGHT  =  ( buf[1] & (1<<3));   if ( isRIGHT ) DBG("RT");                         // 0001 1000b  18
        isUP     = !( buf[1] & (1<<4)) & !( buf[1] & (1<<5));   if ( isUP ) DBG("UP");      // 0000 0100b  04
        isDOWN   =  ( buf[1] & (1<<5));   if ( isDOWN ) DBG("DN");                          // 0010 0100b  24
        //----------------------------------------------------------------------------------------------------
        Input_MASTER(port_no, len, buf);    
        break;

      case TYPE_SNES:
        // [[ iBUFFALO SNES CLASSIC USB GAMEPAD Parser -------------------------------------------------------
                                                                          // buf[0] neutral // 1000 0000b  80
        isLEFT   =  ( buf[0] == 0x00 );   if ( isLEFT ) DBG("LT");                          // 0000 0000b  00
        isRIGHT  =  ( buf[0] == 0xFF );   if ( isRIGHT ) DBG("RT");                         // 1111 1111b  FF
                                                                          // buf[1] neutral // 1000 0000b  80
        isUP     =  ( buf[1] == 0x00 );   if ( isUP ) DBG("UP");                            // 0000 0000b  00
        isDOWN   =  ( buf[1] == 0xFF );   if ( isDOWN ) DBG("DN");                          // 1111 1111b  FF
                                                                          // buf[2] neutral // 0000 0000b  80
        isCIRCLE =  ( buf[2] & 0x01 );    if ( isCIRCLE ) DBG("A");       // A .. circle    // 0000 0001b  01
        isCROSS  =  ( buf[2] & 0x02 );    if ( isCROSS ) DBG("B");        // B .. cross     // 0000 0010b  02
        isTRI    =  ( buf[2] & 0x04 );    if ( isTRI ) DBG("X");          // X .. triangle  // 0000 0100b  04
        isSQUARE =  ( buf[2] & 0x08 );    if ( isSQUARE ) DBG("Y");       // Y .. square    // 0000 1000b  08
        isR1     =  ( buf[2] & 0x10 );    if ( isR1 ) DBG("R1");          // Z ..           // 0001 0000b  10
        isR2     =  ( buf[2] & 0x20 );    if ( isR2 ) DBG("R2");          // C ..           // 0010 0000b  20
//      isL1     =  ( buf[2] & 0x10 );    if ( isL1 ) DBG("L1");                            // 0001 0000b  10 //スト2のために逆にしてみた
//      isR1     =  ( buf[2] & 0x20 );    if ( isR1 ) DBG("R1");                            // 0010 0000b  20
        isSELECT =  ( buf[2] & 0x40 );    if ( isSELECT ) DBG("Select");                    // 0100 0000b  40
        isSTART  =  ( buf[2] & 0x80 );    if ( isSTART )  DBG("Start");                     // 1000 0000b  80
        //----------------------------------------------------------------------------------------------------
        Input_MASTER(port_no, len, buf);    
        break;
      default: // 標準
        Input_PS4(port_no, len, buf);
    }
    Serial.println("");
//  Input_MASTER(port_no, len, buf);    //最終的には個別起動はやめて、ここに集約する！
  }

//************************************************************************************************************
//************************************************************************************************************
//************************************************************************************************************
  void Input_PS4(int port_no, uint8_t len, uint8_t *buf)
  {
    uint8_t w_buf[32];
    int i;
    int d_pointer;
    byte d_work1;
    byte d_work2;

    if(buf[0] == 0x01) d_pointer = 0;
    else if(buf[0] == 0x11) {
      if (len < 4) return;
      d_pointer = 2;
    }
    else return;

    switch(cnv_mode) {
      case IF_MODE_ATARI:
      case IF_MODE_CPSF:
        if(port_no!=0) break;
        d_work1 = d_work2 = B11111111;

        if(byte(buf[d_pointer+1]) < AD_CNV_DATA1) d_work1 &= B11111011; // 左
        if(byte(buf[d_pointer+1]) > AD_CNV_DATA2) d_work1 &= B11110111; // 右
        if(byte(buf[d_pointer+2]) < AD_CNV_DATA1) d_work1 &= B11111110; // 上
        if(byte(buf[d_pointer+2]) > AD_CNV_DATA2) d_work1 &= B11111101; // 下

        if(cnv_mode == IF_MODE_ATARI) {
          if(byte(buf[d_pointer+3]) < AD_CNV_DATA1) d_work2 &= B11111011; // 左
          if(byte(buf[d_pointer+3]) > AD_CNV_DATA2) d_work2 &= B11110111; // 右
          if(byte(buf[d_pointer+4]) < AD_CNV_DATA1) d_work2 &= B11111110; // 上
          if(byte(buf[d_pointer+4]) > AD_CNV_DATA2) d_work2 &= B11111101; // 下
        }

        switch(byte(buf[d_pointer+5])&B00001111) {
          case 0: // 上
            d_work1 &= B11111110;
            break;
          case 1: // 上+右
            d_work1 &= B11110110;
            break;
          case 2: // 右
            d_work1 &= B11110111;
            break;
          case 3: // 下+右
            d_work1 &= B11110101;
            break;
          case 4: // 下
            d_work1 &= B11111101;
            break;
          case 5: // 下+左
            d_work1 &= B11111001;
            break;
          case 6: // 左
            d_work1 &= B11111011;
            break;
          case 7: // 上+左
            d_work1 &= B11111010;
            break;
        }

        if(buf[d_pointer+5]&0x0020) { // Aボタン // ☓ボタン
          d_work1 &= B11101111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11101111;
        }
        if(buf[d_pointer+5]&0x0040) { // Bボタン // ○ボタン
          d_work1 &= B11011111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11011111;
        }
        if(buf[d_pointer+6]&0x0008) { // Cボタン // R2ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11101111;
        }
/*
        if(buf[d_pointer+6]&0x0002) { // R1ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11101111;
        }
*/
        if(buf[d_pointer+5]&0x0010) { // Xボタン // □ボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11011111; // d_work1 &= B11101111;
            d_work2 &= B11011111; // d_work2 &= B11101111;
          }
          else d_work2 &= B11111011;
        }
        if(buf[d_pointer+5]&0x0080) { // Yボタン // △ボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11101111; // d_work1 &= B11011111;
            d_work2 &= B11101111; // d_work2 &= B11011111;
          }
          else d_work2 &= B11111101;
        }
        if(buf[d_pointer+6]&0x0002) { // Zボタン / R1ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11111110;
        }
/*
        if(buf[d_pointer+6]&0x0001) { // L1ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11111110;
        }
*/
        if((buf[d_pointer+6]&0x0020) || (buf[d_pointer+7]&0x0001)) { // STARTボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11011111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11110011;
        }
        if(buf[d_pointer+6]&0x0010) { // SELECTボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11110111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11111100;
        }
        cpsfatari_data1 = d_work1;
        cpsfatari_data2 = d_work2;
        break;

      case IF_MODE_CYBER:
        if(port_no!=0) break;
        for(i=0;i<12;i+=2) {
          cyber_data[i]=B11001111;
          cyber_data[i+1]=B11011111;
        }

        /* 左右アナログ */
        cyber_data[3] &= ((byte(buf[d_pointer+1])>>4)|B11110000); // 上位4ビット
        cyber_data[7] &= (byte(buf[d_pointer+1])|B11110000); // 下位4ビット

        /* 上下アナログ */
        cyber_data[2] &= ((byte(buf[d_pointer+2])>>4)|B11110000); // 上位4ビット
        cyber_data[6] &= (byte(buf[d_pointer+2])|B11110000); // 下位4ビット

        /* 左右アナログ */
        cyber_data[5] &= ((byte(buf[d_pointer+3])>>4)|B11110000); // 上位4ビット
        cyber_data[9] &= (byte(buf[d_pointer+3])|B11110000); // 下位4ビット

        /* 上下アナログ */
        cyber_data[4] &= ((byte(buf[d_pointer+4])>>4)|B11110000); // 上位4ビット
        cyber_data[8] &= (byte(buf[d_pointer+4])|B11110000); // 下位4ビット

        if(buf[d_pointer+5]&0x0020) { // Aボタン
          cyber_data[0]  &= B11110111;
          cyber_data[10] &= B11110111;
        }
        if(buf[d_pointer+5]&0x0040) { // Bボタン
          cyber_data[0]  &= B11111011;
          cyber_data[10] &= B11111011;
        }
        if(buf[d_pointer+6]&0x0002) // R1ボタン
          cyber_data[0] &= B11111101;
        if(buf[d_pointer+5]&0x0010) { // Xボタン
          cyber_data[0]  &= B11110111;
          cyber_data[10] &= B11111101;
        }
        if(buf[d_pointer+5]&0x0080) { // Yボタン
          cyber_data[0]  &= B11111011;
          cyber_data[10] &= B11111110;
        }
        if(buf[d_pointer+6]&0x0001) // L1ボタン
          cyber_data[0] &= B11111110;

        if((buf[d_pointer+6]&0x0020) || (buf[d_pointer+7]&0x0001)) // STARTボタン
          cyber_data[1] &= B11111101;
        if(buf[d_pointer+6]&0x0010) // SELECTボタン
          cyber_data[1] &= B11111110;
        break;

      case IF_MODE_JOYDRV:
        joydrv_snddata[port_no][5] = byte(buf[d_pointer+1]); // 左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[d_pointer+2]); // 上下アナログ
        joydrv_snddata[port_no][7] = byte(buf[d_pointer+3]); // 左右アナログ
        joydrv_snddata[port_no][6] = byte(buf[d_pointer+4]); // 上下アナログ
        /* アナログ変化なし */
        for(i=8;i<16;i++) joydrv_snddata[port_no][i] = 0;
        joydrv_snddata[port_no][8]  = byte(buf[d_pointer+8]); // 左アナログボタン
        joydrv_snddata[port_no][12] = byte(buf[d_pointer+9]); // 右アナログボタン

        joydrv_snddata[port_no][0] = joydrv_snddata[port_no][1] = joydrv_snddata[port_no][2] = joydrv_snddata[port_no][3] = B11111111;

        switch(byte(buf[d_pointer+5])&B00001111) {
          case 0: // 上
            joydrv_snddata[port_no][3] &= B11111110;
            break;
          case 1: // 上+右
            joydrv_snddata[port_no][3] &= B11110110;
            break;
          case 2: // 右
            joydrv_snddata[port_no][3] &= B11110111;
            break;
          case 3: // 下+右
            joydrv_snddata[port_no][3] &= B11110101;
            break;
          case 4: // 下
            joydrv_snddata[port_no][3] &= B11111101;
            break;
          case 5: // 下+左
            joydrv_snddata[port_no][3] &= B11111001;
            break;
          case 6: // 左
            joydrv_snddata[port_no][3] &= B11111011;
            break;
          case 7: // 上+左
            joydrv_snddata[port_no][3] &= B11111010;
            break;
        }

        if(buf[d_pointer+5]&0x0020) // Aボタン
          joydrv_snddata[port_no][2] &= B11111110;
        if(buf[d_pointer+5]&0x0040) // Bボタン
          joydrv_snddata[port_no][2] &= B11111101;
        if(buf[d_pointer+6]&0x0002) // R1ボタン
          joydrv_snddata[port_no][1] &= B11101111;
        if(buf[d_pointer+6]&0x0008) // R2ボタン
          joydrv_snddata[port_no][1] &= B11011111;
        if(buf[d_pointer+6]&0x0080) // R3ボタン
          joydrv_snddata[port_no][1] &= B10111111;
        if(buf[d_pointer+5]&0x0010) // Xボタン
          joydrv_snddata[port_no][2] &= B11101111;
        if(buf[d_pointer+5]&0x0080) // Yボタン
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

//************************************************************************************************
  void Input_MDmini(int port_no, uint8_t len, uint8_t *buf)
  {
    int i;
    byte d_work1;
    byte d_work2;

    switch(cnv_mode) {
      case IF_MODE_ATARI:
      case IF_MODE_CPSF:
        if(port_no!=0) break;
        d_work1 = d_work2 = B11111111;

        if(byte(buf[3]) < AD_CNV_DATA1) d_work1 &= B11111011; // 左
        if(byte(buf[3]) > AD_CNV_DATA2) d_work1 &= B11110111; // 右
        if(byte(buf[4]) < AD_CNV_DATA1) d_work1 &= B11111110; // 上
        if(byte(buf[4]) > AD_CNV_DATA2) d_work1 &= B11111101; // 下

        if(buf[5]&0x0040) { // Aボタン
          d_work1 &= B11101111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11101111;
        }
        if(buf[5]&0x0020) { // Bボタン
          d_work1 &= B11011111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11011111;
        }
        if(buf[6]&0x0002) { // Cボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11101111;
//        if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11111110;
        }
        if(buf[5]&0x0080) { // Xボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11101111;
            d_work2 &= B11101111;
          }
          else d_work2 &= B11111011;
        }
        if(buf[5]&0x0010) { // Yボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11011111;
            d_work2 &= B11011111;
          }
          else d_work2 &= B11111101;
        }
        if(buf[6]&0x0001) { // Zボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11111110;
//        if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11101111;
        }

        if(buf[6]&0x0020) { // STARTボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11011111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11110011;
        }
        if(buf[6]&0x0010) { // SELECT(MODE)ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11110111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11111100;
        }
        cpsfatari_data1 = d_work1;
        cpsfatari_data2 = d_work2;
        break;

      case IF_MODE_CYBER:
        if(port_no!=0) break;
        for(i=0;i<12;i+=2) {
          cyber_data[i]=B11001111;
          cyber_data[i+1]=B11011111;
        }

        /* 左右アナログ */
        cyber_data[3] &= ((byte(buf[3])>>4)|B11110000); // 上位4ビット
        cyber_data[7] &= (byte(buf[3])|B11110000); // 下位4ビット

        /* 上下アナログ */
        cyber_data[2] &= ((byte(buf[4])>>4)|B11110000); // 上位4ビット
        cyber_data[6] &= (byte(buf[4])|B11110000); // 下位4ビット

        /* アナログ変化なし */
        cyber_data[5] &= B11111000; // 上位4ビット
        cyber_data[9] &= B11110000; // 下位4ビット
        cyber_data[4] &= B11111000; // 上位4ビット
        cyber_data[8] &= B11110000; // 下位4ビット

        if(buf[5]&0x0040) { // Aボタン
          cyber_data[0]  &= B11110111;
          cyber_data[10] &= B11110111;
        }
        if(buf[5]&0x0020) { // Bボタン
          cyber_data[0]  &= B11111011;
          cyber_data[10] &= B11111011;
        }
        if(buf[6]&0x0002) // Cボタン
          cyber_data[0] &= B11111101;
        if(buf[5]&0x0080) { // Xボタン
          cyber_data[0]  &= B11110111;
          cyber_data[10] &= B11111101;
        }
        if(buf[5]&0x0010) { // Yボタン
          cyber_data[0]  &= B11111011;
          cyber_data[10] &= B11111110;
        }
        if(buf[6]&0x0001) // Zボタン
          cyber_data[0] &= B11111110;

        if(buf[6]&0x0020) // STARTボタン
          cyber_data[1] &= B11111101;
        if(buf[6]&0x0010) // SELECT(MODE)ボタン
          cyber_data[1] &= B11111110;
        break;

      case IF_MODE_JOYDRV:
        joydrv_snddata[port_no][5] = byte(buf[3]); // 左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[4]); // 上下アナログ
        /* アナログ変化なし */
        joydrv_snddata[port_no][7] = joydrv_snddata[port_no][6] = B10000000;
        for(i=8;i<16;i++) joydrv_snddata[port_no][i] = 0;

        joydrv_snddata[port_no][0] = joydrv_snddata[port_no][1] = joydrv_snddata[port_no][2] = joydrv_snddata[port_no][3] = B11111111;
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
    }
  }

  
/*******************************************************************
* Input_MASTER
* https://gamesx.com/wiki/doku.php?id=x68000:joystick_regs
*===================================================================
* LAYOUT
*    □  △  R1  L1      SEL  STA
*    ☓  ○  R2  L2      L3   R3
*-------------------------------------------------------------------
* 0:ATARI mode
*    B   A   NA  NA      SEL  STA
*    A   B   NA  NA      NA   NA
*-------------------------------------------------------------------
* 1:CPSF mode
*    X   Y   Z   NA      SEL  STA
*    A   B   C   NA      NA   NA
*-------------------------------------------------------------------
*
********************************************************************/
  void Input_MASTER(int port_no, uint8_t len, uint8_t *buf)
  {
    int i;
    byte d_work1;
    byte d_work2;

    switch(cnv_mode) {
      case IF_MODE_ATARI:
      case IF_MODE_CPSF:
        if(port_no!=0) break;
        d_work1 = d_work2 = B11111111;

        if (isUP)     { d_work1 &= B11111110; }
        if (isDOWN)   { d_work1 &= B11111101; }
        if (isLEFT)   { d_work1 &= B11111011; }
        if (isRIGHT)  { d_work1 &= B11110111; }

        if (isCROSS) {
          d_work1 &= B11101111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11101111;
        }
        if (isCIRCLE) {
          d_work1 &= B11011111;
          if(cnv_mode == IF_MODE_ATARI) d_work2 &= B11011111;
        }
        if (isR2) { // Cボタン // R2ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11101111;
        }
         
        if(isSQUARE) { // Xボタン // □ボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11011111; // d_work1 &= B11101111;
            d_work2 &= B11011111; // d_work2 &= B11101111;
          }
          else d_work2 &= B11111011;
        }
        if(isTRI) { // Yボタン // △ボタン
          if(cnv_mode == IF_MODE_ATARI) {
            d_work1 &= B11101111; // d_work1 &= B11011111;
            d_work2 &= B11101111; // d_work2 &= B11011111;
          }
          else d_work2 &= B11111101;
        }
        if(isR1) { // Zボタン // R1ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11111110;
        }
        if(isSELECT) { // SELECT(MODE)ボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11110111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11111100;
        }
        if(isSTART) { // STARTボタン
          if(cnv_mode == IF_MODE_CPSF) d_work2 &= B11011111;
          else if(cnv_mode == IF_MODE_ATARI) d_work1 &= B11110011;
        }
        cpsfatari_data1 = d_work1;
        cpsfatari_data2 = d_work2;
        break;

//-----------------------------------------------------------------------------------------
      case IF_MODE_CYBER:
        if(port_no!=0) break;
        for(i=0;i<12;i+=2) {
          cyber_data[i]=B11001111;
          cyber_data[i+1]=B11011111;
        }

//***********************************************************************
        /* 左右アナログ */
//        cyber_data[3] &= ((byte(buf[3])>>4)|B11110000); // 上位4ビット
//        cyber_data[7] &= (byte(buf[3])|B11110000); // 下位4ビット

        /* 上下アナログ */
//        cyber_data[2] &= ((byte(buf[4])>>4)|B11110000); // 上位4ビット
//        cyber_data[6] &= (byte(buf[4])|B11110000); // 下位4ビット
//************************************************************************

        /* アナログ変化なし */
        cyber_data[5] &= B11111000; // 上位4ビット
        cyber_data[9] &= B11110000; // 下位4ビット
        cyber_data[4] &= B11111000; // 上位4ビット
        cyber_data[8] &= B11110000; // 下位4ビット

        if(isCROSS) {cyber_data[0]  &= B11110111;
                     cyber_data[10] &= B11110111;}  // Aボタン
        if(isCIRCLE) {cyber_data[0]  &= B11111011;
                      cyber_data[10] &= B11111011;} // Bボタン
        if(isR2)      cyber_data[0] &= B11111101;   // Cボタン

        if(isTRI) {   cyber_data[0]  &= B11110111;
                      cyber_data[10] &= B11111101;} // Xボタン
        if(isTRI) {   cyber_data[0]  &= B11111011;
                      cyber_data[10] &= B11111110;} // Yボタン
        if(isR1)      cyber_data[0] &= B11111110;   // Zボタン

        if(isSTART)   cyber_data[1] &= B11111101;   // STARTボタン
        if(isSELECT)  cyber_data[1] &= B11111110;   // SELECT(MODE)ボタン
        break;

//-----------------------------------------------------------------------------------------
      case IF_MODE_JOYDRV:
        joydrv_snddata[port_no][5] = byte(buf[3]); // 左右アナログ
        joydrv_snddata[port_no][4] = byte(buf[4]); // 上下アナログ
        /* アナログ変化なし */
        joydrv_snddata[port_no][7] = joydrv_snddata[port_no][6] = B10000000;
        for(i=8;i<16;i++) joydrv_snddata[port_no][i] = 0;

        joydrv_snddata[port_no][0] = joydrv_snddata[port_no][1] = joydrv_snddata[port_no][2] = joydrv_snddata[port_no][3] = B11111111;
        if(isCROSS)  joydrv_snddata[port_no][2] &= B11111110; // Aボタン ☓ボタン
        if(isCIRCLE) joydrv_snddata[port_no][2] &= B11111101; // Bボタン ○ボタン
        if(isR2)     joydrv_snddata[port_no][2] &= B11111011; // Cボタン R2ボタン
        if(isSQUARE) joydrv_snddata[port_no][2] &= B11101111; // Xボタン □ボタン
        if(isTRI)    joydrv_snddata[port_no][2] &= B11011111; // Yボタン △ボタン
        if(isR1)     joydrv_snddata[port_no][2] &= B10111111; // Zボタン R1ボタン
        if(isSTART)  joydrv_snddata[port_no][0] &= B11111110; // STARTボタン
        if(isSELECT) joydrv_snddata[port_no][0] &= B11111101; // SELECT(MODE)ボタン

        if(stick_ctrldata[port_no].flg_change)
          stick_ctrldata[port_no].flg_change = false;
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

  cpsfatari_data1 = cpsfatari_data2 = B11111111;
  for(i=0;i<12;i+=2) {
    cyber_data[i]=B11001111;
    cyber_data[i+1]=B11011111;
  }

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
  Serial.print(F("\r\nUSB controller(PS4,MD mini,RAP,etc.) -> ATARI ver 2.00\r\n"));
  cyber_if_flg = 0; /* 起動時割込み誤検知防止 */
  joydrv_if_flg = 0; /* 起動時割込み誤検知防止 */
}

void loop() {
  int joydrv_port;
  int motor1;
  int motor2;
  int i;

  Usb.Task();

  if (cyber_if_flg == 1) {
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
  else if (cnv_mode == IF_MODE_ATARI || cnv_mode == IF_MODE_CPSF) {
    PORTC = ((PIND&B00000100) ? cpsfatari_data2 : cpsfatari_data1);
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
  PORTC = ((PIND&B00000100) ? cpsfatari_data2 : cpsfatari_data1);
}

void int_cyber()
{
  cyber_if_flg = 1;
}

void int_joydrv()
{
  joydrv_if_flg = 1;
}
