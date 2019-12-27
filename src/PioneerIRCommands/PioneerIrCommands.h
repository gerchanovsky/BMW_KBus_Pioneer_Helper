#include "ESP32Rmt.h" //https://github.com/ExploreEmbedded/ESP32_RMT
static ESP32Rmt ir_remote(PIN_IR);

// Pioneer CXE5116 IR Remote
typedef enum : public uint8_t {
  PIO_VOLUME_DN   = 0xF40B,
  PIO_VOLUME_UP   = 0xF50A,
  PIO_MODE        = 0xC837,//"Mode" > "Camera"
  PIO_SRC         = 0xE51A,//2 sec OFF
  PIO_MUTE        = 0xCF30,//0x0CF3
  PIO_MENU        = 0x926D,
  PIO_TOP_MENU    = 0x936C,

  PIO_ENTER       = 0x8679,//Select
  PIO_LEFT        = 0x8778,//0x42BD
  PIO_RIGHT       = 0x8877,//0x43BC
  PIO_UP          = 0x8A75,//0x40BF
  PIO_DOWN        = 0x8976,//0x41BE

  PIO_AUDIO       = 0x837C,
  PIO_SUBTITLE    = 0x847B,
  PIO_ANGLE       = 0x827D,
  PIO_RETURN      = 0x857A,
  PIO_STOP        = 0x8B74,
  PIO_REW         = 0x13EC,
  PIO_PLAY_PAUSE  = 0x8C73,
  PIO_FF          = 0x14EB,

  PIO_BAND        = 0xED12,//Escape
  PIO_PREV        = 0x8D72,
  PIO_NEXT        = 0x8E71,
  PIO_FOLDER_UP   = 0xBF40,
  PIO_FOLDER_DOWN = 0xBE41,

  // functions from https://www.bmwgm5.com/bmwgm5/IBUS.htm
  // Pioneer Premier DEH-P670MP head with CXC3173 wireless IR remote
  PIO_Function    = 0xE619,
  PIO_Audio       = 0xF20D,
  PIO_CD          = 0xE11E,
  PIO_Pause       = 0xA758,
  PIO_Tuner       = 0xE31C,
} PIO_IR_COMMANDS;

static void ir_command(uint16_t code)
{
  ir_remote.necSend(0x52AD, code);
}
