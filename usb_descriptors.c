#include <string.h>

#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/unique_id.h"

#if CFG_AUDIO_DEBUG
#include "common_types.h"
#endif

/* Unique PID for SX1280 QO-100 SSB TX project */
#define USB_PID  0x4073  // 'QO' in hex (0x51 0x4F -> 0x4073 unique)

//--------------------------------------------------------------------+
// Device Descriptor
//--------------------------------------------------------------------+
static tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // IAD dla kompozytu
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0101,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

#if CFG_AUDIO_DEBUG
//--------------------------------------------------------------------+
// HID Report Descriptor (opcjonalnie)
//--------------------------------------------------------------------+
uint8_t const desc_hid_report[] = {
  HID_USAGE_PAGE_N ( HID_USAGE_PAGE_VENDOR, 2   ),
  HID_USAGE        ( 0x01                       ),
  HID_COLLECTION   ( HID_COLLECTION_APPLICATION ),
    HID_USAGE        ( 0x02 ),
    HID_LOGICAL_MIN  ( 0x00 ),
    HID_LOGICAL_MAX_N( 0xff, 2 ),
    HID_REPORT_SIZE  ( 8 ),
    HID_REPORT_COUNT ( sizeof(audio_debug_info_t) ),
    HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ),
  HID_COLLECTION_END
};

uint8_t const * tud_hid_descriptor_report_cb(uint8_t itf)
{
  (void) itf;
  return desc_hid_report;
}
#endif

#if CFG_TUSB_MCU == OPT_MCU_LPC175X_6X || CFG_TUSB_MCU == OPT_MCU_LPC177X_8X || CFG_TUSB_MCU == OPT_MCU_LPC40XX
  #define EPNUM_AUDIO         0x03
  #define EPNUM_AUDIO_FB      0x03

  #define EPNUM_CDC_NOTIF     0x81
  #define EPNUM_CDC_OUT       0x02
  #define EPNUM_CDC_IN        0x82

  #define EPNUM_DEBUG         0x04

#elif TU_CHECK_MCU(OPT_MCU_NRF5X)
  #define EPNUM_AUDIO         0x08
  #define EPNUM_AUDIO_FB      0x08

  #define EPNUM_CDC_NOTIF     0x81
  #define EPNUM_CDC_OUT       0x02
  #define EPNUM_CDC_IN        0x82

  #define EPNUM_DEBUG         0x04

#elif defined(TUD_ENDPOINT_ONE_DIRECTION_ONLY)
  // konserwatywnie: unikamy współdzielenia numerów
  #define EPNUM_AUDIO         0x02
  #define EPNUM_AUDIO_FB      0x01

  #define EPNUM_CDC_NOTIF     0x83
  #define EPNUM_CDC_OUT       0x04
  #define EPNUM_CDC_IN        0x84

  #define EPNUM_DEBUG         0x05

#else
  #define EPNUM_AUDIO         0x01
  #define EPNUM_AUDIO_FB      0x01

  #define EPNUM_CDC_OUT       0x02
  #define EPNUM_CDC_IN        0x82
  #define EPNUM_CDC_NOTIF     0x83

  #define EPNUM_DEBUG         0x84
#endif

//--------------------------------------------------------------------+
// Configuration Descriptor (UAC1 + CDC (+ optional HID))
//--------------------------------------------------------------------+

#define EPNUM_AUDIO_CAPTURE 0x86u
#define AUDIO_CONTROL_TERMINALS_LEN (2 * TUD_AUDIO10_DESC_INPUT_TERM_LEN + \
                                    2 * TUD_AUDIO10_DESC_OUTPUT_TERM_LEN + \
                                    TUD_AUDIO10_DESC_FEATURE_UNIT_LEN(2))
#define AUDIO_STREAM_LEN (2 * TUD_AUDIO10_DESC_STD_AS_LEN + TUD_AUDIO10_DESC_CS_AS_INT_LEN + \
                          TUD_AUDIO10_DESC_TYPE_I_FORMAT_LEN(1) + \
                          TUD_AUDIO10_DESC_STD_AS_ISO_EP_LEN + TUD_AUDIO10_DESC_CS_AS_ISO_EP_LEN)
#define AUDIO_DUPLEX_LEN (8 + TUD_AUDIO10_DESC_STD_AC_LEN + TUD_AUDIO10_DESC_CS_AC_LEN(2) + \
                          AUDIO_CONTROL_TERMINALS_LEN + 2 * AUDIO_STREAM_LEN)

#if CFG_AUDIO_DEBUG
  #define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + AUDIO_DUPLEX_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)
#else
  #define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + AUDIO_DUPLEX_LEN + TUD_CDC_DESC_LEN)
#endif

uint8_t const desc_configuration[] =
{
  // Config: 1, interface count, string index, total length, attribute, power(mA)
  // Attribute: 0x80 = bus powered
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x80, 100),

  // One UAC1 function with independent 48 kHz playback and capture streams.
  8, TUSB_DESC_INTERFACE_ASSOCIATION, ITF_NUM_AUDIO_CONTROL, 3, TUSB_CLASS_AUDIO, 0, 0, 5,
  TUD_AUDIO10_DESC_STD_AC(ITF_NUM_AUDIO_CONTROL, 0, 5),
  TUD_AUDIO10_DESC_CS_AC(0x0100, AUDIO_CONTROL_TERMINALS_LEN,
                        ITF_NUM_AUDIO_STREAMING, ITF_NUM_AUDIO_CAPTURE),
  TUD_AUDIO10_DESC_INPUT_TERM(UAC1_ENTITY_INPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, 0, 2,
                              AUDIO10_CHANNEL_CONFIG_LEFT_FRONT | AUDIO10_CHANNEL_CONFIG_RIGHT_FRONT, 0, 0),
  TUD_AUDIO10_DESC_FEATURE_UNIT(UAC1_ENTITY_FEATURE_UNIT, UAC1_ENTITY_INPUT_TERMINAL, 0,
                               AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME,
                               AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME,
                               AUDIO10_FU_CONTROL_BM_MUTE | AUDIO10_FU_CONTROL_BM_VOLUME),
  TUD_AUDIO10_DESC_OUTPUT_TERM(UAC1_ENTITY_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER,
                               0, UAC1_ENTITY_FEATURE_UNIT, 0),
  TUD_AUDIO10_DESC_INPUT_TERM(UAC1_ENTITY_RX_INPUT_TERMINAL, AUDIO_TERM_TYPE_IN_GENERIC_MIC,
                              0, 1, AUDIO10_CHANNEL_CONFIG_NON_PREDEFINED, 0, 6),
  TUD_AUDIO10_DESC_OUTPUT_TERM(UAC1_ENTITY_RX_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING,
                               0, UAC1_ENTITY_RX_INPUT_TERMINAL, 0),

  TUD_AUDIO10_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 0, 0, 0),
  TUD_AUDIO10_DESC_STD_AS_INT(ITF_NUM_AUDIO_STREAMING, 1, 1, 0),
  TUD_AUDIO10_DESC_CS_AS_INT(UAC1_ENTITY_INPUT_TERMINAL, 0, AUDIO10_DATA_FORMAT_TYPE_I_PCM),
  TUD_AUDIO10_DESC_TYPE_I_FORMAT(2, CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX,
                                 CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX, 48000),
  TUD_AUDIO10_DESC_STD_AS_ISO_EP(EPNUM_AUDIO, TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_SYNCHRONOUS,
                                CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_FS, 1, 0),
  TUD_AUDIO10_DESC_CS_AS_ISO_EP(0, AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0),

  TUD_AUDIO10_DESC_STD_AS_INT(ITF_NUM_AUDIO_CAPTURE, 0, 0, 6),
  TUD_AUDIO10_DESC_STD_AS_INT(ITF_NUM_AUDIO_CAPTURE, 1, 1, 6),
  TUD_AUDIO10_DESC_CS_AS_INT(UAC1_ENTITY_RX_OUTPUT_TERMINAL, 0, AUDIO10_DATA_FORMAT_TYPE_I_PCM),
  TUD_AUDIO10_DESC_TYPE_I_FORMAT(1, 2, 16, USB_RX_AUDIO_RATE),
  TUD_AUDIO10_DESC_STD_AS_ISO_EP(EPNUM_AUDIO_CAPTURE,
                                TUSB_XFER_ISOCHRONOUS | TUSB_ISO_EP_ATT_SYNCHRONOUS,
                                CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX, 1, 0),
  TUD_AUDIO10_DESC_CS_AS_ISO_EP(0, AUDIO10_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_UNDEFINED, 0),

  // ---- CDC ACM (Serial) ----
  // stridx = 4 (patrz stringi poniżej)
  TUD_CDC_DESCRIPTOR(
    ITF_NUM_CDC_COMM,
    4,
    EPNUM_CDC_NOTIF, 8,
    EPNUM_CDC_OUT,
    EPNUM_CDC_IN, 64
  ),

#if CFG_AUDIO_DEBUG
  // ---- Optional HID debug ----
  TUD_HID_DESCRIPTOR(ITF_NUM_DEBUG, 0, HID_ITF_PROTOCOL_NONE,
                     sizeof(desc_hid_report), (uint8_t)EPNUM_DEBUG,
                     CFG_TUD_HID_EP_BUFSIZE, 7)
#endif
};

TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN, "Incorrect size");

uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index;
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC,       // 4
  STRID_UAC1,      // 5
  STRID_RX_AUDIO,  // 6
};

static char const *string_desc_arr[] =
{
  (const char[]) { 0x09, 0x04 },        // 0: English (0x0409)
  "SP8ESA",                             // 1: Manufacturer
  "SX1280 QO-100 SSB TX",               // 2: Product
  NULL,                                 // 3: Serial (generated)
  "TX Console",                     // 4: CDC interface string
  "QO-100 USB Audio",                   // 5: UAC1 function
  "Si4732 RX Audio",                    // 6: receive audio capture
};

static uint16_t _desc_str[32 + 1];

uint16_t const * tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;
  size_t chr_count;

  switch ( index )
  {
    case STRID_LANGID:
      memcpy(&_desc_str[1], string_desc_arr[0], 2);
      chr_count = 1;
      break;

    case STRID_SERIAL:
      {
        // Use Pico's unique board ID for truly unique serial number
        char serial_str[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
        pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
        chr_count = strlen(serial_str);
        if (chr_count > 31) chr_count = 31;
        for (size_t i = 0; i < chr_count; i++) {
          _desc_str[1 + i] = serial_str[i];
        }
      }
      break;

    default:
    {
      if ( !(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) ) return NULL;

      const char *str = string_desc_arr[index];
      if (!str) return NULL;

      chr_count = strlen(str);

      size_t const max_count = sizeof(_desc_str) / sizeof(_desc_str[0]) - 1;
      if ( chr_count > max_count ) chr_count = max_count;

      for ( size_t i = 0; i < chr_count; i++ ) _desc_str[1 + i] = str[i];
    }
    break;
  }

  _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
  return _desc_str;
}
