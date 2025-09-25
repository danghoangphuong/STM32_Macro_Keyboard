#include "hid_report_desc.h"

__ALIGN_BEGIN uint8_t HID_ReportDesc_FS[] __ALIGN_END =
{
// ================== Keyboard ==================
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,        // USAGE (Keyboard)
    0xA1, 0x01,        // COLLECTION (Application)
    0x85, 0x01,        //   REPORT_ID (1)

    // Modifier keys (8 bits)
    0x05, 0x07,        //   USAGE_PAGE (Keyboard)
    0x19, 0xE0,        //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xE7,        //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)

    // Reserved
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x81, 0x03,        //   INPUT (Cnst,Var,Abs)

    // LED output (Num Lock, Caps Lock, Scroll Lock, Compose, Kana)
    0x95, 0x05,        //   REPORT_COUNT (5)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x05, 0x08,        //   USAGE_PAGE (LEDs)
    0x19, 0x01,        //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,        //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,        //   OUTPUT (Data,Var,Abs)

    // LED padding
    0x95, 0x01,        //   REPORT_COUNT (1)
    0x75, 0x03,        //   REPORT_SIZE (3)
    0x91, 0x03,        //   OUTPUT (Cnst,Var,Abs)

    // Keycodes (6 bytes)
    0x95, 0x06,        //   REPORT_COUNT (6)
    0x75, 0x08,        //   REPORT_SIZE (8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x65,        //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,        //   USAGE_PAGE (Keyboard)
    0x19, 0x00,        //   USAGE_MINIMUM (Reserved)
    0x29, 0x65,        //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,        //   INPUT (Data,Ary,Abs)

    0xC0,              // END_COLLECTION

	// ================== Consumer Control (Volume + Mute + Media) ==================
	0x05, 0x0C,        // USAGE_PAGE (Consumer Devices)
	0x09, 0x01,        // USAGE (Consumer Control)
	0xA1, 0x01,        // COLLECTION (Application)
	0x85, 0x02,        //   REPORT_ID (2)

	0x15, 0x00,        //   LOGICAL_MINIMUM (0)
	0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,        //   REPORT_SIZE (1 bit per control)

	// -------- Bit 0 --------
	0x95, 0x01,        //   REPORT_COUNT (1)
	0x09, 0xE9,        //   USAGE (Volume Up)
	0x81, 0x02,        //   INPUT (Data,Var,Abs)

	// -------- Bit 1 --------
	0x95, 0x01,        
	0x09, 0xEA,        //   USAGE (Volume Down)
	0x81, 0x02,        

	// -------- Bit 2 --------
	0x95, 0x01,        
	0x09, 0xE2,        //   USAGE (Mute)
	0x81, 0x02,        

	// -------- Bit 3 --------
	0x95, 0x01,        
	0x09, 0xCD,        //   USAGE (Play/Pause)
	0x81, 0x02,        

	// -------- Bit 4 --------
	0x95, 0x01,        
	0x09, 0xB5,        //   USAGE (Scan Next Track)
	0x81, 0x02,        

	// -------- Bit 5 --------
	0x95, 0x01,        
	0x09, 0xB6,        //   USAGE (Scan Previous Track)
	0x81, 0x02, 
	// -------- Padding --------
	0x95, 0x02,        //   REPORT_COUNT (3) -> padding bits (bit6..bit7)
	0x81, 0x03,        //   INPUT (Const,Var,Abs)

	0xC0              // END_COLLECTION
};

const uint16_t HID_ReportDesc_FS_Size = sizeof(HID_ReportDesc_FS);

