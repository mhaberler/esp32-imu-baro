#pragma once


// BLE: Name: TY, Address: dc:23:4d:59:a9:fa, 
// manufacturer data: d007 80 030 0 00 01 00a44ac8a1ee80a0db7041bcd35052e90b
// serviceUUID: 0xa201


#define MFID_X20V 0x07D0
#define X20V_SERVICE_UUID ((uint16_t)0xa201)

// decoded
typedef struct {
  uint8_t unknown[22];
} x20v_t;

// wire format
typedef struct {
  uint8_t unknown[22];
} x20v_raw_t;
