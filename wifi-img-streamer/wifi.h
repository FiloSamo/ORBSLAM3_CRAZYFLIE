#pragma once

#include <stdint.h>

typedef enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,

  WIFI_POSITION_SENDED             = 0x40, // Position data sent
} __attribute__((packed)) WiFiCTRLType;

typedef struct {
  WiFiCTRLType cmd;
  uint8_t data[50];
} __attribute__((packed)) WiFiCTRLPacket_t;