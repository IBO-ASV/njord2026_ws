#pragma once

// Set to 1 on the Ground-PC USB adapter and 0 on the vessel/miniPC adapter.
#define CRITICAL_LINK_ROLE_GROUND 1

#define CRITICAL_LINK_ESPNOW_CHANNEL 1
#define CRITICAL_LINK_SERIAL_BAUD 921600
#define CRITICAL_LINK_RETRY_INTERVAL_MS 15
#define CRITICAL_LINK_MAX_ATTEMPTS 3

// MAC address of the ESP32 at the opposite end.
static constexpr uint8_t CRITICAL_LINK_PEER_MAC[6] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Replace both keys before deployment. Each value must contain exactly 16 bytes.
static constexpr uint8_t CRITICAL_LINK_PMK[16] = {
  'c', 'h', 'a', 'n', 'g', 'e', '-', 'p', 'm', 'k', '-', '1', '2', '3', '4', '5'
};
static constexpr uint8_t CRITICAL_LINK_LMK[16] = {
  'c', 'h', 'a', 'n', 'g', 'e', '-', 'l', 'm', 'k', '-', '1', '2', '3', '4', '5'
};
