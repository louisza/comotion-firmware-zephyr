/*
 * ═══════════════════════════════════════════════════════════════
 * CoMotion Tracker — BLE (Dual Advertising + NUS)
 * ═══════════════════════════════════════════════════════════════
 */
#ifndef COMOTION_BLE_H
#define COMOTION_BLE_H

#include "common.h"

int  ble_init(void);
void ble_update_advertising(void);
void ble_send(const char *msg);
void ble_send_raw(const uint8_t *data, uint16_t len);
void ble_set_impact_flag(void);
bool ble_is_connected(void);

#endif
