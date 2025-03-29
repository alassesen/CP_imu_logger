/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include "ble.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

LOG_MODULE_REGISTER(ble);
extern int requested_state;
extern char filename[];

/* Button value. */
static uint16_t but_val;

/* Prototype */
static ssize_t recv(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                    const void *buf, uint16_t len, uint16_t offset,
                    uint8_t flags);

static ssize_t file_recv(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags);

/* ST Custom Service  */
static const struct bt_uuid_128 st_service_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0000fe40, 0xcc7a, 0x482a, 0x984a, 0x7f2ed5b3e58f));

/* ST LED service */
static const struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0000fe41, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

static const struct bt_uuid_128 file_char_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0000fe43, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

/* ST Notify button service */
static const struct bt_uuid_128 but_notif_uuid = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x0000fe42, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static uint8_t manuf_data[ADV_LEN] = {
    0x01 /*SKD version */,
    0x83 /* STM32WB - P2P Server 1 */,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)};

/* BLE connection */
struct bt_conn *ble_conn;
/* Notification state */
volatile bool notify_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                uint16_t value) {
  ARG_UNUSED(attr);
  notify_enable = (value == BT_GATT_CCC_NOTIFY);
  LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
}

/* The embedded board is acting as GATT server.
 * The ST BLE Android app is the BLE GATT client.
 */

/* ST BLE Sensor GATT services and characteristic */

BT_GATT_SERVICE_DEFINE(
    stsensor_svc, BT_GATT_PRIMARY_SERVICE(&st_service_uuid),
    BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, recv, (void *)1),
    BT_GATT_CHARACTERISTIC(&file_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, file_recv, (void *)1),
    BT_GATT_CHARACTERISTIC(&but_notif_uuid.uuid, BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, NULL, NULL, &but_val),
    BT_GATT_CCC(mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static ssize_t recv(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                    const void *buf, uint16_t len, uint16_t offset,
                    uint8_t flags) {

  int recieved_data = *(int *)buf;

  LOG_INF("Bluetooth Recieved integer: %d", recieved_data);

  LOG_INF("Bluetooth Recieved: %s", recieved_data == 1   ? "Idle state"
                                    : recieved_data == 2 ? "Starting Logging"
                                    : recieved_data == 3 ? "Stop Logging"
                                                         : "Unknown result");

  requested_state = recieved_data;

  return 0;
}

static ssize_t file_recv(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len, uint16_t offset,
                         uint8_t flags) {

  char *recieved_data = (char *)buf;

  LOG_INF("Bluetooth Recieved filename: %s", recieved_data);
  strncpy(filename, recieved_data, MAX_FILENAME_SIZE);
  filename[MAX_FILENAME_SIZE - 1] = '\0';
  return 0;
}

void bt_ready(int err) {
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return;
  }
  LOG_INF("Bluetooth initialized");
  /* Start advertising */
  err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
  if (err) {
    LOG_ERR("Advertising failed to start (err %d)", err);
    return;
  }

  LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, uint8_t err) {
  if (err) {
    LOG_ERR("Connection failed (err %u)", err);
  } else {
    LOG_INF("Connected");
    if (!ble_conn) {
      ble_conn = bt_conn_ref(connected);
    }
  }
}

static void disconnected(struct bt_conn *disconn, uint8_t reason) {
  if (ble_conn) {
    bt_conn_unref(ble_conn);
    ble_conn = NULL;

    int err =
        bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
      LOG_ERR("Advertising failed to start (err %d)", err);
      return;
    }

    LOG_INF("Configuration mode: waiting connections...");
  }

  LOG_INF("Disconnected, reason %u %s", reason, bt_hci_err_to_str(reason));
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};
