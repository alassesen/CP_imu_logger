/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mass_storage_device.h"
#include <ff.h>
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>

LOG_MODULE_REGISTER(mass_storage);

#define FS_RET_OK FR_OK
#define STORAGE_PARTITION storage_partition
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(STORAGE_PARTITION)

USBD_DEFINE_MSC_LUN(nand, "NAND", "Zephyr", "FlashDisk", "0.00");

static FATFS fat_fs = {
    .fsize = 4096, .csize = 8, .n_fats = 1, .n_rootdir = 512};
struct fs_mount_t fs_mnt = {
    .type = FS_FATFS,
    .fs_data = &fat_fs,
    // .flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
    .mnt_point = DISK_MOUNT_PT,
};

static int setup_flash(struct fs_mount_t *mnt) {
  int rc = 0;
  unsigned int id;
  const struct flash_area *pfa;

  mnt->storage_dev = (void *)STORAGE_PARTITION_ID;
  id = STORAGE_PARTITION_ID;
  rc = flash_area_open(id, &pfa);
  printk("Area %u at 0x%x on %s for %u bytes\n", id, (unsigned int)pfa->fa_off,
         pfa->fa_dev->name, (unsigned int)pfa->fa_size);

  if (rc < 0 && IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
    printk("Erasing flash area ... ");
    rc = flash_area_flatten(pfa, 0, pfa->fa_size);
    printk("%d\n", rc);
  }

  if (rc < 0) {
    flash_area_close(pfa);
  }
  return rc;
}

int setup_disk(void) {
  struct fs_mount_t *mp = &fs_mnt;
  int rc;

  rc = setup_flash(mp);
  if (rc < 0) {
    LOG_ERR("Failed to setup flash area");
    return -1;
  }

  return 0;
}
