#include <sample_usbd.h>
#include <stdio.h>

#include <ff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

#include <zephyr/storage/flash_map.h>

#define DISK_DRIVE_NAME "NAND"
#define DISK_MOUNT_PT "/" DISK_DRIVE_NAME ":"

extern struct fs_mount_t fs_mnt;
extern const char *disk_mount_pt;

int setup_disk(void);

int littlefs_mount(struct fs_mount_t *mp);