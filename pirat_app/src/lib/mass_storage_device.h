#include <sample_usbd.h>
#include <stdio.h>

#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>


#define DISK_DRIVE_NAME "NAND"
#define DISK_MOUNT_PT "/" DISK_DRIVE_NAME ":"

extern struct fs_mount_t fs_mnt;
extern const char *disk_mount_pt;

int enable_usb_device_next(void);

void setup_disk(void);