/******************************************************************************
*
*   Copyright (c) 2022 Intel.
*
*   Licensed under the Apache License, Version 2.0 (the "License");
*   you may not use this file except in compliance with the License.
*   You may obtain a copy of the License at
*
*       http://www.apache.org/licenses/LICENSE-2.0
*
*   Unless required by applicable law or agreed to in writing, software
*   distributed under the License is distributed on an "AS IS" BASIS,
*   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*   See the License for the specific language governing permissions and
*   limitations under the License.
*
*******************************************************************************/

#ifndef __BB_ACC_H__
#define __BB_ACC_H__

#include <stdlib.h>
#include <stdint.h>
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <strings.h>
#include <unistd.h>
#include <stdbool.h>
#include <limits.h>
#include <linux/vfio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <libgen.h>
#include <sys/eventfd.h>
#include "pf_bb_config_cli.h"

#define SYS_DIR "/sys/bus/pci/devices"
#define CUR_DIR "."
#define PREV_DIR ".."

#define DEVICE_FILE  "device"
#define VENDOR_FILE  "vendor"
#define BAR0_FILE    "resource0"

#define PCI_STR_SIZE 15
#define NULL_PAD     2

#define ACC100_VENDOR_ID        0x8086
#define ACC100_DEVICE_ID        0x0D5C
#define VRB1_VENDOR_ID          0x8086
#define VRB1_DEVICE_ID          0x57C0
#define VRB2_VENDOR_ID          0x8086
#define VRB2_DEVICE_ID          0x57C2
#define FPGA_LTE_FEC_VENDOR_ID  0x1172
#define FPGA_LTE_FEC_DEVICE_ID  0x5052
#define FPGA_5GNR_FEC_VENDOR_ID 0x8086
#define FPGA_5GNR_FEC_DEVICE_ID 0x0D8F
#define AGX100_VENDOR_ID        0x8086
#define AGX100_DEVICE_ID        0x5799

#define VFIO_VF_TOKEN_LEN 16
#define VFIO_VF_TOKEN_STR_LEN 36
#define VFIO_PCI_CONFIG_REGION_SIZE 256
#define VFIO_IRQ_SET_BUF_LEN  (sizeof(struct vfio_irq_set) + sizeof(int))

#define BB_ACC_MAX_VFS              64
#define BB_ACC_MAX_WIN              16

#define BB_ACC_PF_TO_VF_DBELL_REG_OFFSET 0x100

#define __bf_shf(x) (__builtin_ffsll(x) - 1)

#define GENMASK(h, l) (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))

#define FIELD_GET(mask, reg) \
	((typeof(mask))(((reg) & (mask)) >> __bf_shf(mask)))

#define BITS_PER_LONG   (__SIZEOF_LONG__ * 8)

#define BIT(nr) (1UL << (nr))

#define BB_ACC_INFO_RING_NUM_ENTRIES   1024
/* Mask used to calculate an index in an Info Ring array (not a byte offset) */
#define BB_ACC_INFO_RING_MASK           (BB_ACC_INFO_RING_NUM_ENTRIES - 1)
#define BB_ACC_INFO_RING_PTR_MASK       ((BB_ACC_INFO_RING_NUM_ENTRIES * 4) - 1)
#define BB_ACC_INFO_RING_SIZE           (BB_ACC_INFO_RING_NUM_ENTRIES * sizeof(uint32_t))

#define BB_ACC_FIRST_CFG true
#define BB_ACC_RECFG     false

#define MAX(x, y) ((x) >= (y) ? (x) : (y))

/**
 * Flags indicate the status of the device to the application
 */
enum bb_acc_device_status {
	RTE_BBDEV_DEV_NOSTATUS,        /**< Nothing being reported */
	RTE_BBDEV_DEV_NOT_SUPPORTED,   /**< Device status is not supported on the PMD */
	RTE_BBDEV_DEV_RESET,           /**< Device in reset and un-configured state */
	RTE_BBDEV_DEV_CONFIGURED,      /**< Device is configured and ready to use */
	RTE_BBDEV_DEV_ACTIVE,          /**< Device is configured and VF is being used */
	RTE_BBDEV_DEV_FATAL_ERR,       /**< Device has hit a fatal uncorrectable error */
	RTE_BBDEV_DEV_RESTART_REQ,     /**< Device requires application to restart */
	RTE_BBDEV_DEV_RECONFIG_REQ,    /**< Device requires application to reconfigure queues */
	RTE_BBDEV_DEV_CORRECT_ERR,     /**< Warning of a correctable error event happened */
};

/**
 * Flags indicate the status of the device to the application
 */
enum bb_acc_device_request {
	REQ_DEV_STATUS = 1,        /**< Request the device status report */
	REQ_DEV_NEW = 2,           /**< New VF device being used */
	REQ_DEV_LUT_VER = 3,       /**< Request the device LUT version number. */
	REQ_DEV_FFT_WIN_SIZE = 4,  /**< Request window width. */
	REQ_DEV_FFT_WIN_START = 5,  /**< Request window start point. */
	REQ_DEV_MASK = 0xFFFF
};

/* Function pointers for bb dev operations */
struct bb_acc_operations {
	int   (*open)(void *dev);
	int   (*conf)(void *dev, void *bar0addr, const char *arg_cfg_filename, bool firstConfig);
	int   (*enable_intr)(void *dev);
	int   (*disable_intr)(void *dev);
	int   (*dev_enable_intr)(void *dev);
	int   (*dev_disable_intr)(void *dev);
	int   (*dev_isr)(void *dev);
	int   (*flr)(void *dev);
	void  (*cluster_reset)(void *dev);
	void *(*get_bar0_addr)(const char *pci_addr, unsigned int bar_size, void *dev);
	void  (*close)(void *dev);
	void  (*device_data)(void *dev);
};

typedef struct hw_device {
	const char *device_name;
	char *config_file;
	char *fft_lut_filename;
	int vendor_id;
	int device_id;
	char pci_address[PCI_STR_SIZE];
	bool driver_found;
	struct bb_acc_operations ops;

	void *info_ring;
	uint64_t info_ring_phys_addr;
	size_t info_ring_total_size;
	uint16_t info_ring_head;
	void *bar0Addr;
	unsigned int bar_size;

	/* vfio specific */
	int vfio_mode;
	int vfio_dev_fd;
	int irq_event_fd;
	int vfio_int_mode;
	int num_regions;
	void *iommu_map_dma_vaddr;
	int   iommu_map_dma_size;
	unsigned char vfio_vf_token[VFIO_VF_TOKEN_LEN];

	unsigned long dev_status[BB_ACC_MAX_VFS];
	unsigned long prev_status[BB_ACC_MAX_VFS];
	int auto_reconfig_on_fatal_error;
	int device_reset_using_flr;
	int numvfs;
	uint16_t fft_version_md5sum;
	int fft_win_start[BB_ACC_MAX_WIN];
	int fft_win_size[BB_ACC_MAX_WIN];
} hw_device;

#define PCI_VENDOR_ID		0x00	/* 16 bits */
#define PCI_DEVICE_ID   0x02	/* 16 bits */
#define PCI_COMMAND     0x04	/* 16 bits */
#define PCI_COMMAND_MEMORY 0x2	/* Enable response in Memory space */
#define PCI_COMMAND_MASTER 0x4	/* Enable bus mastering */
#define PCI_REVISION_ID		0x08	/* Revision ID */

#define MAX_BB_ACC_DEV_COUNT 10
#define MAX_EVENTS (MAX_BB_ACC_DEV_COUNT + 1)

#define DEVICE_RESET_USING_CLUSTER 0
#define DEVICE_RESET_USING_FLR     1
#define DEVICE_RESET_AUTO_RECONFIG 1

typedef	void (*cbFp_t)(int fd, void *data);

extern void *vfio_dma_alloc(void *dev, size_t size, uint64_t *phy);
extern void  vfio_dma_free(void *virt, size_t size);
extern void *vfio_get_bar0_mapping(const char *pci_addr, unsigned int bar_size, void *hwd);
extern int   vfio_device_open(void *dev);
extern int   vfio_device_reset(void *dev);
extern int   vfio_enable_intr(void *dev);
extern int   vfio_disable_intr(void *dev);
extern int   vfio_irq_loopback_test(hw_device *hwd);
extern int   vfio_write(int fd, void *buf, size_t len, off_t off);
extern int   vfio_read(int fd, void *buf, size_t len, off_t off);
extern int   vfio_uuid_parse(char *uuid_str, unsigned char *uuid);
extern void  vfio_device_close(hw_device *dev);
extern int   event_add_fd(int fd, cbFp_t cbFp,  void *cbData);
extern int   event_del_fd(int fd);
extern void event_init_fd(void);

extern int bb_acc_dev_reset_and_reconfig(void *accel_dev);
extern int bb_acc_cluster_reset_and_reconfig(void *accel_dev);
extern int bb_acc_cluster_reset_and_flr_reconfig(void *accel_dev);
extern void bb_acc_set_all_device_status(void *dev, unsigned int status);
extern const char *bb_acc_device_status_str(unsigned int status);

extern int cmd_handler(int argc, char *argv[]);
extern void daemonize(hw_device *dev);

extern int update_reset_mode(void *dev, int mode);
extern int auto_reset_mode(void *dev, int mode);

extern void clear_log_file(void *dev);
extern void clear_log_resp_file(void *dev);
extern void exit_app_mode(void *dev);
extern int acc_reg_dump(void *dev, int devId);
extern int acc_mem_read(void *dev, int rwFlag, int regAddr, int wPayload);
extern void acc_device_data(void *dev);

int unix_channel_init(void);
int unix_channel_rx(void);

extern int
print_stat32(char *buf, int offset, int len, uint32_t stat);

extern void
print_all_stat32(hw_device *accel_dev, uint32_t address, int num, int reg_offset);

/*
 * Configure ACC100
 */
extern int
acc100_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);

extern void
acc100_device_data(void *dev);

/*
 * Configure VRB1
 */
extern int
vrb1_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);

extern int
vrb1_enable_intr(void *);

extern int
vrb1_disable_intr(void *);

extern int
vrb1_irq_handler(void *dev);

extern void
vrb1_cluster_reset(void *dev);

extern int vrb1_get_info_ring_size(void);

extern void
vrb1_device_data(void *dev);

/*
 * Configure VRB2
 */
extern int
vrb2_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);

extern int
vrb2_enable_intr(void *);

extern int
vrb2_disable_intr(void *);

extern int
vrb2_irq_handler(void *dev);

extern void
vrb2_cluster_reset(void *dev);

extern int
vrb2_get_info_ring_size(void);

extern void
vrb2_device_data(void *dev);

extern void
vrb_fft_lut_md5sum(const char *lut_filename, hw_device *accel_pci_dev);

extern void
vrb_fft_win_check(int16_t *gTDWinCoff, hw_device *accel_pci_dev);

/*
 * Configure FPGA
 */
extern int
fpga_lte_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);
extern int
fpga_5gnr_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);
extern int
agx100_configure(void *dev, void *bar0addr, const char *arg_cfg_filename, const bool first_cfg);

extern void sig_fun(int sig);
#endif /* __BB_ACC_H__ */
