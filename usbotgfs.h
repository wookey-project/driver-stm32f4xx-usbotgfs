/*
 *
 * Copyright 2018 The wookey project team <wookey@ssi.gouv.fr>
 *   - Ryad     Benadjila
 *   - Arnauld  Michelizza
 *   - Mathieu  Renard
 *   - Philippe Thierry
 *   - Philippe Trebuchet
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * the Free Software Foundation; either version 3 of the License, or (at
 * ur option) any later version.
 *
 * This package is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this package; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#ifndef USBOTGHS_H_
# define USBOTGHS_H_

#include "autoconf.h"

#include "libc/regutils.h"
#include "libc/types.h"
#include "libc/stdio.h"

#include "api/libusbotgfs.h"
#include "usbotgfs_regs.h"

#define USBOTGFS_REG_CHECK_TIMEOUT 50
#define CPT_HARD 100

#define MAX_EP_HW 4

#define MAX_TIME_DETACH     4000

#define USB_GLOBAL_OUT_NAK        0b0001 /* Global OUT NAK (triggers an interrupt) */
#define USB_OUT_PACKET_RECEIVED   0b0010 /* OUT data packet received */
#define USB_OUT_TRANSFERT_COMPLETED   0b0011 /* OUT transfer completed (triggers an interrupt) */
#define USB_SETUP_TRANS_COMPLETED 0b0100 /* SETUP transaction completed (triggers an interrupt) */
#define USB_SETUP_PACKET_RECEIVED 0b0110 /* SETUP data packet received */


/*********************************************************
 * General tooling
 */

#if CONFIG_USR_DRV_USBOTGFS_DEBUG
# define log_printf(...) printf(__VA_ARGS__)
#else
# define log_printf(...)
#endif

/********************************************************
 * Driver private structures and types
 */

typedef enum {
    USBOTG_FS_SPEED_LS = 0, /* aka Low speed (USB 1.0) */
    USBOTG_FS_SPEED_FS = 1, /* aka Full Speed (USB 1.1) */
    USBOTG_FS_SPEED_HS = 2, /* aka High speed (USB 2.0) */
} usbotgfs_speed_t;

/*
 * local context hold by the driver
 */
typedef struct {
    uint8_t                      id;           /* EP id (libusbctrl view) */
    bool                         configured;   /* is EP configured in current configuration ? */
    uint16_t                     mpsize;       /* max packet size (bitfield, 11 bits, in bytes) */
    usbotgfs_ep_type_t           type;         /* EP type */
    usbotgfs_ep_state_t          state;        /* EP current state */
    usbotgfs_ep_dir_t            dir;
    usbotgfs_ioep_handler_t      handler;      /* EP Handler for (I|O)EPEVENT */

    uint8_t            *fifo;         /* associated RAM FIFO (recv) */
    uint32_t            fifo_idx;     /* current FIFO index  (recv) */
    uint32_t            fifo_size;    /* associated RAM FIFO max size (recv) */
    bool                fifo_lck;     /* DMA, locking mechanism (recv) */
    bool                core_txfifo_empty; /* core TxFIFO (Half) empty */
} usbotgfs_ep_t;

#define USBOTGFS_MAX_IN_EP   8
#define USBOTGFS_MAX_OUT_EP  3

/* current context of the USB OTG HS Core */
typedef struct {
    device_t            dev;             /* associated device_t structure */
    int                 dev_desc;        /* device descriptor */
    usbotgfs_dev_mode_t mode;            /* current OTG mode (host or device) */
    bool                gonak_req;       /* global OUT NAK requested */
    bool                gonak_active;    /* global OUT NAK effective */
    uint16_t            fifo_idx;        /* consumed Core FIFO */
    usbotgfs_ep_t       in_eps[USBOTGFS_MAX_IN_EP];       /* list of HW supported IN EPs */
    usbotgfs_ep_t       out_eps[USBOTGFS_MAX_OUT_EP];      /* list of HW supported OUT EPs */
    usbotgfs_speed_t    speed;        /* device enumerated speed, default HS */
} usbotgfs_context_t;

usbotgfs_context_t *usbotgfs_get_context(void);

#endif /*!USBOTGHS_H_ */
