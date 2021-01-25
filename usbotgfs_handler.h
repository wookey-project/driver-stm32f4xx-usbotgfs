/*
 *
 * Copyright 2019 The wookey project team <wookey@ssi.gouv.fr>
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
#ifndef USBOTGFS_HANDLER_H_
#define USBOTGFS_HANDLER_H_

#include "libc/types.h"


/*
 * OTG_HS_GRXSTSR/OTG_HS_GRXSTSP fields typeseting, needed by oepint/iepint handlers
 */
/*
 * Host IN packet possible status. Other values are reserved
 */
typedef enum {
    PKT_STATUS_IN_DATA_PKT_RECV     = 0x02,
    PKT_STATUS_IN_TRANSFER_COMPLETE = 0x03,
    PKT_STATUS_DATA_TOGGLE_ERROR    = 0x05,
    PKT_STATUS_CHANNEL_HALTED       = 0x07,
} host_pkt_status_t;


/*
 * Device OUT packet possible status. Other values are reserved
 */
typedef enum {
    PKT_STATUS_GLOBAL_OUT_NAK        = 0x01,
    PKT_STATUS_OUT_DATA_PKT_RECV     = 0x02,
    PKT_STATUS_OUT_TRANSFER_COMPLETE = 0x03,
    PKT_STATUS_SETUP_TRANS_COMPLETE  = 0x04,
    PKT_STATUS_SETUP_PKT_RECEIVED    = 0x06,
} device_pkt_status_t;


/*
 * The packet status field content doesn't have the same meaning depending on the current
 * mode (Device or host)
 */
typedef union {
    host_pkt_status_t   hoststs;
    device_pkt_status_t devsts;
} pkt_status_t;

typedef enum {
    DATA_PID_DATA0  = 0x0,
    DATA_PID_DATA1  = 0x1,
    DATA_PID_DATA2  = 0x2,
    DATA_PID_MDATA  = 0x3,
} data_pid_t;

void USBOTGFS_IRQHandler(uint8_t interrupt,
                         uint32_t sr,
                         uint32_t dr);

#endif/*!USBOTGFS_HANDLER_H_*/
