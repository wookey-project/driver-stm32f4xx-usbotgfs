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
#ifndef LIBUSBOTGFS_H_
#define LIBUSBOTGFS_H_

#include "libc/types.h"
#include "autoconf.h"

typedef enum {
    USBOTG_FS_PORT_LOWSPEED = 0,
    USBOTG_FS_PORT_FULLSPEED = 1,
    USBOTG_FS_PORT_HIGHSPEED = 2
} usbotgfs_port_speed_t;


typedef mbed_error_t (*usbotgfs_ioep_handler_t)(uint32_t dev_id, uint32_t size, uint8_t ep);

/*
 * The USB OTG support On-The-Go configuration (i.e. Host or Device mode, configurable
 * by software stack. This enumerate define which mode to use
 */
typedef enum {
    USBOTGFS_MODE_HOST = 0,
    USBOTGFS_MODE_DEVICE = 1
} usbotgfs_dev_mode_t;

/*
 * Device Endpoint identifiers
 */
typedef enum {
    USBOTG_FS_EP0 = 0,
    USBOTG_FS_EP1 = 1,
    USBOTG_FS_EP2 = 2,
    USBOTG_FS_EP3 = 3,
    USBOTG_FS_EP4 = 4,
    USBOTG_FS_EP5 = 5,
    USBOTG_FS_EP6 = 6,
    USBOTG_FS_EP7 = 7,
} usbotgfs_ep_nb_t;

/*
 * Max packet size in EP0 is a specific field.
 * Here are the supported size
 */
typedef enum {
    USBOTG_FS_EP0_MPSIZE_64BYTES = 0,
    USBOTG_FS_EP0_MPSIZE_32BYTES = 1,
    USBOTG_FS_EP0_MPSIZE_16BYTES = 2,
    USBOTG_FS_EP0_MPSIZE_8BYTES  = 3,
} usbotgfs_ep0_mpsize_t;


/*
 * Other EPs support various sizes for their
 * max packet size. Although, we limit these size to
 * various standard sizes.
 */
typedef enum {
    USBOTG_FS_EPx_MPSIZE_64BYTES = 64,
    USBOTG_FS_EPx_MPSIZE_128BYTES = 128,
    USBOTG_FS_EPx_MPSIZE_512BYTES = 512,
    USBOTG_FS_EPx_MPSIZE_1024BYTES  = 1024,
} usbotgfs_epx_mpsize_t;

typedef enum {
    USB_FS_DXEPCTL_SD0PID_SEVNFRM  = 0,
    USB_FS_DXEPCTL_SD1PID_SODDFRM
} usbotgfs_ep_toggle_t;

/*
 * USB standard EP type
 */
typedef enum {
    USBOTG_FS_EP_TYPE_CONTROL     = 0,
    USBOTG_FS_EP_TYPE_ISOCHRONOUS = 1,
    USBOTG_FS_EP_TYPE_BULK        = 2,
    USBOTG_FS_EP_TYPE_INT         = 3,
} usbotgfs_ep_type_t;

/*
 * Global device state, depending on currently send/received data.
 * This flags are (mostly) set by rxflvl handler and can be read back
 * at oepint/iepint time to be informed of which data type is currently
 * waiting for treatment (reception case) or has been sent (transmission
 * case). The driver reset the current flag to IDLE automatically when the
 * data has be treated in iepint/oepint end of function.
 */
typedef enum {
    USBOTG_FS_EP_STATE_IDLE  = 0,
    USBOTG_FS_EP_STATE_SETUP_WIP = 1,
    USBOTG_FS_EP_STATE_SETUP = 2,
    USBOTG_FS_EP_STATE_STATUS = 3,
    USBOTG_FS_EP_STATE_STALL = 4,
    USBOTG_FS_EP_STATE_DATA_IN_WIP = 5,
    USBOTG_FS_EP_STATE_DATA_IN = 6,
    USBOTG_FS_EP_STATE_DATA_OUT_WIP = 7,
    USBOTG_FS_EP_STATE_DATA_OUT = 8,
    USBOTG_FS_EP_STATE_INVALID = 9,
} usbotgfs_ep_state_t;

typedef enum {
    USBOTG_FS_EP_DIR_IN = 0,
    USBOTG_FS_EP_DIR_OUT = 1,
} usbotgfs_ep_dir_t;


/*********************************************************************************
 * About handlers
 *
 * The Control plane must declare some handlers for various events (see usbotgfs_handlers.c
 * for more informations). These handlers are called on these events in order to execute
 * control level (or potentially upper level(s)) programs. They can use the USB OTG FS
 * driver API during their execution.
 *
 * Control level handlers are linked directly through their prototype definition here.
 *
 * We prefer to use prototype and link time symbol resolution instead of callbacks as:
 *   1. The USB control plane is not an hotpluggable element
 *   2. callbacks have security impacts, as they can be corrupted, generating arbitrary
 *      code execution
 *
 *  WARNING: as we use prototypes (and not callbacks), these functions *must* exists at
 *  link time, for symbol resolution.
 *  It has been decided that the driver doesn't hold weak symbols for these functions,
 *  as their absence would make the USB stack unfonctional.
 *  If one of these function is not set in the control plane (or in any element of the
 *  application to be linked) it would generate a link time error, corresponding to a
 *  missing symbol.
 *
 */

mbed_error_t usbctrl_handle_earlysuspend(uint32_t dev_id);
mbed_error_t usbctrl_handle_reset(uint32_t dev_id);
mbed_error_t usbctrl_handle_usbsuspend(uint32_t dev_id);
mbed_error_t usbctrl_handle_inepevent(uint32_t dev_id, uint32_t size, uint8_t ep);
mbed_error_t usbctrl_handle_outepevent(uint32_t dev_id, uint32_t size, uint8_t ep);
mbed_error_t usbctrl_handle_wakeup(uint32_t dev_id);


/********************************************************************************
 * About functional API
 *
 * This is the USB OTG FS functional API. The goal is to abstract as much as
 * possible all device-specific content and to declare a protocol orented API.
 *
 * Nevertheless, all control plane (requests, events) must be handled by the USB
 * control stack, not the driver itself.
 * As a consequence, the following API is made in order to be controlled by an
 * external USB 2.0 control stack.
 */

/*
 * Declaring the device against EwoK
 */
mbed_error_t usbotgfs_declare(void);

/*
 * Core initial setup and config. At the end of the initialization, the Core should
 * have its USB control pipe ready to receive the first requests from the host.
 */
mbed_error_t usbotgfs_configure(usbotgfs_dev_mode_t mode,
                                usbotgfs_ioep_handler_t ieph,
                                usbotgfs_ioep_handler_t oeph);

/*
 * Sending data (whatever data type is (i.e. status on control pipe or data on
 * data (Bulk, IT or isochronous) pipe)
 * This is not a syncrhonous request, i.e. data are stored into the USB OTG FS
 * interanal FIFO, waiting for bus transmission. When data are fully transmitted,
 * a iepint (device mode) or oepint (host mode) is triggered to inform the upper
 * layer that the content has been sent. Although, it is possible to push some
 * other data in the internal FIFO if needed, while this FIFO is not full
 * (check for this function return value)
 *
 * @src the RAM FIFO from which the data are read
 * @size the amount of data bytes to send
 * @ep the endpoint on which the data are to be sent
 *
 * @return MBED_ERROR_NONE if data has been correctly transmitted into the internal
 * core FIFO, or MBED_ERROR_BUSY if the interal core FIFO for the given EP is full
 */
mbed_error_t usbotgfs_send_data(uint8_t *src, uint32_t size, uint8_t ep);

/*
 * Configure for receiving data. Receiving data is a triggering event, not a direct call.
 * As a consequence, the upper layers have to specify the amount of data requested for
 * the next USB transaction on the given OUT (device mode) or IN (host mode) enpoint.
 *
 * @dst is the destination buffer that will be used to hold  @size amount of data bytes
 * @size is the amount of data bytes to load before await the upper stack
 * @ep is the active endpoint on which this action is done
 *
 * On data reception:
 * - if there is enough data loaded in the output buffer, the upper stack is awoken
 * - If not, data is silently stored in FIFO RAM (targetted by dst), and the driver waits
 *   for the next content while 'size' amount of data is not reached
 *
 * @return MBED_ERROR_NONE if setup is ok, or various possible other errors (INVSTATE
 * for invalid enpoint type, INVPARAM if dst is NULL or size invalid)
 */
mbed_error_t usbotgfs_set_recv_fifo(uint8_t *dst, uint32_t size, uint8_t ep);

/*
 * Send a special zero-length packet on EP ep
 */
mbed_error_t usbotgfs_send_zlp(uint8_t ep);

/*
 * Activate the configuration global stall mode. If any EP has its stall mode configured,
 * it can override the global stall mode
 * INFO: stall mode for Control and data EP don't have the same meaning. See datasheet,
 * chap 35.13.7
 */
mbed_error_t usbotgfs_global_stall(void);

/*
 * Clear the global stall mode.
 */
mbed_error_t usbotgfs_global_stall_clear(void);

/*
 * Set the STALL mode for the given EP
 */
mbed_error_t usbotgfs_endpoint_stall(uint8_t ep_id, usbotgfs_ep_dir_t dir);

/*
 * Clear the STALL mode for the given EP
 */
mbed_error_t usbotgfs_endpoint_stall_clear(uint8_t ep, usbotgfs_ep_dir_t dir);

mbed_error_t usbotgfs_endpoint_set_nak(uint8_t ep_id, usbotgfs_ep_dir_t dir);

mbed_error_t usbotgfs_endpoint_clear_nak(uint8_t ep_id, usbotgfs_ep_dir_t dir);

/*
 * Activate the given EP (for e.g. to transmit data)
 */
mbed_error_t usbotgfs_configure_endpoint(uint8_t               ep,
                                         usbotgfs_ep_type_t    type,
                                         usbotgfs_ep_dir_t     dir,
                                         usbotgfs_epx_mpsize_t mpsize,
                                         usbotgfs_ep_toggle_t  dtoggle,
                                         usbotgfs_ioep_handler_t handler);

/*
 * Deactivate the given EP (Its configuration is keeped, the EP is *not* deconfigured)
 */
mbed_error_t usbotgfs_deconfigure_endpoint(uint8_t ep);

/*
 * Configure the given EP in order to be ready to work
 */
mbed_error_t usbotgfs_activate_endpoint(uint8_t               id,
                                        usbotgfs_ep_dir_t     dir);

/*
 * Deconfigure the given EP. The EP is no more usable after this call. A new configuration
 * of the EP must be done before reuse it.
 * This function is typically used on SetConfiguration Requests schedule, or on
 * Reset frame reception to reconfigure the Core in a known clear state.
 */
mbed_error_t usbotgfs_deactivate_endpoint(uint8_t ep,
                                          usbotgfs_ep_dir_t     dir);


/**
 * usb_driver_set_address - Set the address of the device
 * @addr: Device's address
 */
void usbotgfs_set_address(uint16_t addr);

/* Map USB device. TODO */
void usbotgfs_bind(void);

void usbotgfs_unbind(void);

usbotgfs_ep_state_t usbotgfs_get_ep_state(uint8_t epnum, usbotgfs_ep_dir_t dir);

uint32_t usbotgfs_get_ep_mpsize(void);


usbotgfs_port_speed_t usbotgfs_get_speed(void);

#endif /*!LIBUSBOTGFS_H_ */
