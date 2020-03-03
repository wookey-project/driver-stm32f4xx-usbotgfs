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
#ifndef USBOTGFS_INIT_H_
# define USBOTGFS_INIT_H_

/*
 * This package handle the USB OTG HS device and core initialization actions
 * initialization are complex actions and are defined in the
 * DesignWare Cores USB 2.0 Hi-Speed On-The-Go (OTG) Progamming Guide, v3.30a
 */

#include "autoconf.h"

#include "libc/regutils.h"
#include "libc/types.h"

#include "api/libusbotgfs.h"
#include "usbotgfs_regs.h"


/*
 * Power-On generic Core initialization, independent of the USB core configuration.
 * This action must be done at USB power-on in any case.
 */
mbed_error_t usbotgfs_initialize_core(usbotgfs_dev_mode_t mode);

/*
 * Device mode core initialization. In this mode, the USB OTG HS Core is configured
 * in device mode.
 */
mbed_error_t usbotgfs_initialize_device(void);

/*
 * Host mode core initialization. In this mode, the USB OTG HS Core is configured
 * in host mode.
 */
mbed_error_t usbotgfs_initialize_host(void);

#endif/*!USBOTGFS_INIT_H_*/
