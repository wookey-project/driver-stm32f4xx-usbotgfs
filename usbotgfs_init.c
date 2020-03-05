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

#include "libc/regutils.h"
#include "libc/types.h"
#include "libc/stdio.h"

#include "api/libusbotgfs.h"
#include "usbotgfs_regs.h"
#include "usbotgfs.h"

#include "usbotgfs_init.h"

#define TRIGGER_TXFE_ON_HALF_EMPTY 0
#define TRIGGER_TXFE_ON_FULL_EMPTY 1

/*
 * Core initialization after Power-On. This configuration must
 * be done irrespective to whatever the DWC_Core is going to be
 * configured in Host or Device Mode of Operation.
 * This initialization is common to any use.
 *
 *
 * If the cable is connected during power-up, the Current Mode of
 * Operation bit in the Core Interrupt register  (GINTSTS.CurMod)
 * reflects the mode. The DWC_otg core enters Host mode when an “A” plug is
 * connected, or Device mode when a “B” plug is connected.
 *
 * This section explains the initialization of the DWC_otg core after
 * power-on. The application must follow this initialization sequence
 * irrespective of whether the DWC_otg core is going to be configured
 * in Host or Device mode of operations. All core global registers are
 * initialized according to the core’s configuration. The parameters
 * referred to in this section are the DWC_otg configuration parameters
 * defined in the Databook.
 *
 * 1. Read the User Hardware Configuration registers (GHWCFG1, 2, 3, and
 * 4) to find the configuration parameters selected for DWC_otg core.
 *
 * 2. Program the following fields in the Global AHB Configuration (GAHBCFG)
 * register.
 *
 *  - Global Interrupt Mask bit = 1
 *  - Non-periodic TxFIFO Empty Level (can be enabled only when the core is
 *    operating in Slave mode as a host or as a device in Shared FIFO operation.)
 *  - Periodic TxFIFO Empty Level (can be enabled only when the core is
 *    operating in Slave mode)
 *
 * 3. Program the following field in the Global Interrupt Mask (GINTMSK)
 *    register:
 *
 *  - GINTMSK.RxFLvlMsk = 1’b0
 *
 * 4. Program the following fields in GUSBCFG register.
 *
 * - HNP Capable bit (applicable only when the OTG_MODE parameter is set
 *   to 0)
 * - SRP Capable bit (not applicable when the OTG_MODE parameter is set to
 *   Device Only)
 * - ULPI DDR Selection bit (applicable only when the OTG_HSPHY_INTERFACE
 *   parameter is selected for ULPI)
 * - External HS PHY or Internal FS Serial PHY Selection bit (not applicable
 *   when “None” is selected for the OTG_FSPHY_INTERFACE parameter)
 * - ULPI or UTMI+ Selection bit (not applicable when “None” is selected for
 *   the OTG_HSPHY_INTERFACE parameter)
 * - PHY Interface bit (applicable only when the OTG_HSPHY_INTERFACE parameter
 *   is selected for UTMI+)
 * - HS/FS TimeOUT Time-Out Calibration field
 * - USB Turnaround Time field
 *
 * 5. The software must unmask the following bits in the GINTMSK register.
 *
 * - OTG Interrupt Mask
 * - Mode Mismatch Interrupt Mask
 *
 * 6. If the GUID register is selected for implementation in coreConsultant,
 *    the software has the option of programming this register.
 *
 * 7. The software can read the GINTSTS.CurMod bit to determine whether the
 *    DWC_otg core is operating in Host or Device mode. The software then follows
 *    either the “Programming Flow for Application Selection of PHY Interface” or
 *    "Device Initialization" sequence.
 *
 */
mbed_error_t usbotgfs_initialize_core(usbotgfs_dev_mode_t mode)
{
    mode = mode; /* TODO */
    log_printf("[USB FS] initializing the core\n");
    mbed_error_t errcode = MBED_ERROR_NONE;
    int count = 0;

	set_reg(r_CORTEX_M_USBOTG_FS_GCCFG, 1, USBOTG_FS_GCCFG_PWRDWN);
    set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 1, USBOTG_FS_GUSBCFG_PHYSEL); /* Full Speed serial transceiver select */
    /* 1. Read the User Hardware Configuration registers
     *
     * INFO: this not needed in the STM32F4 IP instanciation, as GHWCFG registers
     * are not mapped
     */
    clear_reg_bits(r_CORTEX_M_USBOTG_FS_GINTSTS, USBOTG_FS_GINTSTS_SOF_Msk);


    /* 2. Program the following fields in the Global AHB Configuration (GAHBCFG)
     * register.
     */
    log_printf("[USB FS] core init: set global AHB config\n");
    /* mask the global interrupt mask */
    log_printf("[USB FS] core init: mask the global interrupt msk\n");
    set_reg(r_CORTEX_M_USBOTG_FS_GAHBCFG, 1, USBOTG_FS_GAHBCFG_GINTMSK);
    /* non-periodic TxFIFO empty level (host mode or device with SFO mode) */
    /* both Host & Device mode, TXFE rise on TxFIFO completely empty */
    /* non-periodic TxFIFO empty level rise an interrupt when *full* empty */
	set_reg(r_CORTEX_M_USBOTG_FS_GAHBCFG, 0, USBOTG_FS_GAHBCFG_TXFELVL);

	/* Wait for master AHB automaton to be in IDLE state */
    log_printf("[USB FS] core init: clear PWRDWN\n");

    /*
     * 3. Program the following field in the Global Interrupt Mask (GINTMSK)
     *    register:
     */
    log_printf("[USB FS] core init: clear global interrupt mask\n");
    set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 0, USBOTG_FS_GINTMSK_RXFLVLM);


    /*
     * 4. Program the following fields in GUSBCFG register (configure PHY
     * interactions and HNP/SRP capabilities).
     */
	set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 0, USBOTG_FS_GUSBCFG_HNPCAP); /* FIXME Device Only Configuration :  HNP capability is enabled. */

	set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 0, USBOTG_FS_GUSBCFG_SRPCAP); /* FIXME Device Only Configuration :  SRP capability is not enabled. */

	set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 0, USBOTG_FS_GUSBCFG_TOCAL); /* FS timeout calibration */

	set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 6, USBOTG_FS_GUSBCFG_TRDT); /* USB turnaround time see TRDT values (DocID018909 Rev 15) */

	set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_MMISM); /* Unmask  Mode mismatch interrupt */

    /* SRP and HNP capable bits are not described in the USB OTG HS
     * TODO: STM32F4 datasheet. It would be intereseting to check if they can
     * be used in device and host mode in HS case too */

	/* Core soft reset must be issued after PHY configuration */
	/* Wait for AHB master idle */
    while (!get_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, USBOTG_FS_GRSTCTL_AHBIDL)) {
        if (++count >  USBOTGFS_REG_CHECK_TIMEOUT) {
            log_printf("HANG! AHB Idle GRSTCTL:AHBIDL\n");
            errcode = MBED_ERROR_BUSY;
            goto err;
        }
    }

    log_printf("[USB FS] AHB idle after %d loops\n", count);


    log_printf("[USB FS] Core acknowledged reset after %d loops\n", count);
    /* 3 PHY clocks wait, (active wait here, as sys_sleep() is too slow */
	for (uint32_t i = 0; i < 0xff; i++) {
		continue;
    }

    /*
     * 5. The software must unmask the following bits in the GINTMSK register.
     */
    log_printf("[USB FS] core init: unmask OTGINT & MMISM Int\n");
	set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK,
                 USBOTG_FS_GINTMSK_OTGINT_Msk | USBOTG_FS_GINTMSK_MMISM_Msk);

    log_printf("[USB FS] core init: clear SOF (soft reset case)\n");
	clear_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_SOFM_Msk);
    /*
     * 6. not needed here.
     */
    /*
     * 7. checking curMod at core init
     */
err:
    return errcode;
}


mbed_error_t usbotgfs_initialize_device(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    log_printf("[USB FS] dev init: Device mode initialization...\n");
//    set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_RXFLVLM);
//XXX:    set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_NPTXFEM);
	set_reg(r_CORTEX_M_USBOTG_FS_GUSBCFG, 1, USBOTG_FS_GUSBCFG_FDMOD); /* FIXME Force device mode */
    /* sleep at least 25 ms after forcing device mode */
    sys_sleep(SLEEP_MODE_DEEP, 30);


    log_printf("[USB FS] dev init: set speed to FS\n");
    /* set device speed to Full Speed */
	set_reg(r_CORTEX_M_USBOTG_FS_DCFG, 0x3, USBOTG_FS_DCFG_DSPD);
    /* send packets to the application. send handshake based on NAK & STALL bits for
     * current EP */
	set_reg(r_CORTEX_M_USBOTG_FS_DCFG, 0, USBOTG_FS_DCFG_NZLSOHSK);

    log_printf("[USB FS] dev init: enable nominal Ints\n");
    /* enable reset, enumeration done, early suspend, usb suspend & start-of-frame
     * IEP and OEP int are handled at USB reset handling time */
	set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK,
        		 USBOTG_FS_GINTMSK_USBRST_Msk   |
                 USBOTG_FS_GINTMSK_ENUMDNEM_Msk |
                 USBOTG_FS_GINTMSK_ESUSPM_Msk   |
                 USBOTG_FS_GINTMSK_USBSUSPM_Msk);

    log_printf("[USB FS] dev init: unmask the global interrupt msk\n");
    set_reg(r_CORTEX_M_USBOTG_FS_GAHBCFG, 1, USBOTG_FS_GAHBCFG_GINTMSK);


	set_reg(r_CORTEX_M_USBOTG_FS_GCCFG, 1, USBOTG_FS_GCCFG_VBUSBSEN); /* Enable the VBUS sensing “B” device */
    /* connect device */
    set_reg(r_CORTEX_M_USBOTG_FS_DCTL, 0, USBOTG_FS_DCTL_SDIS);
    return errcode;
}

/*
 * TODO: this mode is not yet supported by the device
 */
mbed_error_t usbotgfs_initialize_host(void)
{
    mbed_error_t errcode = MBED_ERROR_UNSUPORTED_CMD;

    return errcode;

}

