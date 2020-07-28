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
#include "autoconf.h"

#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "generated/usb_otg_hs.h"

#include "api/libusbotgfs.h"
#include "usbotgfs.h"
#include "usbotgfs_init.h"
#include "usbotgfs_fifos.h"
#include "usbotgfs_handler.h"
#include "usbotgfs_regs.h"
#include "libs/usbctrl/api/libusbctrl.h"


#define ZERO_LENGTH_PACKET 0
#define OUT_NAK		0x01
#define DataOUT		0x02
#define Data_Done	0x03
#define SETUP_Done	0x04
#define SETUP		0x06

#define MAX_EPx_PKT_SIZE 64

#define USB_REG_CHECK_TIMEOUT 50

#define USBOTG_FS_RX_FIFO_SZ 	512
#define USBOTG_FS_TX_FIFO_SZ	512

#define USBOTG_FS_DEBUG 0

/******************************************************************
 * Utilities
 */
/* wait while the iepint (or oepint in host mode) clear the DATA_OUT state */
static void usbotgfs_wait_for_xmit_complete(usbotgfs_ep_t *ep)
{
#if CONFIG_USR_DEV_USBOTGFS_TRIGER_XMIT_ON_HALF
    /* wait for iepint interrupt & DIEPINTx TXFE flag set, specifying that
     * the TxFIFO is half empty
     */
    do {
        ;
    } while (ep->state != USBOTG_FS_EP_STATE_IDLE);
#else
    /* wait for iepint interrupt & DIEPINTx TXFC flag set, specifying that
     * the TxFIFO is half empty
     */
    do {
        ;
    } while (ep->state != USBOTG_FS_EP_STATE_IDLE);
#endif
    return;
}

/******************************************************************
 * Defining functional API
 */

static const char *devname = "usb-otg-fs";
/* buffer for setup packets */
// TODO to use static uint8_t 	setup_packet[8];

/* local context. Only one as there is one USB OTG device per SoC */
static volatile usbotgfs_context_t usbotgfs_ctx = { 0 };

usbotgfs_context_t *usbotgfs_get_context(void)
{
    return (usbotgfs_context_t *)&usbotgfs_ctx;
}

mbed_error_t usbotgfs_declare(void)
{
    e_syscall_ret ret = 0;

    log_printf("[USBOTG][FS] Declaring device\n");
    memset((void*)&(usbotgfs_ctx.dev), 0, sizeof(device_t));

    memcpy((void*)usbotgfs_ctx.dev.name, devname, strlen(devname));

    usbotgfs_ctx.dev.address = USB_OTG_FS_BASE;
    usbotgfs_ctx.dev.size = 0x4000;
    usbotgfs_ctx.dev.irq_num = 1;
    /* device is mapped voluntary and will be activated after the full
     * authentication sequence
     */
    usbotgfs_ctx.dev.map_mode = DEV_MAP_VOLUNTARY;

    /* IRQ configuration */
    usbotgfs_ctx.dev.irqs[0].handler = USBOTGFS_IRQHandler;
    usbotgfs_ctx.dev.irqs[0].irq = OTG_FS_IRQ; /* starting with STACK */
    usbotgfs_ctx.dev.irqs[0].mode = IRQ_ISR_FORCE_MAINTHREAD; /* if ISR force MT immediat execution, use FORCE_MAINTHREAD instead of STANDARD, and activate FISR permission */

    /*
     * IRQ posthook configuration
     * The posthook is executed at the end of the IRQ handler mode, *before* the ISR.
     * It permit to clean potential status registers (or others) that may generate IRQ loops
     * while the ISR has not been executed.
     * register read can be saved into 'status' and 'data' and given to the ISR in 'sr' and 'dr' argument
     */
    usbotgfs_ctx.dev.irqs[0].posthook.status = 0x0014; /* SR is first read */
    usbotgfs_ctx.dev.irqs[0].posthook.data = 0x0018; /* Data reg  is 2nd read */


    usbotgfs_ctx.dev.irqs[0].posthook.action[0].instr = IRQ_PH_READ;
    usbotgfs_ctx.dev.irqs[0].posthook.action[0].read.offset = 0x0014;


    usbotgfs_ctx.dev.irqs[0].posthook.action[1].instr = IRQ_PH_READ;
    usbotgfs_ctx.dev.irqs[0].posthook.action[1].read.offset = 0x0018;


    usbotgfs_ctx.dev.irqs[0].posthook.action[2].instr = IRQ_PH_MASK;
    usbotgfs_ctx.dev.irqs[0].posthook.action[2].mask.offset_dest = 0x14; /* MASK register offset */
    usbotgfs_ctx.dev.irqs[0].posthook.action[2].mask.offset_src = 0x14; /* MASK register offset */
    usbotgfs_ctx.dev.irqs[0].posthook.action[2].mask.offset_mask = 0x18; /* MASK register offset */
    usbotgfs_ctx.dev.irqs[0].posthook.action[2].mask.mode = 0; /* no binary inversion */


    /* mask only for bits that are 'r', other bits of GINTSTS are rc_w1, handle by MASK PH */
    usbotgfs_ctx.dev.irqs[0].posthook.action[3].instr = IRQ_PH_AND;
    usbotgfs_ctx.dev.irqs[0].posthook.action[3].and.offset_dest = 0x18; /* MASK register offset */
    usbotgfs_ctx.dev.irqs[0].posthook.action[3].and.offset_src = 0x14; /* MASK register offset */
    usbotgfs_ctx.dev.irqs[0].posthook.action[3].and.mask =
                 USBOTG_FS_GINTMSK_OEPINT_Msk   |
                 USBOTG_FS_GINTMSK_IEPINT_Msk   |
//XXX:                 USBOTG_FS_GINTMSK_NPTXFEM_Msk  |
                 USBOTG_FS_GINTMSK_PTXFEM_Msk   |
				 USBOTG_FS_GINTMSK_RXFLVLM_Msk;
    usbotgfs_ctx.dev.irqs[0].posthook.action[3].and.mode = 1; /* binary inversion */

    /* Now let's configure the GPIOs */
    usbotgfs_ctx.dev.gpio_num = 4;

	/* SOF -> PA 8 */
    usbotgfs_ctx.dev.gpios[0].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    usbotgfs_ctx.dev.gpios[0].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_SOF].port;
    usbotgfs_ctx.dev.gpios[0].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_SOF].pin;
    usbotgfs_ctx.dev.gpios[0].mode         = GPIO_PIN_ALTERNATE_MODE;
    usbotgfs_ctx.dev.gpios[0].pupd         = GPIO_NOPULL;
    usbotgfs_ctx.dev.gpios[0].type         = GPIO_PIN_OTYPER_PP;
    usbotgfs_ctx.dev.gpios[0].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    usbotgfs_ctx.dev.gpios[0].afr          = GPIO_AF_OTG_FS;
	/* VBUS -> PA 9 */
    usbotgfs_ctx.dev.gpios[1].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    usbotgfs_ctx.dev.gpios[1].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_VBUS].port;
    usbotgfs_ctx.dev.gpios[1].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_VBUS].pin;
    usbotgfs_ctx.dev.gpios[1].mode         = GPIO_PIN_INPUT_MODE;
    usbotgfs_ctx.dev.gpios[1].pupd         = GPIO_NOPULL;
    usbotgfs_ctx.dev.gpios[1].type         = GPIO_PIN_OTYPER_PP;
    usbotgfs_ctx.dev.gpios[1].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    usbotgfs_ctx.dev.gpios[1].afr          = GPIO_AF_OTG_FS;

	/* DM pin -> PA11 */
    usbotgfs_ctx.dev.gpios[2].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    usbotgfs_ctx.dev.gpios[2].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_DM].port;
    usbotgfs_ctx.dev.gpios[2].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_DM].pin;
    usbotgfs_ctx.dev.gpios[2].mode         = GPIO_PIN_ALTERNATE_MODE;
    usbotgfs_ctx.dev.gpios[2].pupd         = GPIO_NOPULL;
    usbotgfs_ctx.dev.gpios[2].type         = GPIO_PIN_OTYPER_PP;
    usbotgfs_ctx.dev.gpios[2].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    usbotgfs_ctx.dev.gpios[2].afr          = GPIO_AF_OTG_FS;

	/* DP pin -> PA12 */
    usbotgfs_ctx.dev.gpios[3].mask         = GPIO_MASK_SET_MODE | GPIO_MASK_SET_PUPD | GPIO_MASK_SET_TYPE | GPIO_MASK_SET_SPEED | GPIO_MASK_SET_AFR;
    usbotgfs_ctx.dev.gpios[3].kref.port    = usb_otg_fs_dev_infos.gpios[USB_FS_DP].port;
    usbotgfs_ctx.dev.gpios[3].kref.pin     = usb_otg_fs_dev_infos.gpios[USB_FS_DP].pin;
    usbotgfs_ctx.dev.gpios[3].mode         = GPIO_PIN_ALTERNATE_MODE;
    usbotgfs_ctx.dev.gpios[3].pupd         = GPIO_NOPULL;
    usbotgfs_ctx.dev.gpios[3].type         = GPIO_PIN_OTYPER_PP;
    usbotgfs_ctx.dev.gpios[3].speed        = GPIO_PIN_VERY_HIGH_SPEED;
    usbotgfs_ctx.dev.gpios[3].afr          = GPIO_AF_OTG_FS;

    if ((ret == sys_init(INIT_DEVACCESS, (device_t*)&(usbotgfs_ctx.dev), (int*)&(usbotgfs_ctx.dev_desc))) != SYS_E_DONE) {
        return MBED_ERROR_UNKNOWN;
    }
    return MBED_ERROR_NONE;
}


/*
 * This function initialize the USB OTG HS Core.
 *
 * The driver must meet the following conditions to set up the device core to handle traffic:
 *
 *  -  In Slave mode, GINTMSK.NPTxFEmpMsk, and GINTMSK.RxFLvlMsk must be unset.
 *
 * The driver must perform the following steps to initialize the core at device on, power on, or after a
 * mode change from Host to Device.
 *
 * 1. Program the following fields in DCFG register.
 *  -  Device Speed
 *  -  NonZero Length Status OUT Handshake
 *  - Periodic Frame Interval (If Periodic Endpoints are supported)
 *
 * 3. Clear the DCTL.SftDiscon bit. The core issues a connect after this bit is cleared.
 *
 * 4. Program the GINTMSK register to unmask the following interrupts.
 * -  USB Reset
 * -  Enumeration Done
 * -  Early Suspend
 * -  USB Suspend
 * -  SOF
 *
 * 5. Wait for the GINTSTS.USBReset interrupt, which indicates a reset has been detected on the USB and
 *    lasts for about 10 ms. On receiving this interrupt, the application must perform the steps listed in
 *    "Initialization on USB Reset" on page 157.
 *
 * 6. Wait for the GINTSTS.EnumerationDone interrupt. This interrupt indicates the end of reset on the
 * USB. On receiving this interrupt, the application must read the DSTS register to determine the
 * enumeration speed and perform the steps listed in “Initialization on Enumeration Completion” on
 * page 158.
 *
 * At this point, the device is ready to accept SOF packets and perform control transfers on control endpoint 0.
 */
//static mbed_error_t usbotgfs_core_init;

mbed_error_t usbotgfs_configure(usbotgfs_dev_mode_t mode,
                                usbotgfs_ioep_handler_t ieph,
                                usbotgfs_ioep_handler_t oeph)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    /* First, reset the PHY device connected to the core through ULPI interface */
    log_printf("[USB FS] Mapping device\n");
    if (sys_cfg(CFG_DEV_MAP, usbotgfs_ctx.dev_desc)) {
        log_printf("[USB FS] Unable to map USB device !!!\n");
        errcode = MBED_ERROR_NOMEM;
        goto err;
    }
    usbotgfs_ctx.mode = mode;
    /* first, we need to initialize the core */
    log_printf("[USB FS] initialize the Core\n");
    if ((errcode = usbotgfs_initialize_core(mode)) != MBED_ERROR_NONE) {
        goto err;
    }
    /* host/device mode */
    switch (mode) {
        case USBOTGFS_MODE_HOST: {
            log_printf("[USB FS][HOST] initialize in Host mode\n");
            if ((errcode = usbotgfs_initialize_host()) != MBED_ERROR_NONE) {
                goto err;
            }
            /* IT Indicates that Periodic TxFIFO is half empty */
            break;
        }
        case USBOTGFS_MODE_DEVICE: {
            log_printf("[USB FS][DEVICE] initialize in Device mode\n");
            if ((errcode = usbotgfs_initialize_device()) != MBED_ERROR_NONE) {
                goto err;
            }
            break;
        }
        default:
            errcode = MBED_ERROR_INVPARAM;
            goto err;
            break;
    }

    usbotgfs_ctx.fifo_idx = 0;
    /* initialize EP0, which is both IN & OUT EP */
    usbotgfs_ctx.in_eps[0].id = 0;
    usbotgfs_ctx.in_eps[0].configured = true; /* wait for reset, but EP0 ctrl is ready to recv
XXX: shouldn't it be false, without FIFO as RXFLVL should not be received before
reset ? */
    usbotgfs_ctx.in_eps[0].mpsize = USBOTG_FS_EPx_MPSIZE_64BYTES;
    usbotgfs_ctx.in_eps[0].type = USBOTG_FS_EP_TYPE_CONTROL;
    usbotgfs_ctx.in_eps[0].state = USBOTG_FS_EP_STATE_IDLE;
    usbotgfs_ctx.in_eps[0].handler = ieph;
    usbotgfs_ctx.in_eps[0].fifo = NULL; /* not yet configured */
    usbotgfs_ctx.in_eps[0].fifo_idx = 0; /* not yet configured */
    usbotgfs_ctx.in_eps[0].fifo_size = 0; /* not yet configured */
    usbotgfs_ctx.in_eps[0].fifo_lck = false;
    usbotgfs_ctx.in_eps[0].dir = USBOTG_FS_EP_DIR_IN;
    if (mode == USBOTGFS_MODE_DEVICE) {
        usbotgfs_ctx.in_eps[0].core_txfifo_empty = true;
    }

    usbotgfs_ctx.out_eps[0].id = 0;
    usbotgfs_ctx.out_eps[0].configured = true; /* wait for reset */
    usbotgfs_ctx.out_eps[0].mpsize = USBOTG_FS_EPx_MPSIZE_64BYTES;
    usbotgfs_ctx.out_eps[0].type = USBOTG_FS_EP_TYPE_CONTROL;
    usbotgfs_ctx.out_eps[0].state = USBOTG_FS_EP_STATE_IDLE;
    usbotgfs_ctx.out_eps[0].handler = oeph;
    usbotgfs_ctx.out_eps[0].dir = USBOTG_FS_EP_DIR_OUT;
    usbotgfs_ctx.out_eps[0].fifo = 0; /* not yet configured */
    usbotgfs_ctx.out_eps[0].fifo_idx = 0; /* not yet configured */
    usbotgfs_ctx.out_eps[0].fifo_size = 0; /* not yet configured */
    usbotgfs_ctx.in_eps[0].fifo_lck = false;

    usbotgfs_ctx.speed = USBOTG_FS_SPEED_FS; /* default. In device mode, wait for enumeration */

err:
    return errcode;
}

/*
 * Returns, for the current IP, the max data endpoint (not control) packet size
 * supported
 */
uint32_t usbotgfs_get_ep_mpsize(void)
{
    return MAX_EPx_PKT_SIZE;
}

/*
 * Sending data put content in the USB OTG FIFO and ask the EP to read from it to
 * send the data on the line (by activating the EP (field USBAEP of out EPs))
 * We must wait data sent IT to be sure that content is effectively transmitted
 */
mbed_error_t usbotgfs_send_data(uint8_t *src, uint32_t size, uint8_t ep_id)
{
    uint32_t packet_count = 0;
    mbed_error_t errcode = MBED_ERROR_NONE;
    uint32_t fifo_size = 0;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    usbotgfs_ep_t *ep = NULL;

#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
      ep = &ctx->in_eps[ep_id];
#else
      ep = &ctx->out_eps[ep_id];
#endif
    if (!ep->configured) {
        log_printf("[USBOTG][FS] ep %d not configured\n", ep->id);
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    fifo_size = USBOTG_FS_TX_CORE_FIFO_SZ;
    /* configure EP FIFO internal informations */
    if ((errcode = usbotgfs_set_xmit_fifo(src, size, ep_id)) != MBED_ERROR_NONE) {
       log_printf("[USBOTG][FS] failed to set EP%d TxFIFO!\n", ep_id);
        goto err;
    }
    /*
     * Here, we have to split the src content, taking into account the
     * current EP mpsize, and schedule transmission into the Core TxFIFO.
     */

    /* XXX: Here we assume fifo size == mpsize, which is bad..., fifo is bigger */
    uint32_t residual_size = size;

    /*
     * We can configure the core to handle the transmission of upto:
     * - 1024 packets (independently of their size)
     * - 1048575 bytes (2^19 - 1, independently of the number of packets)
     *
     * We consider here, that there is not request bigger than the max packet
     * size in bytes (i.e. ~1Mbytes), and no request bigger than 1024 packets
     * (in "data" EP such as mass storage where MPSize is 512, we can transmit
     * upto 512*1024 = 512KBytes per transfer, which is huge).
     */

    /*
     * First we configure the number of packets to transfer and the number of
     * bytes to transfer
     */
    packet_count = (size / ep->mpsize) + ((size % ep->mpsize) ? 1: 0);

    log_printf("[USBOTG][FS] need to write %d pkt on ep %d, init_size: %d\n", packet_count, ep_id, size);
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
    /* 1. Program the OTG_FS_DIEPTSIZx register for the transfer size
     * and the corresponding packet count. */
    /* EP 0 is not able to handle more than one packet of mpsize size per transfer. For bigger
     * transfers, the driver must fragment data transfer transparently */
    if (ep_id > 0 || size < ep->mpsize) {
        set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                packet_count,
                USBOTG_FS_DIEPTSIZ_PKTCNT_Msk(ep_id),
                USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(ep_id));

        set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                size,
                USBOTG_FS_DIEPTSIZ_XFRSIZ_Msk(ep_id),
                USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(ep_id));
    } else {
        log_printf("[USBOTG][FS] need to write more data than the EP is able in a single transfer\n");
        set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                1,
                USBOTG_FS_DIEPTSIZ_PKTCNT_Msk(ep_id),
                USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(ep_id));
        set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                ep->mpsize,
                USBOTG_FS_DIEPTSIZ_XFRSIZ_Msk(ep_id),
                USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(ep_id));
    }
    /* 2. Enable endpoint for transmission. */
    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id),
            USBOTG_FS_DIEPCTL_CNAK_Msk | USBOTG_FS_DIEPCTL_EPENA_Msk);

    ep->state = USBOTG_FS_EP_STATE_DATA_IN_WIP;
#else
    /* EP 0 is not able to handle more than one packet of mpsize size per transfer. For bigger
     * transfers, the driver must fragment data transfer transparently */
    if (ep_id > 0 || size <= ep->mpsize) {
    /* 1. Program the OTG_FS_DOEPTSIZx register for the transfer size
     * and the corresponding packet count. */
    set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(ep_id),
            packet_count,
            USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(ep_id),
            USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(ep_id));

    set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(ep_id),
            size,
            USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(ep_id),
            USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(ep_id));
    } else {
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(ep_id),
                1,
                USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(epid),
                USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(epid));
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(ep_id),
                ep->mpsize,
                USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(epid),
                USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(epid));
    }
    ep->state = USBOTG_FS_EP_STATE_DATA_OUT_WIP;
#endif
    /* Fragmentation on EP0 case: we don't loop on the input FIFO to
     * synchronously transmit the data, we just write the first packet
     * into the FIFO, and we wait for IEPINT. The successive next
     * contents will be transmitted by iepint by detecting that
     * ep->fifo_idx is smaller than ep->fifo_size (data transmission
     * not finished) */
    if (ep_id == 0 && size > ep->mpsize) {
       log_printf("[USBOTG][FS] fragment: initiate the first fragment to send (MPSize) on EP0\n");
        /* wait for enough space in TxFIFO */
        while (get_reg(r_CORTEX_M_USBOTG_FS_DTXFSTS(ep_id), USBOTG_FS_DTXFSTS_INEPTFSAV) < (ep->mpsize / 4)) {
            if (get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_SUSPSTS)){
                log_printf("[USBOTG][FS] Suspended!\n");
                errcode = MBED_ERROR_BUSY;
                goto err;
            }
        }
        /* write data from SRC to FIFO */
        usbotgfs_write_epx_fifo(ep->mpsize, ep);
        goto err;
    }

    /*
     * Case of packets WITHOUT fragmentation
     * Now, we need to loop on the FIFO write and transmit, while there is
     * data to send. The Core FIFO will handle the decrement of XFRSIZ and
     * PKTCNT automatically, and will rise the XFRC interrupt when both reach
     * 0.
     */
    /*
     * First, we push FIFO size multiple into the FIFO
     */
    while (residual_size >= fifo_size) {
        while (get_reg(r_CORTEX_M_USBOTG_FS_DTXFSTS(ep_id), USBOTG_FS_DTXFSTS_INEPTFSAV) < (fifo_size / 4)) {
            if (get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_SUSPSTS)){
                log_printf("[USBOTG][FS] Suspended!\n");
                errcode = MBED_ERROR_BUSY;
                goto err;
            }
        }
        /* write data from SRC to FIFO */
        usbotgfs_write_epx_fifo(fifo_size, ep);
        /* wait for XMIT data to be transfered (wait for iepint (or oepint in
         * host mode) to set the EP in correct state */
        //usbotgfs_wait_for_xmit_complete(ep);
        residual_size -= fifo_size;
        log_printf("[USBOTG][FS] EP: %d: residual: %d\n", ep_id, residual_size);
    }
    /* Now, if there is residual size shorter than FIFO size, just send it */
    if (residual_size > 0) {
        /* wait while there is enough space in TxFIFO */
        while (get_reg(r_CORTEX_M_USBOTG_FS_DTXFSTS(ep_id), USBOTG_FS_DTXFSTS_INEPTFSAV)  < ((residual_size / 4) + (residual_size & 3 ? 1 : 0))) {
            if (get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_SUSPSTS)){
                log_printf("[USBOTG][FS] Suspended!\n");
                errcode = MBED_ERROR_BUSY;
                goto err;
            }
        }
        /* set the EP state to DATA OUT WIP (not yet transmitted) */
        usbotgfs_write_epx_fifo(residual_size, ep);
        /* wait for XMIT data to be transfered (wait for iepint (or oepint in
         * host mode) to set the EP in correct state */
        //usbotgfs_wait_for_xmit_complete(ep);

        residual_size = 0;
    }

    if(get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_SUSPSTS)) {
        errcode = MBED_ERROR_BUSY;
    }
err:
    /* From whatever we come from to this point, the current transfer is complete
     * (with failure or not on upper level). IEPINT can inform the upper layer */
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        ep->state = USBOTG_FS_EP_STATE_DATA_IN;
#else
        ep->state = USBOTG_FS_EP_STATE_DATA_OUT;
#endif
    return errcode;
}

/*
 * Send a Zero-length packet into EP 'ep'
 */
mbed_error_t usbotgfs_send_zlp(uint8_t ep_id)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    usbotgfs_ep_t *ep = NULL;

#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
    ep = &ctx->in_eps[ep_id];
#else
    ep = &ctx->out_eps[ep_id];
#endif
    if (!ep->configured) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }

    /*
     * Be sure that previous transmission is finished before configuring another one
     */
    while (get_reg(r_CORTEX_M_USBOTG_FS_DTXFSTS(ep_id), USBOTG_FS_DTXFSTS_INEPTFSAV) <
            USBOTG_FS_TX_CORE_FIFO_SZ / 4) {
        /* Are we suspended? */
        if (get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_SUSPSTS)){
            log_printf("[USBOTG][FS] Suspended!\n");
            errcode = MBED_ERROR_BUSY;
            goto err;
        }
    }

    log_printf("[USBOTG][FS] Sending ZLP on ep %d\n", ep_id);
    /* device mode ONLY */
    /* EP is now in DATA_OUT state */
    // XXX: needed for ZLP ? ep->state = USBOTG_FS_EP_STATE_DATA_OUT;
    /* 1. Program the OTG_FS_DIEPTSIZx register for the transfer size
     * and the corresponding packet count. */
    set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
            1,
            USBOTG_FS_DIEPTSIZ_PKTCNT_Msk(ep_id),
            USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(ep_id));

    set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
            0,
            USBOTG_FS_DIEPTSIZ_XFRSIZ_Msk(ep_id),
            USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(ep_id));
    /* 2. Enable endpoint for transmission. */
    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id),
            USBOTG_FS_DIEPCTL_CNAK_Msk | USBOTG_FS_DIEPCTL_EPENA_Msk);

err:
    return errcode;
}

/*
 * Set the STALL mode for the device. Per-EP STALL mode can still override
 */
mbed_error_t usbotgfs_global_stall(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    return errcode;
}

mbed_error_t usbotgfs_endpoint_set_nak(uint8_t ep_id, usbotgfs_ep_dir_t dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    uint32_t count = 0;
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }

    /*
     * FIXME: For IN endpoint, implicit fallthrough to IN+OUT NAK
     * It seems that in the other case, the USB Core stalls.
     */
    switch (dir) {
        case USBOTG_FS_EP_DIR_IN:
            if (ep_id >= USBOTGFS_MAX_IN_EP) {
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->in_eps[ep_id].configured == false) {
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            /* wait for end of current transmission */
            while (get_reg_value(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_EPENA_Msk, USBOTG_FS_DIEPCTL_EPENA_Pos))  {
                if (++count > USBOTGFS_REG_CHECK_TIMEOUT){
                    log_printf("[USBOTG][FS] HANG! DIEPCTL:EPENA\n");
                    errcode = MBED_ERROR_BUSY;
                    goto err;
                }
            }

            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_SNAK_Msk);
            __explicit_fallthrough
        case USBOTG_FS_EP_DIR_OUT:
            if (ep_id >= USBOTGFS_MAX_OUT_EP) {
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->out_eps[ep_id].configured == false) {
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            /* wait for end of current transmission */
            while (get_reg_value(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_EPENA_Msk, USBOTG_FS_DOEPCTL_EPENA_Pos))  {
                if (++count > USBOTGFS_REG_CHECK_TIMEOUT){
                    log_printf("[USBOTG][FS] HANG! DOEPCTL:EPENA\n");
                    errcode = MBED_ERROR_BUSY;
                    goto err;
                }
            }

            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DIEPCTL_SNAK_Msk);
            break;
        default:
            errcode = MBED_ERROR_INVPARAM;
            goto err;
    }
err:
    return errcode;
}

mbed_error_t usbotgfs_endpoint_clear_nak(uint8_t ep_id, usbotgfs_ep_dir_t dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    //uint32_t count = 0;
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }

    /*
     * FIXME: For IN endpoint, implicit fallthrough to IN+OUT ACK
     * It seems that in the other case, the USB Core stalls.
     */
    switch (dir) {
        case USBOTG_FS_EP_DIR_IN:
            log_printf("[USBOTG][FS] CNAK on IN ep %d\n", ep_id);
            if (ep_id >= USBOTGFS_MAX_IN_EP) {
                log_printf("[USBOTG][FS] invalid IN EP %d\n", ep_id);
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->in_eps[ep_id].configured == false) {
                log_printf("[USBOTG][FS] invalid IN EP %d: not configured\n", ep_id);
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_CNAK_Msk);
            __explicit_fallthrough
        case USBOTG_FS_EP_DIR_OUT:
            log_printf("[USBOTG][FS] CNAK on OUT ep %d\n", ep_id);
            if (ep_id >= USBOTGFS_MAX_OUT_EP) {
                log_printf("[USBOTG][FS] invalid OUT EP %d\n", ep_id);
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->out_eps[ep_id].configured == false) {
                log_printf("[USBOTG][FS] invalid OUT EP %d: not configured\n", ep_id);
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_CNAK_Msk);
            break;
        default:
            log_printf("[USBOTG][FS] CNAK: invalid direction for ep %d\n", ep_id);
            errcode = MBED_ERROR_INVPARAM;
            goto err;
    }
err:
    return errcode;
}


/*
 * Clear the global STALL mode for the device
 */
mbed_error_t usbotgfs_global_stall_clear(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    return errcode;
}


/*
 * Set the STALL mode for the given EP. This mode has priority on the global STALL mode
 */
mbed_error_t usbotgfs_endpoint_stall(uint8_t ep_id, usbotgfs_ep_dir_t dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    uint32_t count = 0;
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    switch (dir) {
        case USBOTG_FS_EP_DIR_IN:
            if (ep_id >= USBOTGFS_MAX_IN_EP) {
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->in_eps[ep_id].configured == false) {
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            /* wait for end of current transmission */
            while (get_reg_value(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_EPENA_Msk, USBOTG_FS_DIEPCTL_EPENA_Pos))  {
                if (++count > USBOTGFS_REG_CHECK_TIMEOUT){
                    log_printf("[USBOTG][FS] HANG! DIEPCTL:EPENA\n");
                    errcode = MBED_ERROR_BUSY;
                    goto err;
                }

                continue; //FIXME TIMEOUT
            }
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_EPDIS_Msk);
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id), USBOTG_FS_DIEPCTL_STALL_Msk);
            break;
        case USBOTG_FS_EP_DIR_OUT:
            if (ep_id >= USBOTGFS_MAX_OUT_EP) {
                errcode = MBED_ERROR_INVPARAM;
                goto err;
            }
            if (ctx->out_eps[ep_id].configured == false) {
                errcode = MBED_ERROR_INVSTATE;
                goto err;
            }
            /* wait for end of current transmission */
            while (get_reg_value(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_EPENA_Msk, USBOTG_FS_DOEPCTL_EPENA_Pos))  {
                if (++count > USBOTGFS_REG_CHECK_TIMEOUT){
                    log_printf("[USBOTG][FS] HANG! DIEPCTL:EPENA\n");
                    errcode = MBED_ERROR_BUSY;
                    goto err;
                }
            }
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_EPDIS_Msk);
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_STALL_Msk);
            break;
        default:
            errcode = MBED_ERROR_INVPARAM;
            goto err;
    }

err:
    return errcode;
}

/*
 * Clear the STALL mode for the given EP
 */
mbed_error_t usbotgfs_endpoint_stall_clear(uint8_t ep, usbotgfs_ep_dir_t dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    ep = ep;
    dir = dir;
    return errcode;
}

/*
 * Activate EP (for e.g. before sending data). It can also be used in order to
 * configure a new endpoint with the given configuration (type, mode, data toggle,
 * FIFO informations)
 */
mbed_error_t usbotgfs_configure_endpoint(uint8_t                 ep,
                                         usbotgfs_ep_type_t      type,
                                         usbotgfs_ep_dir_t       dir,
                                         usbotgfs_epx_mpsize_t   mpsize,
                                         usbotgfs_ep_toggle_t    dtoggle,
                                         usbotgfs_ioep_handler_t handler)
 {
     mbed_error_t errcode = MBED_ERROR_NONE;
     log_printf("[USBOTGFS] configure EP %d: dir %d, mpsize %d, type %x\n", ep, dir, mpsize, type);
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    /* truncating to max 64 bytes whatever mpsize is */
    uint32_t local_mpsize = 64;
    if (mpsize < 64) {
        local_mpsize = mpsize;
    }

    switch (dir) {
        case USBOTG_FS_EP_DIR_IN:
            log_printf("[USBOTGFS] enable EP %d: dir IN, mpsize %d, type %x\n", ep, local_mpsize, type);

            if (ep >= USBOTGFS_MAX_IN_EP) {
                errcode = MBED_ERROR_NOSTORAGE;
                goto err;
            }
            ctx->in_eps[ep].id = ep;
            ctx->in_eps[ep].dir = dir;
            ctx->in_eps[ep].configured = true;
            ctx->in_eps[ep].mpsize = local_mpsize;
            ctx->in_eps[ep].type = type;
            ctx->in_eps[ep].state = USBOTG_FS_EP_STATE_IDLE;
            ctx->in_eps[ep].handler = handler;
            if (ep <= USBOTGFS_MAX_OUT_EP) {
                ctx->out_eps[ep].configured = false;
            }

            /* set EP configuration */
            set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep), type,
                    USBOTG_FS_DIEPCTL_EPTYP_Msk,
                    USBOTG_FS_DIEPCTL_EPTYP_Pos);

            set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep), local_mpsize,
                          USBOTG_FS_DIEPCTL_MPSIZ_Msk(ep),
                          USBOTG_FS_DIEPCTL_MPSIZ_Pos(ep));

            if (type == USBOTG_FS_EP_TYPE_BULK || type == USBOTG_FS_EP_TYPE_INT) {
                set_reg(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep), dtoggle, USBOTG_FS_DIEPCTL_SD0PID);
            }
            /* set EP FIFO */
            usbotgfs_reset_epx_fifo(&(ctx->in_eps[ep]));

            /* Enable endpoint */
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep), USBOTG_FS_DIEPCTL_USBAEP_Msk);
            set_reg(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep), ep, USBOTG_FS_DIEPCTL_CNAK);
            //set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_IEPINT_Msk);
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DAINTMSK, USBOTG_FS_DAINTMSK_IEPM(ep));
            break;
        case USBOTG_FS_EP_DIR_OUT:
            log_printf("[USBOTGFS] enable EP %d: dir OUT, mpsize %d, type %x\n", ep, local_mpsize, type);

            if (ep >= USBOTGFS_MAX_OUT_EP) {
                errcode = MBED_ERROR_NOSTORAGE;
                goto err;
            }
            ctx->out_eps[ep].id = ep;
            ctx->out_eps[ep].dir = dir;
            ctx->out_eps[ep].configured = true;
            /* FS mode: MPSize = 64 */
            ctx->out_eps[ep].mpsize = local_mpsize;
            ctx->out_eps[ep].type = type;
            ctx->out_eps[ep].state = USBOTG_FS_EP_STATE_IDLE;
            ctx->out_eps[ep].handler = handler;
            if (ep <= USBOTGFS_MAX_IN_EP) {
                ctx->in_eps[ep].configured = false;
            }

            /* Maximum packet size */
            set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep),
                    local_mpsize, USBOTG_FS_DOEPCTL_MPSIZ_Msk(ep),
                    USBOTG_FS_DOEPCTL_MPSIZ_Pos(ep));

            /*  USB active endpoint */
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep), USBOTG_FS_DOEPCTL_USBAEP_Msk);

            /* FIXME Start data toggle */
            if (type == USBOTG_FS_EP_TYPE_BULK || type == USBOTG_FS_EP_TYPE_INT) {
                set_reg(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep), dtoggle, USBOTG_FS_DOEPCTL_SD0PID);
            }

            /* Endpoint type */
            set_reg(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep), type, USBOTG_FS_DOEPCTL_EPTYP);
            /* set EP FIFO */
            usbotgfs_reset_epx_fifo(&(ctx->out_eps[ep]));


            set_reg_bits(r_CORTEX_M_USBOTG_FS_DAINTMSK, USBOTG_FS_DAINTMSK_OEPM(ep));
            break;
    }
err:
    return errcode;
}

/*
 * Dectivate EP.
 * This can be requested on SetConfiguration or SetInterface, when
 * a configuration change is required, which implies that some old EPs need to be
 * removed before creating new ones.
 */
mbed_error_t usbotgfs_deconfigure_endpoint(uint8_t ep)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }

    clear_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_NPTXFEM_Msk | USBOTG_FS_GINTMSK_RXFLVLM_Msk);
    if (ctx->in_eps[ep].configured == true) {
        clear_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep),
                USBOTG_FS_DIEPCTL_EPENA_Msk);
    }
    if (ctx->out_eps[ep].configured == true) {
        clear_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep),
                USBOTG_FS_DOEPCTL_EPENA_Msk);
    }

    set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_NPTXFEM_Msk | USBOTG_FS_GINTMSK_RXFLVLM_Msk);

err:
    return errcode;
}


/*
 * Configure given EP with given params
 * This function should be called on Set_Configuration & Set_Interface requests,
 * in compliance with the currently enabled configuration and interface(s)
 * hold by the libUSBCtrl
 */
mbed_error_t usbotgfs_activate_endpoint(uint8_t               ep_id,
                                        usbotgfs_ep_dir_t     dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    if (dir == USBOTG_FS_EP_DIR_IN) {
        set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id),
                USBOTG_FS_DIEPCTL_EPENA_Msk);
    } else {
        set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_CNAK_Msk);
        set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id),
                  USBOTG_FS_DOEPCTL_EPENA_Msk);
    }
err:
    return errcode;
}

mbed_error_t usbotgfs_deactivate_endpoint(uint8_t ep,
                                          usbotgfs_ep_dir_t     dir)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    /* sanitize */
    if (ctx == NULL) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }

    clear_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_NPTXFEM_Msk | USBOTG_FS_GINTMSK_RXFLVLM_Msk);
    if (dir == USBOTG_FS_EP_DIR_IN) {
        clear_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep),
                USBOTG_FS_DIEPCTL_EPENA_Msk);
    } else {
        clear_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep),
                USBOTG_FS_DOEPCTL_EPENA_Msk);
    }
    set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK, USBOTG_FS_GINTMSK_NPTXFEM_Msk | USBOTG_FS_GINTMSK_RXFLVLM_Msk);

err:
    return errcode;
}

/*
 * Force EP to stop transmit (IN EP) or receive (OUT EP)
 */
mbed_error_t usbotgfs_enpoint_nak(uint8_t ep)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    ep = ep;
    return errcode;
}

/*
 * Leave the NAK (freezed) mode for given EP
 */
mbed_error_t usbotgfs_enpoint_nak_clear(uint8_t ep)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    ep = ep;
    return errcode;
}

void usbotgfs_set_address(uint16_t addr)
{
    set_reg(r_CORTEX_M_USBOTG_FS_DCFG, addr, USBOTG_FS_DCFG_DAD);
}

usbotgfs_ep_state_t usbotgfs_get_ep_state(uint8_t epnum, usbotgfs_ep_dir_t dir)
{
    if (dir == USBOTG_FS_EP_DIR_IN && epnum >= USBOTGFS_MAX_IN_EP) {
        return USBOTG_FS_EP_STATE_INVALID;
    }
    if (dir == USBOTG_FS_EP_DIR_OUT && epnum >= USBOTGFS_MAX_OUT_EP) {
        return USBOTG_FS_EP_STATE_INVALID;
    }
    switch (dir) {
        case USBOTG_FS_EP_DIR_IN:
            return usbotgfs_ctx.in_eps[epnum].state;
            break;
        case USBOTG_FS_EP_DIR_OUT:
            return usbotgfs_ctx.out_eps[epnum].state;
            break;
        default:
            return USBOTG_FS_EP_STATE_INVALID;
            break;
    }
    return USBOTG_FS_EP_STATE_INVALID;
}

usbotgfs_port_speed_t usbotgfs_get_speed(void)
{
    return USBOTG_FS_PORT_FULLSPEED;
}

/*
 * About generic part:
 * This part translate libusbctrl forward-declaration symbols to local symbols.
 * This permits to resolve all symbols of the libctrl abstraction layer into this
 * very driver one.
 * WARNING: this method has one single restriction: only one driver can be used
 * at a time by a given ELF binary (i.e. an application), as symbols are resolved
 * at link time.
 */
mbed_error_t usb_backend_drv_configure(usb_backend_drv_mode_t mode,
                                       usb_backend_drv_ioep_handler_t ieph,
                                       usb_backend_drv_ioep_handler_t oeph)
    __attribute__ ((alias("usbotgfs_configure")));

mbed_error_t usb_backend_drv_declare(void)
    __attribute__ ((alias("usbotgfs_declare")));
mbed_error_t usb_backend_drv_activate_endpoint(uint8_t               id,
                                         usb_backend_drv_ep_dir_t     dir)
    __attribute__ ((alias("usbotgfs_activate_endpoint")));
mbed_error_t usb_backend_drv_configure_endpoint(uint8_t               ep,
                                         usb_backend_drv_ep_type_t    type,
                                         usb_backend_drv_ep_dir_t     dir,
                                         usb_backend_drv_epx_mpsize_t mpsize,
                                         usb_backend_drv_ep_toggle_t  dtoggle,
                                         usb_backend_drv_ioep_handler_t handler)
    __attribute__ ((alias("usbotgfs_configure_endpoint")));

mbed_error_t usb_backend_drv_get_ep_state(uint8_t epnum, usb_backend_drv_ep_dir_t dir)
    __attribute__ ((alias("usbotgfs_get_ep_state")));
mbed_error_t usb_backend_drv_send_data(uint8_t *src, uint32_t size, uint8_t ep)
    __attribute__ ((alias("usbotgfs_send_data")));
mbed_error_t usb_backend_drv_send_zlp(uint8_t ep)
    __attribute__ ((alias("usbotgfs_send_zlp")));
void         usb_backend_drv_set_address(uint16_t addr)
    __attribute__ ((alias("usbotgfs_set_address")));
/* USB protocol standard handshaking */
mbed_error_t usb_backend_drv_ack(uint8_t ep_id, usb_backend_drv_ep_dir_t dir)
    __attribute__ ((alias("usbotgfs_endpoint_clear_nak")));
mbed_error_t usb_backend_drv_nak(uint8_t ep_id, usb_backend_drv_ep_dir_t dir)
    __attribute__ ((alias("usbotgfs_endpoint_set_nak")));
mbed_error_t usb_backend_drv_stall(uint8_t ep_id, usb_backend_drv_ep_dir_t dir)
    __attribute__ ((alias("usbotgfs_endpoint_stall")));

uint32_t usb_backend_get_ep_mpsize(void) __attribute__ ((alias("usbotgfs_get_ep_mpsize")));
usb_backend_drv_port_speed_t usb_backend_drv_get_speed(void) __attribute__ ((alias("usbotgfs_get_speed")));

