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
#include "libc/sanhandlers.h"

#include "api/libusbotgfs.h"
#include "usbotgfs_regs.h"
#include "usbotgfs.h"

#include "generated/usb_otg_fs.h"
#include "usbotgfs_fifos.h"
#include "usbotgfs_handler.h"

/************************************************
 * About ISR handlers
 */
/*
 * interrupt flag bit identifier, as set in the INTSTS global interrupt status register
 * Some of these interrupts required an execution of a stack level handler, other not.
 *
 * When an interrupt is handled, all the bits must be checked against the following
 * bitfield definition, as multiple event may occur in the same time, and as a consequence be
 * handled in the same ISR.
 *
 * In all these interrupts, the following must be pushed to the upper layer:
 * - ESUSP (Early suspend, i.e. BUS_INACTIVE transition)
 * - USBRST (USB reset transition)
 * - USBSUSP (USB Suspend, i.e. BUS_INACTIVE transition, starting the USB DEFAULT state)
 * - IEPINT (IN Endpoint event interrupt)
 * - OEPINT (OUT Endpoint event interrupt)
 * - RXFLVL (RxFIFO non-empty interrupt)
 * - WKUPINT (Wakeup event interrupt)
 *
 * The USB OTG HS driver is neither responsible for dispatching received data between
 * interfaces, nor responsible for parsing setup packets or dispatching differencially
 * data or setup packets for various endpoint.
 * O(I)EPINT handlers get back the packet, its size, its EP num and type, and push
 * it back to the control plane for parsing/dispatching.
 */
typedef enum {
    USBOTGFS_IT_CMOD       = 0,    /*< Current Mode of Operation  */
    USBOTGFS_IT_MMIS       = 1,    /*< Mode mismatch */
    USBOTGFS_IT_OTGINT     = 2,    /*< OTG interrupt */
    USBOTGFS_IT_SOF        = 3,    /*< Start of Frame */
    USBOTGFS_IT_RXFLVL     = 4,    /*< RxFifo non-empty */
    USBOTGFS_IT_NPTXE      = 5,    /*< Non-periodic TxFIFO empty (Host mode) */
    USBOTGFS_IT_GINAKEFF   = 6,    /*< Global IN NAK effective */
    USBOTGFS_IT_GONAKEFF   = 7,    /*< Global OUT NAK effective*/
    USBOTGFS_IT_RESERVED8  = 8,    /*< Reserved */
    USBOTGFS_IT_RESERVED9  = 9,    /*< Reserved */
    USBOTGFS_IT_ESUSP      = 10,   /*< Early suspend */
    USBOTGFS_IT_USBSUSP    = 11,   /*< USB Suspend */
    USBOTGFS_IT_USBRST     = 12,   /*< Reset */
    USBOTGFS_IT_ENUMDNE    = 13,   /*< Speed enumeration done */
    USBOTGFS_IT_ISOODRP    = 14,   /*< Isochronous OUT pkt dropped */
    USBOTGFS_IT_EOPF       = 15,   /*< End of periodic frame */
    USBOTGFS_IT_RESERVED16 = 16,   /*< Reserved */
    USBOTGFS_IT_EPMISM     = 17,   /*< Endpoint mismatch */
    USBOTGFS_IT_IEPINT     = 18,   /*< IN Endpoint event */
    USBOTGFS_IT_OEPINT     = 19,   /*< OUT Endpoint event */
    USBOTGFS_IT_IISOIXFR   = 20,   /*< Incomplete Isochronous IN transfer */
    USBOTGFS_IT_IPXFR      = 21,   /*< Incomplete periodic transfer */
    USBOTGFS_IT_RESERVED22 = 22,   /*< Reserved */
    USBOTGFS_IT_RESERVED23 = 23,   /*< Reserved */
    USBOTGFS_IT_HPRTINT    = 24,   /*< Host port event (Host mode) */
    USBOTGFS_IT_HCINTT     = 25,   /*< Host channels event (Host mode) */
    USBOTGFS_IT_PTXFE      = 26,   /*< Periodic TxFIFO empty (Host mode) */
    USBOTGFS_IT_RESERVED27 = 27,   /*< Reserved */
    USBOTGFS_IT_CIDSCHG    = 28,   /*< Connector ID status change */
    USBOTGFS_IT_DISCINT    = 29,   /*< Disconnect event (Host mode) */
    USBOTGFS_IT_SRQINT     = 30,   /*< Session request/new session event*/
    USBOTGFS_IT_WKUPINT    = 31,   /*< Resume/Wakeup event */
} usbotgfs_int_id_t;


#if CONFIG_USR_DRV_USBOTGFS_DEBUG

static volatile uint32_t    usbotgfs_int_cnt[32] = { 0 };
/* This table is made only as a debug helper, in order to assicate the
 * interrupt identifier with a full-text name for pretty printing.
 * This consume space in the device flash memory and should be used only
 * for debug purpose.
 */
#endif
/*
 * Generic handler, used by default.
 */
static mbed_error_t default_handler(void)
{
    return MBED_ERROR_NONE;
}

/*
 * Reserved handler, should never be executed, as interrupt flag corresponds to
 * 'Reserved' field.
 */
static mbed_error_t reserved_handler(void)
{
    return MBED_ERROR_UNSUPORTED_CMD;
}

/*
 * USB Reset handler. Handling USB reset requests. These requests can be received in
 * various state and handle a core soft reset and configuration.
 *
 * 1. Set the NAK bit for all OUT endpoints:
 *  - DOEPCTLn.SNAK = 1 (for all OUT endpoints)
 * Unmask the following interrupt bits:
 *  - DAINTMSK.INEP0 = 1 (control 0 IN endpoint)
 *  - DAINTMSK.OUTEP0 = 1 (control 0 OUT endpoint)
 *  - DOEPMSK.SETUP = 1
 *  - DOEPMSK.XferCompl = 1
 *  - DIEPMSK.XferCompl = 1
 *  - DIEPMSK.TimeOut = 1
 *
 *  3. To transmit or receive data, the device must initialize more registers as
 *  specified in Device Initialization on page 154.
 *
 *  4. Set up the Data FIFO RAM for each of the FIFOs (only if Dynamic FIFO Sizing is
 *  enabled)
 *  - Program the GRXFSIZ Register, to be able to receive control OUT data and setup
 *    data. If thresholding is not enabled, at a minimum, this must be equal to 1 max
 *    packet size of control endpoint 0 + 2 DWORDs (for the status of the control OUT
 *    data packet) + 10 DWORDs (for setup packets). If thresholding is enabled, at a
 *    minimum, this must be equal to 2 * (DTHRCTL.RxThrLen/4 + 1) + 2 DWORDs (for the
 *    status of the control OUT data packet) + 10 DWORDs (for setup packets)
 *  - Program the GNPTXFSIZ Register in Shared FIFO operation or dedicated FIFO size
 *    register (depending on the FIFO number chosen) in Dedicated FIFO operation, to
 *    be able to transmit control IN data. At a minimum, this must be equal to 1 max
 *    packet size of control endpoint 0.If thresholding is enabled, this can be
 *    programmed to less than one max packet size.
 *
 * 5. Reset the Device Address field in Device Configuration Register (DCFG).
 * 6. Program the following fields in the endpoint-specific registers for control OUT
 *    endpoint 0 to receive a SETUP packet
 *  - DOEPTSIZ0.SetUP Count = 3 (to receive up to 3 back-to-back SETUP packets)
 *
 *    At this point, all initialization required to receive SETUP packets is done
 */
static mbed_error_t reset_handler(void)
{
    log_printf("[USB FS][RESET] received USB Reset\n");
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    for (uint8_t i = 0; i < USBOTGFS_MAX_OUT_EP; ++i) {
        /* if Out EPi is configured, set DOEPCTLi.SNAK to 1 */
        if (ctx->out_eps[i].configured) {
            log_printf("[USB FS][RESET] activate SNAK for out_ep %d\n", i);
            set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(i),
                         USBOTG_FS_DOEPCTL_CNAK_Msk);
        }
    }
    /* unmask control pipe requested interrupt bits:
     * activate OEPInt, IEPInt & RxFIFO non-empty.
     * Ready to receive requests on EP0.
     */
	set_reg_bits(r_CORTEX_M_USBOTG_FS_GINTMSK,
                 USBOTG_FS_GINTMSK_OEPINT_Msk   |
                 USBOTG_FS_GINTMSK_IEPINT_Msk   |
				 USBOTG_FS_GINTMSK_RXFLVLM_Msk);
    /* unmask control 0 IN & OUT endpoint interrupts */
	set_reg_bits(r_CORTEX_M_USBOTG_FS_DAINTMSK,
                 USBOTG_FS_DAINTMSK_IEPM(0)   |
                 USBOTG_FS_DAINTMSK_OEPM(0));
    /* unmask global setup mask, xfr completion, timeout, for in and out */
	set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPMSK,
                 USBOTG_FS_DOEPMSK_STUPM_Msk   |
                 USBOTG_FS_DOEPMSK_XFRCM_Msk);
	set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPMSK,
                 USBOTG_FS_DIEPMSK_XFRCM_Msk |
                 USBOTG_FS_DIEPMSK_TOM_Msk);

    log_printf("[USB FS][RESET] initialize global fifo\n");
    if ((errcode = usbotgfs_init_global_fifo()) != MBED_ERROR_NONE) {
        goto err;
    }

    log_printf("[USB FS][RESET] initialize EP0 fifo\n");
    /* fifo is RESET, in both Core registers and EP context. The FIFO will need
     * to be reconfigured later by the driver API (typically through upper
     * reset handler */
# if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /* set TxFIFO for EP0 (in_eps[0]) */
        log_printf("[USB FS][RESET] initialize EP0 TxFIFO in device mode\n");
        if ((errcode = usbotgfs_reset_epx_fifo(&(ctx->in_eps[0]))) != MBED_ERROR_NONE) {
            goto err;
        }
#else
        /* set TxFIFO for EP0 (out_eps[0]) */
        log_printf("[USB FS][RESET] initialize EP0 TxFIFO in host mode\n");
        if ((errcode = usbotgfs_reset_epx_fifo(&(ctx->out_eps[0]))) != MBED_ERROR_NONE) {
            goto err;
        }
#endif
    /* flushing FIFOs */
    usbotgfs_txfifo_flush(0);
    usbotgfs_rxfifo_flush(0);

    log_printf("[USB FS][RESET] set EP0 as configured\n");
    /* Now EP0 is configued. Set this information in the driver context */
    ctx->in_eps[0].configured = true;
    ctx->out_eps[0].configured = true;

    /* execute upper layer (USB Control plane) reset handler. This
     * function should always reconfigure the FIFO structure */
    log_printf("[USB FS][RESET] call usb ctrl plane reset\n");
    usbctrl_handle_reset(usb_otg_fs_dev_infos.id);

    /* now that USB full stack execution is done, Enable Endpoint.
     * From now on, data can be received or sent on Endpoint 0 */
# if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        log_printf("[USB FS][RESET] enable EP0 out (reception)\n");
        set_reg(r_CORTEX_M_USBOTG_FS_DOEPCTL(0),
                1, USBOTG_FS_DOEPCTL_EPENA);
#else
        log_printf("[USB FS][RESET] host mode TODO\n");
#endif
err:
    return errcode;
}

/*
 * Enumeration done interrupt handler
 */
static mbed_error_t enumdone_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

	/* 1. read the OTG_HS_DSTS register to determine the enumeration speed. */
	uint8_t speed = get_reg(r_CORTEX_M_USBOTG_FS_DSTS, USBOTG_FS_DSTS_ENUMSPD);

	if (speed == USBOTG_FS_DSTS_ENUMSPD_FS) {
		log_printf("[USB FS][ENUMDONE] Full speed enumerated !\n");
	} else {
		log_printf("[USB FS][ENUMDONE] invalid speed 0x%x !\n", speed);
        errcode = MBED_ERROR_INITFAIL;
        goto err;
    }

    /* TODO Program the MPSIZ field in OTG_HS_DIEPCTL0 to set the maximum packet size. This
     * step configures control endpoint 0.
     */
    /*
     * INFO: maximum packet size of control EP in FS and HS is not the same:
     * LS: 8 bytes
     * FS: 8, 16, 32 or 64 bytes
     * HS: 64 bytes only
     * Here we use 64 bytes, compliant with both FS and HS.
     * LS is not supported.
     */
	set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPCTL(0),
		      USBOTG_FS_DIEPCTL0_MPSIZ_64BYTES,
		      USBOTG_FS_DIEPCTL_MPSIZ_Msk(0),
		      USBOTG_FS_DIEPCTL_MPSIZ_Pos(0));

err:
    return errcode;
}


/*
 * OUT endpoint event (reception in device mode, transmission in Host mode)
 *
 * In device mode:
 * OEPINT is executed when the RxFIFO has been fully read by the software (in RXFLVL handler)
 * In Host mode:
 * OEPINNT is executed when the TxFIFO has been flushed by the core and content sent
 */
static mbed_error_t oepint_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    uint16_t daint = 0;
    /* get EPx on which the event came */
    daint = (uint16_t)((read_reg_value(r_CORTEX_M_USBOTG_FS_DAINT) >> 16) & 0xff);
    /* checking current mode */
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /* here, this is a 'data received' interrupt */
        uint16_t val = 0x1;
        uint8_t ep_id = 0;
        log_printf("[USBOTG][HS] handling received data\n");
        for (int i = 0; daint; i++) {
            if (daint & val) {
                log_printf("[USBOTG][HS] received data on ep %d\n", ep_id);
                /* calling upper handler */
                uint32_t doepint = read_reg_value(r_CORTEX_M_USBOTG_FS_DOEPINT(ep_id));
                bool callback_to_call = false;
                bool end_of_transfer = false;
                if (doepint & USBOTG_FS_DOEPINT_STUP_Msk) {
                    log_printf("[USBOTG][HS] oepint: entering STUP\n");
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPINT(ep_id), USBOTG_FS_DOEPINT_STUP_Msk);
                    callback_to_call = true;
                }
                /* Bit 0 XFRC: Data received complete */
                if (doepint & USBOTG_FS_DOEPINT_XFRC_Msk) {
                    log_printf("[USBOTG][HS] oepint: entering XFRC\n");
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPINT(ep_id), USBOTG_FS_DOEPINT_XFRC_Msk);
                    if (ctx->out_eps[ep_id].fifo_idx == 0) {
                        /* ZLP transfer initialited from the HOST */
                        continue;
                    }
                    end_of_transfer = true;
                    /* Here we set SNAK bit to avoid receiving data before the next read cmd config.
                     * If not, a race condition can happen, if RXFLVL handler is executed *before* the EP
                     * RxFIFO is set by the upper layer */
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_SNAK_Msk);
                    /* XXX: defragmentation need to be checked for others (not EP0) EPs */
                    if (ctx->out_eps[ep_id].fifo_idx < ctx->out_eps[ep_id].fifo_size) {
                        /* handle defragmentation for DATA OUT packets on EP0 */
                        log_printf("[USBOTG][HS] fragment pkt %d total, %d read\n", ctx->out_eps[ep_id].fifo_size, ctx->out_eps[ep_id].fifo_idx);
                        set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_CNAK_Msk);
                    } else {
                        log_printf("[USBOTG][HS] oepint for %d data size read\n", ctx->out_eps[ep_id].fifo_idx);
                        callback_to_call = true;
                    }
                }
                if (callback_to_call == true) {
                    log_printf("[USBOTG][HS] oepint: calling callback\n");
                    if (handler_sanity_check_with_panic((physaddr_t)ctx->out_eps[ep_id].handler)) {
                        goto err;
                    }
                    errcode = ctx->out_eps[ep_id].handler(usb_otg_fs_dev_infos.id, ctx->out_eps[ep_id].fifo_idx, ep_id);
                    ctx->out_eps[ep_id].fifo_idx = 0;
                    if (end_of_transfer == true && ep_id == 0) {
                        /* We synchronously handle CNAK only for EP0 data. others EP are handled by dedicated upper layer
                         * class level handlers */
                        log_printf("[USBOTG][HS] oepint: set CNAK (end of transfer)\n");
                        set_reg_bits(r_CORTEX_M_USBOTG_FS_DOEPCTL(ep_id), USBOTG_FS_DOEPCTL_CNAK_Msk);
                    }
                }
                /* XXX: only if SNAK set */
                /* now that data has been handled, consider FIFO as empty */
                ctx->out_eps[ep_id].state = USBOTG_FS_EP_STATE_IDLE;
            }
            ep_id++;
            daint >>= 1;
        }
        set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_OEPINT);
err:
#else
        /* TODO: (FIXME host mode not working yet) here, this is a 'end of transmission' interrupt. Let's handle each
         * endpoint for which the interrupt rised */
        uint16_t val = 0x1;
        uint8_t ep_id = 0;
        for (uint8_t i = 0; i < 16; ++i) {
            if (daint & val) {
                /* an iepint for this EP is active */
                log_printf("[USBOTG][HS] iepint: ep %d\n", ep_id);
                /* now that transmit is complete, set ep state as IDLE */
                /* calling upper handler, transmitted size read from DOEPSTS */
                errcode = usbctrl_handle_outepevent(usb_otg_fs_dev_infos.id, ctx->out_eps[ep_id].fifo_idx, ep_id);
                ctx->out_eps[ep_id].state = USBOTG_FS_EP_STATE_IDLE;
            }
            ep_id++;
            val = val << 1;
        }
#endif
    return errcode;
}

/*
 * IN endpoint event (transmission in device mode, reception in Host mode)
 *
 * In device mode:
 * IEPINNT is executed when the TxFIFO has been flushed by the core and content sent
 * In Host mode:
 * IEPINT is executed when the RxFIFO has been fully read by the software (in RXFLVL handler)
 */
static mbed_error_t iepint_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    uint16_t daint = 0;
    uint32_t diepintx = 0;
    /* get EPx on which the event came */
    daint = (uint16_t)(read_reg_value(r_CORTEX_M_USBOTG_FS_DAINT) & 0xff);
    /* checking current mode */
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /*
         * An event rose for one or more IN EPs.
         * First, for each EP, we handle driver level events (NAK, errors, etc.)
         * if there is upper layer that need to be executed, we handle it.
         */
        uint16_t val = 0x1;
        uint8_t ep_id = 0;
        for (int i = 0; daint; i++) {
            if (daint & val) {
                /* an iepint for this EP is active */
                log_printf("[USBOTG][HS] iepint: ep %d\n", ep_id);
                /*
                 * Get back DIEPINTx for this EP
                 */
                diepintx = read_reg_value(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id));

                /* Bit 7 TXFE: Transmit FIFO empty */
                if (diepintx & USBOTG_FS_DIEPINT_TOC_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_TXFE_Msk);
                    ctx->in_eps[ep_id].core_txfifo_empty = true;
                    log_printf("[USBOTG][HS] iepint: ep %d: TxFifo empty\n", ep_id);
                }

                /* Bit 6 INEPNE: IN endpoint NAK effective */
                if (diepintx & USBOTG_FS_DIEPINT_INEPNE_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_INEPNE_Msk);
                    log_printf("[USBOTG][HS] iepint: ep %d: NAK effective\n", ep_id);
                }

                /* Bit 4 ITTXFE: IN token received when TxFIFO is empty */
                if (diepintx & USBOTG_FS_DIEPINT_ITTXFE_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_ITTXFE_Msk);
                    log_printf("[USBOTG][HS] iepint: ep %d: token rcv when fifo empty\n", ep_id);
                }

                /* Bit 3 TOC: Timeout condition */
                if (diepintx & USBOTG_FS_DIEPINT_TOC_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_TOC_Msk);
                    log_printf("[USBOTG][HS] iepint: ep %d: timeout cond\n", ep_id);
                }

                /* bit 1 EPDISD: Endpoint disabled interrupt */
                if (diepintx & USBOTG_FS_DIEPINT_EPDISD_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_EPDISD_Msk);
                    log_printf("[USBOTG][HS] iepint: ep %d: EP disabled\n", ep_id);
                    /* Now the endpiont is really disabled
                     * We should update enpoint status
                     */
                }

                /* Bit 0 XFRC: Transfer completed interrupt */
                if (diepintx & USBOTG_FS_DIEPINT_XFRC_Msk) {
                    set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPINT(ep_id), USBOTG_FS_DIEPINT_XFRC_Msk);

                    log_printf("[USBOTG][HS] iepint: ep %d: transfert completed\n", ep_id);

                    /* inform upper layer only on end of effetvive transfer. A transfer may be
                     * the consequence of multiple FIFO flush, depending on the transfer size and
                     * the FIFO size */
                    if (ctx->in_eps[ep_id].state == USBOTG_FS_EP_STATE_DATA_IN) {
                        if (ctx->in_eps[ep_id].fifo_idx < ctx->in_eps[ep_id].fifo_size) {

                            log_printf("[USBOTG][HS] iepint: ep %d: still in fragmented transfer (%d on %d), continue...\n", ep_id, ctx->in_eps[ep_id].fifo_idx, ctx->in_eps[ep_id].fifo_size);
                            /* still in fragmentation transfer. We need to start a new
                             * transmission of the bigger size between mpsize and residual size
                             * in order to finish the current transfer. The EP state is untouched */
                            /* 1. Configure the endpoint to specify the amount of data to send, at
                             * most MPSize */
                            uint32_t datasize = ctx->in_eps[ep_id].fifo_size - ctx->in_eps[ep_id].fifo_idx;
                            if (datasize > ctx->in_eps[ep_id].mpsize) {
                                datasize = ctx->in_eps[ep_id].mpsize;
                            }
                            set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                                    1,
                                    USBOTG_FS_DIEPTSIZ_PKTCNT_Msk(ep_id),
                                    USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(ep_id));
                            set_reg_value(r_CORTEX_M_USBOTG_FS_DIEPTSIZ(ep_id),
                                    datasize,
                                    USBOTG_FS_DIEPTSIZ_XFRSIZ_Msk(ep_id),
                                    USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(ep_id));
                            set_reg_bits(r_CORTEX_M_USBOTG_FS_DIEPCTL(ep_id),
                                    USBOTG_FS_DIEPCTL_CNAK_Msk | USBOTG_FS_DIEPCTL_EPENA_Msk);
                            /* 2. write data to fifo */
                            usbotgfs_write_epx_fifo(ctx->in_eps[ep_id].mpsize, &(ctx->in_eps[ep_id]));
                        } else {
                            /* now EP is idle */
                            ctx->in_eps[ep_id].state = USBOTG_FS_EP_STATE_IDLE;
                            /* inform libctrl of transfert complete */
                            if (handler_sanity_check_with_panic((physaddr_t)ctx->in_eps[ep_id].handler)) {
                                goto err;
                            }
                            errcode = ctx->in_eps[ep_id].handler(usb_otg_fs_dev_infos.id, ctx->in_eps[ep_id].fifo_idx, ep_id);
                            ctx->in_eps[ep_id].fifo = 0;
                            ctx->in_eps[ep_id].fifo_idx = 0;
                            ctx->in_eps[ep_id].fifo_size = 0;
                        }
                    } else {
                        /* the EP is only set as IDLE to inform the send process
                         * that the FIFO content is effectively sent */
                        /* clear current FIFO, now that content is sent */
                        ctx->in_eps[ep_id].state = USBOTG_FS_EP_STATE_IDLE;
                    }
                }
#if 0
                /* handle various events of the current IN EP */
                /* TxFIFO (half) empty ? */
                if (diepintx & USBOTG_FS_DIEPINT_TXFE_Msk) {
                    /* set TxFIFO has (half) empty */
                    ctx->in_eps[ep_id].core_txfifo_empty = true;
#if CONFIG_USR_DEV_USBOTGFS_TRIGER_XMIT_ON_HALF
                    ctx->in_eps[ep_id].state = USBOTG_FS_EP_STATE_DATA_OUT
#endif
                }
#endif
                /* now that transmit is complete, set ep state as IDLE */
                /* calling upper handler, transmitted size read from DIEPSTS */
            }
            ep_id++;
            daint >>= 1;
        }
        set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_IEPINT);
err:
#else
        /* here, this is a 'data received' interrupt  (Host mode) */
        uint16_t val = 0x1;
        uint8_t ep_id = 0;
        for (uint8_t i = 0; i < 16; ++i) {
            if (daint & val) {
                /* an iepint for this EP is active */
                log_printf("[USBOTG][HS] iepint: ep %d\n", ep_id);
                /* calling upper handler */
                errcode = usbctrl_handle_outepevent(usb_otg_fs_dev_infos.id, ctx->in_eps[ep_id].fifo_idx, ep_id);
                ctx->in_eps[ep_id].state = USBOTG_FS_EP_STATE_IDLE;
            }
            ep_id++;
            val = val << 1;
        }
#endif
    /* calling upper handler... needed ? */
    return errcode;
}

/*
 * RXFLV handler, This interrupt is executed when the core as written a complete packet in the RxFIFO.
 */
static mbed_error_t rxflvl_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
	uint32_t grxstsp;
	pkt_status_t pktsts;
	data_pid_t dpid;
	uint16_t bcnt;
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
	uint8_t epnum = 0; /* device case */
#else
	uint8_t chnum = 0; /* host case */
#endif
	uint32_t size;
    usbotgfs_context_t *ctx;


   	/* 1. Mask the RXFLVL interrupt (in OTG_HS_GINTSTS) by writing to RXFLVL = 0
     * (in OTG_HS_GINTMSK),  until it has read the packet from the receive FIFO
     */
	set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 0, USBOTG_FS_GINTMSK_RXFLVLM);

 	/* 2. Read the Receive status pop register */
    grxstsp = read_reg_value(r_CORTEX_M_USBOTG_FS_GRXSTSP);

    ctx = usbotgfs_get_context();
    log_printf("[USBOTG][HS] Rxflvl handler\n");

    /* what is our mode (Host or Dev) ? Set corresponding variables */
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
            pktsts.devsts = USBOTG_FS_GRXSTSP_GET_STATUS(grxstsp);
            epnum = USBOTG_FS_GRXSTSP_GET_EPNUM(grxstsp);
#else
            pktsts.hoststs = USBOTG_FS_GRXSTSP_GET_STATUS(grxstsp);
            chnum = USBOTG_FS_GRXSTSP_GET_CHNUM(grxstsp);
#endif
	dpid = USBOTG_FS_GRXSTSP_GET_DPID(grxstsp);
	bcnt =  USBOTG_FS_GRXSTSP_GET_BCNT(grxstsp);
	size = 0;

#if CONFIG_USR_DRV_USBOTGFS_DEBUG
# if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        log_printf("EP:%d, PKTSTS:%x, BYTES_COUNT:%x,  DATA_PID:%x\n", epnum, pktsts.devsts, bcnt, dpid);
# else
        log_printf("CH:%d, PKTSTS:%x, BYTES_COUNT:%x,  DATA_PID:%x\n", chnum, pktsts.hoststs, bcnt, dpid);
# endif
#endif
    /* 3. If the received packet’s byte count is not 0, the byte count amount of data
     * is popped from the receive Data FIFO and stored in memory. If the received packet
     * byte count is 0, no data is popped from the receive data FIFO.
     *
     *   /!\ Reading an empty receive FIFO can result in undefined core behavior.
     */
# if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /* 4. The receive FIFO’s packet status readout indicates one of the following: */
        switch (pktsts.devsts) {
            case PKT_STATUS_GLOBAL_OUT_NAK:
                {
                    if (epnum != 0) {
                        errcode = MBED_ERROR_UNSUPORTED_CMD;
                        goto err;
                    }
                    log_printf("[USB FS][RXFLVL] EP%d Global OUT NAK effective\n", epnum);
                    ctx->gonak_active = true;
                    ctx->out_eps[epnum].state = USBOTG_FS_EP_STATE_IDLE;
                    break;
                }
            case PKT_STATUS_OUT_DATA_PKT_RECV:
                {
                    log_printf("[USB FS][RXFLVL] EP%d OUT Data PKT (size %d) Recv\n", epnum, bcnt);
                    if (ctx->out_eps[epnum].configured != true)
                    {
                        log_printf("[USB FS][RXFLVL] EP%d OUT Data PKT on invalid EP!\n", epnum);
                        /* to clear RXFLVL IT, we must read from FIFO. read to garbage here */
                        if (bcnt > 0) {
                            usbotgfs_rxfifo_flush(epnum);
                        }
                        errcode = MBED_ERROR_INVSTATE;
                        goto err;
                    }
                    if (ctx->out_eps[epnum].state == USBOTG_FS_EP_STATE_SETUP) {
                        /* associated oepint not yet executed, return NYET to host */
                        log_printf("[RXFLVL] recv DATA while in STUP mode!\n");
                        if (bcnt > 0) {
                            usbotgfs_rxfifo_flush(epnum);
                        }
                        usbotgfs_endpoint_set_nak(epnum, USBOTG_FS_EP_DIR_OUT);
                        errcode = MBED_ERROR_INVSTATE;
                        goto err;
                    }
                    if (bcnt == 0) {
                        goto err;
                    }
                    log_printf("[USB FS][RXFLVL] EP%d OUT Data PKT (size %d) Read EPx FIFO\n", epnum, bcnt);
                    if (usbotgfs_read_epx_fifo(bcnt, &(ctx->out_eps[epnum])) != MBED_ERROR_NONE) {
                        /* empty fifo on error */
                        usbotgfs_rxfifo_flush(epnum);
                        usbotgfs_endpoint_set_nak(epnum, USBOTG_FS_EP_DIR_OUT);
                    }
                    ctx->out_eps[epnum].state = USBOTG_FS_EP_STATE_DATA_OUT_WIP;
                    if (epnum == 0) {
                        if (ctx->out_eps[epnum].fifo_idx < ctx->out_eps[epnum].fifo_size) {
                            /* rise oepint to permit refragmentation at oepint layer */
                            set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epnum), 1, USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(epnum), USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(epnum));
                            set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epnum), ctx->out_eps[epnum].mpsize, USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(epnum), USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(epnum));
                        }
                    }
                    break;
                }
            case PKT_STATUS_OUT_TRANSFER_COMPLETE:
                {
                    log_printf("[USB FS][RXFLVL] OUT Transfer complete on EP %d\n", epnum);
                    if (ctx->out_eps[epnum].configured != true) /* which state on OUT TRSFER Complete ? */
                    {
                        log_printf("[USB FS][RXFLVL] OUT Data PKT on invalid EP!\n");
                        errcode = MBED_ERROR_INVSTATE;
                        goto err;
                    }
                    ctx->out_eps[epnum].state = USBOTG_FS_EP_STATE_DATA_OUT;
                    break;
                }
            case PKT_STATUS_SETUP_TRANS_COMPLETE:
                {
                    log_printf("[USB FS][RXFLVL] Setup Transfer complete on ep %d (bcnt %d)\n", epnum, bcnt);
                    if (epnum != 0 || bcnt != 0) {
                        errcode = MBED_ERROR_UNSUPORTED_CMD;
                        goto err;
                    }
                    /* setup transfer complete, no wait oepint to handle this */
                    ctx->out_eps[epnum].state = USBOTG_FS_EP_STATE_SETUP;
                    break;
                }
            case PKT_STATUS_SETUP_PKT_RECEIVED:
                {
                    log_printf("[USB FS][RXFLVL] Setup pkt (%dB) received on ep %d\n", bcnt, epnum);
                    if (epnum != 0) {

                        uint8_t buf[16];
                        for (; bcnt > 16; bcnt -= 16) {
                            usbotgfs_read_core_fifo(&(buf[0]), 16, epnum);
                        }
                        usbotgfs_read_core_fifo(&(buf[0]), bcnt, epnum);

                        errcode = MBED_ERROR_UNSUPORTED_CMD;
                        goto err;
                    }
                    if (ctx->out_eps[epnum].state == USBOTG_FS_EP_STATE_SETUP) {
                        /* associated oepint not yet executed, return NYET to host */
                        if (bcnt > 0) {
                            uint8_t buf[16];
                            for (; bcnt > 16; bcnt -= 16) {
                                usbotgfs_read_core_fifo(&(buf[0]), 16, epnum);
                            }
                            usbotgfs_read_core_fifo(&(buf[0]), bcnt, epnum);
                        }
                        usbotgfs_endpoint_set_nak(epnum, USBOTG_FS_EP_DIR_OUT);
                        errcode = MBED_ERROR_INVSTATE;
                        goto err;
                    }
                    if (bcnt == 0) {
                        /* This is a Zero-length-packet reception, nothing to do */
                        goto err;
                    }
                    /* INFO: here, We don't check the setup pkt size, this is under the responsability of the
                     * control plane, as the setup pkt size is USB-standard defined, not driver specific */
                    usbotgfs_read_epx_fifo(bcnt, &(ctx->out_eps[epnum]));
                    // TODO: read_fifo(setup_packet, bcnt, epnum);
                    /* After this, the Data stage begins. A Setup stage done should be received, which triggers
                     * a Setup interrupt */
                    ctx->out_eps[epnum].state = USBOTG_FS_EP_STATE_SETUP_WIP;
                    break;
                }
            default:
                log_printf("[USB FS][RXFLVL] RXFLVL bad status %x!", pktsts.devsts);
                break;
        }

#else
        /* TODO: handle Host mode RXFLVL behavior */
#endif
err:
	set_reg(r_CORTEX_M_USBOTG_FS_GINTMSK, 1, USBOTG_FS_GINTMSK_RXFLVLM);
    return errcode;
}


/*
 * Start-offrame event (new USB frame)
 */
static mbed_error_t sof_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    return errcode;
}

static mbed_error_t otg_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    return errcode;
}

static mbed_error_t mmism_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    return errcode;
}

/*
 * Early suspend handler. Received when an Idle state has been
 * detected on the USB for 3ms.
 */
static mbed_error_t esuspend_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    return errcode;
}

/*
 * USB suspend handler. Received when there is no activity on the data
 * lines (including EP0) for a period of 3ms.
 */
static mbed_error_t ususpend_handler(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    return errcode;
}




/************************************************
 * About ISR handlers global table
 */
typedef mbed_error_t (*usb_otg_hs_isr_handler_t)(void);

/*
 * This table handle the overall possible ISR handlers, ordered by their bit identifier in the
 * INTSTS and GINTMSK registers
 */
static const usb_otg_hs_isr_handler_t usb_otg_hs_isr_handlers[32] = {
    default_handler,    /*< Current Mode of Operation  */
    mmism_handler,      /*< Mode mismatch */
    otg_handler,        /*< OTG interrupt */
    sof_handler,        /*< Start of Frame */
    rxflvl_handler,     /*< RxFifo non-empty */
    default_handler,    /*< Non-periodic TxFIFO empty */
    default_handler,    /*< Global IN NAK effective */
    default_handler,    /*< Global OUT NAK effective*/
    reserved_handler,   /*< Reserved */
    reserved_handler,   /*< Reserved */
    esuspend_handler,   /*< Early suspend */
    ususpend_handler,   /*< USB Suspend */
    reset_handler,      /*< Reset */
    enumdone_handler,   /*< Speed enumeration done */
    default_handler,    /*< Isochronous OUT pkt dropped */
    default_handler,    /*< End of periodic frame */
    default_handler,    /*< Reserved */
    default_handler,    /*< Endpoint mismatch */
    iepint_handler,     /*< IN Endpoint event */
    oepint_handler,     /*< OUT Endpoint event */
    default_handler,    /*< Incomplete Isochronous IN transfer */
    default_handler,    /*< Incomplete periodic transfer */
    reserved_handler,   /*< Reserved */
    reserved_handler,   /*< Reserved */
    default_handler,    /*< Host port event (Host mode) */
    default_handler,    /*< Host channels event (Host mode) */
    default_handler,    /*< Periodic TxFIFO empty (Host mode) */
    reserved_handler,   /*< Reserved */
    default_handler,    /*< Connector ID status change */
    default_handler,    /*< Disconnect event (Host mode) */
    default_handler,    /*< Session request/new session event*/
    default_handler,    /*< Resume/Wakeup event */
};

/************************************************
 * About ISR dispatchers
 */
void USBOTGFS_IRQHandler(uint8_t interrupt __attribute__((unused)),
                         uint32_t sr,
                         uint32_t dr)
{
	uint8_t i;
	uint32_t intsts = sr;
	uint32_t intmsk = dr;

	if (intsts & USBOTG_FS_GINTSTS_CMOD_Msk) {
		log_printf("[USB FS] Int in Host mode !\n");
	}
    uint32_t val = intsts;
    val &= intmsk;

    /*
     * Here, for each status flag active, execute the corresponding handler if
     * the global interrupt mask is also enabled
     */
	for (i = 0; val; i++,val>>=1) {
		/* Below code is equivalent to
         * calculating (!(intsts & ((uint32_t)1 << i)) || !(intmsk & ((uint32_t)1 << i)))
         */
        if (val & 1)
        {
#if CONFIG_USR_DRV_USBOTGFS_DEBUG
            usbotgfs_int_cnt[i]++;
#endif
            /* INFO: as log_printf is a *macro* only resolved by cpp in debug mode,
             * usbotgfs_int_name is accedded only in this mode. There is no
             * invalid memory access in the other case. */
            usb_otg_hs_isr_handlers[i]();
        }
    }
}

