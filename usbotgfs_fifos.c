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
#include "libc/sync.h"
#include "libc/stdio.h"

#include "api/libusbotgfs.h"
#include "usbotgfs_regs.h"
#include "usbotgfs_fifos.h"
#include "usbotgfs.h"
#include "usbotgfs_handler.h"
#include "generated/usb_otg_fs.h"

#if defined(__FRAMAC__)
#include "socs/stm32f439/usbctrl_backend.h"
#else
#include "libs/usbctrl/api/libusbctrl.h"
#endif/*!__FRAMAC__*/



/* Hardware IP FIFO size */
#define CORE_FIFO_LENGTH 4096


#if defined(__FRAMAC__)
void usbotgfs_read_core_fifo( uint8_t *dest, const uint32_t size, uint8_t ep)
#else
void usbotgfs_read_core_fifo(volatile uint8_t *dest, volatile const uint32_t size, uint8_t ep)
#endif/*__FRAMAC__*/
{
    uint32_t size_4bytes = size / 4;
    uint32_t tmp;

    /* 4 bytes aligned copy from EP FIFO */
	for (uint32_t i = 0; i < size_4bytes; i++, dest += 4){
		*(volatile uint32_t *)dest = *(USBOTG_FS_DEVICE_FIFO(ep));
	}
    /* read the residue */
	switch (size % 4) {
	case 1:
		*dest = (*(USBOTG_FS_DEVICE_FIFO(ep))) & 0xff;
		break;
	case 2:
		*(volatile uint16_t *)dest = (*(USBOTG_FS_DEVICE_FIFO(ep))) & 0xffff;
		break;
	case 3:
		tmp = *(USBOTG_FS_DEVICE_FIFO(ep));
		dest[0] = tmp & 0xff;
		dest[1] = (tmp >> 8) & 0xff;
		dest[2] = (tmp >> 16) & 0xff;
		break;
	default:
		break;
	}
}


/*@
    @ requires ep <= USBOTGFS_MAX_OUT_EP ;
    @ requires \valid_read(src);
    @ requires (uint32_t *)USB_BACKEND_MEMORY_BASE <= USBOTG_FS_DEVICE_FIFO(ep) <= (uint32_t *)USB_BACKEND_MEMORY_END ;
    @ requires \separated(src,((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)) ) ;
    @ assigns *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)) ;
*/

/*  requires ep <= USBOTGFS_MAX_OUT_EP needed for memory space :
        if ep > USBOTGFS_MAX_OUT_EP, USBOTG_FS_DEVICE_FIFO(ep) >= USB_BACKEND_MEMORY_END
        this limit is hardware dependant (even if USBOTGFS_MAX_IN_EP > USBOTGFS_MAX_OUT_EP, it is not possible
        to handle more than USBOTGFS_MAX_OUT_EP (+ EP0))
 */

#if defined(__FRAMAC__)
static inline void usbotgfs_write_core_fifo(uint8_t *src, const uint32_t size, uint8_t ep)
#else
static inline void usbotgfs_write_core_fifo(volatile uint8_t *src, volatile const uint32_t size, uint8_t ep)
#endif/*__FRAMAC__*/
{
	uint32_t size_4bytes = size / 4;
    uint32_t tmp = 0;
    if (!src || size == 0) {
        return;
    }
    log_printf("[USBOTG][FS] writing %d bytes to EP %d core TxFIFO\n", size, ep);
    // IP should has its own interrupts disable during ISR execution
    uint32_t oldmask = read_reg_value(r_CORTEX_M_USBOTG_FS_GINTMSK);
    /* mask interrupts while writting Core FIFO */
    set_reg_value(r_CORTEX_M_USBOTG_FS_GINTMSK, 0, 0xffffffff, 0);

    /* manual copy to Core FIFO */
    /* there is no overflow on src here, as the C divisor is natural integer
     * divisor, truncating the divised size value to the first integer below
     */

    /*@
        @ loop invariant 0 <= i <= size_4bytes;
        @ loop invariant ep <= 3 ;
        @ loop invariant (uint32_t *)USB_BACKEND_MEMORY_BASE <= USBOTG_FS_DEVICE_FIFO(ep) <= (uint32_t *)USB_BACKEND_MEMORY_END ;
        @ loop invariant \separated(src,((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)) ) ;
        @ loop assigns i, *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)), tmp, src ;
        @ loop variant (size_4bytes - i) ;
    */
    for (uint32_t i = 0; i < size_4bytes; i++, src += 4){
        tmp = src[0];
        tmp |= (uint32_t)(src[1] & 0xff) << 8;
        tmp |= (uint32_t)(src[2] & 0xff) << 16;
        tmp |= (uint32_t)(src[3] & 0xff) << 24;
        write_reg_value(USBOTG_FS_DEVICE_FIFO(ep), tmp);
    }
    tmp = 0;
    switch (size & 3) {
        /* sequencialy write up to 3 bytes into tmp (depending on the carry)
         * and write tmp to Core FIFO
         * INFO: the sequencial bytes inclusion (up to 3) is managed by *removing*
         * the switch/case breaks. Do not re-add it ! */
        case 3:
            tmp = ((const uint8_t*) src)[2] << 16;
            __explicit_fallthrough
        case 2:
            tmp |= ((const uint8_t*) src)[1] << 8;
            __explicit_fallthrough
        case 1:
            tmp  |= ((const uint8_t*) src)[0];
            write_reg_value(USBOTG_FS_DEVICE_FIFO(ep), tmp);
            break;
        default:
            /* should never happend, complete switch */
            break;
    }

    /* IP should has its own interrupts disable during ISR execution */
    set_reg_value(r_CORTEX_M_USBOTG_FS_GINTMSK, oldmask, 0xffffffff, 0);
}

mbed_error_t usbotgfs_init_global_fifo(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    if (ctx == NULL) {
        log_printf("[USBOTG] ctx null\n");
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    /*
     * 	  Set up the Data FIFO RAM for each of the FIFOs
	 *      – Program the OTG_HS_GRXFSIZ register, to be able to receive control OUT data
	 *        and setup data. If thresholding is not enabled, at a minimum, this must be equal to
	 *        1 max packet size of control endpoint 0 + 2 Words (for the status of the control
	 *        OUT data packet) + 10 Words (for setup packets).
     *        for  e.g. giving 64 bytes for max EP0 Pkt size, this means 64 + 2*wz + 10*wz
	 *
	 * See reference manual section 34.11 for peripheral FIFO architecture.
	 * XXX: The sizes of TX FIFOs seems to be the size of TX FIFO #0 for
	 * all FIFOs. We don't know if it is really the case or if the DTXFSTS
	 * register does not give the free space for the right FIFO.
	 *
	 * 0                512                1024             1536
	 * +-----------------+------------------+-----------------+-----------
	 * |     RX FIFO     |     TX0 FIFO     | TXi FIFO (EP i) |
	 * |    128 Words    |    128 Words     |    128 Words    |...
	 * +-----------------+------------------+-----------------+------------
     * Settings FIFOs for Endpoint 0
     * RXFD (RxFIFO depth, in 32bits DWORD)
     */
	set_reg(r_CORTEX_M_USBOTG_FS_GRXFSIZ, USBOTG_FS_RX_CORE_FIFO_SZ, USBOTG_FS_GRXFSIZ_RXFD);
    ctx->fifo_idx +=  USBOTG_FS_RX_CORE_FIFO_SZ;
    /* setting TX0FSIZ to */

err:
    return errcode;
}

/*@
    @ requires \valid(ep);
    @ requires \separated(((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)), &usbotghs_ctx) ;
    @ assigns *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)), usbotgfs_ctx, *ep ;

    @ behavior ctx_null:
    @   assumes &usbotgfs_ctx == \null;
    @   ensures \result == MBED_ERROR_INVSTATE ;

    @ behavior epid_null_NOSTORAGE:
    @   assumes &usbotgfs_ctx != \null;
    @   assumes (ep->id == 0) ;
    @   assumes usbotgfs_ctx.fifo_idx + (uint16_t)USBOTG_FS_TX_CORE_FIFO_SZ >= (uint16_t)CORE_FIFO_LENGTH ;
    @   ensures \result == MBED_ERROR_NOSTORAGE ;
    @   ensures ep->mpsize == \old(ep->mpsize) ;

    @ behavior epid_null_STORAGE_OK:
    @   assumes &usbotgfs_ctx != \null;
    @   assumes (ep->id == 0) ;
    @   assumes !(usbotgfs_ctx.fifo_idx + (uint16_t)USBOTG_FS_TX_CORE_FIFO_SZ >= (uint16_t)CORE_FIFO_LENGTH) ;
    @   ensures \result == MBED_ERROR_NONE ;

    @ behavior epid_not_null :
    @   assumes &usbotgfs_ctx != \null;
    @   assumes !(ep->id == 0) ;
    @   ensures \result == MBED_ERROR_NONE || \result == MBED_ERROR_NOSTORAGE ;

    @ complete behaviors;
    @ disjoint behaviors ;

*/

/*
    TODO : be more precise for behavior epid_not_null : problem proving \result == MBED_ERROR_NOSTORAGE, problably
            du to some assume about ep->mpsize
*/

mbed_error_t usbotgfs_reset_epx_fifo(usbotgfs_ep_t *ep)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    if (ctx == NULL) {
        log_printf("[USBOTG] ctx null\n");
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    if (ep->id == 0) {
        /*
         * EndPoint 0 TX FIFO configuration (should store a least 4 64byte paquets)
         */
        /*  – Program the OTG_HS_TX0FSIZ register (depending on the FIFO number
         *  chosen) to be able to transmit control IN data. At a minimum, this must be equal to
         *  1 max packet size of control endpoint 0.
         *
         *  FIXME: this work is not made in the previous driver... Maybe we should correct this here.
         */

        if (ctx->fifo_idx + USBOTG_FS_TX_CORE_FIFO_SZ >= CORE_FIFO_LENGTH) {
            errcode = MBED_ERROR_NOSTORAGE;
            goto err;
        }

        /*
         */
        set_reg(r_CORTEX_M_USBOTG_FS_DIEPTXF0, USBOTG_FS_RX_CORE_FIFO_SZ, USBOTG_FS_DIEPTXF_INEPTXSA);
        set_reg(r_CORTEX_M_USBOTG_FS_DIEPTXF0, USBOTG_FS_TX_CORE_FIFO_SZ / 4, USBOTG_FS_DIEPTXF_INEPTXFD);
        /*
         * 4. Program STUPCNT in the endpoint-specific registers for control OUT endpoint 0 to receive a SETUP packet
         *      – STUPCNT = 3 in OTG_HS_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP packets)
         */
        set_reg(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(0),
                3, USBOTG_FS_DOEPTSIZ_STUPCNT);
        set_reg(r_CORTEX_M_USBOTG_FS_DOEPCTL(0),
                1, USBOTG_FS_DOEPCTL_CNAK);
        usbotgfs_txfifo_flush(0);
        ctx->fifo_idx += USBOTG_FS_TX_CORE_FIFO_SZ;
    } else {
        /* all other EPs have their DIEPTXF registers accesible through a single macro */
        if (ep->dir == USBOTG_FS_EP_DIR_OUT) {
            /* using global RX fifo... GRXFIFOSZ set as global RX FIFO */
        } else {
            set_reg(r_CORTEX_M_USBOTG_FS_DIEPTXF(ep->id), ctx->fifo_idx, USBOTG_FS_DIEPTXF_INEPTXSA);
            /* this field is in 32bits words unit */
            /* for very small mpsize EP (e.g. keyboards, we must support at list
             * URB size + mpsize */
            uint32_t fifosize;
            if (ep->mpsize <= 64) {
                fifosize = 128;
            } else {
                fifosize = ep->mpsize;
            }


            if (ctx->fifo_idx + fifosize >= CORE_FIFO_LENGTH) {
                errcode = MBED_ERROR_NOSTORAGE;
                goto err;
            }

            /* FIXME: DIEPTXF fifo size is in word unit, shouldn't it be fifosize/4 ? */
            set_reg(r_CORTEX_M_USBOTG_FS_DIEPTXF(ep->id), fifosize, USBOTG_FS_DIEPTXF_INEPTXFD);
            ctx->fifo_idx += fifosize;
        }
    }

    set_bool_with_membarrier(&(ep->fifo_lck), true);
    ep->fifo_idx = 0;
    ep->fifo = NULL;
    ep->fifo_size = 0;
    ep->core_txfifo_empty = true;
    set_bool_with_membarrier(&(ep->fifo_lck), false);
err:
    return errcode;
}

/*
 * read from Core EPx FIFO to associated RAM FIFO for given EP.
 * The EP must be a receiver EP (IN in host mode, OUT in device mode)
 *
 */
mbed_error_t usbotgfs_read_epx_fifo(uint32_t size, usbotgfs_ep_t *ep)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    /* sanitation */
    if (ep->configured == false) {
        log_printf("[USBOTG][FS] EPx %d not configured\n", ep->id);
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
    /* TODO: checking that EP is in correct direction before continuing */
    if (size == 0) {
        log_printf("[USBOTG][FS] nothing to read on EPx %d\n", ep->id);
        goto err;
    }
    if (size > (ep->fifo_size - ep->fifo_idx)) {
        printf("[USBOTG][FS] invalid or too big size in ep %d: %d (fifo: 0x%x, fifo size: %d, idx: %d)\n", ep->id, size, ep->fifo, ep->fifo_size, ep->fifo_idx);
        /* Why reading 0 bytes from Core FIFO ? */
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
    /* Let's now do the read transaction itself... */
    if (ep->fifo_lck != false) {
        log_printf("[USBOTG][FS] invalid state! fifo already locked\n");
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    set_bool_with_membarrier(&(ep->fifo_lck), true);
    usbotgfs_read_core_fifo(&(ep->fifo[ep->fifo_idx]), size, ep->id);
    ep->fifo_idx += size;
    set_bool_with_membarrier(&(ep->fifo_lck), false);
err:
    return errcode;
}

/*
 * read from the EPx RAM FIFO to the Core EPx FIFO.
 * The EP must be a sender EP (OUT in host mode, IN in device mode).
 *
 * Here, size is already splitted to be smaller than current EP mpsize
 * Moreover, the TX FIFO ZS is (must) be bigger than EP MPSize in order to
 * permit packet transmission. As a consequence, comparison to FIFO MAX SZ is not
 * needed.
 */

/*@
    @ requires \valid(ep);
    @ requires \separated(ep,&ep->fifo[ep->fifo_idx],((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)));
    @ assigns *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)), *ep ;
    @ ensures size > USBOTG_FS_TX_CORE_FIFO_SZ <==> \result == MBED_ERROR_INVPARAM ;
    @ ensures (size <= USBOTG_FS_TX_CORE_FIFO_SZ && \old(ep->fifo_lck) == \true) <==> \result == MBED_ERROR_INVSTATE ;
    @ ensures (size <= USBOTG_FS_TX_CORE_FIFO_SZ && \old(ep->fifo_lck) == \false && \old(ep->fifo_idx) >= ((uint32_t)4*1024*1024*1000 - size) ) <==> \result == MBED_ERROR_NOMEM ;
    @ ensures (size <= USBOTG_FS_TX_CORE_FIFO_SZ && \old(ep->fifo_lck) == \false && \old(ep->fifo_idx) < ((uint32_t)4*1024*1024*1000 - size) ) ==> (\result == MBED_ERROR_NONE && ep->fifo_lck == \false);
*/

mbed_error_t usbotgfs_write_epx_fifo(const uint32_t size, usbotgfs_ep_t *ep)
{

    mbed_error_t errcode = MBED_ERROR_NONE;
/* sanitation */
    /* we consider that packet splitting is made by the caller (i.e. usbotgfs_send()) */
    /* fixme: size > (USBOTG_FS_TX_CORE_FIFO_SZ - ep->fifo_idx) */
    if (size > USBOTG_FS_TX_CORE_FIFO_SZ) {
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
    /* Let's now do the read transaction itself... */
    if (ep->fifo_lck != false) {
        log_printf("[USBOTG][FS] invalid state! fifo already locked\n");
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    set_bool_with_membarrier(&(ep->fifo_lck), true);
    usbotgfs_write_core_fifo(&(ep->fifo[ep->fifo_idx]), size, ep->id);
    /* int overflow check */
    if (ep->fifo_idx >= ((uint32_t)4*1024*1024*1000 - size)) {
        /* In a nominal embedded usage, this should never arise as embedded devices never
         * handle such amount of memory */
        log_printf("USBOTG][FS] overflow detected!\n");
        errcode = MBED_ERROR_NOMEM;
        goto err;
    }
    ep->fifo_idx += size;
err:
    set_bool_with_membarrier(&(ep->fifo_lck), false);
    return errcode;
}


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
/*@
    @ assigns *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)), usbotghs_ctx;

    @   ensures \result == MBED_ERROR_INVPARAM
    <==> ((usbotgfs_ctx.out_eps[epid].configured == \false) || (usbotgfs_ctx.out_eps[epid].mpsize == 0))
     || (!((usbotgfs_ctx.out_eps[epid].configured == \false) || (usbotgfs_ctx.out_eps[epid].mpsize == 0)) && size == 0) ;

    @   ensures \result == MBED_ERROR_NONE
    ==> (usbotgfs_ctx.out_eps[epid].configured == \true && usbotgfs_ctx.out_eps[epid].mpsize != 0 && size != 0) ;

*/
/* ep check is done by calling functions */
mbed_error_t usbotgfs_set_recv_fifo(uint8_t *dst, uint32_t size, uint8_t epid)
{
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    usbotgfs_ep_t*      ep;
    mbed_error_t        errcode = MBED_ERROR_NONE;

    if (dst == NULL) {
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
    if (epid >= USBOTGFS_MAX_OUT_EP) {
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /* reception is done ON out_eps in device mode */
        ep = &(ctx->out_eps[epid]);
#else
        /* reception is done IN out_eps in device mode */
        ep = &(ctx->in_eps[epid]);
#endif
    if (!ep->configured || !ep->mpsize ) {  // ep->mpsize check to avoid RTE later
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }

    if (size == 0) {
        printf("[USBOTG] try to set FIFO of size 0\n");
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }
    /* Lock FIFO handling here */
    set_bool_with_membarrier(&(ep->fifo_lck), true);
    ep->fifo = dst;
    ep->fifo_idx = 0;
    ep->fifo_size = size;
    set_bool_with_membarrier(&(ep->fifo_lck), false);

    if (epid > 0) {
        /* configure EP for receiving size amount of data */
        uint32_t pktcount = size / ep->mpsize + (size & (ep->mpsize - 1) ? 1: 0);
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epid), pktcount, USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(epid), USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(epid));
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epid), size, USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(epid), USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(ep));
    } else {
        /* for EP0, the IP is not able to handle more than 64 bytes per
         * transfer. As a consequence, even for bigger transfers (e.g. 4K)
         * a fragmentation step is needed. This is done by:
         * 1. settting pktcunt and pktsize so that oepint is executed for each
         * 64 bytes packet
         * 2. oepint (in DATA_OUT mode ) check that fifo_idx == fifo_size.
         * If not, oepting does NOT call the upper class handler, silently
         * acknowledge. */
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epid), 1, USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(epid), USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(epid));
        set_reg_value(r_CORTEX_M_USBOTG_FS_DOEPTSIZ(epid), ep->mpsize, USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(epid), USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(epid));
    }
    /* FIFO is now configured */
    /* CNAK is done by endpoint activation */
err:
    /*@ assert errcode == MBED_ERROR_NONE ==> (usbotgfs_ctx.out_eps[epid].configured == \true && usbotgfs_ctx.out_eps[epid].mpsize != 0 && size != 0) ; */
// without this assert, global ensures about MBED_ERROR_NONE is not prooved
    return errcode;
}

/*@
    @ requires \valid_read(src);
    @ requires \separated(src,&usbotgfs_ctx);
     @   assigns usbotgfs_ctx ;

    @ behavior not_configured:
    @   assumes (usbotgfs_ctx.in_eps[epid].configured == \false) ;
    @   ensures \result == MBED_ERROR_INVPARAM ;

    @ behavior fifo_not_null:
    @   assumes !(usbotgfs_ctx.in_eps[epid].configured == \false) ;
    @   assumes (usbotgfs_ctx.in_eps[epid].fifo_lck == \true)  ;
    @   ensures \result == MBED_ERROR_INVSTATE ;

    @ behavior fifo_null:
    @   assumes !(usbotgfs_ctx.in_eps[epid].configured == \false) ;
    @   assumes !(usbotgfs_ctx.in_eps[epid].fifo_lck == \true)  ;
    @   ensures \result == MBED_ERROR_NONE ;


    @ complete behaviors;
    @ disjoint behaviors ;

*/

mbed_error_t usbotgfs_set_xmit_fifo(uint8_t *src, uint32_t size, uint8_t epid)
{
    usbotgfs_context_t *ctx = usbotgfs_get_context();
    usbotgfs_ep_t*      ep;
    mbed_error_t        errcode = MBED_ERROR_NONE;

#if CONFIG_USR_DRV_USBOTGFS_MODE_DEVICE
        /* transmition is done using in_eps in device mode */
        ep = &(ctx->in_eps[epid]);
#else
        /* transmition is done using out_eps in device mode */
        ep = &(ctx->out_eps[epid]);
#endif
    if (!ep->configured) {
        errcode = MBED_ERROR_INVPARAM;
        goto err;
    }

    log_printf("[USBOTG][FS] set ep %d TxFIFO to %p (size %d)\n", ep->id, src, size);

    set_bool_with_membarrier(&(ep->fifo_lck), true);
    /* set RAM FIFO for current EP. */
    ep->fifo = src;
    ep->fifo_idx = 0;
    ep->fifo_size = size;
    set_bool_with_membarrier(&(ep->fifo_lck), false);

    /* FIFO is now configured */
err:
    return errcode;
}


/*@
    @ assigns *((uint32_t *) (USB_BACKEND_MEMORY_BASE .. USB_BACKEND_MEMORY_END)) ;
    @ ensures \result == MBED_ERROR_NONE || \result == MBED_ERROR_BUSY ;
*/
mbed_error_t usbotgfs_txfifo_flush(uint8_t ep_id)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
	/* Select which ep to flush and do it
 	 * This is the FIFO number that must be flushed using the TxFIFO Flush bit.
 	 * This field must not be changed until the core clears the TxFIFO Flush bit.
 	 */
    /*
     * Is there a previous flush being executed ? If yes, wait for this flush to
     * end.
     */
    if (get_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, USBOTG_FS_GRSTCTL_TXFFLSH)){
        errcode = MBED_ERROR_BUSY;
        goto err;
    }
	/*
	 * The application must write this bit only after checking that the core is neither writing to the
	 * TxFIFO nor reading from the TxFIFO. Verify using these registers:
	 */

	/* FIXME Read: the NAK effective interrupt ensures the core is not reading from the FIFO */

	/* Write: the AHBIDL bit in OTG_HS_GRSTCTL ensures that the core is not writing anything to the FIFO */
	set_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, ep_id, USBOTG_FS_GRSTCTL_TXFNUM);
	set_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, 1, USBOTG_FS_GRSTCTL_TXFFLSH);

    /* wait for fifo flush to be executed */

    /*@
        @ loop invariant 0 <= cpt <= CPT_HARD;
        @ loop assigns cpt;
        @ loop variant (CPT_HARD - cpt);
    */
    for(uint8_t cpt=0; cpt<CPT_HARD; cpt++){
        if (get_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, USBOTG_FS_GRSTCTL_TXFFLSH)) {
            if (cpt > USBOTGFS_REG_CHECK_TIMEOUT) {
                log_printf("[USBOTG][FS] HANG! Waiting for the core to clear the TxFIFO Flush bit GRSTCTL:TXFFLSH\n");
                errcode = MBED_ERROR_BUSY;
                goto err;
            }
        }
    }
err:
    return errcode;
}

mbed_error_t usbotgfs_txfifo_flush_all(void)
{
    mbed_error_t errcode = MBED_ERROR_NONE;

    usbotgfs_context_t *ctx = usbotgfs_get_context();
    if (!ctx) {
        errcode = MBED_ERROR_INVSTATE;
        goto err;
    }
    /* Device mode, TxFIFO set in IN EPs */
    for (uint8_t i = 0; i < USBOTGFS_MAX_IN_EP; ++i) {
        if (ctx->in_eps[i].configured) {
            usbotgfs_txfifo_flush(i);
        }
    }
err:
    return errcode;

}

mbed_error_t usbotgfs_rxfifo_flush(uint8_t ep_id)
{
    mbed_error_t errcode = MBED_ERROR_NONE;
	uint32_t count = 0;
    ep_id = ep_id;
	/* Select which ep to flush and do it
 	 * This is the FIFO number that must be flushed using the RxFIFO Flush bit.
 	 * This field must not be changed until the core clears the RxFIFO Flush bit.
 	 */
    while (get_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, USBOTG_FS_GRSTCTL_RXFFLSH)) {
        if (++count > USBOTGFS_REG_CHECK_TIMEOUT) {
            log_printf("[USBOTG][FS] HANG! Waiting for the core to clear the RxFIFO Flush bit GRSTCTL:RXFFLSH\n");
            errcode = MBED_ERROR_BUSY;
            goto err;
        }
    }
	/*
	 * The application must write this bit only after checking that the core is neither writing to the
	 * TxFIFO nor reading from the TxFIFO. Verify using these registers:
	 */

	/* FIXME Read: the NAK effective interrupt ensures the core is not reading from the FIFO */

	/* Write: the AHBIDL bit in OTG_HS_GRSTCTL ensures that the core is not writing anything to the FIFO */
	set_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, 1, USBOTG_FS_GRSTCTL_RXFFLSH);
	count = 0;
    while (get_reg(r_CORTEX_M_USBOTG_FS_GRSTCTL, USBOTG_FS_GRSTCTL_RXFFLSH)) {
        if (++count > USBOTGFS_REG_CHECK_TIMEOUT) {
            log_printf("[USBOTG][FS] HANG! Waiting for the core to clear the RxFIFO Flush bit GRSTCTL:RXFFLSH\n");
            errcode = MBED_ERROR_BUSY;
            goto err;
        }
    }
err:
    return errcode;
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
#ifndef __FRAMAC__
mbed_error_t usb_backend_drv_set_recv_fifo(uint8_t *dst, uint32_t size, uint8_t ep)
    __attribute__ ((alias("usbotgfs_set_recv_fifo")));
#endif/*!__FRAMAC__*/
