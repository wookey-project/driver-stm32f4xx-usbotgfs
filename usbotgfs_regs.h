#ifndef STM32F4XX_USBOTG_FS_REGS_H
#define STM32F4XX_USBOTG_FS_REGS_H

#include "libc/regutils.h"
#include "generated/usb_otg_fs.h"

#if 1 // MR DEBUG
#define USBOTG_FS_DIEPCTL0_MPSIZ_64BYTES      0
#define USBOTG_FS_DIEPCTL0_MPSIZ_32BYTES      1
#define USBOTG_FS_DIEPCTL0_MPSIZ_16BYTES      2
#define USBOTG_FS_DIEPCTL0_MPSIZ_8BYTES       3
#define USBOTG_FS_DIEPCTL_EPTYP_CONTROL       0
#define USBOTG_FS_DIEPCTL_EPTYP_ISOCHRO       1
#define USBOTG_FS_DIEPCTL_EPTYP_BULK          2
#define USBOTG_FS_DIEPCTL_EPTYP_INT           3
#define USBOTG_FS_DOEPCTL_EPTYP_CONTROL       0
#define USBOTG_FS_DOEPCTL_EPTYP_ISOCHRO       1
#define USBOTG_FS_DOEPCTL_EPTYP_BULK  	   2
#define USBOTG_FS_DOEPCTL_EPTYP_INT   	   3
#endif


#define AHB2_BASE           0x50000000

/* The following registers addresses are defined as soon as they are needed */
#define r_CORTEX_M_USBOTG_FS_GOTGCTL   REG_ADDR(USB_OTG_FS_BASE + 0x000)
#define r_CORTEX_M_USBOTG_FS_GAHBCFG   REG_ADDR(USB_OTG_FS_BASE + 0x008)
#define r_CORTEX_M_USBOTG_FS_GUSBCFG   REG_ADDR(USB_OTG_FS_BASE + 0x00C)
#define r_CORTEX_M_USBOTG_FS_GRSTCTL   REG_ADDR(USB_OTG_FS_BASE + 0x010)
#define r_CORTEX_M_USBOTG_FS_GINTSTS   REG_ADDR(USB_OTG_FS_BASE + 0x014)
#define r_CORTEX_M_USBOTG_FS_GINTMSK   REG_ADDR(USB_OTG_FS_BASE + 0x018)
#define r_CORTEX_M_USBOTG_FS_GRXSTSP   REG_ADDR(USB_OTG_FS_BASE + 0x020)
#define r_CORTEX_M_USBOTG_FS_GRXFSIZ   REG_ADDR(USB_OTG_FS_BASE + 0x024)
/* The register 'Enpoint 0 Transmit FIFO size' is called OTG_FS_TX0FSIZ in
 * section 34.17.5 of the reference manual but it is called OTG_FS_DIEPTXF0 in
 * the section 34.16.2.
 */
#define r_CORTEX_M_USBOTG_FS_DIEPTXF0  REG_ADDR(USB_OTG_FS_BASE + 0x028)
#define r_CORTEX_M_USBOTG_FS_GCCFG     REG_ADDR(USB_OTG_FS_BASE + 0x038)
#define r_CORTEX_M_USBOTG_FS_DIEPTXF(EP)   REG_ADDR(USB_OTG_FS_BASE + 0x100 + ((EP) * 0x4))
#define r_CORTEX_M_USBOTG_FS_DCFG      REG_ADDR(USB_OTG_FS_BASE + 0x800)
#define r_CORTEX_M_USBOTG_FS_DCTL      REG_ADDR(USB_OTG_FS_BASE + 0x804)
#define r_CORTEX_M_USBOTG_FS_DSTS      REG_ADDR(USB_OTG_FS_BASE + 0x808)
#define r_CORTEX_M_USBOTG_FS_DIEPMSK   REG_ADDR(USB_OTG_FS_BASE + 0x810)
#define r_CORTEX_M_USBOTG_FS_DOEPMSK   REG_ADDR(USB_OTG_FS_BASE + 0x814)
#define r_CORTEX_M_USBOTG_FS_DAINT     REG_ADDR(USB_OTG_FS_BASE + 0x818)
#define r_CORTEX_M_USBOTG_FS_DAINTMSK  REG_ADDR(USB_OTG_FS_BASE + 0x81C)
#define r_CORTEX_M_USBOTG_FS_DIEPCTL(EP)   REG_ADDR(USB_OTG_FS_BASE + 0x900 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DIEPINT(EP)   REG_ADDR(USB_OTG_FS_BASE + 0x908 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DIEPTSIZ(EP)  REG_ADDR(USB_OTG_FS_BASE + 0x910 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DTXFSTS(EP)   REG_ADDR(USB_OTG_FS_BASE + 0x918 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DOEPCTL(EP)   REG_ADDR(USB_OTG_FS_BASE + 0xB00 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DOEPINT(EP)   REG_ADDR(USB_OTG_FS_BASE + 0xB08 + ((EP) * 0x20))
#define r_CORTEX_M_USBOTG_FS_DOEPTSIZ(EP)  REG_ADDR(USB_OTG_FS_BASE + 0xB10 + ((EP) * 0x20))

/* FIFO buffers */
#define USBOTG_FS_DEVICE_FIFO(EP)      REG_ADDR(USB_OTG_FS_BASE + (0x1000 * ((EP) + 1)))

/*
 * The following macros defines bitfield used in the driver. In order to save
 * time, only used bits in registers are defined here.
 */
/* AHB configuration register */
#define USBOTG_FS_GAHBCFG_GINTMSK_Pos  0
#define USBOTG_FS_GAHBCFG_GINTMSK_Msk  ((uint32_t)1 << USBOTG_FS_GAHBCFG_GINTMSK_Pos)
#define USBOTG_FS_GAHBCFG_TXFELVL_Pos  7
#define USBOTG_FS_GAHBCFG_TXFELVL_Msk  ((uint32_t)1 << USBOTG_FS_GAHBCFG_TXFELVL_Pos)

/* USB configuration register */
#define USBOTG_FS_GUSBCFG_TOCAL_Pos    0
#define USBOTG_FS_GUSBCFG_TOCAL_Msk    ((uint32_t)0x07 << USBOTG_FS_GUSBCFG_TOCAL_Pos)
#define USBOTG_FS_GUSBCFG_PHYSEL_Pos   6
#define USBOTG_FS_GUSBCFG_PHYSEL_Msk   ((uint32_t)1 << USBOTG_FS_GUSBCFG_PHYSEL_Pos)
#define USBOTG_FS_GUSBCFG_SRPCAP_Pos   8
#define USBOTG_FS_GUSBCFG_SRPCAP_Msk   ((uint32_t)1 << USBOTG_FS_GUSBCFG_SRPCAP_Pos)
#define USBOTG_FS_GUSBCFG_HNPCAP_Pos   9
#define USBOTG_FS_GUSBCFG_HNPCAP_Msk   ((uint32_t)1 << USBOTG_FS_GUSBCFG_HNPCAP_Pos)
#define USBOTG_FS_GUSBCFG_TRDT_Pos     10
#define USBOTG_FS_GUSBCFG_TRDT_Msk     ((uint32_t)0x0F << USBOTG_FS_GUSBCFG_TRDT_Pos)
#define USBOTG_FS_GUSBCFG_FDMOD_Pos    30
#define USBOTG_FS_GUSBCFG_FDMOD_Msk    ((uint32_t)1 << USBOTG_FS_GUSBCFG_FDMOD_Pos)

/* Reset register */
#define USBOTG_FS_GRSTCTL_CSRST_Pos    0
#define USBOTG_FS_GRSTCTL_CSRST_Msk    ((uint32_t)1 << USBOTG_FS_GRSTCTL_CSRST_Pos)
#define USBOTG_FS_GRSTCTL_HSRST_Pos    1
#define USBOTG_FS_GRSTCTL_HSRST_Msk    ((uint32_t)1 << USBOTG_FS_GRSTCTL_HSRST_Pos)
#define USBOTG_FS_GRSTCTL_FCRST_Pos    2
#define USBOTG_FS_GRSTCTL_FCRST_Msk    ((uint32_t)1 << USBOTG_FS_GRSTCTL_FCRST_Pos)
#define USBOTG_FS_GRSTCTL_RXFFLSH_Pos  4
#define USBOTG_FS_GRSTCTL_RXFFLSH_Msk  ((uint32_t)1 << USBOTG_FS_GRSTCTL_RXFFLSH_Pos)
#define USBOTG_FS_GRSTCTL_TXFFLSH_Pos  5
#define USBOTG_FS_GRSTCTL_TXFFLSH_Msk  ((uint32_t)1 << USBOTG_FS_GRSTCTL_TXFFLSH_Pos)
#define USBOTG_FS_GRSTCTL_TXFNUM_Pos   6
#define USBOTG_FS_GRSTCTL_TXFNUM_Msk   ((uint32_t)0x1F << USBOTG_FS_GRSTCTL_TXFNUM_Pos)
#define USBOTG_FS_GRSTCTL_AHBIDL_Pos   31
#define USBOTG_FS_GRSTCTL_AHBIDL_Msk   ((uint32_t)1 << USBOTG_FS_GRSTCTL_AHBIDL_Pos)

/* Core interrupt register */
#define USBOTG_FS_GINTSTS_CMOD_Pos     0
#define USBOTG_FS_GINTSTS_CMOD_Msk     ((uint32_t)1 << USBOTG_FS_GINTSTS_CMOD_Pos)
#define USBOTG_FS_GINTSTS_MMIS_Pos     1
#define USBOTG_FS_GINTSTS_MMIS_Msk     ((uint32_t)1 << USBOTG_FS_GINTSTS_MMIS_Pos)
#define USBOTG_FS_GINTSTS_OTGINT_Pos   2
#define USBOTG_FS_GINTSTS_OTGINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_OTGINT_Pos)
#define USBOTG_FS_GINTSTS_SOF_Pos      3
#define USBOTG_FS_GINTSTS_SOF_Msk      ((uint32_t)1 << USBOTG_FS_GINTSTS_SOF_Pos)
#define USBOTG_FS_GINTSTS_RXFLVL_Pos   4
#define USBOTG_FS_GINTSTS_RXFLVL_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_RXFLVL_Pos)
#define USBOTG_FS_GINTSTS_NPTXFE_Pos   5
#define USBOTG_FS_GINTSTS_NPTXFE_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_NPTXFE_Pos)
#define USBOTG_FS_GINTSTS_GINAKEFF_Pos 6
#define USBOTG_FS_GINTSTS_GINAKEFF_Msk ((uint32_t)1 << USBOTG_FS_GINTSTS_GINAKEFF_Pos)
#define USBOTG_FS_GINTSTS_GOUTNAKEFF_Pos   7
#define USBOTG_FS_GINTSTS_GOUTNAKEFF_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_GOUTNAKEFF_Pos)




#define USBOTG_FS_GINTSTS_ESUSP_Pos    10
#define USBOTG_FS_GINTSTS_ESUSP_Msk    ((uint32_t)1 << USBOTG_FS_GINTSTS_ESUSP_Pos)
#define USBOTG_FS_GINTSTS_USBSUSP_Pos  11
#define USBOTG_FS_GINTSTS_USBSUSP_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_USBSUSP_Pos)
#define USBOTG_FS_GINTSTS_USBRST_Pos   12
#define USBOTG_FS_GINTSTS_USBRST_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_USBRST_Pos)
#define USBOTG_FS_GINTSTS_ENUMDNE_Pos  13
#define USBOTG_FS_GINTSTS_ENUMDNE_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_ENUMDNE_Pos)
#define USBOTG_FS_GINTSTS_ISOODRP_Pos  14
#define USBOTG_FS_GINTSTS_ISOODRP_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_ISOODRP_Pos)
#define USBOTG_FS_GINTSTS_EOPF_Pos     15
#define USBOTG_FS_GINTSTS_EOPF_Msk     ((uint32_t)1 << USBOTG_FS_GINTSTS_EOPF_Pos)


#define USBOTG_FS_GINTSTS_EPMISM_Pos       17
#define USBOTG_FS_GINTSTS_EPMISM_Msk       ((uint32_t)1 << USBOTG_FS_GINTSTS_EPMISM_Pos)
#define USBOTG_FS_GINTSTS_IEPINT_Pos   18
#define USBOTG_FS_GINTSTS_IEPINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_IEPINT_Pos)
#define USBOTG_FS_GINTSTS_OEPINT_Pos   19
#define USBOTG_FS_GINTSTS_OEPINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_OEPINT_Pos)
#define USBOTG_FS_GINTSTS_IISOIXFR_Pos 20
#define USBOTG_FS_GINTSTS_IISOIXFR_Msk ((uint32_t)1 << USBOTG_FS_GINTSTS_IISOIXFR_Pos)
#define USBOTG_FS_GINTSTS_IPXFR_Pos    21




#define USBOTG_FS_GINTSTS_IPXFR_Msk    ((uint32_t)1 << USBOTG_FS_GINTSTS_IPXFR_Pos)
#define USBOTG_FS_GINTSTS_HPRTINT_Pos  24
#define USBOTG_FS_GINTSTS_HPRTINT_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_HPRTINT_Pos)
#define USBOTG_FS_GINTSTS_HCINT_Pos    25
#define USBOTG_FS_GINTSTS_HCINT_Msk    ((uint32_t)1 << USBOTG_FS_GINTSTS_HCINT_Pos)
#define USBOTG_FS_GINTSTS_PTXFE_Pos    26
#define USBOTG_FS_GINTSTS_PTXFE_Msk    ((uint32_t)1 << USBOTG_FS_GINTSTS_PTXFE_Pos)


#define USBOTG_FS_GINTSTS_CIDSCHG_Pos  28
#define USBOTG_FS_GINTSTS_CIDSCHG_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_CIDSCHG_Pos)
#define USBOTG_FS_GINTSTS_DISCINT_Pos  29
#define USBOTG_FS_GINTSTS_DISCINT_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_DISCINT_Pos)
#define USBOTG_FS_GINTSTS_SRQINT_Pos   30
#define USBOTG_FS_GINTSTS_SRQINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTSTS_SRQINT_Pos)
#define USBOTG_FS_GINTSTS_WKUPINT_Pos  31
#define USBOTG_FS_GINTSTS_WKUPINT_Msk  ((uint32_t)1 << USBOTG_FS_GINTSTS_WKUPINT_Pos)


/* Interrupt mask register */




#define USBOTG_FS_GINTMSK_MMISM_Pos    1
#define USBOTG_FS_GINTMSK_MMISM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_MMISM_Pos)
#define USBOTG_FS_GINTMSK_OTGINT_Pos   2
#define USBOTG_FS_GINTMSK_OTGINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_OTGINT_Pos)
#define USBOTG_FS_GINTMSK_SOFM_Pos     3
#define USBOTG_FS_GINTMSK_SOFM_Msk     ((uint32_t)1 << USBOTG_FS_GINTMSK_SOFM_Pos)
#define USBOTG_FS_GINTMSK_RXFLVLM_Pos  4
#define USBOTG_FS_GINTMSK_RXFLVLM_Msk  ((uint32_t)1 << USBOTG_FS_GINTMSK_RXFLVLM_Pos)
#define USBOTG_FS_GINTMSK_NPTXFEM_Pos  5
#define USBOTG_FS_GINTMSK_NPTXFEM_Msk  ((uint32_t)1 << USBOTG_FS_GINTMSK_NPTXFEM_Pos)
#define USBOTG_FS_GINTMSK_GINAKEFFM_Pos    6
#define USBOTG_FS_GINTMSK_GINAKEFFM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_GINAKEFFM_Pos)
#define USBOTG_FS_GINTMSK_GONAKEFFM_Pos    7
#define USBOTG_FS_GINTMSK_GONAKEFFM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_GONAKEFFM_Pos)

#define USBOTG_FS_GINTMSK_ESUSPM_Pos   10
#define USBOTG_FS_GINTMSK_ESUSPM_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_ESUSPM_Pos)
#define USBOTG_FS_GINTMSK_USBSUSPM_Pos 11
#define USBOTG_FS_GINTMSK_USBSUSPM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_USBSUSPM_Pos)
#define USBOTG_FS_GINTMSK_USBRST_Pos   12
#define USBOTG_FS_GINTMSK_USBRST_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_USBRST_Pos)
#define USBOTG_FS_GINTMSK_ENUMDNEM_Pos 13
#define USBOTG_FS_GINTMSK_ENUMDNEM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_ENUMDNEM_Pos)
#define USBOTG_FS_GINTMSK_ISOODRPM_Pos 14
#define USBOTG_FS_GINTMSK_ISOODRPM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_ISOODRPM_Pos)
#define USBOTG_FS_GINTMSK_EOPFM_Pos    15
#define USBOTG_FS_GINTMSK_EOPFM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_EOPFM_Pos)

#define USBOTG_FS_GINTMSK_EPMISM_Pos   17
#define USBOTG_FS_GINTMSK_EPMISM_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_EPMISM_Pos)
#define USBOTG_FS_GINTMSK_IEPINT_Pos   18
#define USBOTG_FS_GINTMSK_IEPINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_IEPINT_Pos)
#define USBOTG_FS_GINTMSK_OEPINT_Pos   19
#define USBOTG_FS_GINTMSK_OEPINT_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_OEPINT_Pos)

#define USBOTG_FS_GINTMSK_IISOIXFRM_Pos    20
#define USBOTG_FS_GINTMSK_IISOIXFRM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_IISOIXFRM_Pos)
#define USBOTG_FS_GINTMSK_IPXFRM_IISOOXFRM_Pos 21
#define USBOTG_FS_GINTMSK_IPXFRM_IISOOXFRM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_IPXFRM_IISOOXFRM_Pos)

#define USBOTG_FS_GINTMSK_PRTIM_Pos    24
#define USBOTG_FS_GINTMSK_PRTIM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_PRTIM_Pos)
#define USBOTG_FS_GINTMSK_HCIM_Pos 25
#define USBOTG_FS_GINTMSK_HCIM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_HCIM_Pos)
#define USBOTG_FS_GINTMSK_PTXFEM_Pos   26
#define USBOTG_FS_GINTMSK_PTXFEM_Msk   ((uint32_t)1 << USBOTG_FS_GINTMSK_PTXFEM_Pos)

#define USBOTG_FS_GINTMSK_CIDSCHGM_Pos 28
#define USBOTG_FS_GINTMSK_CIDSCHGM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_CIDSCHGM_Pos)

#define USBOTG_FS_GINTMSK_DISCINT_Pos  29
#define USBOTG_FS_GINTMSK_DISCINT_Msk  ((uint32_t)1 << USBOTG_FS_GINTMSK_DISCINT_Pos)
#define USBOTG_FS_GINTMSK_SRQIM_Pos    30
#define USBOTG_FS_GINTMSK_SRQIM_Msk    ((uint32_t)1 << USBOTG_FS_GINTMSK_SRQIM_Pos)
#define USBOTG_FS_GINTMSK_WUIM_Pos 31
#define USBOTG_FS_GINTMSK_WUIM_Msk ((uint32_t)1 << USBOTG_FS_GINTMSK_WUIM_Pos)


#define USBOTG_FS_GRXSTSP_GET_STATUS(grxstsp) ((grxstsp & 0x1e0000) >> 17)
#define USBOTG_FS_GRXSTSP_GET_DPID(grxstsp) ((grxstsp & 0x18000) >> 15)
#define USBOTG_FS_GRXSTSP_GET_BCNT(grxstsp) ((grxstsp & 0x7ff0) >> 4)
#define USBOTG_FS_GRXSTSP_GET_EPNUM(grxstsp) (grxstsp & 0xf)
#define USBOTG_FS_GRXSTSP_GET_CHNUM(grxstsp) (grxstsp & 0xf)



/* Receive FIFO size register */
#define USBOTG_FS_GRXFSIZ_RXFD_Pos     0
#define USBOTG_FS_GRXFSIZ_RXFD_Msk     ((uint32_t)0xFFFF << USBOTG_FS_GRXFSIZ_RXFD_Pos)

/* Device IN endpoint transmit FIFO size register (for DIEPTXF0 and DIEPTXFx) */
#define USBOTG_FS_DIEPTXF_INEPTXSA_Pos 0
#define USBOTG_FS_DIEPTXF_INEPTXSA_Msk ((uint32_t)0xffff << USBOTG_FS_DIEPTXF_INEPTXSA_Pos)
#define USBOTG_FS_DIEPTXF_INEPTXFD_Pos 16
#define USBOTG_FS_DIEPTXF_INEPTXFD_Msk ((uint32_t)0xffff << USBOTG_FS_DIEPTXF_INEPTXFD_Pos)

/* General core configuration */
#define USBOTG_FS_GCCFG_PWRDWN_Pos     16
#define USBOTG_FS_GCCFG_PWRDWN_Msk     ((uint32_t)1 << USBOTG_FS_GCCFG_PWRDWN_Pos)
#define USBOTG_FS_GCCFG_VBUSBSEN_Pos   19
#define USBOTG_FS_GCCFG_VBUSBSEN_Msk   ((uint32_t)1 << USBOTG_FS_GCCFG_VBUSBSEN_Pos)

/* Device configuration register */
#define USBOTG_FS_DCFG_DSPD_Pos        0
#define USBOTG_FS_DCFG_DSPD_Msk        ((uint32_t)0x3 << USBOTG_FS_DCFG_DSPD_Pos)
#define USBOTG_FS_DCFG_NZLSOHSK_Pos    2
#define USBOTG_FS_DCFG_NZLSOHSK_Msk    ((uint32_t)1 << USBOTG_FS_DCFG_NZLSOHSK_Pos)
#define USBOTG_FS_DCFG_DAD_Pos     4
#define USBOTG_FS_DCFG_DAD_Msk     ((uint32_t)0x7f << USBOTG_FS_DCFG_DAD_Pos)

/* Device status register */
#define USBOTG_FS_DSTS_SUSPSTS_Pos    0
#define USBOTG_FS_DSTS_SUSPSTS_Msk    ((uint32_t)1 << USBOTG_FS_DSTS_SUSPSTS_Pos)
#define USBOTG_FS_DSTS_ENUMSPD_Pos     1
#define USBOTG_FS_DSTS_ENUMSPD_Msk     ((uint32_t)0x3 << USBOTG_FS_DSTS_ENUMSPD_Pos)
#define USBOTG_FS_DSTS_ENUMSPD_FS      ((uint8_t)3)

/* Device IN enpoint common interrupt mask register */
#define USBOTG_FS_DIEPMSK_XFRCM_Pos    0
#define USBOTG_FS_DIEPMSK_XFRCM_Msk    ((uint32_t)1 << USBOTG_FS_DIEPMSK_XFRCM_Pos)
#define USBOTG_FS_DIEPMSK_TOM_Pos      3
#define USBOTG_FS_DIEPMSK_TOM_Msk      ((uint32_t)1 << USBOTG_FS_DIEPMSK_TOM_Pos)

/* Device OUT endpoint common interrupt mask register */
#define USBOTG_FS_DOEPMSK_XFRCM_Pos    0
#define USBOTG_FS_DOEPMSK_XFRCM_Msk    ((uint32_t)1 << USBOTG_FS_DOEPMSK_XFRCM_Pos)
#define USBOTG_FS_DOEPMSK_STUPM_Pos    3
#define USBOTG_FS_DOEPMSK_STUPM_Msk    ((uint32_t)1 << USBOTG_FS_DOEPMSK_STUPM_Pos)

/* Device all endpoints interrupt register */
#define USBOTG_FS_DAINT_IEPINT(EP)     ((uint32_t)1 << (EP))
#define USBOTG_FS_DAINT_OEPINT(EP)     ((uint32_t)1 << ((EP) + 16))

/* Device all endpoints interrupt mask register */
#define USBOTG_FS_DAINTMSK_IEPM(EP)    ((uint32_t)1 << (EP))
#define USBOTG_FS_DAINTMSK_OEPM(EP)    ((uint32_t)1 << ((EP) + 16))

/* Device control IN endpoint X control register */
#define USBOTG_FS_DIEPCTL_MPSIZ_Pos(EP)    0
#define USBOTG_FS_DIEPCTL_MPSIZ_Msk(EP)    ((EP) > 0 ? ((uint32_t)0x7ff << USBOTG_FS_DIEPCTL_MPSIZ_Pos(EP)) \
                     : ((uint32_t)0x3 << USBOTG_FS_DIEPCTL_MPSIZ_Pos(EP)))

#define USBOTG_FS_DIEPCTL_USBAEP_Pos   15
#define USBOTG_FS_DIEPCTL_USBAEP_Msk   ((uint32_t)1 << USBOTG_FS_DIEPCTL_USBAEP_Pos)
#define USBOTG_FS_DIEPCTL_EONUM_Pos    16
#define USBOTG_FS_DIEPCTL_EONUM_Msk    ((uint32_t)1 << USBOTG_FS_DIEPCTL_EONUM_Pos)
#define USBOTG_FS_DIEPCTL_NAKSTS_Pos   17
#define USBOTG_FS_DIEPCTL_NAKSTS_Msk   ((uint32_t)1 << USBOTG_FS_DIEPCTL_NAKSTS_Pos)
#define USBOTG_FS_DIEPCTL_EPTYP_Pos    18
#define USBOTG_FS_DIEPCTL_EPTYP_Msk    ((uint32_t)0x3 << USBOTG_FS_DIEPCTL_EPTYP_Pos)

#define USBOTG_FS_DIEPCTL_SNPM_Pos     20
#define USBOTG_FS_DIEPCTL_SNPM_Msk     ((uint32_t)1 << USBOTG_FS_DIEPCTL_SNPM_Pos)
#define USBOTG_FS_DIEPCTL_STALL_Pos    21
#define USBOTG_FS_DIEPCTL_STALL_Msk    ((uint32_t)1 << USBOTG_FS_DIEPCTL_STALL_Pos)

#define USBOTG_FS_DIEPCTL_CNAK_Pos     26
#define USBOTG_FS_DIEPCTL_CNAK_Msk     ((uint32_t)1 << USBOTG_FS_DIEPCTL_CNAK_Pos)
#define USBOTG_FS_DIEPCTL_SNAK_Pos     27
#define USBOTG_FS_DIEPCTL_SNAK_Msk     ((uint32_t)1 << USBOTG_FS_DIEPCTL_SNAK_Pos)
#define USBOTG_FS_DIEPCTL_SD0PID_Pos   28
#define USBOTG_FS_DIEPCTL_SD0PID_Msk   ((uint32_t)1 << USBOTG_FS_DIEPCTL_SD0PID_Pos)
#define USBOTG_FS_DIEPCTL_SD1PID_Pos   29
#define USBOTG_FS_DIEPCTL_SD1PID_Msk   ((uint32_t)1 << USBOTG_FS_DIEPCTL_SD1PID_Pos)
#define USBOTG_FS_DIEPCTL_EPDIS_Pos    30
#define USBOTG_FS_DIEPCTL_EPDIS_Msk    ((uint32_t)1 << USBOTG_FS_DIEPCTL_EPDIS_Pos)
#define USBOTG_FS_DIEPCTL_EPENA_Pos    31
#define USBOTG_FS_DIEPCTL_EPENA_Msk    ((uint32_t)1 << USBOTG_FS_DIEPCTL_EPENA_Pos)

/* Device endpoint interrupt register */
#define USBOTG_FS_DIEPINT_XFRC_Pos     0
#define USBOTG_FS_DIEPINT_XFRC_Msk     ((uint32_t)1 << USBOTG_FS_DIEPINT_XFRC_Pos)
#define USBOTG_FS_DIEPINT_EPDISD_Pos   1
#define USBOTG_FS_DIEPINT_EPDISD_Msk   ((uint32_t)1 << USBOTG_FS_DIEPINT_EPDISD_Pos)
#define USBOTG_FS_DIEPINT_TOC_Pos      3
#define USBOTG_FS_DIEPINT_TOC_Msk      ((uint32_t)1 << USBOTG_FS_DIEPINT_TOC_Pos)
#define USBOTG_FS_DIEPINT_ITTXFE_Pos   4
#define USBOTG_FS_DIEPINT_ITTXFE_Msk   ((uint32_t)1 << USBOTG_FS_DIEPINT_ITTXFE_Pos)
#define USBOTG_FS_DIEPINT_INEPNE_Pos   6
#define USBOTG_FS_DIEPINT_INEPNE_Msk   ((uint32_t)1 << USBOTG_FS_DIEPINT_INEPNE_Pos)
#define USBOTG_FS_DIEPINT_TXFE_Pos 7
#define USBOTG_FS_DIEPINT_TXFE_Msk ((uint32_t)1 << USBOTG_FS_DIEPINT_TXFE_Pos)

/* Device IN endpoint X transfert size register */
#define USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(EP)  0
#define USBOTG_FS_DIEPTSIZ_XFRSIZ_Msk(EP)  ((EP) > 0 ? ((uint32_t)0x7ffff << USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(EP)) \
                        : ((uint32_t)0x7f << USBOTG_FS_DIEPTSIZ_XFRSIZ_Pos(EP)))
#define USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(EP)  19
#define USBOTG_FS_DIEPTSIZ_PKTCNT_Msk(EP)  ((EP) > 0 ? ((uint32_t)0x3ff << USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(EP)) \
                        : ((uint32_t)0x3 << USBOTG_FS_DIEPTSIZ_PKTCNT_Pos(EP)))

/* Device IN endpoint transmit FIFO status register */
#define USBOTG_FS_DTXFSTS_INEPTFSAV_Pos    0
#define USBOTG_FS_DTXFSTS_INEPTFSAV_Msk    ((uint32_t)0xffff << USBOTG_FS_DTXFSTS_INEPTFSAV_Pos)

/* Device control OUT endpoint X control register */
#define USBOTG_FS_DOEPCTL_MPSIZ_Pos(EP)    0
#define USBOTG_FS_DOEPCTL_MPSIZ_Msk(EP)    ((EP) > 0 ? ((uint32_t)0x7ff << USBOTG_FS_DOEPCTL_MPSIZ_Pos(EP)) \
                     : ((uint32_t)0x3 << USBOTG_FS_DOEPCTL_MPSIZ_Pos(EP)))

#define USBOTG_FS_DOEPCTL_USBAEP_Pos   15
#define USBOTG_FS_DOEPCTL_USBAEP_Msk   ((uint32_t)1 << USBOTG_FS_DOEPCTL_USBAEP_Pos)
#define USBOTG_FS_DOEPCTL_EONUM_Pos    16
#define USBOTG_FS_DOEPCTL_EONUM_Msk    ((uint32_t)1 << USBOTG_FS_DOEPCTL_EONUM_Pos)
#define USBOTG_FS_DOEPCTL_NAKSTS_Pos   17
#define USBOTG_FS_DOEPCTL_NAKSTS_Msk   ((uint32_t)1 << USBOTG_FS_DOEPCTL_NAKSTS_Pos)
#define USBOTG_FS_DOEPCTL_EPTYP_Pos    18
#define USBOTG_FS_DOEPCTL_EPTYP_Msk    ((uint32_t)0x3 << USBOTG_FS_DOEPCTL_EPTYP_Pos)

#define USBOTG_FS_DOEPCTL_SNPM_Pos     20
#define USBOTG_FS_DOEPCTL_SNPM_Msk     ((uint32_t)1 << USBOTG_FS_DOEPCTL_SNPM_Pos)
#define USBOTG_FS_DOEPCTL_STALL_Pos    21
#define USBOTG_FS_DOEPCTL_STALL_Msk    ((uint32_t)1 << USBOTG_FS_DOEPCTL_STALL_Pos)

#define USBOTG_FS_DOEPCTL_CNAK_Pos     26
#define USBOTG_FS_DOEPCTL_CNAK_Msk     ((uint32_t)1 << USBOTG_FS_DOEPCTL_CNAK_Pos)
#define USBOTG_FS_DOEPCTL_SNAK_Pos     27
#define USBOTG_FS_DOEPCTL_SNAK_Msk     ((uint32_t)1 << USBOTG_FS_DOEPCTL_SNAK_Pos)
#define USBOTG_FS_DOEPCTL_SD0PID_Pos   28
#define USBOTG_FS_DOEPCTL_SD0PID_Msk   ((uint32_t)1 << USBOTG_FS_DOEPCTL_SD0PID_Pos)
#define USBOTG_FS_DOEPCTL_SD1PID_Pos   29
#define USBOTG_FS_DOEPCTL_SD1PID_Msk   ((uint32_t)1 << USBOTG_FS_DOEPCTL_SD1PID_Pos)
#define USBOTG_FS_DOEPCTL_EPDIS_Pos    30
#define USBOTG_FS_DOEPCTL_EPDIS_Msk    ((uint32_t)1 << USBOTG_FS_DOEPCTL_EPDIS_Pos)
#define USBOTG_FS_DOEPCTL_EPENA_Pos    31
#define USBOTG_FS_DOEPCTL_EPENA_Msk    ((uint32_t)1 << USBOTG_FS_DOEPCTL_EPENA_Pos)

/* Device endpoint interrupt register */
#define USBOTG_FS_DOEPINT_XFRC_Pos     0
#define USBOTG_FS_DOEPINT_XFRC_Msk     ((uint32_t)1 << USBOTG_FS_DOEPINT_XFRC_Pos)
#define USBOTG_FS_DOEPINT_STUP_Pos     3
#define USBOTG_FS_DOEPINT_STUP_Msk     ((uint32_t)1 << USBOTG_FS_DOEPINT_STUP_Pos)

/* Device OUT enpoint 0 transfer size register */
#define USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(EP)  0
#define USBOTG_FS_DOEPTSIZ_XFRSIZ_Msk(EP)  ((EP) > 0 ? ((uint32_t)0x7ffff << USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(EP)) \
                        : ((uint32_t)0x7f << USBOTG_FS_DOEPTSIZ_XFRSIZ_Pos(EP)))
#define USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(EP)  19
#define USBOTG_FS_DOEPTSIZ_PKTCNT_Msk(EP)  ((EP) > 0 ? ((uint32_t)0x3ff << USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(EP)) \
                        : ((uint32_t)1 << USBOTG_FS_DOEPTSIZ_PKTCNT_Pos(EP)))
#define USBOTG_FS_DOEPTSIZ_STUPCNT_Pos 29
#define USBOTG_FS_DOEPTSIZ_STUPCNT_Msk ((uint32_t)3 << USBOTG_FS_DOEPTSIZ_STUPCNT_Pos)

/* Device control register */
#define USBOTG_FS_DCTL_RWUSIG_Pos      0
#define USBOTG_FS_DCTL_RWUSIG_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_RWUSIG_Pos)
#define USBOTG_FS_DCTL_SDIS_Pos        1
#define USBOTG_FS_DCTL_SDIS_Msk        ((uint32_t)1 << USBOTG_FS_DCTL_SDIS_Pos)
#define USBOTG_FS_DCTL_GINSTS_Pos      2
#define USBOTG_FS_DCTL_GINSTS_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_GINSTS_Pos)
#define USBOTG_FS_DCTL_GONSTS_Pos      3
#define USBOTG_FS_DCTL_GONSTS_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_GONSTS_Pos)
#define USBOTG_FS_DCTL_TCTL_Pos        4
#define USBOTG_FS_DCTL_TCTL_Msk        ((uint32_t)0x07 << USBOTG_FS_DCTL_TCTL_Pos)
#define USBOTG_FS_DCTL_SGINAK_Pos      7
#define USBOTG_FS_DCTL_SGINAK_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_SGINAK_Pos)
#define USBOTG_FS_DCTL_CGINAK_Pos      8
#define USBOTG_FS_DCTL_CGINAK_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_CGINAK_Pos)
#define USBOTG_FS_DCTL_SGONAK_Pos      9
#define USBOTG_FS_DCTL_SGONAK_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_SGONAK_Pos)
#define USBOTG_FS_DCTL_CGONAK_Pos      10
#define USBOTG_FS_DCTL_CGONAK_Msk      ((uint32_t)1 << USBOTG_FS_DCTL_CGONAK_Pos)
#define USBOTG_FS_DCTL_POPRGDNE_Pos    11
#define USBOTG_FS_DCTL_POPRGDNE_Msk    ((uint32_t)1 << USBOTG_FS_DCTL_POPRGDNE_Pos)


#endif /* STM32F4XX_USBOTG_FS_REGS_H */
