/*! @file vfsSpiDrv.h
 *******************************************************************************
 **  SPI Driver Interface Functions
 **
 *******************************************************************************
 **
 **                           NDA AND NEED-TO-KNOW REQUIRED
 **
 *****************************************************************************
 **
 **  Copyright (C) 2011-2016 Synaptics Incorporated. All rights reserved.
 **
 **
 ** This file contains information that is proprietary to Synaptics
 ** Incorporated ("Synaptics"). The holder of this file shall treat all
 ** information contained herein as confidential, shall use the
 ** information only for its intended purpose, and shall not duplicate,
 ** disclose, or disseminate any of this information in any manner unless
 ** Synaptics has otherwise provided express, written permission.
 ** Use of the materials may require a license of intellectual property
 ** from a third party or from Synaptics. Receipt or possession of this
 ** file conveys no express or implied licenses to any intellectual
 ** property rights belonging to Synaptics.
 **
 **
 ** INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND
 ** SYNAPTICS EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES,
 ** INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 ** PARTICULAR PURPOSE, AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY
 ** INTELLECTUAL PROPERTY RIGHTS. IN NO EVENT SHALL SYNAPTICS BE LIABLE
 ** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, PUNITIVE, OR
 ** CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION WITH THE USE OF
 ** THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED AND BASED
 ** ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 ** NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED
 ** OF THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT
 ** JURISDICTION DOES NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY
 ** OTHER DAMAGES, SYNAPTICS' TOTAL CUMULATIVE LIABILITY TO ANY PARTY
 ** SHALL NOT EXCEED ONE HUNDRED U.S. DOLLARS.
*/


#ifndef VFSSPIDRV_H_
#define VFSSPIDRV_H_

#define DEBUG 1

#define PR_ERR(fmt, args...)   pr_err("synafpspi:"fmt, ## args)
#if DEBUG
#define PR_DEBUG(fmt, args...) pr_debug("synafpspi:"fmt, ## args)
#define PR_INFO(fmt, args...)  pr_info("synafpspi:"fmt, ## args)
#else /* DEBUG */
#define PR_DEBUG(fmt, args...)
#define PR_INFO(fmt, args...)
#endif /* DEBUG */

#define SYNA_PART_NAME "syna_fingerprint"
#define SYNA_DEV_NAME  "vfsspi"

#define PLATFORM_BIG_ENDIAN 1

/* DRDY GPIO pin number */
#define VFSSPI_DRDY_PIN     133
/* Sleep GPIO pin number */
#define VFSSPI_SLEEP_PIN    132
/* GPIO 0 pin number */
#define VFSSPI_HBM_REQ_PIN  0 /* TBD just for reference*/
/* GPIO 1 pin number */
#define VFSSPI_HBM_READY_PIN    1 /* TBD just for reference*/

#define DRDY_ACTIVE_STATUS      1
#define BITS_PER_WORD           8
#define DRDY_IRQ_FLAG           IRQF_TRIGGER_RISING
#define HBM_REQ_IRQ_FLAG        IRQF_TRIGGER_RISING

/* Max baud rate supported by Validity sensor. */
#define MAX_BAUD_RATE   12000000

/* The coefficient which is multiplying with value retrieved from the
 * VFSSPI_IOCTL_SET_CLK IOCTL command for getting the final baud rate. */
#define BAUD_RATE_COEF  1000

/* Maximum transfer size */
#define DEFAULT_BUFFER_SIZE  (18 * 4096)

/* confing spi in QSEE */
#define CONFIG_FINGERPRINT_IN_QSEE

/* Indicates DRDY IRQ enabled or disabled */
#define DRDY_IRQ_ENABLE         1
#define DRDY_IRQ_DISABLE        0

/* Indicates HBM IRQ enabled or disabled */
#define HBM_REQ_IRQ_ENABLE      1
#define HBM_REQ_IRQ_DISABLE     0

#endif /* VFSSPIDRV_H__ */
