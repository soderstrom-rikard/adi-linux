/*
 * can4linux.h - can4linux CAN driver module
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2001 port GmbH Halle/Saale
 *------------------------------------------------------------------
 * $Header: /z2/cvsroot/products/0530/software/can4linux/src/can4linux.h,v 1.12 2006/09/08 09:45:19 oe Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *--------------------------------------------------------------------------
 */


/**
* \file can.h
* \author Heinz-Jürgen Oertel, port GmbH
* $Revision: 1.12 $
* $Date: 2006/09/08 09:45:19 $
*
* can4linux interface definitions
*
*
*
*/


#ifndef __CAN_H
#define __CAN_H

/*  #include "/usr/include/stdint.h" */

# define CAN4LINUXVERSION 0x0304 /*(Version 3.4)*/

#ifndef __KERNEL__
#include <sys/time.h>
#endif
 /*---------- the can message structure */

#define CAN_MSG_LENGTH 8		/**< maximum length of a CAN frame */


#define MSG_RTR		(1<<0)		/**< RTR Message */
#define MSG_OVR		(1<<1)		/**< CAN controller Msg overflow error */
#define MSG_EXT		(1<<2)		/**< extended message format */
#define MSG_SELF	(1<<3)		/**< message received from own tx */
#define MSG_PASSIVE	(1<<4)		/**< controller in error passive */
#define MSG_BUSOFF      (1<<5)		/**< controller Bus Off  */
#define MSG_       	(1<<6)		/**<  */
#define MSG_BOVR	(1<<7)		/**< receive/transmit buffer overflow */
/**
* mask used for detecting CAN errors in the canmsg_t flags field
*/
#define MSG_ERR_MASK	(MSG_OVR + MSG_PASSIVE + MSG_BUSOFF + MSG_BOVR)

/**
* The CAN message structure.
* Used for all data transfers between the application and the driver
* using read() or write().
*/
typedef struct {
    /** flags, indicating or controlling special message properties */
    int32_t         flags;
    int16_t         cob;	 /**< CAN object number, used in Full CAN  */
    uint32_t        id;		 /**< CAN message ID, 4 bytes  */
    int16_t        length;	 /**< number of bytes in the CAN message */
    unsigned   char data[CAN_MSG_LENGTH]; /**< data, 0...8 bytes */
    struct timeval  timestamp;	 /**< time stamp for received messages */
} canmsg_t;



/**
---------- IOCTL requests */
/* Use 'c' as magic number, follow chapter 6 of LDD3 */
#define CAN4L_IOC_MAGIC 'c'

#define CAN_IOCTL_COMMAND 	 0	/**< IOCTL command request */
#define CAN_IOCTL_CONFIG 	 1	/**< IOCTL configuration request */
#define CAN_IOCTL_SEND 		 2	/**< IOCTL request */
#define CAN_IOCTL_RECEIVE 	 3	/**< IOCTL request */
#define CAN_IOCTL_CONFIGURERTR 	 4	/**< IOCTL request */
#define CAN_IOCTL_STATUS         5      /**< IOCTL status request */

/*---------- CAN ioctl parameter types */
/**
 IOCTL Command request parameter structure */
struct Command_par {
    int16_t cmd;			/**< special driver command */
    int16_t target;			/**< special configuration target */
    uint32_t val1;		/**< 1. parameter for the target */
    uint32_t val2;		/**< 2. parameter for the target */
    int16_t error;	 		/**< return value */
    uint32_t retval;	/**< return value */
};


/**
 IOCTL Command request parameter structure */
typedef struct Command_par Command_par_t ; /**< Command parameter struct */
/**
 IOCTL CConfiguration request parameter structure */
typedef struct Command_par  Config_par_t ; /**< Configuration parameter struct */


/**
 IOCTL generic CAN controller status request parameter structure */
typedef struct CanStatusPar { 
    uint16_t baud;			/**< actual bit rate */
    uint16_t status;		/**< CAN controller status register */
    uint16_t error_warning_limit;	/**< the error warning limit */
    uint16_t rx_errors;		/**< content of RX error counter */
    uint16_t tx_errors;		/**< content of TX error counter */
    uint16_t error_code;		/**< content of error code register */
    uint16_t rx_buffer_size;	/**< size of rx buffer  */
    uint16_t rx_buffer_used;	/**< number of messages */
    uint16_t tx_buffer_size;	/**< size of tx buffer  */
    uint16_t tx_buffer_used;	/**< number of messages */
    uint16_t retval;		/**< return value */
    uint16_t type;			/**< CAN controller / driver type */
} CanStatusPar_t;

/**
 IOCTL  CanStatusPar.type CAN controller hardware chips */
#define CAN_TYPE_UNSPEC		0
#define CAN_TYPE_SJA1000	1
#define CAN_TYPE_FlexCAN	2
#define CAN_TYPE_TouCAN		3
#define CAN_TYPE_82527		4
#define CAN_TYPE_TwinCAN	5
#define CAN_TYPE_BlackFinCAN	6


/**
 IOCTL Send request parameter structure */
typedef struct Send_par {
    int16_t error;	 		/**< return value for errno */
    uint32_t retval;	/**< return value */
    canmsg_t *Tx;		/**< CAN message struct  */
} Send_par_t ;

/**
 IOCTL Receive request parameter structure */
typedef struct Receive_par {
    int16_t error;	 		/**< return value for errno */
    uint32_t retval;	/**< return value */
    canmsg_t *Rx;		/**< CAN message struct  */
} Receive_par_t ;

/**
IOCTL ConfigureRTR request parameter structure */
typedef struct ConfigureRTR_par {
    uint32_t message;		/**< CAN message ID */
    int16_t error;	 		/**< return value for errno */
    uint32_t retval;	/**< return value */
    canmsg_t *Tx;		/**< CAN message struct  */
} ConfigureRTR_par_t ;

/**
---------- IOCTL Command subcommands and there targets */

# define CMD_START		1
# define CMD_STOP 		2
# define CMD_RESET		3
# define CMD_CLEARBUFFERS	4




/**
---------- IOCTL Configure targets */

# define CONF_ACC   	0	/* mask and code */
# define CONF_ACCM   	1	/* mask only */
# define CONF_ACCC   	2	/* code only */
# define CONF_TIMING	3	/* bit timing */
# define CONF_OMODE 	4	/* output control register */
# define CONF_FILTER	5
# define CONF_FENABLE	6
# define CONF_FDISABLE	7
# define CONF_LISTEN_ONLY_MODE	8	/* for SJA1000 PeliCAN */
# define CONF_SELF_RECEPTION	9	/* */
# define CONF_BTR   		10      /* set direct bit timing registers
					   (SJA1000) */
# define CONF_TIMESTAMP  	11      /* use TS in received messages */
# define CONF_WAKEUP		12      /* wake up processes */

#endif 	/* __CAN_H */
