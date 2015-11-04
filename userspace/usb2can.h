/*
Copyright (c) 2009  Sygmi

Author:		 
		sygi

Module Name:	
		usb2can.h

Description:  
		This is commercial software
		Native USB to CAN device driver based on
		FTDI FT245 and FTD2XX library definitions

		Tested with FTD2xx

		3.1.18.0, 3.1.19.0

		version library

Revision History:

	13/09/09	sygi	 First draft
	05/10/10	sygi	 Added support for 33,3; 62,5; 83,3 kbps

*/

#ifndef _USB2CAN_H_
#define _USB2CAN_H_

#ifdef WINTYPES  // use -DWINTYPES flag if you need Windows types
 #include "WinTypes.h"
#endif

// The following ifdef block is the standard way of creating macros
// which make exporting from a DLL simpler.  All files within this DLL
// are compiled with the USB2CAN_EXPORTS symbol defined on the command line.
// This symbol should not be defined on any project that uses this DLL.
// This way any other project whose source files include this file see
// USB2CAN_API functions as being imported from a DLL, whereas this DLL
// sees symbols defined with this macro as being exported.

#ifdef CCWINREL
#if USB2CAN_EXPORTS
# define USB2CAN_API __declspec (dllexport)
#else /* Not USB2CAN_EXPORTS */
# define USB2CAN_API __declspec (dllimport) // used for load-time dynamic linking
#endif
#else
#define USB2CAN_API
#endif

//! Library version eg. 1.0.0
#define CANCTRL_LIBVER                  "1.0.0"

//! Maximal length of the string
#define CANCTRL_MAX_STRING_LEN          25

//! Library thread priorities
#define CANCTRL_PRIORITY_LOW            0
#define CANCTRL_PRIORITY_NORMAL         1
#define CANCTRL_PRIORITY_HIGH           2
#define CANCTRL_PRIORITY_RT             3

//! Controller modes
#define CANCTRL_MODE_LOOPBACK           4
#define CANCTRL_MODE_LISTENONLY         2
#define CANCTRL_MODE_NORMAL             0

//! Controller baudrates
#define CANCTRL_BAUDRATE_1000           1000
#define CANCTRL_BAUDRATE_500            500
#define CANCTRL_BAUDRATE_250            250
#define CANCTRL_BAUDRATE_125            125
#define CANCTRL_BAUDRATE_100            100
#define CANCTRL_BAUDRATE_83_3           83
#define CANCTRL_BAUDRATE_62_5           62
#define CANCTRL_BAUDRATE_50             50
#define CANCTRL_BAUDRATE_33_3           33
#define CANCTRL_BAUDRATE_20             20


//! Can message frame info flags
#define CANCTRL_FRAME_NORET             0x80    ///< no frame retransmission in case of error
#define CANCTRL_FRAME_EXT               0x20    ///< Extended frame format
#define CANCTRL_FRAME_STD               0x00    ///< Standard frame format
#define CANCTRL_FRAME_RTR               0x10    ///< Remote transmission request
#define CANCTRL_FRAME_DLC               0xF     ///< DLC


/*!
 * \brief Library exceptions and return codes
 * \enum USB2CAN_LIBSTATUS
 */
typedef enum {
	USB2CAN_OK = 0,         ///< OK
	USB2CAN_USB_ERR,        ///< internal usb bus error  eg. ftdi device transfer timeout
	USB2CAN_APP_ERR,        ///< internal application error eg. mutexes, lists or thread error
	USB2CAN_TX_FIFO_FULL,   ///< tx fifo full
	USB2CAN_RX_FIFO_FULL,   ///< rx fifo full
	USB2CAN_RX_FIFO_EMPTY,  ///< rx fifo empty
	USB2CAN_FILTER_ERR,     ///< software filter error
	USB2CAN_RESOURCE_ERR,   ///< insufficient resources  eg. malloc error
	USB2CAN_PARAM_ERR,      ///< wrong parameter
	USB2CAN_DEVICE_ERR,     ///< device error  eg. device not found
	USB2CAN_HANDLE_ERR      ///< internal library handle error eg. device not open, ftd2xx.dll missing ?
} USB2CAN_LIBSTATUS;

/*!
 * \brief CAN controller state
 * \enum CANBUSSTATE
 */
typedef enum
{
	CANCTRL_STATE_ACTIVE = 0,     ///< active controller state
	CANCTRL_STATE_BUSHEAVY,       ///< bus heavy state
	CANCTRL_STATE_PASSIVE,        ///< passive controller state
	CANCTRL_STATE_BUSOFF          ///< bus off state
} CANBUSSTATE;

/*!
	Macro to obtain bus error type
*/
#define CANCTRL_BUSERRTYPE(busError)  (busError >> 6)

/*!
 * \brief CAN bus error type
 * \enum CANBUSERRORTYPE
 */
typedef enum
{
	CANCTRL_EBUSTYPE_BIT = 0,     ///< 00 -> Bit error
	CANCTRL_EBUSTYPE_FORM,        ///< 01 -> Form error
	CANCTRL_EBUSTYPE_STUFF,       ///< 10 -> Stuff error
	CANCTRL_EBUSTYPE_OTHER        ///< 11 -> Other error
} CANBUSERRORTYPE;

/*!
	Macro to obtain bus error direction
*/
#define CANCTRL_BUSERRDIR(busError)  ((busError >> 5) & 0x1)

/*!
 * \brief CAN bus error direction
 * \enum CANBUSERRORDIR
 */
typedef enum
{
	CANCTRL_EBUSDIR_TX = 0,      ///< 0 -> Error occurred during transmitting
	CANCTRL_EBUSDIR_RX           ///< 1 -> Error occurred during receiving
} CANBUSERRORDIR;

/*!
	Macro to obtain bus error code
*/
#define CANCTRL_BUSERRCODE(busError)  ((busError) & 0x1F)

/*!
 * \brief CAN bus error code value
 * \enum CANBUSERRORCODE
 */
typedef enum
{
	CANCTRL_EBUSCODE_SOF    = 3,    ///< 00011 -> Start of Frame
	CANCTRL_EBUSCODE_ID2821 = 2,    ///< 00010 -> ID28 ... ID21
	CANCTRL_EBUSCODE_ID2018 = 6,    ///< 00110 -> ID20 ... ID18
	CANCTRL_EBUSCODE_SRTR   = 4,    ///< 00100 -> SRTR bit
	CANCTRL_EBUSCODE_IDE    = 5,    ///< 00101 -> IDE bit
	CANCTRL_EBUSCODE_ID1713 = 7,    ///< 00111 -> ID17 ... ID13
	CANCTRL_EBUSCODE_ID125  = 15,   ///< 01111 -> ID12 ... ID5
	CANCTRL_EBUSCODE_ID40   = 14,   ///< 01110 -> ID4 .. ID0
	CANCTRL_EBUSCODE_RTR    = 12,   ///< 01100 -> RTR bit
	CANCTRL_EBUSCODE_RSVD1  = 13,   ///< 01101 -> Reserved Bit 1
	CANCTRL_EBUSCODE_RSVD0  = 9,    ///< 01001 -> Reserved Bit 0
	CANCTRL_EBUSCODE_DLC    = 11,   ///< 01011 -> Data Length Code
	CANCTRL_EBUSCODE_DATA   = 10,   ///< 01010 -> Data Field
	CANCTRL_EBUSCODE_CRC    = 8,    ///< 01000 -> CRC Sequence
	CANCTRL_EBUSCODE_CRCD   = 24,   ///< 11000 -> CRC Delimiter
	CANCTRL_EBUSCODE_ACK    = 25,   ///< 11001 -> Acknowledge Slot
	CANCTRL_EBUSCODE_ACKD   = 27,   ///< 11011 -> Acknowledge Delimiter
	CANCTRL_EBUSCODE_EOF    = 26,   ///< 11010 -> End of Frame
	CANCTRL_EBUSCODE_INT    = 18,   ///< 10010 -> Intermission
	CANCTRL_EBUSCODE_AEF    = 17,   ///< 10001 -> Active Error Flag
	CANCTRL_EBUSCODE_PEF    = 22,   ///< 10110 -> Passive Error Flag
	CANCTRL_EBUSCODE_TDB    = 19,   ///< 10011 -> Tolerate Dominant Bits
	CANCTRL_EBUSCODE_ED     = 23,   ///< 10111 -> Error Delimiter
	CANCTRL_EBUSCODE_OVF    = 28    ///< 11100 -> Overload Flag
} CANBUSERRORCODE;

/*!
 * \brief CAN counters selection bits
 * \enum CANCOUNTERTYPE
 */
typedef enum
{
	CANCTRL_CNT_RX_ERROR    = 1,    ///< rx error counter flag
	CANCTRL_CNT_TX_ERROR    = 2,    ///< tx error counter flag
	CANCTRL_CNT_OVR         = 4,    ///< ovverrun counter flag
	CANCTRL_CNT_RX_FRAME    = 8,    ///< received frames counter flag
	CANCTRL_CNT_TX_FRAME    = 16,   ///< transmitted frames counter flag
	CANCTRL_CNT_BUS_OFF     = 32,   ///< bus off counter flag
	CANCTRL_CNT_BUS_WRN     = 64,   ///< bus warnning counter flag
	CANCTRL_CNT_RX_LOST     = 128,  ///< receive lost frame counter flag
	CANCTRL_CNT_TX_LOST     = 256,  ///< transmit lost counter flag
	CANCTRL_CNT_ALL         = 511   ///< all counters flag
} CANCOUNTERTYPE;

typedef DWORD USB2CAN_RET;

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1) // align to 1 byte
/*!
 * \brief CAN message structure
 * \struct CANCTRLMSG
 */
typedef struct
{
	DWORD   id;             ///< identifier
	BYTE    frameInfo;      ///< dlc, rtr, std/ext
	BYTE    data[8];        ///< data bytes
	DWORD   timeStamp;      
	///< 32 bit time stamp  - accuracy -> 1ms
	///< 31-27->rsvd|26-22->Hour|21-16->Min|15-10->Sec|9-0->Ms;
} CANCTRLMSG;

#pragma pack(1) // align to 1 byte
/*!
 * \brief CAN internal counters
 * \struct CANCTRLCNTS
 */
typedef struct
{
	BYTE   cRxErr;       ///< Receive Error Counter
	BYTE   cTxErr;       ///< Transmit Error Counter
	DWORD  cOvr;         ///< Data Overrun Counter
	DWORD  cRxFrames;    ///< Received Frames Counter
	DWORD  cTxFrames;    ///< Transmitted Frames Counter
	DWORD  cBusOff;      ///< Bus Off Counter
	DWORD  cBusWrn;      ///< Bus Warning Counter
	DWORD  cRxLost;      ///< Receive Lost Frames Counter
	DWORD  cTxLost;      ///< Transmit Lost Frames Counter
} CANCTRLCNTS;

#pragma pack(1) // align to 1 byte
/*!
 * \brief CAN bus status
 * \struct CANCTRLSTATUS
 */
typedef struct
{
	BYTE state;     ///<  0 - active, 1 - heavy, 2- passive, 3 - off
	BYTE busError;  ///<  7-6 bit error type, 5 bit error direction, 4-0 error code
} CANCTRLSTATUS;

/*!
 * \brief CAN filter type
 * \enum CANCTRLFILTERTYPE
 */
enum CANCTRLFILTERTYPE
{
	CANCTRLFILTER_REJECT = 0,     ///< reject filter
	CANCTRLFILTER_ACCEPT          ///< accept filter
};

/*! \brief
	This function obtains product description as string.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pProduct - pointer to local buffer
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetProduct (char* pProduct);

/*! \brief
	This function obtains firmware version as string.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pFirmwareVer - pointer to local buffer
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetFirmwareVer (char* pFirmwareVer);

/*! \brief
	This function fetches Library version as integer eg. 1.0.0.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pLibraryVer - pointer to local buffer
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetLibVer (char* pLibraryVer);

/*! \brief
	This function initiates library together with dependencies 
	and exception callback function.
	Shall be executed before opening CAN device.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pExceptionCallback - pointer to callback function
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_InitLib (DWORD (*pExceptionCallback)(DWORD));

/*! \brief
	This function releases resources used by library.
	Shall be executed at the end of the application, after CAN device close procedure.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_ReleaseLib (void);

/*! \brief
	This function reads undirectly (non blocking) frame from CAN device.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pMsg - pointer to CANCTRLMSG message structure
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_Pull (CANCTRLMSG* pMsg);

/*! \brief
	This function writes undirectly (non blocking) frame to CAN device.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param msg - CANCTRLMSG message structure
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_Push (const CANCTRLMSG msg);

/*! \brief
	This function sets internal filter with id range and type.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param idMin - minimal ID value
	\param idMax - maximal ID value
	\param type - filter type
	\param pFilterID - pointer to DWORD where filter identifier will be stored
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_SetFilter (const DWORD idMin,
                               const DWORD idMax,
                               const BYTE type,
                                     DWORD* pFilterID);

/*! \brief
	This function resets internal filter with specified number.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param filterID - filter identifier number
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_ResetFilter (const DWORD filterID);

/*! \brief
	This function gets can controller status bus status.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pBusStat - pointer to CANCTRLSTATUS structure
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_BusStatus (CANCTRLSTATUS* pBusStat);

/*! \brief
	This function gets can controller internal counters.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pCounters - pointer to CANCTRLCNTS structure
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetCounters (CANCTRLCNTS* pCounters);

/*! \brief
	This function resets can controller internal counters.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param cntFlag - CANCOUNTERTYPE value
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_ResetCounters (const SHORT cntFlag);

/*! \brief
	This function gets actual received frames fifo (queue) size.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pFifoInCount - pointer to DWORD where incomming buffer size will be stored
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetRxFifo (DWORD* pFifoInCount);

/*! \brief
	This function gets actual not transmitted frames fifo (queue) size.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param pFifoOutCount - pointer to DWORD where outgoing buffer size will be stored
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_GetTxFifo (DWORD* pFifoOutCount);

/*! \brief
	This function releases actual received frames fifo (queue).
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_ReleaseRxFifo (void);

/*! \brief
	This function releases actual not transmitted frames fifo (queue).
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_ReleaseTxFifo (void);

/*! \brief
	This function resets a CAN device. Shall be called after device open.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param baudRate - human readable baudrate value
	\param operationMode - controller mode
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_Reset (const DWORD baudRate,
                           const BYTE operationMode);

/*! \brief
	This function opens a CAN device, at specified baudrate, in specified mode,
	priority, and maximal fifo size (1 - 65535).
	Shall be executed after Library initiate function.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\param baudRate - human readable baudrate value
	\param operationMode - controller mode
	\param priority - thread priority
	\param fifoSize - buffer size (max 65535 frames)
	\return USB2CAN_LIBSTATUS code
*/
USB2CAN_API
USB2CAN_RET Usb2Can_Open (const DWORD baudRate,
                          const BYTE operationMode,
                          const BYTE priority,
                          const WORD fifoSize);

/*! \brief
	This function closes a CAN device.
	Shall be executed before Library release function.
	Returns USB2CAN_OK when everything is OK, otherwise proper USB2CAN_LIBSTATUS code.
	\return USB2CAN_LIBSTATUS code
*/

USB2CAN_API
USB2CAN_RET Usb2Can_Close (void);


#ifdef __cplusplus
}
#endif

#endif /* _USB2CAN_H_ */
