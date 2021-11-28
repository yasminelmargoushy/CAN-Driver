/************************************************************************************************/
/* Author	: Yasmin El Margoushy						   										*/
/* Driver	: CAN - TM4C123GH6PM																*/
/* Date		: 7 sept 2020								   										*/
/* Version	: V01										   										*/
/************************************************************************************************/
#ifndef CAN_INTERAFCE_H
#define CAN_INTERAFCE_H

/*************************************************************************************************/
/***************************************	CAN Prototypes	**************************************/
/*************************************************************************************************/

// Initializes the Clock for the Appropriate PORT & Configure the Corresponding TX & RX Pins
// Initializes the Clock for the Appropriate CAN unit
// Initializes the CAN Unit by Clearing all MSG Objects
extern void CAN_voidInit(uint8 Copy_u8Unit);

// Trys to find a suitable bit time parameters from a given system clock & desired bit rate
// If it's impossible to operate at this bit rate, then it decreases the desired bit rate
// Return the final Bit rate value
extern uint32 CAN_u32SetBitRate(uint8 Copy_u8Unit, uint32 Copy_u32SourceClock, uint32 Copy_u32BitRate);

// Sets the given bit time parameters, if they are suitable
extern void CAN_voidSetBitTiming(uint8 Copy_u8Unit, uint8 Copy_u8SyncPropPhase1Seg, uint8 Copy_u8Phase2Seg, uint8 Copy_u8SJW, uint16 Copy_u16QuantumPrescaler);

// Enables the Appropriate CAN unit
extern void CAN_voidEnable(uint8 Copy_u8Unit);

// Disables the Appropriate CAN unit
extern void CAN_voidDisable(uint8 Copy_u8Unit);

// Returns one of the Status Registers Depending on the Copy_u8StatusType Parameter
extern uint32 CAN_GetStatus(uint8 Copy_u8Unit, uint8 Copy_u8StatusType);

// Configures the MSG object whose Number is Copy_u8ObjID & Transmits MSG if need be
extern void CAN_voidSetMSG(uint8 Copy_u8Unit, uint8 Copy_u8ObjID, CAN_Msg_Object * ptrCANMsgObject);

// Fills the ptrReturnCANMsgObject with the data & configuration of the MSG Object whose Number is Copy_u8ObjID
extern void CAN_voidGetMSG(uint8 Copy_u8Unit, uint8 Copy_u8ObjID, CAN_Msg_Object * ptrReturnCANMsgObject);

// Sets function that needs to be executed when a interrupt occurs
extern void CAN_voidSetCallBack(uint8 Copy_u8Unit, void (*Copy_ptrFn)(void));

//Clears the Pending Interrupt Flag in the MSG object whose Number is Copy_u8ObjID
extern void CAN_voidClearINTPND(uint8 Copy_u8Unit, uint8 Copy_u8ObjID);


/*************************************************************************************************/
/********************************	CAN_GetStatus Options	**************************************/
/*************************************************************************************************/

// Read the full CAN status.
#define    CAN_STS_TYPE_STS			0

// Read the full CANTXRQ1 & CANTXRQ2 in one 32-bit variable
#define    CAN_STS_TYPE_TXREQUEST	1

// Read the full CANNWDA1 & CANNWDA2 in one 32-bit variable
#define    CAN_STS_TYPE_NEWDAT		2

// Read the full CANMSG1INT & CANMSG2INT in one 32-bit variable
#define    CAN_STS_TYPE_MSGINT		3

// Read the full CANMSG1VAL & CANMSG2VAL in one 32-bit variable
#define    CAN_STS_TYPE_MSGVAL		4

/*************************************************************************************************/
/********************************   CAN_MSG_TYPE Options    **************************************/
/*************************************************************************************************/

// Transmit message object
#define     MSG_TYPE_TX             0

// Receive message object
#define     MSG_TYPE_RX             1

// Remote frame receive remote, with auto-transmit message object
#define     MSG_TYPE_TXRX_REMOTE    2


#endif
