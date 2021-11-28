/************************************************************************************************/
/* Author	: Yasmin El Margoushy						   										*/
/* Driver	: CAN - TM4C123GH6PM																*/
/* Date		: 7 sept 2020								   										*/
/* Version	: V01										   										*/
/************************************************************************************************/
#ifndef CAN_PRIVATE_H
#define CAN_PRIVATE_H


/*************************************************************************************************/
/*****************************************	CAN Structs	******************************************/
/*************************************************************************************************/

typedef struct
{
    // The message identifier.
    uint32 u32MsgID;				//

    // The message identifier mask.
    uint32 u32MsgIDMask;			//
	
	// Pointer to Data
	uint8 * ptru8MsgData;			//

	// The Type of the message object.
    uint8 u8MsgType;		
	
    // The number of bytes of data in the message object.	//
    uint8 u8MsgLen;
	
	// Extended ID
	bool bExtendedID;				//

	// Remote Frame
	bool bRemote;					//

	// Direction in Filter			//
	bool bDirectFilter;

	// Not Last Entry				//
	bool bNotLastEntry;
	
	// Transmit Interrupt Enable
	bool TxINTEnable;				//

	// Receive Interrupt Enable
	bool RxINTEnable;				//
	
}
CAN_Msg_Object;


/*************************************************************************************************/
/***********************************	CAN base addresses	**************************************/
/*************************************************************************************************/

#define CAN0_BASE               0x40040000  // CAN0
#define CAN1_BASE               0x40041000  // CAN1


/*************************************************************************************************/
/*******************************	CAN register offsets	**************************************/
/*************************************************************************************************/

#define CAN_O_CTL               0x00000000  // CAN Control
#define CAN_O_STS               0x00000004  // CAN Status
#define CAN_O_ERR               0x00000008  // CAN Error Counter
#define CAN_O_BIT               0x0000000C  // CAN Bit Timing
#define CAN_O_INT               0x00000010  // CAN Interrupt
#define CAN_O_TST               0x00000014  // CAN Test
#define CAN_O_BRPE              0x00000018  // CAN Baud Rate Prescaler

#define CAN_O_IF1CRQ            0x00000020  // CAN IF1 Command Request
#define CAN_O_IF1CMSK           0x00000024  // CAN IF1 Command Mask
#define CAN_O_IF1MSK1           0x00000028  // CAN IF1 Mask 1
#define CAN_O_IF1MSK2           0x0000002C  // CAN IF1 Mask 2
#define CAN_O_IF1ARB1           0x00000030  // CAN IF1 Arbitration 1
#define CAN_O_IF1ARB2           0x00000034  // CAN IF1 Arbitration 2
#define CAN_O_IF1MCTL           0x00000038  // CAN IF1 Message Control
#define CAN_O_IF1DA1            0x0000003C  // CAN IF1 Data A1
#define CAN_O_IF1DA2            0x00000040  // CAN IF1 Data A2
#define CAN_O_IF1DB1            0x00000044  // CAN IF1 Data B1
#define CAN_O_IF1DB2            0x00000048  // CAN IF1 Data B2


#define CAN_O_IF2CRQ            0x00000080  // CAN IF2 Command Request
#define CAN_O_IF2CMSK           0x00000084  // CAN IF2 Command Mask
#define CAN_O_IF2MSK1           0x00000088  // CAN IF2 Mask 1
#define CAN_O_IF2MSK2           0x0000008C  // CAN IF2 Mask 2
#define CAN_O_IF2ARB1           0x00000090  // CAN IF2 Arbitration 1
#define CAN_O_IF2ARB2           0x00000094  // CAN IF2 Arbitration 2
#define CAN_O_IF2MCTL           0x00000098  // CAN IF2 Message Control
#define CAN_O_IF2DA1            0x0000009C  // CAN IF2 Data A1
#define CAN_O_IF2DA2            0x000000A0  // CAN IF2 Data A2
#define CAN_O_IF2DB1            0x000000A4  // CAN IF2 Data B1
#define CAN_O_IF2DB2            0x000000A8  // CAN IF2 Data B2


#define CAN_O_TXRQ1             0x00000100  // CAN Transmission Request 1
#define CAN_O_TXRQ2             0x00000104  // CAN Transmission Request 2


#define CAN_O_NWDA1             0x00000120  // CAN New Data 1
#define CAN_O_NWDA2             0x00000124  // CAN New Data 2


#define CAN_O_MSG1INT           0x00000140  // CAN Message 1 Interrupt Pending
#define CAN_O_MSG2INT           0x00000144  // CAN Message 2 Interrupt Pending


#define CAN_O_MSG1VAL           0x00000160  // CAN Message 1 Valid
#define CAN_O_MSG2VAL           0x00000164  // CAN Message 2 Valid

/*************************************************************************************************/
/***********************************	External registers 	**************************************/
/*************************************************************************************************/

/***************************	System Control External registers 	******************************/

#define SYSCTL_RCGCGPIO         0x400FE608  // General-Purpose Input/Output Run
                                            // Mode Clock Gating Control
											
#define SYSCTL_RCGCCAN          0x400FE634  // Controller Area Network Run Mode
                                            // Clock Gating Control

#define SYSCTL_RCGC0            0x400FE100  // Run Mode Clock Gating Control
                                            // Register 0									
#define SYSCTL_RCGC2            0x400FE108  // Run Mode Clock Gating Control
                                            // Register 2

/**********	General Purpose Input/Output External registers & offsets 	**************************/

#define GPIO_PORTA_BASE         0x40004000  // GPIO Port A
#define GPIO_PORTB_BASE         0x40005000  // GPIO Port B
#define GPIO_PORTE_BASE         0x40024000  // GPIO Port E
#define GPIO_PORTF_BASE         0x40025000  // GPIO Port F


#define GPIO_O_DIR              0x00000400  // GPIO Direction
#define GPIO_O_AFSEL            0x00000420  // GPIO Alternate Function Select
#define GPIO_O_DEN              0x0000051C  // GPIO Digital Enable
#define GPIO_O_LOCK             0x00000520  // GPIO Lock
#define GPIO_O_CR               0x00000524  // GPIO Commit
#define GPIO_O_AMSEL            0x00000528  // GPIO Analog Mode Select
#define GPIO_O_PCTL             0x0000052C  // GPIO Port Control

/*****************	Nested Vector Interrupt Controller External registers 	**********************/

#define NVIC_EN1                0xE000E104  // Interrupt 32-63 Set Enable

#define NVIC_PRI9               0xE000E424  // Interrupt 36-39 Priority
#define NVIC_PRI10              0xE000E428  // Interrupt 40-43 Priority


/*************************************************************************************************/
/***************************************	External Masks 	**************************************/
/*************************************************************************************************/

/********************************	System Control External Masks 	******************************/

#define SYSCTL_RCGCGPIO_PORTF_M 	0x00000020  	// GPIO Port F Run Mode Clock
													// Gating Control
#define SYSCTL_RCGCGPIO_PORTE_M 	0x00000010  	// GPIO Port E Run Mode Clock
													// Gating Control
#define SYSCTL_RCGCGPIO_PORTB_M 	0x00000002  	// GPIO Port B Run Mode Clock
													// Gating Control
#define SYSCTL_RCGCGPIO_PORTA_M		0x00000001  	// GPIO Port A Run Mode Clock
													// Gating Control

#define SYSCTL_RCGCCAN_CAN1_M       0x00000002  	// CAN Module 1 Run Mode Clock
													// Gating Control
#define SYSCTL_RCGCCAN_CAN0_M       0x00000001  	// CAN Module 0 Run Mode Clock
													// Gating Control

#define SYSCTL_RCGC0_CAN1       0x02000000  // CAN1 Clock Gating Control
#define SYSCTL_RCGC0_CAN0       0x01000000  // CAN0 Clock Gating Control

#define SYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control
#define SYSCTL_RCGC2_GPIOE      0x00000010  // Port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // Port B Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // Port A Clock Gating Control


/**********	General Purpose Input/Output External registers & offsets 	**************************/

#define		CAN0_PORTB_TX_M		0x00000020
#define		CAN0_PORTB_RX_M		0x00000010

#define		CAN0_PORTE_TX_M		0x00000020
#define		CAN0_PORTE_RX_M		0x00000010

#define		CAN0_PORTF_TX_M		0x00000008
#define		CAN0_PORTF_RX_M		0x00000001

#define		CAN1_PORTA_TX_M		0x00000002
#define		CAN1_PORTA_RX_M		0x00000001

/*************************************************************************************************/

#define		CAN0_PORTB_PCTL_TX_M		0x00F00000
#define		CAN0_PORTB_PCTL_RX_M		0x000F0000

#define		CAN0_PORTE_PCTL_TX_M		0x00F00000
#define		CAN0_PORTE_PCTL_RX_M		0x000F0000

#define		CAN0_PORTF_PCTL_TX_M		0x0000000F
#define		CAN0_PORTF_PCTL_RX_M		0x0000F000

#define		CAN1_PORTA_PCTL_TX_M		0x000000F0
#define		CAN1_PORTA_PCTL_RX_M		0x0000000F

/*************************************************************************************************/

#define		CAN0_PORTB_PCTL_TX_S		20
#define		CAN0_PORTB_PCTL_RX_S		16

#define		CAN0_PORTE_PCTL_TX_S		20
#define		CAN0_PORTE_PCTL_RX_S		16

#define		CAN0_PORTF_PCTL_TX_S		0
#define		CAN0_PORTF_PCTL_RX_S		12

#define		CAN1_PORTA_PCTL_TX_S		4
#define		CAN1_PORTA_PCTL_RX_S		0

/*************************************************************************************************/

#define		CAN0_PORTB_PCTL_VALUE		8
#define		CAN0_PORTE_PCTL_VALUE		8
#define		CAN0_PORTF_PCTL_VALUE		3
#define		CAN1_PORTA_PCTL_VALUE		8


/*******************	Nested Vector Interrupt Controller External Masks 	**********************/

#define NVIC_EN1_CAN0_M				0x00000080  	// Interrupt CAN 0 Enable Mask
#define NVIC_EN1_CAN1_M				0x00000100  	// Interrupt CAN 1 Enable Mask

#define NVIC_PRI9_CAN0_M 			0xE0000000		// Interrupt CAN 0 Priority Mask
#define NVIC_PRI10_CAN1_M			0x000000E0		// Interrupt CAN 1 Priority Mask
												
#define NVIC_PRI9_CAN0_S 			29				// Interrupt CAN 0 Priority Shift
#define NVIC_PRI10_CAN1_S			5				// Interrupt CAN 1 Priority Shift													
													

/*************************************************************************************************/
/*****************************************	Bit Masks	******************************************/
/*************************************************************************************************/

/***************************************	CANCTL register	**************************************/

#define CAN_CTL_TEST            0x00000080  // Test Mode Enable
#define CAN_CTL_CCE             0x00000040  // Configuration Change Enable
#define CAN_CTL_DAR             0x00000020  // Disable Automatic-Retransmission
#define CAN_CTL_EIE             0x00000008  // Error Interrupt Enable
#define CAN_CTL_SIE             0x00000004  // Status Interrupt Enable
#define CAN_CTL_IE              0x00000002  // CAN Interrupt Enable
#define CAN_CTL_INIT            0x00000001  // Initialization


/***************************************	CANSTS register	**************************************/

#define CAN_STS_BOFF            0x00000080  // Bus-Off Status
#define CAN_STS_EWARN           0x00000040  // Warning Status
#define CAN_STS_EPASS           0x00000020  // Error Passive
#define CAN_STS_RXOK            0x00000010  // Received a Message Successfully
#define CAN_STS_TXOK            0x00000008  // Transmitted a Message
                                            // Successfully
#define CAN_STS_LEC_M           0x00000007  // Last Error Code
#define CAN_STS_LEC_NONE        0x00000000  // No Error
#define CAN_STS_LEC_STUFF       0x00000001  // Stuff Error
#define CAN_STS_LEC_FORM        0x00000002  // Format Error
#define CAN_STS_LEC_ACK         0x00000003  // ACK Error
#define CAN_STS_LEC_BIT1        0x00000004  // Bit 1 Error
#define CAN_STS_LEC_BIT0        0x00000005  // Bit 0 Error
#define CAN_STS_LEC_CRC         0x00000006  // CRC Error
#define CAN_STS_LEC_NOEVENT     0x00000007  // No Event


/***************************************	CANERR register	**************************************/

#define CAN_ERR_RP              0x00008000  // Received Error Passive
#define CAN_ERR_REC_M           0x00007F00  // Receive Error Counter
#define CAN_ERR_TEC_M           0x000000FF  // Transmit Error Counter
#define CAN_ERR_REC_S           8
#define CAN_ERR_TEC_S           0


/***************************************	CANBIT register	**************************************/

#define CAN_BIT_TSEG2_M         0x00007000  // Time Segment after Sample Point
#define CAN_BIT_TSEG1_M         0x00000F00  // Time Segment Before Sample Point
#define CAN_BIT_SJW_M           0x000000C0  // (Re)Synchronization Jump Width
#define CAN_BIT_BRP_M           0x0000003F  // Baud Rate Prescaler
#define CAN_BIT_TSEG2_S         12
#define CAN_BIT_TSEG1_S         8
#define CAN_BIT_SJW_S           6
#define CAN_BIT_BRP_S           0


/***************************************	CANINT register	**************************************/

#define CAN_INT_INTID_M         0x0000FFFF  // Interrupt Identifier
#define CAN_INT_INTID_NONE      0x00000000  // No interrupt pending
#define CAN_INT_INTID_STATUS    0x00008000  // Status Interrupt


/***************************************	CANTST register	**************************************/

#define CAN_TST_RX              0x00000080  // Receive Observation
#define CAN_TST_TX_M            0x00000060  // Transmit Control
#define CAN_TST_TX_CANCTL       0x00000000  // CAN Module Control
#define CAN_TST_TX_SAMPLE       0x00000020  // Sample Point
#define CAN_TST_TX_DOMINANT     0x00000040  // Driven Low
#define CAN_TST_TX_RECESSIVE    0x00000060  // Driven High
#define CAN_TST_LBACK           0x00000010  // Loopback Mode
#define CAN_TST_SILENT          0x00000008  // Silent Mode
#define CAN_TST_BASIC           0x00000004  // Basic Mode


/*************************************	CANBRPE register	**************************************/

#define CAN_BRPE_BRPE_M         0x0000000F  // Baud Rate Prescaler Extension
#define CAN_BRPE_BRPE_S         0


/*************************************	CANIF1CRQ register	**************************************/

#define CAN_IF1CRQ_BUSY         0x00008000  // Busy Flag
#define CAN_IF1CRQ_MNUM_M       0x0000003F  // Message Number
#define CAN_IF1CRQ_MNUM_S       0


/***************************************	CANIF1CMSK register	**********************************/

#define CAN_IF1CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF1CMSK_MASK        0x00000040  // Access Mask Bits
#define CAN_IF1CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF1CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF1CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit
#define CAN_IF1CMSK_NEWDAT      0x00000004  // Access New Data
#define CAN_IF1CMSK_TXRQST      0x00000004  // Access Transmission Request
#define CAN_IF1CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF1CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7


/***************************************	CANIF1MSK1 register	**********************************/

#define CAN_IF1MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask
#define CAN_IF1MSK1_IDMSK_S     0


/***************************************	CANIF1MSK2 register	**********************************/

#define CAN_IF1MSK2_MXTD        0x00008000  // Mask Extended Identifier
#define CAN_IF1MSK2_MDIR        0x00004000  // Mask Message Direction
#define CAN_IF1MSK2_IDMSK_M     0x00001FFF  // Identifier Mask
#define CAN_IF1MSK2_IDMSK_S     0


/***************************************	CANIF1ARB1 register	**********************************/

#define CAN_IF1ARB1_ID_M        0x0000FFFF  // Message Identifier
#define CAN_IF1ARB1_ID_S        0


/***************************************	CANIF1ARB2 register	**********************************/

#define CAN_IF1ARB2_MSGVAL      0x00008000  // Message Valid
#define CAN_IF1ARB2_XTD         0x00004000  // Extended Identifier
#define CAN_IF1ARB2_DIR         0x00002000  // Message Direction
#define CAN_IF1ARB2_ID_M        0x00001FFF  // Message Identifier
#define CAN_IF1ARB2_ID_S        0


/***************************************	CANIF1MCTL register	**********************************/

#define CAN_IF1MCTL_NEWDAT      0x00008000  // New Data
#define CAN_IF1MCTL_MSGLST      0x00004000  // Message Lost
#define CAN_IF1MCTL_INTPND      0x00002000  // Interrupt Pending
#define CAN_IF1MCTL_UMASK       0x00001000  // Use Acceptance Mask
#define CAN_IF1MCTL_TXIE        0x00000800  // Transmit Interrupt Enable
#define CAN_IF1MCTL_RXIE        0x00000400  // Receive Interrupt Enable
#define CAN_IF1MCTL_RMTEN       0x00000200  // Remote Enable
#define CAN_IF1MCTL_TXRQST      0x00000100  // Transmit Request
#define CAN_IF1MCTL_EOB         0x00000080  // End of Buffer
#define CAN_IF1MCTL_DLC_M       0x0000000F  // Data Length Code
#define CAN_IF1MCTL_DLC_S       0


/*************************************	CANIF1DA1 register	**************************************/

#define CAN_IF1DA1_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DA1_DATA_S       0


/*************************************	CANIF1DA2 register	**************************************/


#define CAN_IF1DA2_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DA2_DATA_S       0


/*************************************	CANIF1DB1 register	**************************************/

#define CAN_IF1DB1_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DB1_DATA_S       0


/*************************************	CANIF1DB2 register	**************************************/

#define CAN_IF1DB2_DATA_M       0x0000FFFF  // Data
#define CAN_IF1DB2_DATA_S       0


/*************************************	CANIF2CRQ register	**************************************/

#define CAN_IF2CRQ_BUSY         0x00008000  // Busy Flag
#define CAN_IF2CRQ_MNUM_M       0x0000003F  // Message Number
#define CAN_IF2CRQ_MNUM_S       0


/****************************************	CANIF2CMSK register	**********************************/

#define CAN_IF2CMSK_WRNRD       0x00000080  // Write, Not Read
#define CAN_IF2CMSK_MASK        0x00000040  // Access Mask Bits
#define CAN_IF2CMSK_ARB         0x00000020  // Access Arbitration Bits
#define CAN_IF2CMSK_CONTROL     0x00000010  // Access Control Bits
#define CAN_IF2CMSK_CLRINTPND   0x00000008  // Clear Interrupt Pending Bit
#define CAN_IF2CMSK_NEWDAT      0x00000004  // Access New Data
#define CAN_IF2CMSK_TXRQST      0x00000004  // Access Transmission Request
#define CAN_IF2CMSK_DATAA       0x00000002  // Access Data Byte 0 to 3
#define CAN_IF2CMSK_DATAB       0x00000001  // Access Data Byte 4 to 7


/***************************************	CANIF2MSK1 register	**********************************/

#define CAN_IF2MSK1_IDMSK_M     0x0000FFFF  // Identifier Mask
#define CAN_IF2MSK1_IDMSK_S     0


/***************************************	CANIF2MSK2 register	**********************************/

#define CAN_IF2MSK2_MXTD        0x00008000  // Mask Extended Identifier
#define CAN_IF2MSK2_MDIR        0x00004000  // Mask Message Direction
#define CAN_IF2MSK2_IDMSK_M     0x00001FFF  // Identifier Mask
#define CAN_IF2MSK2_IDMSK_S     0


/***************************************	CANIF2ARB1 register	**********************************/

#define CAN_IF2ARB1_ID_M        0x0000FFFF  // Message Identifier
#define CAN_IF2ARB1_ID_S        0


/***************************************	CANIF2ARB2 register	**********************************/

#define CAN_IF2ARB2_MSGVAL      0x00008000  // Message Valid
#define CAN_IF2ARB2_XTD         0x00004000  // Extended Identifier
#define CAN_IF2ARB2_DIR         0x00002000  // Message Direction
#define CAN_IF2ARB2_ID_M        0x00001FFF  // Message Identifier
#define CAN_IF2ARB2_ID_S        0


/***************************************	CANIF2MCTL register	**********************************/

#define CAN_IF2MCTL_NEWDAT      0x00008000  // New Data
#define CAN_IF2MCTL_MSGLST      0x00004000  // Message Lost
#define CAN_IF2MCTL_INTPND      0x00002000  // Interrupt Pending
#define CAN_IF2MCTL_UMASK       0x00001000  // Use Acceptance Mask
#define CAN_IF2MCTL_TXIE        0x00000800  // Transmit Interrupt Enable
#define CAN_IF2MCTL_RXIE        0x00000400  // Receive Interrupt Enable
#define CAN_IF2MCTL_RMTEN       0x00000200  // Remote Enable
#define CAN_IF2MCTL_TXRQST      0x00000100  // Transmit Request
#define CAN_IF2MCTL_EOB         0x00000080  // End of Buffer
#define CAN_IF2MCTL_DLC_M       0x0000000F  // Data Length Code
#define CAN_IF2MCTL_DLC_S       0


/*************************************	CANIF2DA1 register	**************************************/

#define CAN_IF2DA1_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DA1_HALFDATA_M   0x000000FF
#define CAN_IF2DA1_DATA_S       0


/*************************************	CANIF2DA2 register	**************************************/

#define CAN_IF2DA2_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DA2_HALFDATA_M   0x000000FF
#define CAN_IF2DA2_DATA_S       0


/*************************************	CANIF2DB1 register	**************************************/

#define CAN_IF2DB1_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DB1_HALFDATA_M   0x000000FF
#define CAN_IF2DB1_DATA_S       0


/*************************************	CANIF2DB2 register	**************************************/

#define CAN_IF2DB2_DATA_M       0x0000FFFF  // Data
#define CAN_IF2DB2_HALFDATA_M   0x000000FF
#define CAN_IF2DB2_DATA_S       0


/*************************************	CANTXRQ1 register	**************************************/

#define CAN_TXRQ1_TXRQST_M      0x0000FFFF  // Transmission Request Bits
#define CAN_TXRQ1_TXRQST_S      0


/*************************************	CANTXRQ2 register	**************************************/

#define CAN_TXRQ2_TXRQST_M      0x0000FFFF  // Transmission Request Bits
#define CAN_TXRQ2_TXRQST_S      0


/*************************************	CANNWDA1 register	**************************************/

#define CAN_NWDA1_NEWDAT_M      0x0000FFFF  // New Data Bits
#define CAN_NWDA1_NEWDAT_S      0


/*************************************	CANNWDA2 register	**************************************/

#define CAN_NWDA2_NEWDAT_M      0x0000FFFF  // New Data Bits
#define CAN_NWDA2_NEWDAT_S      0


/***************************************	CANMSG1INT register	**********************************/

#define CAN_MSG1INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits
#define CAN_MSG1INT_INTPND_S    0


/***************************************	CANMSG2INT register	**********************************/

#define CAN_MSG2INT_INTPND_M    0x0000FFFF  // Interrupt Pending Bits
#define CAN_MSG2INT_INTPND_S    0


/***************************************	CANMSG1VAL register	**********************************/

#define CAN_MSG1VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits
#define CAN_MSG1VAL_MSGVAL_S    0


/***************************************	CANMSG2VAL register	**********************************/

#define CAN_MSG2VAL_MSGVAL_M    0x0000FFFF  // Message Valid Bits
#define CAN_MSG2VAL_MSGVAL_S    0



/*************************************************************************************************/
/***************************************	CAN Constants	**************************************/
/*************************************************************************************************/

#define CAN_MAX_BIT_DIVISOR     19
#define CAN_MIN_BIT_DIVISOR     4
#define CAN_MAX_PRE_DIVISOR     1024
#define CAN_MIN_PRE_DIVISOR     1
#define CAN_MAX_11BIT_MSG_ID    0x7FF

/*************************************************************************************************/
/***************************************	CAN Macros	******************************************/
/*************************************************************************************************/

#define CAN_BIT_VALUE(seg1, seg2, sjw)                                        					\
                                ((((seg1 - 1) << CAN_BIT_TSEG1_S) & CAN_BIT_TSEG1_M) |			\
                                 (((seg2 - 1) << CAN_BIT_TSEG2_S) & CAN_BIT_TSEG2_M) |			\
                                 (((sjw - 1)  << CAN_BIT_SJW_S)   & CAN_BIT_SJW_M))

/*************************************************************************************************/
/**********************************	CAN Time Bit Table	******************************************/
/*************************************************************************************************/
// It's Better to divide Number of Quantum between Phase 1 & Phase 2 Equally.
//
// But when Calculating TSEG1 we must take into consideration the propagation delay as well.
// Which causes TSEG1 to almost always be greater than TSEG2 unless stated otherwise.
//
// If we assumed the smallest propagation delay then we can assume that the propagation delay is 1 or 2 Quantum
// Based on these Assumtions the value of TSEG1 is always Greater than TSEG2 by either 1 or 2.
//
// The value of SJW is the least of Phase 1 & Phase 2 & 4, so at First it's equal to TSEG2 (Phase 2)
// then it becomes stable at 4.

const uint16 CAN_u16TimeBitValues[] =
{
    CAN_BIT_VALUE(2, 1, 1),     // 4 clocks/bit     // Minimum number of Quantum per Bit is 4
    CAN_BIT_VALUE(3, 1, 1),     // 5 clocks/bit
    CAN_BIT_VALUE(3, 2, 2),     // 6 clocks/bit
    CAN_BIT_VALUE(4, 2, 2),     // 7 clocks/bit
    CAN_BIT_VALUE(4, 3, 3),     // 8 clocks/bit
    CAN_BIT_VALUE(5, 3, 3),     // 9 clocks/bit
    CAN_BIT_VALUE(5, 4, 4),     // 10 clocks/bit
    CAN_BIT_VALUE(6, 4, 4),     // 11 clocks/bit
    CAN_BIT_VALUE(6, 5, 4),     // 12 clocks/bit
    CAN_BIT_VALUE(7, 5, 4),     // 13 clocks/bit
    CAN_BIT_VALUE(7, 6, 4),     // 14 clocks/bit
    CAN_BIT_VALUE(8, 6, 4),     // 15 clocks/bit
    CAN_BIT_VALUE(8, 7, 4),     // 16 clocks/bit
    CAN_BIT_VALUE(9, 7, 4),     // 17 clocks/bit
    CAN_BIT_VALUE(9, 8, 4),     // 18 clocks/bit
    CAN_BIT_VALUE(10, 8, 4)     // 19 clocks/bit    // Maximum Value for TSEG2 is 8 Quantum (3-Bits in the CANBIT Register)
};

/*************************************************************************************************/
/*******************************	CAN Configuration Defines	**********************************/
/*************************************************************************************************/

#define		IF_1_SET_IF_2_GET	    0
#define		IF_2_SET_IF_1_GET	    1

#define 	CAN0_PORTB		    	0
#define 	CAN0_PORTE	    		1
#define 	CAN0_PORTF  			2

#define     CAN_ERROR_INT_DISABLE   0
#define     CAN_ERROR_INT_ENABLE    1

#endif
