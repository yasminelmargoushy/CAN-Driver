/********************************************************************************************************************/
/* Author	: Yasmin El Margoushy						   															*/
/* Driver	: CAN - TM4C123GH6PM																					*/
/* Date		: 7 sept 2020								   															*/
/* Version	: V01										   															*/
/********************************************************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

#include "STD_TYPES.h"
#include "BIT_MATH.h"

#include "CAN_config.h"
#include "CAN_private.h"
#include "CAN_interface.h"


void (*CAN0_CallBack) (void);
void (*CAN1_CallBack) (void);


void CAN_voidInit(uint8 Copy_u8Unit)
{
    uint32 Local_u32Msg;
	uint32 Local_u32Base;
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) || (Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			// Allow Clock Gating for CAN 0
			ACC_REG(SYSCTL_RCGCCAN) |= SYSCTL_RCGCCAN_CAN0_M;
			while ((ACC_REG(SYSCTL_RCGC0) & SYSCTL_RCGC0_CAN0) == 0);

			#if CAN0_PORT == CAN0_PORTB
				// Allow Clock Gating to GPIO Port B
				ACC_REG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_PORTB_M;
				while ((ACC_REG(SYSCTL_RCGC2) & SYSCTL_RCGC2_GPIOB) == 0);
				
				/************ Configure Pins ***************/
				// Configure Direction TX -> Output & RX -> Input
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_DIR) |= CAN0_PORTB_TX_M;
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_DIR) &= ~CAN0_PORTB_RX_M;
				
				// Select Alternative Function
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_AFSEL) |= (CAN0_PORTB_TX_M | CAN0_PORTB_RX_M);
				
				// Select Digital Enable
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_DEN) |= (CAN0_PORTB_TX_M | CAN0_PORTB_RX_M);
				
				// Disable Analog Mode
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_AMSEL) &= ~(CAN0_PORTB_TX_M | CAN0_PORTB_RX_M);
				
				// Select CAN Alternative Function
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_PCTL) &= ~(CAN0_PORTB_PCTL_TX_M | CAN0_PORTB_PCTL_RX_M);
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_PCTL) |= ((CAN0_PORTB_PCTL_VALUE << CAN0_PORTB_PCTL_RX_S) & CAN0_PORTB_PCTL_RX_M);
				ACC_REG(GPIO_PORTB_BASE + GPIO_O_PCTL) |= ((CAN0_PORTB_PCTL_VALUE << CAN0_PORTB_PCTL_TX_S) & CAN0_PORTB_PCTL_TX_M);
				
			#elif CAN0_PORT == CAN0_PORTE
				// Allow Clock Gating to GPIO Port E
				ACC_REG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_PORTE_M;
				while ((ACC_REG(SYSCTL_RCGC2) & SYSCTL_RCGC2_GPIOE) == 0);
				
				/************ Configure Pins ***************/
				// Configure Direction TX -> Output & RX -> Input
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_DIR) |= CAN0_PORTE_TX_M;
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_DIR) &= ~CAN0_PORTE_RX_M;
				
				// Select Alternative Function
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_AFSEL) |= (CAN0_PORTE_TX_M | CAN0_PORTE_RX_M);
				
				// Select Digital Enable
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_DEN) |= (CAN0_PORTE_TX_M | CAN0_PORTE_RX_M);
				
				// Disable Analog Mode
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_AMSEL) &= ~(CAN0_PORTE_TX_M | CAN0_PORTE_RX_M);
				
				// Select CAN Alternative Function
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_PCTL) &= ~(CAN0_PORTE_PCTL_TX_M | CAN0_PORTE_PCTL_RX_M);
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_PCTL) |= ((CAN0_PORTE_PCTL_VALUE << CAN0_PORTE_PCTL_RX_S) & CAN0_PORTE_PCTL_RX_M);
				ACC_REG(GPIO_PORTE_BASE + GPIO_O_PCTL) |= ((CAN0_PORTE_PCTL_VALUE << CAN0_PORTE_PCTL_TX_S) & CAN0_PORTE_PCTL_TX_M);
				
			#else
				// Allow Clock Gating to GPIO Port F
				ACC_REG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_PORTF_M;
				while ((ACC_REG(SYSCTL_RCGC2) & SYSCTL_RCGC2_GPIOF) == 0);
				
				/************ Configure Pins ***************/
				// Configure Direction TX -> Output & RX -> Input
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_DIR) |= CAN0_PORTF_TX_M;
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_DIR) &= ~CAN0_PORTF_RX_M;
				
				// Select Alternative Function
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_AFSEL) |= (CAN0_PORTF_TX_M | CAN0_PORTF_RX_M);
				
				// Select Digital Enable
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (CAN0_PORTF_TX_M | CAN0_PORTF_RX_M);
				
				// Disable Analog Mode
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_AMSEL) &= ~(CAN0_PORTF_TX_M | CAN0_PORTF_RX_M);
				
				// Select CAN Alternative Function
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_PCTL) &= ~(CAN0_PORTF_PCTL_TX_M | CAN0_PORTF_PCTL_RX_M);
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_PCTL) |= ((CAN0_PORTF_PCTL_VALUE << CAN0_PORTF_PCTL_RX_S) & CAN0_PORTF_PCTL_RX_M);
				ACC_REG(GPIO_PORTF_BASE + GPIO_O_PCTL) |= ((CAN0_PORTF_PCTL_VALUE << CAN0_PORTF_PCTL_TX_S) & CAN0_PORTF_PCTL_TX_M);
				
			#endif
			
			// Set Base Address
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			// Allow Clock Gating for CAN 1
			ACC_REG(SYSCTL_RCGCCAN) |= SYSCTL_RCGCCAN_CAN1_M;
			while ((ACC_REG(SYSCTL_RCGC0) & SYSCTL_RCGC0_CAN1) == 0);
			
			// Allow Clock Gating to GPIO Port A
			ACC_REG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_PORTA_M;
			while ((ACC_REG(SYSCTL_RCGC2) & SYSCTL_RCGC2_GPIOA) == 0);
							
			/************ Configure Pins ***************/
			// Configure Direction TX -> Output & RX -> Input
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_DIR) |= CAN1_PORTA_TX_M;
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_DIR) &= ~CAN1_PORTA_RX_M;
			
			// Select Alternative Function
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= (CAN1_PORTA_TX_M | CAN1_PORTA_RX_M);
			
			// Select Digital Enable
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_DEN) |= (CAN1_PORTA_TX_M | CAN1_PORTA_RX_M);
			
			// Disable Analog Mode
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_AMSEL) &= ~(CAN1_PORTA_TX_M | CAN1_PORTA_RX_M);
			
			// Select CAN Alternative Function
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_PCTL) &= ~(CAN1_PORTA_PCTL_TX_M | CAN1_PORTA_PCTL_RX_M);
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_PCTL) |= ((CAN1_PORTA_PCTL_VALUE << CAN1_PORTA_PCTL_RX_S) & CAN1_PORTA_PCTL_RX_M);
			ACC_REG(GPIO_PORTA_BASE + GPIO_O_PCTL) |= ((CAN1_PORTA_PCTL_VALUE << CAN1_PORTA_PCTL_TX_S) & CAN1_PORTA_PCTL_TX_M);
				
			
			// Set Base Address
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}
	
    // Place CAN controller in Initialization Mode
	ACC_REG(Local_u32Base + CAN_O_CTL) = CAN_CTL_INIT;

    // Wait for busy bit to clear
    while(ACC_REG(Local_u32Base + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY);
	
	
	// Clear the message value bit in the arbitration register.
    ACC_REG(Local_u32Base + CAN_O_IF1CMSK) = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_ARB | CAN_IF1CMSK_CONTROL);
    ACC_REG(Local_u32Base + CAN_O_IF1ARB2) = 0;
    ACC_REG(Local_u32Base + CAN_O_IF1MCTL) = 0;

    // Loop through all 32 message objects
    for(Local_u32Msg = 1; Local_u32Msg <= 32; Local_u32Msg++)
    {
        // Select & Clear the message object
        ACC_REG(Local_u32Base + CAN_O_IF1CRQ) = Local_u32Msg;
		
		// Wait for busy bit to clear
        while(ACC_REG(Local_u32Base + CAN_O_IF1CRQ) & CAN_IF1CRQ_BUSY);
    }

}

/********************************************************************************************************************/

uint32 CAN_u32SetBitRate(uint8 Copy_u8Unit, uint32 Copy_u32SourceClock, uint32 Copy_u32BitRate)
{
    uint32 Local_u32Base;
    uint32 Local_u32DesiredRatio;
    uint32 Local_u32NumQuantums;
    uint32 Local_u32ActualPreDivide;
    uint32 Local_u32RegValue;


    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));

    // Set the Base Address
    switch(Copy_u8Unit)
    {
        case 0:
            Local_u32Base = CAN0_BASE;
            break;
        case 1:
            Local_u32Base = CAN1_BASE;
            break;
        default:
            return 0;
    }

    // Calculate the desired clock ratio.
    Local_u32DesiredRatio = Copy_u32SourceClock / Copy_u32BitRate;

    // Check all possible values to find a matching value.
    while(Local_u32DesiredRatio <= (CAN_MAX_PRE_DIVISOR * CAN_MAX_BIT_DIVISOR))
    {
        // Loop through all possible CAN bit divisors.
        for(Local_u32NumQuantums = CAN_MAX_BIT_DIVISOR; Local_u32NumQuantums >= CAN_MIN_BIT_DIVISOR; Local_u32NumQuantums--)
        {
            // For a given CAN bit divisor save the pre-divisor.
            // The Desired Ratio MUST be bigger than or equal the number of Quantums per bit.
            // Otherwise this means that the System clock & the Bit rate are close to each other & the processor can't divide the bit into quantum.
            // If the Desired Ratio MUST be Smaller than the number of Quantums per bit,the this nummber of Quantum is is unacceptable.
            Local_u32ActualPreDivide = Local_u32DesiredRatio / Local_u32NumQuantums;


            // If the calculated divisors match the desired clock ratio then return these bit rate and set the CAN bit timing.
            if((Local_u32ActualPreDivide * Local_u32NumQuantums) == Local_u32DesiredRatio)
            {
                // Set the bit timing values.
                Local_u32RegValue = CAN_u16TimeBitValues[Local_u32NumQuantums - CAN_MIN_BIT_DIVISOR];

                // To set the bit timing register, the controller must be
                // placed in Initialization mode, and also configuration change bit enabled.
                ACC_REG(Local_u32Base + CAN_O_CTL) |= (CAN_CTL_INIT | CAN_CTL_CCE);

                // Set  the pre-scalar on the bit rate.
                Local_u32RegValue |= ((Local_u32ActualPreDivide - 1) & CAN_BIT_BRP_M);

                // place in the obtained value in the CANBIT & CANBRPE registers.
                ACC_REG(Local_u32Base + CAN_O_BIT) = Local_u32RegValue;
                ACC_REG(Local_u32Base + CAN_O_BRPE) = ((Local_u32ActualPreDivide - 1) >> 6) & CAN_BRPE_BRPE_M;

                // Restore the CANCTL register.
                ACC_REG(Local_u32Base + CAN_O_CTL) &= ~ (CAN_CTL_INIT | CAN_CTL_CCE);

                // Return the Resulting bit rate.
                return(Copy_u32SourceClock / (Local_u32ActualPreDivide * Local_u32NumQuantums));
            }
        }
        // This means that the Desired Bit Rate is too High (Too close to the system Clock)
        // So there is no sufficient clock cycles for the processor to divde the bit into at least 4 quantum
        // to solve this problem we decrease the Desired Bit rate by increasing the Divisor
        Local_u32DesiredRatio++;
    }

    // A valid combination could not be found
    return(0);
}

/********************************************************************************************************************/

void CAN_voidSetBitTiming(uint8 Copy_u8Unit, uint8 Copy_u8SyncPropPhase1Seg, uint8 Copy_u8Phase2Seg, uint8 Copy_u8SJW, uint16 Copy_u16QuantumPrescaler)
{
    uint32 Local_u32Base;
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}

    // Check phase 1 segment is in the range from 2 to 16.
    assert((Copy_u8SyncPropPhase1Seg >= 2) && (Copy_u8SyncPropPhase1Seg <= 16));

    // Check phase 2 segment is in the range from 1 to 8.
    assert((Copy_u8Phase2Seg >= 1) &&(Copy_u8Phase2Seg <= 8));

    // The synchronous jump windows must be in the range from 1 to 4.
    assert((Copy_u8SJW >= 1) && (Copy_u8SJW <= 4));

    // The CAN clock pre-divider must be in the range from 1 to 1024.
    assert((Copy_u16QuantumPrescaler <= 1024) && (Copy_u16QuantumPrescaler >= 1));

	// To set the bit timing register, the controller must be
	// placed in Initialization mode, and also configuration change bit enabled. 
    ACC_REG(Local_u32Base + CAN_O_CTL) |= (CAN_CTL_INIT | CAN_CTL_CCE);

    // Set the bit fields of the bit timing register according to the parms.
	ACC_REG(Local_u32Base + CAN_O_BIT) = CAN_BIT_VALUE(Copy_u8SyncPropPhase1Seg, Copy_u8Phase2Seg, Copy_u8SJW);
    ACC_REG(Local_u32Base + CAN_O_BIT) |= (Copy_u16QuantumPrescaler - 1) & CAN_BIT_BRP_M;

    // Set the divider upper bits in the extension register.
    ACC_REG(Local_u32Base + CAN_O_BRPE) = ((Copy_u16QuantumPrescaler - 1) >> 6) & CAN_BRPE_BRPE_M;

	// Restore the CANCTL register.
    ACC_REG(Local_u32Base + CAN_O_CTL) &= ~(CAN_CTL_INIT | CAN_CTL_CCE);
}

/********************************************************************************************************************/

void CAN_voidEnable(uint8 Copy_u8Unit)
{
    uint32 Local_u32Base;
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}

    // Clear the init bit in the control register.
    ACC_REG(Local_u32Base + CAN_O_CTL) &= ~CAN_CTL_INIT;
}

/********************************************************************************************************************/

void CAN_voidDisable(uint8 Copy_u8Unit)
{
    uint32 Local_u32Base;
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}
	
    // Set the init bit in the control register.
    ACC_REG(Local_u32Base + CAN_O_CTL) |= CAN_CTL_INIT;
}

/********************************************************************************************************************/

uint32 CAN_GetStatus(uint8 Copy_u8Unit, uint8 Copy_u8StatusType)
{
    uint32 Local_u32Status;
    uint32 Local_u32Base;
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return 0;
	}

    switch(Copy_u8StatusType)
    {
        // Read the full CAN status.
        case CAN_STS_TYPE_STS:
        {
            Local_u32Status = ACC_REG(Local_u32Base + CAN_O_STS);
            ACC_REG(Local_u32Base + CAN_O_STS) = ~(CAN_STS_RXOK | CAN_STS_TXOK | CAN_STS_LEC_M);
            break;
        }

        // Read the full CANTXRQ1 & CANTXRQ2 in one 32-bit variable
        case CAN_STS_TYPE_TXREQUEST:
        {
            Local_u32Status = ACC_REG(Local_u32Base + CAN_O_TXRQ1) |(ACC_REG(Local_u32Base + CAN_O_TXRQ2) << 16);
            break;
        }

		// Read the full CANNWDA1 & CANNWDA2 in one 32-bit variable
        case CAN_STS_TYPE_NEWDAT:
        {
            Local_u32Status = ACC_REG(Local_u32Base + CAN_O_NWDA1) | (ACC_REG(Local_u32Base + CAN_O_NWDA2) << 16);
            break;
        }
		
		// Read the full CANMSG1INT & CANMSG2INT in one 32-bit variable
        case CAN_STS_TYPE_MSGINT:
        {
            Local_u32Status = ACC_REG(Local_u32Base + CAN_O_MSG1INT) | (ACC_REG(Local_u32Base + CAN_O_MSG2INT) << 16);
            break;
        }
		
		// Read the full CANMSG1VAL & CANMSG2VAL in one 32-bit variable
        case CAN_STS_TYPE_MSGVAL:
        {
            Local_u32Status = ACC_REG(Local_u32Base + CAN_O_MSG1VAL) | (ACC_REG(Local_u32Base + CAN_O_MSG2VAL) << 16);
            break;
        }

        // Unknown CAN status requested so return 0.
        default:
        {
            Local_u32Status = 0;
            break;
        }
    }
    return(Local_u32Status);
}

/********************************************************************************************************************/

void CAN_voidSetMSG(uint8 Copy_u8Unit, uint8 Copy_u8ObjID, CAN_Msg_Object * ptrCANMsgObject)
{
    uint32 Local_u32Base;
	uint16 Local_u16CMSKReg;
    uint16 Local_u16MSKReg1, Local_u16MSKReg2;
    uint16 Local_u16ARBReg1, Local_u16ARBReg2;
    uint16 Local_u16MCTLReg;
    bool bTransferData = 0;
	
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}

	// Check on Object ID
    assert((Copy_u8ObjID <= 32) && (Copy_u8ObjID != 0));
	
	// Check on Object Type
    assert((ptrCANMsgObject->u8MsgType == MSG_TYPE_TX) ||
           (ptrCANMsgObject->u8MsgType == MSG_TYPE_RX) ||
           (ptrCANMsgObject->u8MsgType == MSG_TYPE_TXRX_REMOTE));

    // Write to the Message object
	// Set the Arb bit so that this gets transferred to the Message object
	// Set the 2 DATA bits DATAA & DATAB
	// Set the CONTROL bit
	#if INTERFACE_FUNCTIONS == IF_2_SET_IF_1_GET
		ACC_REG(Local_u32Base + CAN_O_IF2CMSK) = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL | CAN_IF1CMSK_ARB);
		Local_u16CMSKReg = ACC_REG(Local_u32Base + CAN_O_IF2CMSK);
	#else
		ACC_REG(Local_u32Base + CAN_O_IF1CMSK) = (CAN_IF1CMSK_WRNRD | CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL | CAN_IF1CMSK_ARB);
		Local_u16CMSKReg = ACC_REG(Local_u32Base + CAN_O_IF1CMSK);
	#endif

		Local_u16CMSKReg =
    // Initialize Control signals in registers
    Local_u16ARBReg1 = 0;
    Local_u16ARBReg2 = 0;
    Local_u16MCTLReg = 0;
    Local_u16MSKReg1 = 0;
    Local_u16MSKReg2 = 0;

    switch(ptrCANMsgObject->u8MsgType)
    {
        // Transmit message object.
        case MSG_TYPE_TX:
        {
            // Set the TXRQST bit and the reset the rest of the register
            Local_u16MCTLReg = CAN_IF1MCTL_TXRQST;
            if (ptrCANMsgObject-> bRemote)
            {
                // Clear the DIR bit along with everything else
                Local_u16ARBReg2 = 0;
            }
            else
            {
                // Set the DIR bit to send Data Frame
                Local_u16ARBReg2 = CAN_IF1ARB2_DIR;

                // The Data Needs to be filled
                bTransferData = 1;
            }
            break;
        }

        // Receive message object.
        case MSG_TYPE_RX:
        {
            if (ptrCANMsgObject-> bRemote)
            {
                // Set the DIR bit to send Remote frame
                Local_u16ARBReg2 = CAN_IF1ARB2_DIR;

                // Use mask in acceptance filtering
                Local_u16MCTLReg = CAN_IF1MCTL_UMASK;

                // Access Mask Bits
                Local_u16CMSKReg |= CAN_IF1CMSK_MASK;
            }
            else
            {
                // Clear the DIR bit along with everything else
                Local_u16ARBReg2 = 0;
            }

            break;
        }

        // Remote frame receive remote, with auto-transmit message object
        case MSG_TYPE_TXRX_REMOTE:
        {
            // The DIR bit is set for auto-transmit once a remote frame is detected
            Local_u16ARBReg2 = CAN_IF1ARB2_DIR;

            // Set Remote Enable
            Local_u16MCTLReg = CAN_IF1MCTL_RMTEN;

            // Use mask in acceptance filtering
            Local_u16MCTLReg |= CAN_IF1MCTL_UMASK;

            // Access Mask Bits
            Local_u16CMSKReg |= CAN_IF1CMSK_MASK;

            // The data to be returned needs to be filled in.
            bTransferData = 1;
            break;
        }

        // never happens due to the assert statement
        default:
        {
            return;
        }
    }


    // Use Direction in acceptance filtering
    if(ptrCANMsgObject->bDirectFilter)
    {
        Local_u16MSKReg2 |= CAN_IF1MSK2_MDIR;
    }

    // Check if we need to use an extended identifier
    if((ptrCANMsgObject->u32MsgID > CAN_MAX_11BIT_MSG_ID) || (ptrCANMsgObject->bExtendedID))
    {
        // Set the 29 bits of Identifier mask
        Local_u16MSKReg1 = ptrCANMsgObject->u32MsgIDMask & CAN_IF1MSK1_IDMSK_M;
        Local_u16MSKReg2 |= ((ptrCANMsgObject->u32MsgIDMask >> 16) & CAN_IF1MSK2_IDMSK_M);

        // Set Mask Extended Identifier
        Local_u16MSKReg2 |= CAN_IF1MSK2_MXTD;

        // Set the 29 bits of Arbitration
        Local_u16ARBReg1 = ptrCANMsgObject->u32MsgID & CAN_IF1ARB1_ID_M;
        Local_u16ARBReg2 |= (ptrCANMsgObject->u32MsgID >> 16) & CAN_IF1ARB2_ID_M;

        // Set valid & Extended bits
        Local_u16ARBReg2 |= CAN_IF1ARB2_MSGVAL | CAN_IF1ARB2_XTD;
    }
    else
    {
        /****** Set the 11 bits of Identifier mask ******/
        // Clear the 16 bits Identifier mask in bits [15:0] of CANIFnMSK1
        Local_u16MSKReg1 = 0;

        // Put the 11 bit Identifier mask in bits [12:2] of CANIFnMSK2
        Local_u16MSKReg2 |= ((ptrCANMsgObject->u32MsgIDMask << 2) & CAN_IF1MSK2_IDMSK_M);


        /****** Set the 11 bits of Arbitration ******/
        // Clear the 16 bits of Arbitration in bits [15:0] of CANIFnARB1
        Local_u16ARBReg1 = 0;

        // Put the 11 bits of Arbitration in bits [12:2] of CANIF1ARB2
        Local_u16ARBReg2 |= (ptrCANMsgObject->u32MsgID << 2) & CAN_IF1ARB2_ID_M;

        // Set valid bit
        Local_u16ARBReg2 |= CAN_IF1ARB2_MSGVAL;
    }

    // Set the data length
    Local_u16MCTLReg |= (ptrCANMsgObject->u8MsgLen & CAN_IF1MCTL_DLC_M);

    // Mark this as the last entry if this is not the last entry in a FIFO.
    if(!(ptrCANMsgObject->bNotLastEntry))
    {
        Local_u16MCTLReg |= CAN_IF1MCTL_EOB;
    }

    // Enable transmit interrupts if they should be enabled.
    if(ptrCANMsgObject->TxINTEnable)
    {
        Local_u16MCTLReg |= CAN_IF1MCTL_TXIE;
    }

    // Enable receive interrupts if they should be enabled.
    if(ptrCANMsgObject->RxINTEnable)
    {
        Local_u16MCTLReg |= CAN_IF1MCTL_RXIE;
    }
	
	// CAN Unit Control & NVIC Control Enable
	if ((ptrCANMsgObject->RxINTEnable) || (ptrCANMsgObject->TxINTEnable))
	{
	    // Enable CAN Unit Interrupt
	    ACC_REG(Local_u32Base + CAN_O_CTL) |= (CAN_CTL_IE | CAN_CTL_SIE);

	    switch(Copy_u8Unit)
		{
			case 0:
			    // Enable Error Interrupts
                #if CAN0_ERROR_INT == CAN_ERROR_INT_ENABLE
			        ACC_REG(Local_u32Base + CAN_O_CTL) |= CAN_CTL_EIE;
                #endif

			    // Set Priority
				ACC_REG(NVIC_PRI9) &= ~NVIC_PRI9_CAN0_M;
				ACC_REG(NVIC_PRI9) |= ((CAN0_PRIORITY << NVIC_PRI9_CAN0_S) & NVIC_PRI9_CAN0_M);
				
				// Enable Interrupt from NVIC
				ACC_REG(NVIC_EN1) |= NVIC_EN1_CAN0_M;
				break;
			
			case 1:
			    // Enable Error Interrupts
                #if CAN1_ERROR_INT == CAN_ERROR_INT_ENABLE
                    ACC_REG(Local_u32Base + CAN_O_CTL) |= CAN_CTL_EIE;
                #endif

				// Set Priority
				ACC_REG(NVIC_PRI10) &= ~NVIC_PRI10_CAN1_M;
				ACC_REG(NVIC_PRI10) |= ((CAN1_PRIORITY << NVIC_PRI10_CAN1_S) & NVIC_PRI10_CAN1_M);
				
				// Enable Interrupt from NVIC
				ACC_REG(NVIC_EN1) |= NVIC_EN1_CAN1_M;
				break;
			
			default:
				break;
		}
	}

    // Write the data out to the CAN Data registers if needed.
    if(bTransferData)
    {
		uint8 * Local_ptru8SourceData = ptrCANMsgObject -> ptru8MsgData;
        #if INTERFACE_FUNCTIONS == IF_2_SET_IF_1_GET
		    uint32 * Local_ptru32DistData = (uint32 *) (Local_u32Base + CAN_O_IF2DA1);
        #else
		    uint32 * Local_ptru32DistData = (uint32 *)(Local_u32Base + CAN_O_IF1DA1);
        #endif
		uint8 Local_u8Counter = 0;
		
		for(Local_u8Counter = 0; Local_u8Counter < ptrCANMsgObject -> u8MsgLen; Local_u8Counter++)
		{
			uint16 Local_u16TempData = 0;
			
			// Write the data 16 bits at a time
			Local_u16TempData = Local_ptru8SourceData[Local_u8Counter];
			Local_u8Counter++;

			// but write the second byte if needed otherwise the value is zero.
			if(Local_u8Counter < ptrCANMsgObject -> u8MsgLen)
			{
				Local_u16TempData |= (Local_ptru8SourceData[Local_u8Counter] << 8);
			}

			ACC_REG(Local_ptru32DistData++) = Local_u16TempData;
		}
    }
	#if INTERFACE_FUNCTIONS == IF_2_SET_IF_1_GET
        // Write on the registers
        ACC_REG(Local_u32Base + CAN_O_IF2CMSK) = Local_u16CMSKReg;
        ACC_REG(Local_u32Base + CAN_O_IF2MSK1) = Local_u16MSKReg1;
        ACC_REG(Local_u32Base + CAN_O_IF2MSK2) = Local_u16MSKReg2;
        ACC_REG(Local_u32Base + CAN_O_IF2ARB1) = Local_u16ARBReg1;
        ACC_REG(Local_u32Base + CAN_O_IF2ARB2) = Local_u16ARBReg2;
        ACC_REG(Local_u32Base + CAN_O_IF2MCTL) = Local_u16MCTLReg;
	#else
        // Write on the registers
        ACC_REG(Local_u32Base + CAN_O_IF1CMSK) = Local_u16CMSKReg;
        ACC_REG(Local_u32Base + CAN_O_IF1MSK1) = Local_u16MSKReg1;
        ACC_REG(Local_u32Base + CAN_O_IF1MSK2) = Local_u16MSKReg2;
        ACC_REG(Local_u32Base + CAN_O_IF1ARB1) = Local_u16ARBReg1;
        ACC_REG(Local_u32Base + CAN_O_IF1ARB2) = Local_u16ARBReg2;
        ACC_REG(Local_u32Base + CAN_O_IF1MCTL) = Local_u16MCTLReg;
	#endif

    // Transfer the message object
	#if INTERFACE_FUNCTIONS == IF_2_SET_IF_1_GET
		ACC_REG(Local_u32Base + CAN_O_IF2CRQ) = Copy_u8ObjID & CAN_IF1CRQ_MNUM_M;
	#else
		ACC_REG(Local_u32Base + CAN_O_IF1CRQ) = Copy_u8ObjID & CAN_IF1CRQ_MNUM_M;
	#endif
}

/********************************************************************************************************************/

void CAN_voidGetMSG(uint8 Copy_u8Unit, uint8 Copy_u8ObjID, CAN_Msg_Object * ptrReturnCANMsgObject)
{
    uint32 Local_u32Base;
    uint16 Local_u16MSKReg1, Local_u16MSKReg2;
    uint16 Local_u16ARBReg1, Local_u16ARBReg2;
    uint16 Local_u16MCTLReg;
	
	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			Local_u32Base = CAN0_BASE;
			break;
		case 1:
			Local_u32Base = CAN1_BASE;
			break;
		default:
			return;
	}
	
	// Check on Object ID
    assert((Copy_u8ObjID <= 32) && (Copy_u8ObjID != 0));

    // Read the Message object
	// Set the Arb bit so that this gets transferred to the Message object
	// Set the 2 DATA bits DATAA & DATAB
	// Set the CONTROL bit
	#if INTERFACE_FUNCTIONS == IF_2_SET_IF_1_GET
		ACC_REG(Local_u32Base + CAN_O_IF1CMSK) = (CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL | CAN_IF1CMSK_MASK | CAN_IF1CMSK_ARB);
		
		// shift the bas address to convert to IF 1 when adding the offsets of IF 2
		Local_u32Base = Local_u32Base - (CAN_O_IF2CRQ - CAN_O_IF1CRQ);
		
	#else
		ACC_REG(Local_u32Base + CAN_O_IF2CMSK) = (CAN_IF1CMSK_DATAA | CAN_IF1CMSK_DATAB | CAN_IF1CMSK_CONTROL | CAN_IF1CMSK_MASK | CAN_IF1CMSK_ARB);
	
	#endif

    // Clear Interrupt pending Flag
	ACC_REG(Local_u32Base + CAN_O_IF2CMSK) |= CAN_IF1CMSK_CLRINTPND;

    // Transfer the message object
    ACC_REG(Local_u32Base + CAN_O_IF2CRQ) = Copy_u8ObjID & CAN_IF1CRQ_MNUM_M;
	
	// Wait for busy bit to clear
    while(ACC_REG(Local_u32Base + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY);

    /***** Read the IF Registers *****/
    Local_u16MSKReg1 = ACC_REG(Local_u32Base + CAN_O_IF2MSK1);
    Local_u16MSKReg2 = ACC_REG(Local_u32Base + CAN_O_IF2MSK2);
    Local_u16ARBReg1 = ACC_REG(Local_u32Base + CAN_O_IF2ARB1);
    Local_u16ARBReg2 = ACC_REG(Local_u32Base + CAN_O_IF2ARB2);
    Local_u16MCTLReg = ACC_REG(Local_u32Base + CAN_O_IF2MCTL);
	
	
	/************************ Get Message Type *********************/
	if(Local_u16MCTLReg & CAN_IF1MCTL_RMTEN)
	{
		ptrReturnCANMsgObject -> u8MsgType = MSG_TYPE_TXRX_REMOTE;
		// Set remote frame Flag
		ptrReturnCANMsgObject->bRemote = 1;
	}
	else if (Local_u16MCTLReg & CAN_IF1MCTL_TXRQST)
	{
		ptrReturnCANMsgObject -> u8MsgType = MSG_TYPE_TX;
	}
	else
	{
		ptrReturnCANMsgObject -> u8MsgType = MSG_TYPE_RX;
	}
	
	
    /***** Get the Identifier & the Mask & the Extended ID Flag *****/
    if(Local_u16ARBReg2 & CAN_IF1ARB2_XTD)
    {
        // Get the 29 bit Identifier
        ptrReturnCANMsgObject->u32MsgID = (Local_u16ARBReg1 | ((Local_u16ARBReg2 & CAN_IF1ARB2_ID_M) << 16));
		
		// Get the 29 bit Mask
		ptrReturnCANMsgObject->u32MsgIDMask = (Local_u16MSKReg1 | ((Local_u16MSKReg2 & CAN_IF1MSK2_IDMSK_M) << 16));
		
		// Set Extended ID Flag
        ptrReturnCANMsgObject-> bExtendedID |= 1;
    }
    else
    {
        // Get the 11 bit Identifier
        ptrReturnCANMsgObject->u32MsgID = (Local_u16ARBReg2 & CAN_IF1ARB2_ID_M) >> 2;
		
		// Get the 11 bit Mask
		ptrReturnCANMsgObject->u32MsgIDMask = (Local_u16MSKReg2 & CAN_IF1MSK2_IDMSK_M) >> 2;
		
		// Clear Extended ID Flag
		ptrReturnCANMsgObject-> bExtendedID |= 0;
    }

    /***** Get remote frame Flag *****/
    if((!(Local_u16MCTLReg & CAN_IF1MCTL_TXRQST) && (Local_u16ARBReg2 & CAN_IF1ARB2_DIR)) ||	// RX_REMOTE
       ((Local_u16MCTLReg & CAN_IF1MCTL_TXRQST) && (!(Local_u16ARBReg2 & CAN_IF1ARB2_DIR))))	// TX_REMOTE
    {
        // Set remote frame Flag
		ptrReturnCANMsgObject->bRemote = 1;
    }
	else
	{
		// Clear remote frame Flag
		ptrReturnCANMsgObject->bRemote = 0;
	}


	/***** Get Direction in Filter Flag *****/
	if(Local_u16MSKReg2 & CAN_IF1MSK2_MDIR)
	{
		// Set Direction in Filter Flag 
		ptrReturnCANMsgObject->bDirectFilter = 1;
	}
	else
	{
		// Clear Direction in Filter Flag 
		ptrReturnCANMsgObject->bDirectFilter = 0;
	}
	
	
	/***** Get Not Last Entry Flag *****/
	if(Local_u16MCTLReg & CAN_IF1MCTL_EOB)
	{
		// Set Not Last Entry Flag 
		ptrReturnCANMsgObject->bNotLastEntry = 1;
	}
	else
	{
		// Clear Not Last Entry Flag 
		ptrReturnCANMsgObject->bNotLastEntry = 0;
	}
	

    /***** Get TX interrupt Flag *****/
    if(Local_u16MCTLReg & CAN_IF1MCTL_TXIE)
    {
        // Set the TX interrupt flag
		ptrReturnCANMsgObject->TxINTEnable = 1;
    }
	else
	{
		// Clear the TX interrupt flag
		ptrReturnCANMsgObject->TxINTEnable = 0;
	}
	
	
	/***** Get RX interrupt Flag *****/
    if(Local_u16MCTLReg & CAN_IF1MCTL_RXIE)
    {
        // Set the RX interrupt flag
		ptrReturnCANMsgObject->RxINTEnable = 1;
    }
	else
	{
		// Clear the RX interrupt flag
		ptrReturnCANMsgObject->RxINTEnable = 0;
	}


	/***** Get the Message Length *****/
	ptrReturnCANMsgObject->u8MsgLen = (Local_u16MCTLReg & CAN_IF1MCTL_DLC_M);
	
	
	/***** Get the Message Data *****/
	uint8 * Local_ptru8DistData = ptrReturnCANMsgObject -> ptru8MsgData;
	uint32 * Local_ptru32SourceData = (uint32 * ) (Local_u32Base + CAN_O_IF2DA1);
	uint8 Local_u8Counter = 0;
	
	for(Local_u8Counter = 0; Local_u8Counter < ptrReturnCANMsgObject -> u8MsgLen; Local_u8Counter++)
	{
		
		// Write the data 16 bits at a time
	    Local_ptru8DistData[Local_u8Counter] = (ACC_REG(Local_ptru32SourceData) & CAN_IF2DA1_HALFDATA_M);
		Local_u8Counter++;

		// but write the second byte if needed otherwise the value is zero.
		if(Local_u8Counter < ptrReturnCANMsgObject -> u8MsgLen)
		{
		    Local_ptru8DistData[Local_u8Counter] = ((ACC_REG(Local_ptru32SourceData) & CAN_IF2DA1_DATA_M) >> 8);
		}
		Local_ptru32SourceData++;
	}
	
	// clear the new data flag.
	ACC_REG(Local_u32Base + CAN_O_IF2CMSK) = CAN_IF1CMSK_NEWDAT;

	// Transfer the message object
	ACC_REG(Local_u32Base + CAN_O_IF2CRQ) = Copy_u8ObjID & CAN_IF1CRQ_MNUM_M;

	// Wait for busy bit to clear
	while(ACC_REG(Local_u32Base + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY);

}

/********************************************************************************************************************/
/****************************************** Notes for using Interrupt ***********************************************/
/********************************************************************************************************************/

// The Function sent to the CAN_voidSetCallBack Function MUST return void & has no parameters
// The Function sent to the CAN_voidSetCallBack Function MUST Clear the Interrupt pending Flag
// The Interrupt pending Flag is Cleared Automatically in CAN_voidGetMSG Function or by Calling CAN_voidClearINTPND
// The Prototypes of the Two Handlers CAN0_Handler & CAN1_Handler MUST be Written in the Startup file "tm4c123gh6pm_startup_ccs.c"
// The Name of the Two Handler Functions CAN0_Handler & CAN1_Handler MUST Written in the Interrupt Vector Table in their Correct Positions Vectors 55 & 56 (Interrupts Number 39 & 40)

void CAN_voidSetCallBack(uint8 Copy_u8Unit, void (*Copy_ptrFn)(void))
{	
    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));
    
	// Set the Base Address
	switch(Copy_u8Unit)
	{
		case 0:
			CAN0_CallBack = Copy_ptrFn;
			break;
		case 1:
			CAN1_CallBack = Copy_ptrFn;
			break;
		default:
			return;
	}
}

/********************************************************************************************************************/

void CAN_voidClearINTPND(uint8 Copy_u8Unit, uint8 Copy_u8ObjID)
{
    uint32 Local_u32Base;

    // Check the Number of CAN Unit
    assert((Copy_u8Unit == 0) ||(Copy_u8Unit == 1));

    // Set the Base Address
    switch(Copy_u8Unit)
    {
        case 0:
            Local_u32Base = CAN0_BASE;
            break;
        case 1:
            Local_u32Base = CAN1_BASE;
            break;
        default:
            return;
    }

    // Check on Object ID
    assert((Copy_u8ObjID <= 32) && (Copy_u8ObjID != 0));

    // Clear Interrupt pending Flag
    ACC_REG(Local_u32Base + CAN_O_IF2CMSK) |= CAN_IF1CMSK_CLRINTPND;
	
	// Transfer the message object
	ACC_REG(Local_u32Base + CAN_O_IF2CRQ) = Copy_u8ObjID & CAN_IF1CRQ_MNUM_M;

	// Wait for busy bit to clear
	while(ACC_REG(Local_u32Base + CAN_O_IF2CRQ) & CAN_IF1CRQ_BUSY);
}

/********************************************************************************************************************/

void CAN0_Handler(void)
{
    // CallBack Function
    CAN0_CallBack();
}

/********************************************************************************************************************/

void CAN1_Handler(void)
{
    // CallBack Function
    CAN1_CallBack();
}
