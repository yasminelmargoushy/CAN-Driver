/************************************************************************************************/
/* Author	: Yasmin El Margoushy						   										*/
/* Driver	: CAN - TM4C123GH6PM																*/
/* Date		: 7 sept 2020								   										*/
/* Version	: V01										   										*/
/************************************************************************************************/
#ifndef CAN_CONFIG_H
#define CAN_CONFIG_H

/*  OPTIONS:
 *      1-	IF_1_SET_IF_2_GET
 *      2-	IF_2_SET_IF_1_GET
 */

#define INTERFACE_FUNCTIONS		IF_1_SET_IF_2_GET


/*  OPTIONS:
 *      1-	CAN0_PORTB
 *      2-	CAN0_PORTE
 *		3-	CAN0_PORTF
 */
 
#define CAN0_PORT	CAN0_PORTB
 
 
/*  OPTIONS:
 *      	0 --> 7
 */
#define CAN0_PRIORITY	0


/*  OPTIONS:
 *      	0 --> 7
 */
#define CAN1_PRIORITY	1


/*  OPTIONS:
 *      1-  CAN_ERROR_INT_DISABLE
 *      2-  CAN_ERROR_INT_ENABLE
 */
#define CAN0_ERROR_INT  CAN_ERROR_INT_ENABLE


/*  OPTIONS:
 *      1-  CAN_ERROR_INT_DISABLE
 *      2-  CAN_ERROR_INT_ENABLE
 */
#define CAN1_ERROR_INT  CAN_ERROR_INT_DISABLE

#endif
