/* ******************************************************* */
/* Author	: Yasmin El Margoushy						   */
/* Date		: 7 sept 2020								   */
/* Version	: V03										   */
/* ******************************************************* */
#ifndef BIT_MATH_H
#define BIT_MATH_H

#define SET_BIT(VAR, BIT)			VAR |= (1 << (BIT))
#define CLR_BIT(VAR, BIT)			VAR &= ~(1 << (BIT))
#define GET_BIT(VAR, BIT)			((VAR >> BIT) & 1)
#define TOG_BIT(VAR, BIT)			VAR ^= (1 << (BIT))
#define ACC_REG(VAR)				(*((volatile uint32_t *)(VAR)))

#endif
